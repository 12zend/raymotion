#import <Foundation/Foundation.h>
#import <Metal/Metal.h>
#include "raymotion/metal.hpp"
#include "metal_types.h"
#include "metal_source.h"
#include <algorithm>
#include <cstdlib>
#include <cstring>
#include <mutex>
#include <unordered_map>
#include <stdexcept>

static_assert(sizeof(metal_data::Triangle)==176, "Metal triangle ABI mismatch");
static_assert(sizeof(metal_data::BvhNode)==36, "Metal BVH ABI mismatch");
static_assert(sizeof(metal_data::Params)==184, "Metal parameters ABI mismatch");
static_assert(sizeof(metal_data::RestirGBuffer)==76, "ReSTIR G-buffer ABI mismatch");
static_assert(sizeof(metal_data::RestirReservoir)==60, "ReSTIR reservoir ABI mismatch");
namespace raymotion {
namespace {
std::string diagnostic(NSError* e) {
    return e ? std::string([[e localizedDescription] UTF8String]) : "Metal resource allocation failed";
}
struct Context {
    id<MTLDevice> device;
    id<MTLCommandQueue> queue;
    id<MTLComputePipelineState> pipeline, simd8, simd32;
    id<MTLComputePipelineState> restirGenerate, restirSpatial, restirShade;
    id<MTLComputePipelineState> denoiseTemporal, denoiseAtrous, denoiseTonemap;
    std::string error;
    bool raytracing=false;
    std::mutex mutex;
    id<MTLBuffer> buffers[5], textures, image, vertices, scratch;
    id<MTLBuffer> restirGbuf, restirInit, restirMerged;
    id<MTLBuffer> hdrShade, denA, histColor;
    // Previous-frame ReSTIR history for temporal reuse (video).
    id<MTLBuffer> histGbuf, histRes;
    metal_data::Camera histCam{};
    int histW=0, histH=0;
    uint64_t histSeed=0, histHash=0;
    bool histValid=false;
    id<MTLAccelerationStructure> acceleration;
    std::vector<metal_data::MetalVec3> previousPositions;
    unsigned refits=0;
    Context() {
        @autoreleasepool {
            device = MTLCreateSystemDefaultDevice();
            if (!device) { error="No Metal GPU found"; return; }
            if (@available(macOS 11.0, *)) raytracing=device.supportsRaytracing;
            if (const char* mode=std::getenv("RAYMOTION_METAL_TRAVERSAL"))
                if (std::string(mode)=="software") raytracing=false;
            NSError* e=nil;
            MTLCompileOptions* options=[MTLCompileOptions new];
            options.preprocessorMacros=@{@"USE_METAL_RT":@(raytracing)};
#if __MAC_OS_X_VERSION_MAX_ALLOWED >= 150000
            if (@available(macOS 15.0, *)) options.mathMode=MTLMathModeFast;
            else
#endif
            {
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wdeprecated-declarations"
                options.fastMathEnabled=YES;
#pragma clang diagnostic pop
            }
            id<MTLLibrary> library=[device newLibraryWithSource:@(metal_source) options:options error:&e];
            if (!library) { error=diagnostic(e); return; }
            pipeline=[device newComputePipelineStateWithFunction:[library newFunctionWithName:@"render_paths"] error:&e];
            if (!pipeline) { error=diagnostic(e); return; }
            for (unsigned lanes: {8u,32u}) {
                auto constants=[MTLFunctionConstantValues new];
                [constants setConstantValue:&lanes type:MTLDataTypeUInt atIndex:0];
                auto function=[library newFunctionWithName:@"render_paths_simd" constantValues:constants error:&e];
                auto specialized=function ? [device newComputePipelineStateWithFunction:function error:&e] : nil;
                if (!specialized) { error=diagnostic(e); return; }
                if (lanes==8) simd8=specialized; else simd32=specialized;
            }
            restirGenerate=[device newComputePipelineStateWithFunction:[library newFunctionWithName:@"restir_generate"] error:&e];
            if (!restirGenerate) { error=diagnostic(e); return; }
            restirSpatial=[device newComputePipelineStateWithFunction:[library newFunctionWithName:@"restir_spatial"] error:&e];
            if (!restirSpatial) { error=diagnostic(e); return; }
            restirShade=[device newComputePipelineStateWithFunction:[library newFunctionWithName:@"restir_shade"] error:&e];
            if (!restirShade) { error=diagnostic(e); return; }
            denoiseTemporal=[device newComputePipelineStateWithFunction:[library newFunctionWithName:@"denoise_temporal"] error:&e];
            if (!denoiseTemporal) { error=diagnostic(e); return; }
            denoiseAtrous=[device newComputePipelineStateWithFunction:[library newFunctionWithName:@"denoise_atrous"] error:&e];
            if (!denoiseAtrous) { error=diagnostic(e); return; }
            denoiseTonemap=[device newComputePipelineStateWithFunction:[library newFunctionWithName:@"denoise_tonemap"] error:&e];
            if (!denoiseTonemap) { error=diagnostic(e); return; }
            queue=[device newCommandQueue];
            if (!queue) error="Could not create Metal command queue";
        }
    }
};
metal_data::MetalVec3 pack(const Vec3& v) { return {float(v.x),float(v.y),float(v.z)}; }
}
bool render_image_metal(const Scene& scene, const Camera& cam, const RenderConfig& cfg,
                        uint64_t seed, std::vector<uint8_t>& output, std::string& error) {
 @autoreleasepool {
    if (cfg.width<=0 || cfg.height<=0 || cfg.spp<=0 || cfg.max_bounces<0 ||
        size_t(cfg.width)*size_t(cfg.height)>100000000) {
        error="Invalid Metal render dimensions or sample count"; return false;
    }
    static Context context;
    if (!context.error.empty()) { error=context.error; return false; }
    std::lock_guard<std::mutex> lock(context.mutex);
    struct CacheGuard {
        Context& context; bool completed=false;
        ~CacheGuard() { if (!completed) { context.acceleration=nil; context.previousPositions.clear(); context.histValid=false; } }
    } guard{context};
    std::vector<metal_data::MetalVec3> texels;
    std::unordered_map<const Texture*,int> offsets;
    std::vector<metal_data::Triangle> tris(scene.tris.size());
    for(size_t i=0;i<tris.size();++i) {
        const auto& s=scene.tris[i]; auto& t=tris[i];
#define V(field) t.field=pack(s.field)
        V(v0); V(v1); V(v2); V(n); V(nn); V(n0); V(n1); V(n2);
#undef V
#define F(field) t.field=s.field
        F(tu0); F(tv0); F(tu1); F(tv1); F(tu2); F(tv2);
        F(ar); F(ag); F(ab); F(er); F(eg); F(eb); F(ior); F(rough); F(shader); F(metallic); F(alpha);
#undef F
        t.texture_offset=-1;
        if(s.texture) {
            auto inserted=offsets.emplace(s.texture.get(),int(texels.size()));
            if(inserted.second) for(auto& pixel:s.texture->pixels) texels.push_back(pack(pixel));
            t.texture_offset=inserted.first->second;
            t.texture_width=s.texture->width; t.texture_height=s.texture->height;
        }
    }
    // Preorder nodes with escape links allow stackless GPU traversal.
    std::vector<metal_data::BvhNode> nodes;
    nodes.reserve(scene.nodes.size());
    std::vector<std::pair<int,bool>> todo;
    if (!context.raytracing && !scene.nodes.empty()) todo.emplace_back(0,false);
    while (!todo.empty()) {
        auto [index,exit]=todo.back(); todo.pop_back();
        if (exit) { nodes[index].right=nodes.size(); continue; }
        const auto& s=scene.nodes.at(index);
        int dest=nodes.size();
        metal_data::BvhNode t{};
        t.mn=pack(s.mn); t.mx=pack(s.mx); t.offset=s.offset; t.count=s.count;
        nodes.push_back(t);
        todo.emplace_back(dest,true);
        if (!s.is_leaf()) { todo.emplace_back(s.right,false); todo.emplace_back(s.left,false); }
    }
    std::vector<float> cdf(scene.light_cdf.begin(),scene.light_cdf.end());
    metal_data::Params p{};
#define C(field) p.camera.field=cam.field
    C(x); C(y); C(z); C(focal);
    C(m0); C(m1); C(m2); C(m3); C(m4); C(m5); C(m6); C(m7); C(m8);
#undef C
    p.width=cfg.width; p.height=cfg.height; p.spp=cfg.spp; p.bounces=cfg.max_bounces;
    p.nodes=scene.nodes.size(); p.lights=scene.light_tri.size(); p.light_total=scene.light_total;
    p.resolution=cfg.resolution; p.clamp_value=cfg.clamp; p.adapt_rel=cfg.adapt_rel; p.adapt_abs=cfg.adapt_abs;
    p.adapt_min=std::getenv("UOW2_NOADAPT") ? 0 : cfg.adapt_min;
    p.adapt_step=cfg.adapt_step>0 ? cfg.adapt_step : 8;
    p.opaque=std::all_of(tris.begin(),tris.end(),[](const auto& t) { return t.alpha>=1; });
    p.nomis=std::getenv("UOW2_NOMIS")!=nullptr;
    // ReSTIR DI: RAYMOTION_RESTIR=0 で完全無効 (従来 NEE 動作)。
    bool restir_off = std::getenv("RAYMOTION_RESTIR") && std::string(std::getenv("RAYMOTION_RESTIR"))=="0";
    p.restir_candidates = restir_off ? 1 : (cfg.restir_candidates>0 ? cfg.restir_candidates : 1);
    if (p.restir_candidates<1) p.restir_candidates=1;
    if (p.restir_candidates>32) p.restir_candidates=32;
    p.restir_spatial = restir_off ? 0 : (cfg.restir_spatial>0 ? cfg.restir_spatial : 0);
    if (p.restir_spatial<0) p.restir_spatial=0;
    if (p.restir_spatial>8) p.restir_spatial=8;
    p.restir_mcap = cfg.restir_mcap>0 ? cfg.restir_mcap : 128;
    if (p.restir_mcap<1) p.restir_mcap=1;
    p.restir_radius = cfg.restir_radius>0 ? float(cfg.restir_radius) : 16.0f;
    if (const char* v=std::getenv("RAYMOTION_RESTIR_CANDIDATES")) p.restir_candidates=std::max(1,std::atoi(v));
    if (const char* v=std::getenv("RAYMOTION_RESTIR_SPATIAL")) p.restir_spatial=std::max(0,std::atoi(v));
    auto upload=[&](id<MTLBuffer> __strong& target,const void* data,size_t size) -> bool {
        size_t capacity=std::max(size_t(4),size);
        if (!target || target.length<capacity)
            target=[context.device newBufferWithLength:capacity options:MTLResourceStorageModeShared];
        if (!target) return false;
        if (size) std::memcpy(target.contents,data,size);
        return true;
    };
    if (!upload(context.textures,texels.data(),texels.size()*sizeof(texels[0])) ||
        !upload(context.buffers[0],tris.data(),tris.size()*sizeof(tris[0])) ||
        !upload(context.buffers[1],nodes.data(),nodes.size()*sizeof(nodes[0])) ||
        !upload(context.buffers[2],scene.bvh_tri.data(),context.raytracing ? 0 : scene.bvh_tri.size()*sizeof(int)) ||
        !upload(context.buffers[3],scene.light_tri.data(),scene.light_tri.size()*sizeof(int)) ||
        !upload(context.buffers[4],cdf.data(),cdf.size()*sizeof(float))) {
        error="Metal scene buffer allocation failed"; return false;
    }
    size_t pixels=size_t(cfg.width)*cfg.height;
    if (!context.image || context.image.length<pixels*3)
        context.image=[context.device newBufferWithLength:pixels*3 options:MTLResourceStorageModeShared];
    id<MTLBuffer> result=context.image;
    if(!result) { error="Metal image buffer allocation failed"; return false; }
    id<MTLCommandBuffer> command=[context.queue commandBuffer];
    if (!command) { error="Metal command allocation failed"; return false; }
    if (context.raytracing) {
        std::vector<metal_data::MetalVec3> positions;
        positions.reserve(std::max(size_t(3),tris.size()*3));
        for (const auto& t:tris) { positions.push_back(t.v0); positions.push_back(t.v1); positions.push_back(t.v2); }
        if (positions.empty()) positions.resize(3);
        bool sameCount=positions.size()==context.previousPositions.size();
        bool changed=!sameCount || std::memcmp(positions.data(),context.previousPositions.data(),positions.size()*sizeof(positions[0]))!=0;
        if (changed || !context.acceleration) {
            if (!upload(context.vertices,positions.data(),positions.size()*sizeof(positions[0]))) {
                error="Metal vertex allocation failed"; return false;
            }
            auto geometry=[MTLAccelerationStructureTriangleGeometryDescriptor descriptor];
            geometry.vertexBuffer=context.vertices; geometry.vertexStride=sizeof(positions[0]);
            geometry.triangleCount=positions.size()/3; geometry.opaque=YES;
            auto descriptor=[MTLPrimitiveAccelerationStructureDescriptor descriptor];
            descriptor.geometryDescriptors=@[geometry]; descriptor.usage=MTLAccelerationStructureUsageRefit;
            auto sizes=[context.device accelerationStructureSizesWithDescriptor:descriptor];
            bool refit=sameCount && context.acceleration && context.refits<31;
            if (!refit) {
                context.acceleration=[context.device newAccelerationStructureWithSize:sizes.accelerationStructureSize];
                context.refits=0;
            }
            NSUInteger scratchSize=std::max(NSUInteger(4),std::max(sizes.buildScratchBufferSize,sizes.refitScratchBufferSize));
            if (!context.scratch || context.scratch.length<scratchSize)
                context.scratch=[context.device newBufferWithLength:scratchSize options:MTLResourceStorageModePrivate];
            if (!context.acceleration || !context.scratch) { error="Metal acceleration allocation failed"; return false; }
            auto builder=[command accelerationStructureCommandEncoder];
            if (!builder) { error="Metal acceleration encoder allocation failed"; return false; }
            if (refit) {
                [builder refitAccelerationStructure:context.acceleration descriptor:descriptor destination:context.acceleration scratchBuffer:context.scratch scratchBufferOffset:0];
                ++context.refits;
            } else {
                [builder buildAccelerationStructure:context.acceleration descriptor:descriptor scratchBuffer:context.scratch scratchBufferOffset:0];
            }
            [builder endEncoding];
            context.previousPositions=std::move(positions);
        }
    }
    // ReSTIR spatial path: G-buffer + reservoir reuse (low-spp quality).
    // Large frames fall back to RIS-only (old kernels with RIS NEE) to bound memory.
    // Transparent scenes (any alpha<1) also fall back: single-primary G-buffer
    // cannot average stochastic coverage correctly (tone-map nonlinearity).
    size_t restirBytes = pixels*(sizeof(metal_data::RestirGBuffer)+2*sizeof(metal_data::RestirReservoir));
    bool useSpatial = (p.restir_spatial>0 && p.lights>0 && p.bounces>0 && p.opaque && restirBytes<=size_t(512)*1024*1024);
    // Temporal history: reusable across video frames with identical scene bytes
    // and a fresh seed. Guards keep unrelated renders isolated:
    // same seed replays exactly (no history), changed geometry disables reuse.
    uint64_t sceneHash = uint64_t(tris.size()) * 0x9E3779B97F4A7C15ULL
        ^ uint64_t(scene.light_tri.size()) * 0xBF58476D1CE4E5B9ULL
        ^ uint64_t(std::hash<double>{}(scene.light_total));
    const unsigned char* tb = reinterpret_cast<const unsigned char*>(tris.data());
    for (size_t i = 0, n = tris.size()*sizeof(tris[0]); i < n; ++i) {
        sceneHash ^= uint64_t(tb[i]);
        sceneHash *= 0x100000001B3ULL;
    }
    bool haveHistory = useSpatial && context.histValid && context.histW==cfg.width && context.histH==cfg.height
        && context.histGbuf && context.histRes && seed != context.histSeed && sceneHash == context.histHash;
    if (const char* tv=std::getenv("RAYMOTION_RESTIR_TEMPORAL"))
        if (std::string(tv)=="0") haveHistory = false;
    p.hasHistory = haveHistory ? 1 : 0;
    p.prevCamera = context.histCam;
    if (useSpatial) {
        auto alloc=[&](id<MTLBuffer> __strong& target,size_t size) -> bool {
            if (!target || target.length<size)
                target=[context.device newBufferWithLength:std::max(size_t(4),size) options:MTLResourceStorageModeShared];
            return target!=nil;
        };
        if (!alloc(context.restirGbuf,pixels*sizeof(metal_data::RestirGBuffer)) ||
            !alloc(context.restirInit,pixels*sizeof(metal_data::RestirReservoir)) ||
            !alloc(context.restirMerged,pixels*sizeof(metal_data::RestirReservoir)) ||
            !alloc(context.hdrShade,pixels*3*sizeof(float)) ||
            !alloc(context.denA,pixels*3*sizeof(float)) ||
            !alloc(context.histColor,pixels*3*sizeof(float))) {
            error="Metal ReSTIR buffer allocation failed"; return false;
        }
        bool denoise = true;
        if (const char* dv=std::getenv("RAYMOTION_DENOISE"))
            if (std::string(dv)=="0") denoise = false;
        auto setScene=[&](id<MTLComputeCommandEncoder> enc) {
            for(int i=0;i<5;++i) [enc setBuffer:context.buffers[i] offset:0 atIndex:i];
            [enc setBytes:&p length:sizeof(p) atIndex:5];
            [enc setBytes:&seed length:sizeof(seed) atIndex:6];
            [enc setBuffer:result offset:0 atIndex:7];
            [enc setBuffer:context.textures offset:0 atIndex:9];
            if (context.raytracing) [enc setAccelerationStructure:context.acceleration atBufferIndex:8];
        };
        auto dispatch1D=[&](id<MTLComputePipelineState> pl) -> id<MTLComputeCommandEncoder> {
            id<MTLComputeCommandEncoder> enc=[command computeCommandEncoder];
            if (!enc) return nil;
            [enc setComputePipelineState:pl];
            setScene(enc);
            return enc;
        };
        // Pass 1: primary G-buffer + initial reservoir.
        {
            id<MTLComputeCommandEncoder> enc=dispatch1D(context.restirGenerate);
            if (!enc) { error="Metal command allocation failed"; return false; }
            [enc setBuffer:context.restirGbuf offset:0 atIndex:10];
            [enc setBuffer:context.restirInit offset:0 atIndex:11];
            NSUInteger w=std::min(NSUInteger(256),context.restirGenerate.maxTotalThreadsPerThreadgroup);
            [enc dispatchThreadgroups:MTLSizeMake((pixels+w-1)/w,1,1) threadsPerThreadgroup:MTLSizeMake(w,1,1)];
            [enc endEncoding];
        }
        // Pass 2: temporal + spatial reuse (reads init/history, writes merged).
        {
            id<MTLComputeCommandEncoder> enc=dispatch1D(context.restirSpatial);
            if (!enc) { error="Metal command allocation failed"; return false; }
            [enc setBuffer:context.restirGbuf offset:0 atIndex:10];
            [enc setBuffer:context.restirInit offset:0 atIndex:11];
            [enc setBuffer:context.restirMerged offset:0 atIndex:12];
            [enc setBuffer:context.histGbuf offset:0 atIndex:13];
            [enc setBuffer:context.histRes offset:0 atIndex:14];
            NSUInteger w=std::min(NSUInteger(256),context.restirSpatial.maxTotalThreadsPerThreadgroup);
            [enc dispatchThreadgroups:MTLSizeMake((pixels+w-1)/w,1,1) threadsPerThreadgroup:MTLSizeMake(w,1,1)];
            [enc endEncoding];
        }
        // Pass 3: shade with merged reservoir + spp-averaged indirect (linear HDR).
        {
            id<MTLComputeCommandEncoder> enc=dispatch1D(context.restirShade);
            if (!enc) { error="Metal command allocation failed"; return false; }
            [enc setBuffer:context.restirGbuf offset:0 atIndex:10];
            [enc setBuffer:context.restirMerged offset:0 atIndex:12];
            [enc setBuffer:context.hdrShade offset:0 atIndex:15];
            NSUInteger w=std::min(NSUInteger(256),context.restirShade.maxTotalThreadsPerThreadgroup);
            [enc dispatchThreadgroups:MTLSizeMake((pixels+w-1)/w,1,1) threadsPerThreadgroup:MTLSizeMake(w,1,1)];
            [enc endEncoding];
        }
        // Passes 4-7: edge-guided spatiotemporal denoise (or tonemap-only).
        if (denoise) {
            {
                id<MTLComputeCommandEncoder> enc=dispatch1D(context.denoiseTemporal);
                if (!enc) { error="Metal command allocation failed"; return false; }
                [enc setBuffer:context.restirGbuf offset:0 atIndex:10];
                [enc setBuffer:context.histGbuf offset:0 atIndex:13];
                [enc setBuffer:context.hdrShade offset:0 atIndex:15];
                [enc setBuffer:context.histColor offset:0 atIndex:16];
                [enc setBuffer:context.denA offset:0 atIndex:17];
                NSUInteger w=std::min(NSUInteger(256),context.denoiseTemporal.maxTotalThreadsPerThreadgroup);
                [enc dispatchThreadgroups:MTLSizeMake((pixels+w-1)/w,1,1) threadsPerThreadgroup:MTLSizeMake(w,1,1)];
                [enc endEncoding];
            }
            for (int step = 0; step < 2; ++step) {
                id<MTLComputeCommandEncoder> enc=dispatch1D(context.denoiseAtrous);
                if (!enc) { error="Metal command allocation failed"; return false; }
                [enc setBuffer:context.restirGbuf offset:0 atIndex:10];
                if (step == 0) {
                    [enc setBuffer:context.denA offset:0 atIndex:18];
                    [enc setBuffer:context.histColor offset:0 atIndex:19];
                } else {
                    [enc setBuffer:context.histColor offset:0 atIndex:18];
                    [enc setBuffer:context.denA offset:0 atIndex:19];
                }
                [enc setBytes:&step length:sizeof(step) atIndex:20];
                NSUInteger w=std::min(NSUInteger(256),context.denoiseAtrous.maxTotalThreadsPerThreadgroup);
                [enc dispatchThreadgroups:MTLSizeMake((pixels+w-1)/w,1,1) threadsPerThreadgroup:MTLSizeMake(w,1,1)];
                [enc endEncoding];
            }
            {
                id<MTLComputeCommandEncoder> enc=dispatch1D(context.denoiseTonemap);
                if (!enc) { error="Metal command allocation failed"; return false; }
                [enc setBuffer:context.denA offset:0 atIndex:18];
                NSUInteger w=std::min(NSUInteger(256),context.denoiseTonemap.maxTotalThreadsPerThreadgroup);
                [enc dispatchThreadgroups:MTLSizeMake((pixels+w-1)/w,1,1) threadsPerThreadgroup:MTLSizeMake(w,1,1)];
                [enc endEncoding];
            }
        } else {
            id<MTLComputeCommandEncoder> enc=dispatch1D(context.denoiseTonemap);
            if (!enc) { error="Metal command allocation failed"; return false; }
            [enc setBuffer:context.hdrShade offset:0 atIndex:18];
            NSUInteger w=std::min(NSUInteger(256),context.denoiseTonemap.maxTotalThreadsPerThreadgroup);
            [enc dispatchThreadgroups:MTLSizeMake((pixels+w-1)/w,1,1) threadsPerThreadgroup:MTLSizeMake(w,1,1)];
            [enc endEncoding];
        }
        [command commit]; [command waitUntilCompleted];
        if(command.status!=MTLCommandBufferStatusCompleted) {
            context.acceleration=nil; context.previousPositions.clear();
            error=diagnostic(command.error); return false;
        }
        output.resize(pixels*3); std::memcpy(output.data(),result.contents,output.size());
        guard.completed=true;
        // Publish this frame as next frame's temporal history (ping-pong swap;
        // generate/spatial fully overwrite the buffers they write).
        std::swap(context.restirGbuf, context.histGbuf);
        std::swap(context.restirMerged, context.histRes);
        if (denoise) std::swap(context.denA, context.histColor);
#define HC(field) context.histCam.field=float(cam.field)
        HC(x); HC(y); HC(z); HC(focal);
        HC(m0); HC(m1); HC(m2); HC(m3); HC(m4); HC(m5); HC(m6); HC(m7); HC(m8);
#undef HC
        context.histW=cfg.width; context.histH=cfg.height;
        context.histSeed=seed; context.histHash=sceneHash;
        context.histValid=true;
        return true;
    }
    id<MTLComputeCommandEncoder> encoder=[command computeCommandEncoder];
    if(!command || !encoder) { error="Metal command allocation failed"; return false; }
    unsigned lanes=1;
    if (cfg.spp>=8 && (p.adapt_min<=0 || p.adapt_step%8==0)) lanes=8;
    if (cfg.spp>=32 && (p.adapt_min<=0 || p.adapt_step%32==0)) lanes=32;
    auto pipeline=lanes==32 ? context.simd32 : lanes==8 ? context.simd8 : context.pipeline;
    if (pipeline.threadExecutionWidth%lanes!=0) { lanes=1; pipeline=context.pipeline; }
    [encoder setComputePipelineState:pipeline];
    for(int i=0;i<5;++i) [encoder setBuffer:context.buffers[i] offset:0 atIndex:i];
    [encoder setBytes:&p length:sizeof(p) atIndex:5];
    [encoder setBytes:&seed length:sizeof(seed) atIndex:6];
    [encoder setBuffer:result offset:0 atIndex:7];
    [encoder setBuffer:context.textures offset:0 atIndex:9];
    if (context.raytracing) [encoder setAccelerationStructure:context.acceleration atBufferIndex:8];
    NSUInteger width=std::min(NSUInteger(256),pipeline.maxTotalThreadsPerThreadgroup);
    size_t workItems=pixels*lanes;
    [encoder dispatchThreadgroups:MTLSizeMake((workItems+width-1)/width,1,1) threadsPerThreadgroup:MTLSizeMake(width,1,1)];
    [encoder endEncoding]; [command commit]; [command waitUntilCompleted];
    if(command.status!=MTLCommandBufferStatusCompleted) {
        context.acceleration=nil; context.previousPositions.clear();
        error=diagnostic(command.error); return false;
    }
    output.resize(pixels*3); std::memcpy(output.data(),result.contents,output.size());
    guard.completed=true;
    return true;
 }
}
}
