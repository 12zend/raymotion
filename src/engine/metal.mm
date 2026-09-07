#import <Foundation/Foundation.h>
#import <Metal/Metal.h>
#include "raymotion/metal.hpp"
#include "metal_types.h"
#include "metal_source.h"
#include <cstdlib>
#include <cstring>
#include <mutex>
#include <unordered_map>
#include <stdexcept>

static_assert(sizeof(metal_data::Triangle)==172, "Metal triangle ABI mismatch");
static_assert(sizeof(metal_data::BvhNode)==36, "Metal BVH ABI mismatch");
static_assert(sizeof(metal_data::Params)==108, "Metal parameters ABI mismatch");
namespace raymotion {
namespace {
std::string diagnostic(NSError* e) {
    return e ? std::string([[e localizedDescription] UTF8String]) : "Metal resource allocation failed";
}
struct Context {
    id<MTLDevice> device;
    id<MTLCommandQueue> queue;
    id<MTLComputePipelineState> pipeline, simd8, simd32;
    std::string error;
    bool raytracing=false;
    std::mutex mutex;
    id<MTLBuffer> buffers[5], textures, image, vertices, scratch;
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
        ~CacheGuard() { if (!completed) { context.acceleration=nil; context.previousPositions.clear(); } }
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
        F(ar); F(ag); F(ab); F(er); F(eg); F(eb); F(ior); F(rough); F(shader); F(metallic);
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
    p.nomis=std::getenv("UOW2_NOMIS")!=nullptr;
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
    NSUInteger width=std::min(NSUInteger(64),pipeline.maxTotalThreadsPerThreadgroup);
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
