#include <raymotion/metal.hpp>
#include <raymotion/bvh.hpp>
#include <chrono>
#include <algorithm>
#include <cstdlib>
#include <iostream>
#include <stdexcept>
using namespace raymotion;
void check(bool ok, const char* why) { if(!ok) throw std::runtime_error(why); }
void triangle(Scene& s,Vec3 a,Vec3 b,Vec3 c,Vec3 color,Vec3 emission={},double metal=0,double ior=1,double rough=.5) {
    Vec3 n=normalize(cross(b-a,c-a));
    s.add_triangle(a,b,c,0,0,0,0,0,0,n,n,n,color.x,color.y,color.z,
        emission.x,emission.y,emission.z,metal,ior,rough,0);
}
void quad(Scene& s, Vec3 a,Vec3 b,Vec3 c,Vec3 d,Vec3 color,Vec3 emission={},double metal=0,double ior=1,double rough=.5) {
    triangle(s,a,b,c,color,emission,metal,ior,rough);
    triangle(s,a,c,d,color,emission,metal,ior,rough);
}
int main() {
    RenderConfig cfg; cfg.width=96; cfg.height=64; cfg.spp=512; cfg.adapt_min=0;
    Camera cam; cam.focal=cfg.width*.8660254; cam.update_trig();
    Scene scene; std::vector<uint8_t> gpu; std::string error;
    if(!render_image_metal(scene,cam,cfg,12345,gpu,error)) {
        std::cerr<<error<<"\n";
        bool unavailable=error=="No Metal GPU found" || error=="Metal backend is not available in this build";
        return unavailable && !std::getenv("RAYMOTION_REQUIRE_METAL") ? 77 : 1;
    }
    check(gpu==std::vector<uint8_t>(cfg.width*cfg.height*3,0),"empty scene is not black");
    // Emission, back wall, diffuse floor, metal and rough/smooth transmission.
    quad(scene,{-4,-2,7},{-4,3,7},{4,3,7},{4,-2,7},{.7,.7,.7});
    quad(scene,{-4,-2,1},{-4,-2,7},{4,-2,7},{4,-2,1},{.7,.2,.1});
    quad(scene,{-1,2,3},{1,2,3},{1,2,5},{-1,2,5},{1,1,1},{6,6,6});
    quad(scene,{-2,-1,4},{-2,1,4},{-1,1,4},{-1,-1,4},{.8,.7,.3},{},1);
    quad(scene,{0,-1,4},{0,1,4},{1,1,4},{1,-1,4},{1,1,1},{},0,1.5,.02);
    quad(scene,{1,-1,5},{1,1,5},{2,1,5},{2,-1,5},{1,1,1},{},0,1.5,.3);
    auto texture=std::make_shared<Texture>(); texture->width=2; texture->height=2;
    texture->pixels={{1,.1,.1},{.1,1,.1},{.1,.1,1},{.5,.5,.5}};
    for(auto& t:scene.tris) if(t.er+t.eg+t.eb==0) {
        t.texture=texture;t.tu0=-.2;t.tv0=.1;t.tu1=1.4;t.tv1=.2;t.tu2=.3;t.tv2=1.7;
    }
    build_bvh(scene,2);
    check(render_image_metal(scene,cam,cfg,12345,gpu,error),error.c_str());
    check(std::any_of(gpu.begin(),gpu.end(),[](uint8_t v){return v>0;}),"lit scene is black");
    std::vector<uint8_t> repeated;
    check(render_image_metal(scene,cam,cfg,12345,repeated,error),error.c_str());
    check(gpu==repeated,"Metal output is not deterministic");
    // Updated geometry and CDF must be uploaded on the next frame.
    scene.clear_triangles(); scene.clear_bvh();
    quad(scene,{-20,-20,3},{-20,20,3},{20,20,3},{20,-20,3},{1,1,1},{1,0,0});
    build_bvh(scene); cfg.adapt_min=16;
    check(render_image_metal(scene,cam,cfg,98765,gpu,error),error.c_str());
    for(size_t i=0;i<gpu.size();i+=3) check(gpu[i]==127 && gpu[i+1]==0 && gpu[i+2]==0,"emission/adaptive/upload mismatch");
    // Alpha coverage on emissive surfaces, with analytic expected coverage.
    cfg.spp=2048; cfg.adapt_min=0; cfg.width=16; cfg.height=16;
    cam.focal=cfg.width*.8660254;
    for(double opacity : {0., .5, 1.}) {
        for(auto& t:scene.tris) t.alpha=opacity;
        check(render_image_metal(scene,cam,cfg,98765,gpu,error),error.c_str());
        if(opacity==.5) {
            double mean=0;
            for(size_t i=0;i<gpu.size();i+=3) mean+=gpu[i];
            check(std::abs(mean/(cfg.width*cfg.height)-85)<3,"alpha coverage mismatch");
        }
        if(opacity==0) for(auto value:gpu) check(value==0,"transparent emission visible");
        if(opacity==1) check(gpu[0]==127,"opaque emission changed");
    }
    // Exercise reused buffers, odd dispatch tails, refits and the periodic rebuild.
    cfg.width=37;cfg.height=23;cfg.spp=9;cfg.adapt_min=0;
    for(int frame=0;frame<34;++frame) {
        scene.clear_triangles(); scene.clear_bvh();
        double x=(frame%2)*100.;
        quad(scene,{x-20,-20,3},{x-20,20,3},{x+20,20,3},{x+20,-20,3},{1,1,1},{1,0,0});
        build_bvh(scene);
        check(render_image_metal(scene,cam,cfg,12345,gpu,error),error.c_str());
        for(size_t i=0;i<gpu.size();i+=3) check(gpu[i]==(frame%2 ? 0 : 127) && gpu[i+1]==0 && gpu[i+2]==0,"refit or tail mismatch");
    }
}
