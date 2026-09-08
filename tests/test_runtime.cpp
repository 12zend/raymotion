// Geometry regression test: a recording renderer isolates CPU scene preparation
// from GPU availability and stochastic sampling.
#include "raymotion/runtime.hpp"
#include <iostream>
#include <chrono>

namespace {
int builds=0, frames=0;
bool benchmark=false;
std::vector<raymotion::Triangle> captured;
void require(bool ok) { if(!ok) throw std::runtime_error("runtime geometry regression"); }
}
namespace raymotion {
Renderer::Renderer() = default;
void Renderer::configure(int w,int h,int spp,int b) {
    config.width=w; config.height=h; config.spp=spp; config.max_bounces=b;
}
void Renderer::build(int leaf) { ++builds; build_bvh(scene,leaf); }
std::vector<uint8_t> Renderer::render(uint64_t,ProgressFn) {
    if(!benchmark) captured=scene.tris;
    ++frames; return std::vector<uint8_t>(3);
}
bool Renderer::render_to_ppm(const std::string&,uint64_t,ProgressFn) { return true; }
bool Renderer::render_to_png(const std::string&,uint64_t,ProgressFn) { return true; }
}
int main(int argc, char**) {
    using namespace raymotion;
    auto model=std::make_shared<Scene>();
    model->add_triangle({0,0,0},{1,0,0},{0,1,0},0,0,0,0,0,0,
                       {0,0,1},{0,0,1},{0,0,1},1,1,1,0,0,0,0,1,0.5,0);
    if(argc>1) {
        benchmark=true;
        const auto triangle=model->tris.front();
        for(int i=1;i<50000;++i) model->tris.push_back(triangle);
        Objects timed([](const auto&,int,int,int){},1,1,1);
        // Warm-up includes initial scene construction; measure steady-state only.
        timed.push(model,{0,0,3}); timed.render();
        const auto start=std::chrono::steady_clock::now();
        for(int i=0;i<100;++i) { timed.push(model,{0,0,3}); timed.render(); }
        std::cout << "CPU scene preparation ms/frame: "
            << std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-start).count()/100 << "\n";
        return 0;
    }
    Objects object([](const auto&,int,int,int){},1,1,1);
    object.push(model,{0,0,3}); object.render();
    require(builds==1 && captured.size()==1 && captured[0].v0.z==3);
    object.camera.x=2;
    object.push(model,{0,0,3}); object.render();
    require(builds==1 && frames==2 && object.u_timer==2.0/30);
    object.push(model,{0,0,4},{},{1,1,1},{0.2,0.3,0.4}); object.render();
    require(captured[0].v0.z==4 && captured[0].ar==0.2);
    object.render(); require(captured.empty());
    object.push(model,{0,0,3}); object.push(model,{0,0,5}); object.render();
    require(captured.size()==2 && captured[1].v0.z==5);
    object.push(model,{0,0,5}); object.push(model,{0,0,3}); object.render();
    require(captured[0].v0.z==5 && captured[1].v0.z==3);
    object.push(model,{}, {0,0,90}, {2,1,1}); object.render();
    require(std::abs(captured[0].v1.y-2)<1e-12 && std::abs(captured[0].v1.x)<1e-12);
    std::cout << "runtime geometry passed\n";
}
