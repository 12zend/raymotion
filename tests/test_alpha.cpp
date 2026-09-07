#include <raymotion/metal.hpp>
#include <raymotion/runtime.hpp>
#include <cmath>
#include <stdexcept>
using namespace raymotion;
void check(bool ok) { if(!ok) throw std::runtime_error("alpha regression"); }
void surface(Scene& s, double z, double alpha, Vec3 emission) {
    Vec3 n{0,0,-1};
    s.add_triangle({-10,-10,z},{0,10,z},{10,-10,z},0,0,0,0,0,0,n,n,n,
                   0,0,0,emission.x,emission.y,emission.z,0,1,.5,0,alpha);
}
int main() {
    for(double opacity : {0., .5, 1.}) {
        Scene s; surface(s,1,opacity,{1,0,0}); surface(s,2,1,{0,1,0}); build_bvh(s);
        RenderConfig cfg; cfg.width=8; cfg.height=8; cfg.spp=4096; cfg.adapt_min=0;
        Camera camera; std::vector<uint8_t> image; std::string error;
        check(render_image_metal(s,camera,cfg,12345,image,error));
        double red=0,green=0;
        for(size_t i=0;i<image.size();i+=3) {red+=image[i];green+=image[i+1];}
        check(std::abs(red/64-255*opacity/(1+opacity))<3);
        check(std::abs(green/64-255*(1-opacity)/(2-opacity))<3);
    }
    Scene layers;
    for(int i=0;i<64;++i) surface(layers,1+i*.01,0,{1,0,0});
    surface(layers,2,1,{0,1,0}); build_bvh(layers);
    RenderConfig cfg; cfg.width=1;cfg.height=1;cfg.spp=1;
    Camera camera;std::vector<uint8_t> image;std::string error;
    check(render_image_metal(layers,camera,cfg,12345,image,error));
    check(image[0]==0 && image[1]==127);
    Scene clamped; surface(clamped,1,-1,{}); surface(clamped,2,2,{});
    check(clamped.tris[0].alpha==0 && clamped.tris[1].alpha==1);
    Objects objects("unused.png",1,1,1,false);
    auto model=std::make_shared<Scene>(clamped);
    objects.push(model);
    objects.push(model,{}, {},{1,1,1},{-1,-1,-1},{-1,-1,-1},{},{},{},.5);
}
