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
        PathTracer pt(&s,12345); Vec3 sum{};
        int blocked=0;
        for(int i=0;i<20000;++i) {
            sum=sum+pt.pathtrace({0,0,0},{0,0,1},1);
            blocked+=pt.cast_alpha_ray({0,0,0},{0,0,1},1.5).tri>=0;
        }
        check(std::abs(sum.x/20000-opacity)<.02);
        check(std::abs(sum.y/20000-(1-opacity))<.02);
        check(std::abs(blocked/20000.-opacity)<.02);
    }
    Scene layers;
    for(int i=0;i<64;++i) surface(layers,1+i*.01,0,{1,0,0});
    surface(layers,2,1,{0,1,0}); build_bvh(layers);
    PathTracer pt(&layers); check(pt.pathtrace({0,0,0},{0,0,1},1).y==1);
    Scene clamped; surface(clamped,1,-1,{}); surface(clamped,2,2,{});
    check(clamped.tris[0].alpha==0 && clamped.tris[1].alpha==1);
    Objects objects("unused.png",1,1,1,false);
    auto model=std::make_shared<Scene>(clamped);
    objects.push(model);
    objects.push(model,{}, {},{1,1,1},{-1,-1,-1},{-1,-1,-1},{},{},{},.5);
}
