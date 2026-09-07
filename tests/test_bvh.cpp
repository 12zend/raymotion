#include <raymotion/bvh.hpp>
#include <raymotion/pathtrace.hpp>
#include <stdexcept>
using namespace raymotion;
void fill(Scene& s,double z,int n) {
    s.clear_triangles();
    for(int i=0;i<n;++i) {
        double x=i*3;
        s.add_triangle({x-1,-1,z},{x+1,-1,z},{x,1,z},0,0,0,0,0,0,
            {0,0,-1},{0,0,-1},{0,0,-1},1,1,1,0,0,0,0,1,0.5,0);
    }
}
int main() {
    Scene s;fill(s,3,40);build_bvh(s,2);
    fill(s,7,40);refit_bvh(s);
    PathTracer tracer(&s);
    auto hit=tracer.cast_ray({0,0,0},{0,0,1},100);
    if(!hit.hit || std::abs(hit.t-7)>1e-5) throw std::runtime_error("refit miss");
    Scene rebuilt=s;build_bvh(rebuilt,2);PathTracer other(&rebuilt);
    for(int i=0;i<40;++i) {
        auto a=tracer.cast_ray({i*3.,0,0},{0,0,1},100);
        auto b=other.cast_ray({i*3.,0,0},{0,0,1},100);
        if(a.hit!=b.hit || std::abs(a.t-b.t)>1e-5) throw std::runtime_error("refit differs from rebuild");
    }
    fill(s,9,1);refit_bvh(s);
    if(std::abs(tracer.cast_ray({0,0,0},{0,0,1},100).t-9)>1e-5) throw std::runtime_error("topology change miss");
    s.clear_triangles();refit_bvh(s);
    if(tracer.cast_ray({0,0,0},{0,0,1},100).hit) throw std::runtime_error("empty scene hit");
}
