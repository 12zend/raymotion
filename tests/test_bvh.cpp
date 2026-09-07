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
    if(s.nodes[0].mn.z!=7 || s.nodes[0].mx.z!=7) throw std::runtime_error("refit bounds");
    Scene rebuilt=s;build_bvh(rebuilt,2);
    if(s.nodes[0].mn.x!=rebuilt.nodes[0].mn.x || s.nodes[0].mx.x!=rebuilt.nodes[0].mx.x)
        throw std::runtime_error("refit differs from rebuild");
    fill(s,9,1);refit_bvh(s);
    if(s.nodes[0].mn.z!=9 || s.bvh_tri.size()!=1) throw std::runtime_error("topology change");
    s.clear_triangles();refit_bvh(s);
    if(!s.nodes.empty()) throw std::runtime_error("empty scene");
}
