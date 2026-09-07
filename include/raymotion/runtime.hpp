#pragma once
#include "raymotion/renderer.hpp"
#include "raymotion/obj.hpp"
#include "raymotion/bvh.hpp"
#include <memory>
#include <unordered_map>
#include <stdexcept>
#include <filesystem>

namespace raymotion {
using Model = std::shared_ptr<const Scene>;
class Objects {
    Renderer renderer;
    std::unordered_map<std::string, Model> models;
    std::vector<const Scene*> topology, previous;
    std::filesystem::path output;
    bool video;
    int frame = 0;
public:
    Camera& camera;
    Objects(std::string path, int width, int height, int sample, bool movie)
        : output(std::move(path)), video(movie), camera(renderer.camera) {
        renderer.configure(width,height,sample);
    }
    Model init(const std::string& path) {
        auto found=models.find(path);
        if(found!=models.end()) return found->second;
        auto model=std::make_shared<Scene>();
        if(add_obj_file(*model,path,0,0,0,1,1,1,1,0,0,0,1,0.5,0)<0)
            throw std::runtime_error("cannot load OBJ: "+path);
        if(model->tris.empty()) throw std::runtime_error("OBJ has no valid triangles: "+path);
        models[path]=model;
        return model;
    }
    void push(const Model& model, Vec3 position={}, Vec3 rotation={}, Vec3 scale={1,1,1},
              Vec3 albedo={1,1,1}, Vec3 emission={}, double refract=1,
              double rougth=0.5, double metallic=0) {
        if(!model) throw std::runtime_error("null object");
        if(scale.x==0 || scale.y==0 || scale.z==0) throw std::runtime_error("scale must be nonzero");
        topology.push_back(model.get());
        auto rotate=[&](Vec3 v) {
            double c=cos_deg(rotation.x),s=sin_deg(rotation.x);
            v={v.x,c*v.y-s*v.z,s*v.y+c*v.z};
            c=cos_deg(rotation.y);s=sin_deg(rotation.y);
            v={c*v.x+s*v.z,v.y,-s*v.x+c*v.z};
            c=cos_deg(rotation.z);s=sin_deg(rotation.z);
            return Vec3{c*v.x-s*v.y,s*v.x+c*v.y,v.z};
        };
        auto point=[&](Vec3 v){return rotate({v.x*scale.x,v.y*scale.y,v.z*scale.z})+position;};
        auto normal=[&](Vec3 v){return normalize(rotate({v.x/scale.x,v.y/scale.y,v.z/scale.z}));};
        for(const auto& t:model->tris) {
            renderer.scene.add_triangle(point(t.v0),point(t.v1),point(t.v2),
                t.tu0,t.tv0,t.tu1,t.tv1,t.tu2,t.tv2,normal(t.n0),normal(t.n1),normal(t.n2),
                albedo.x,albedo.y,albedo.z,emission.x,emission.y,emission.z,metallic,refract,rougth,0);
        }
    }
    void render() {
        if(!video && frame>0) throw std::runtime_error("PNG requires exactly one object.render()");
        if(topology==previous && frame%32!=0) refit_bvh(renderer.scene);
        else renderer.build();
        previous=topology;
        camera.update_trig(); camera.update_focal();
        bool ok;
        if(video) {
            char name[40]; std::snprintf(name,sizeof(name),"f%08d.ppm",frame);
            ok=renderer.render_to_ppm((output/name).string(),12345+frame);
        } else ok=renderer.render_to_png(output.string());
        if(!ok) throw std::runtime_error("cannot write rendered frame");
        ++frame;
        renderer.scene.clear_triangles(); topology.clear();
    }
    void finish() {if(frame==0) throw std::runtime_error("no object.render() executed");}
};
}
