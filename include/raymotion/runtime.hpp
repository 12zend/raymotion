#pragma once
#include "raymotion/renderer.hpp"
#include "raymotion/obj.hpp"
#include "raymotion/bvh.hpp"
#include <memory>
#include <optional>
#include <unordered_map>
#include <stdexcept>
#include <filesystem>

namespace raymotion {
// Signals successful completion after the configured video frame limit.
struct RenderComplete {};

struct Resolution { double x, y; };

using FrameSink = std::function<void(const std::vector<uint8_t>&, int, int, int)>;

using Model = std::shared_ptr<const Scene>;
class Objects {
    Renderer renderer;
    FrameSink frame_sink;
    std::unordered_map<std::string, Model> models;
    std::vector<const Scene*> topology, previous;
    std::filesystem::path output;
    bool video;
    int frame = 0;
    int framerate;
    int max_frames;
    double timer = 0;
public:
    Camera& camera;
    const double& u_timer = timer;
    const Resolution u_resolution;
    Objects(std::string path, int width, int height, int sample, bool movie, int fps = 30, int frame_limit = 0)
        : output(std::move(path)), video(movie), framerate(fps), max_frames(frame_limit), camera(renderer.camera),
          u_resolution{double(width), double(height)} {
        if(fps <= 0) throw std::invalid_argument("framerate must be positive");
        if(frame_limit < 0) throw std::invalid_argument("frame limit must be nonnegative");
        renderer.configure(width,height,sample);
    }
    // A synchronous sink provides backpressure and owns presentation, never file output.
    Objects(FrameSink sink, int width, int height, int sample, int fps = 30)
        : Objects("", width, height, sample, true, fps) {
        if(!sink) throw std::invalid_argument("frame sink is required");
        frame_sink = std::move(sink);
    }
    Model init(const std::string& path, const std::string& mtl="") {
        const std::string key=path+std::string(1,'\0')+mtl;
        auto found=models.find(key);
        if(found!=models.end()) return found->second;
        auto model=std::make_shared<Scene>();
        if((mtl.empty()?add_obj_file(*model,path,0,0,0,1,1,1,1,0,0,0,1,0.5,0):load_obj_materials(*model,path,mtl))<0)
            throw std::runtime_error("cannot load OBJ: "+path);
        if(model->tris.empty()) throw std::runtime_error("OBJ has no valid triangles: "+path);
        models[key]=model;
        return model;
    }
    void push(const Model& model, Vec3 position={}, Vec3 rotation={}, Vec3 scale={1,1,1},
              Vec3 albedo={-1,-1,-1}, Vec3 emission={-1,-1,-1}, std::optional<double> refract={},
              std::optional<double> rougth={}, std::optional<double> metallic={}, std::optional<double> alpha={}) {
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
            int index=renderer.scene.add_triangle(point(t.v0),point(t.v1),point(t.v2),
                t.tu0,t.tv0,t.tu1,t.tv1,t.tu2,t.tv2,normal(t.n0),normal(t.n1),normal(t.n2),
                albedo.x<0?t.ar:albedo.x,albedo.y<0?t.ag:albedo.y,albedo.z<0?t.ab:albedo.z,
                emission.x<0?t.er:emission.x,emission.y<0?t.eg:emission.y,emission.z<0?t.eb:emission.z,
                metallic.value_or(t.metallic),refract.value_or(t.ior),rougth.value_or(t.rough),t.shader,alpha.value_or(t.alpha));
            if(index>=0) renderer.scene.tris[index].texture=t.texture;
        }
    }
    void render() {
        if(video && max_frames > 0 && frame >= max_frames) throw RenderComplete{};
        if(!video && frame>0) throw std::runtime_error("PNG requires exactly one object.render()");
        if(topology==previous && frame%32!=0) refit_bvh(renderer.scene);
        else renderer.build();
        previous=topology;
        camera.update_trig(); camera.update_focal();
        bool ok;
        if(frame_sink) {
            auto pixels = renderer.render(12345 + frame);
            frame_sink(pixels, renderer.config.width, renderer.config.height, frame);
            ok = true;
        } else if(video) {
            char name[40]; std::snprintf(name,sizeof(name),"f%08d.ppm",frame);
            ok=renderer.render_to_ppm((output/name).string(),12345+frame);
        } else ok=renderer.render_to_png(output.string());
        if(!ok) throw std::runtime_error("cannot write rendered frame");
        ++frame;
        timer = double(frame) / framerate;
        renderer.scene.clear_triangles(); topology.clear();
        if(video && max_frames > 0 && frame >= max_frames) throw RenderComplete{};
    }
    void finish() {if(frame==0) throw std::runtime_error("no object.render() executed");}
};
}
