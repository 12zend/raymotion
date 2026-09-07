#include "raymotion/obj.hpp"
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <array>
#include <algorithm>
#include <filesystem>
#include <unordered_map>
#define STB_IMAGE_IMPLEMENTATION
#include "vendor/stb_image.h"

namespace raymotion {
struct Material {
    Vec3 kd{1,1,1}, ke{};
    double ior=1, rough=.5, metallic=0;
    std::shared_ptr<const Texture> texture;
    bool explicit_rough=false;
};
using Materials=std::unordered_map<std::string,Material>;
static int parse_obj(Scene& scene,const std::string& data,double ox,double oy,double oz,
 double scale,double ar,double ag,double ab,double er,double eg,double eb,
 double ior,double rough,int shader,double metallic, const Materials& materials) {
    std::vector<Vec3> vertices,normals,uv;
    std::istringstream input(data);std::string line;
    int added=0;
    Material mat{{ar,ag,ab},{er,eg,eb},ior,rough,metallic,{}};
    const Material fallback=mat;
    auto index=[](const std::string& text,size_t size) {
        size_t used=0;int i=std::stoi(text,&used);
        if(used!=text.size() || i==0) throw std::runtime_error("invalid OBJ index");
        int result=i>0?i-1:int(size)+i;
        if(result<0 || result>=int(size)) throw std::runtime_error("OBJ index out of range");
        return result;
    };
    while(std::getline(input,line)) {
        line=line.substr(0,line.find('#'));
        auto end=line.find_last_not_of(" \t\r");
        if(end!=std::string::npos) line.resize(end+1);
        std::istringstream row(line);std::string kind;row>>kind;
        if(kind=="v" || kind=="vn" || kind=="vt") {
            Vec3 v;
            if(!(row>>v.x>>v.y)) throw std::runtime_error("invalid OBJ vertex");
            if(kind!="vt" && !(row>>v.z)) throw std::runtime_error("invalid OBJ vertex");
            if(!std::isfinite(v.x) || !std::isfinite(v.y) || !std::isfinite(v.z)) throw std::runtime_error("non-finite OBJ vertex");
            if(kind=="v") vertices.push_back({ox+v.x*scale,oy+v.y*scale,oz-v.z*scale});
            else if(kind=="vn") normals.push_back({v.x,v.y,-v.z});
            else uv.push_back(v);
        } else if(kind=="usemtl") {
            std::string name; std::getline(row>>std::ws,name);
            auto found=materials.find(name); mat=found==materials.end()?fallback:found->second;
        } else if(kind=="f") {
            struct Corner {int v=-1,t=-1,n=-1;};
            std::vector<Corner> face;std::string value;
            while(row>>value) {
                std::array<std::string,3> parts;std::istringstream fields(value);
                for(auto& part:parts) std::getline(fields,part,'/');
                Corner c;c.v=index(parts[0],vertices.size());
                if(!parts[1].empty()) c.t=index(parts[1],uv.size());
                if(!parts[2].empty()) c.n=index(parts[2],normals.size());
                face.push_back(c);
            }
            if(face.size()<3) throw std::runtime_error("OBJ face needs three vertices");
            for(size_t i=1;i+1<face.size();++i) {
                Corner cs[3]={face[0],face[i+1],face[i]};
                Vec3 p[3],n[3],t[3];
                for(int j=0;j<3;++j) {p[j]=vertices[cs[j].v];if(cs[j].t>=0)t[j]=uv[cs[j].t];}
                Vec3 geometric=normalize(cross(p[1]-p[0],p[2]-p[0]));
                for(int j=0;j<3;++j)n[j]=cs[j].n>=0?normals[cs[j].n]:geometric;
                int tid=scene.add_triangle(p[0],p[1],p[2],t[0].x,t[0].y,t[1].x,t[1].y,t[2].x,t[2].y,
                    n[0],n[1],n[2],mat.kd.x,mat.kd.y,mat.kd.z,mat.ke.x,mat.ke.y,mat.ke.z,mat.metallic,mat.ior,mat.rough,shader);
                if(tid>=0) {
                    if(cs[0].t>=0 && cs[1].t>=0 && cs[2].t>=0) scene.tris[tid].texture=mat.texture;
                    ++added;
                }
            }
        }
    }
    return added;
}
int add_obj_tokens(Scene& scene,const std::string& data,double ox,double oy,double oz,
 double scale,double ar,double ag,double ab,double er,double eg,double eb,
 double ior,double rough,int shader,double metallic) {
    return parse_obj(scene,data,ox,oy,oz,scale,ar,ag,ab,er,eg,eb,ior,rough,shader,metallic,{});
}
int load_obj_materials(Scene& scene,const std::string& obj,const std::string& mtl) {
    Materials materials;
    std::ifstream file(mtl);
    if(!file) throw std::runtime_error("cannot load MTL: "+mtl);
    Material* current=nullptr;
    std::string line;
    std::unordered_map<std::string,std::shared_ptr<const Texture>> textures;
    while(std::getline(file,line)) {
        line=line.substr(0,line.find('#'));
        auto end=line.find_last_not_of(" \t\r");
        if(end!=std::string::npos) line.resize(end+1);
        std::istringstream row(line); std::string key; row>>key;
        if(key=="newmtl") {std::string name; std::getline(row>>std::ws,name); current=&materials[name]; continue;}
        if(!current) continue;
        auto& m=*current;
        if(key=="Kd" || key=="Ke") {
            auto& c=key=="Kd"?m.kd:m.ke;
            if(!(row>>c.x>>c.y>>c.z) || !std::isfinite(c.x) || !std::isfinite(c.y) || !std::isfinite(c.z)) throw std::runtime_error("invalid MTL color: "+mtl);
        } else if(key=="Ni" || key=="Pr" || key=="Pm" || key=="Ns") {
            double value; if(!(row>>value) || !std::isfinite(value)) throw std::runtime_error("invalid MTL value: "+mtl);
            if(key=="Ni") m.ior=value;
            if(key=="Pr") {m.rough=std::clamp(value,0.0,1.0); m.explicit_rough=true;}
            if(key=="Pm") m.metallic=std::clamp(value,0.0,1.0);
            if(key=="Ns" && !m.explicit_rough) m.rough=std::sqrt(2.0/(std::max(0.0,value)+2.0));
        } else if(key=="map_Kd") {
            std::string name; std::getline(row>>std::ws,name);
            if(name.empty() || name[0]=='-') throw std::runtime_error("map_Kd requires a texture path (options unsupported): "+mtl);
            if(name.size()>1 && name.front()=='"' && name.back()=='"') name=name.substr(1,name.size()-2);
            std::replace(name.begin(),name.end(),'\\','/');
            auto path=(std::filesystem::path(mtl).parent_path()/name).lexically_normal().string();
            auto found=textures.find(path);
            if(found!=textures.end()) {m.texture=found->second; continue;}
            auto texture=std::make_shared<Texture>(); int channels;
            unsigned char* data=stbi_load(path.c_str(),&texture->width,&texture->height,&channels,3);
            if(!data) throw std::runtime_error("cannot load texture: "+path);
            std::unique_ptr<unsigned char,decltype(&stbi_image_free)> guard(data,stbi_image_free);
            texture->pixels.resize(size_t(texture->width)*texture->height);
            auto linear=[](unsigned char c) {double v=c/255.0; return v<=.04045?v/12.92:std::pow((v+.055)/1.055,2.4);};
            for(size_t i=0;i<texture->pixels.size();++i) texture->pixels[i]={linear(data[i*3]),linear(data[i*3+1]),linear(data[i*3+2])};
            m.texture=texture; textures[path]=texture;
        }
    }
    std::ifstream input(obj); if(!input) return -1;
    std::ostringstream data; data<<input.rdbuf();
    return parse_obj(scene,data.str(),0,0,0,1,1,1,1,0,0,0,1,.5,0,0,materials);
}
int add_obj_file(Scene& scene,const std::string& path,double ox,double oy,double oz,
 double scale,double ar,double ag,double ab,double er,double eg,double eb,
 double ior,double rough,int shader,double metallic) {
    std::ifstream file(path);if(!file)return -1;
    std::ostringstream data;data<<file.rdbuf();
    return add_obj_tokens(scene,data.str(),ox,oy,oz,scale,ar,ag,ab,er,eg,eb,ior,rough,shader,metallic);
}
}
