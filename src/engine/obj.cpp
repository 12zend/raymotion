#include "raymotion/obj.hpp"
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <array>

namespace raymotion {
int add_obj_tokens(Scene& scene,const std::string& data,double ox,double oy,double oz,
 double scale,double ar,double ag,double ab,double er,double eg,double eb,
 double ior,double rough,int shader,double metallic) {
    std::vector<Vec3> vertices,normals,uv;
    std::istringstream input(data);std::string line;
    int added=0;
    auto index=[](const std::string& text,size_t size) {
        size_t used=0;int i=std::stoi(text,&used);
        if(used!=text.size() || i==0) throw std::runtime_error("invalid OBJ index");
        int result=i>0?i-1:int(size)+i;
        if(result<0 || result>=int(size)) throw std::runtime_error("OBJ index out of range");
        return result;
    };
    while(std::getline(input,line)) {
        line=line.substr(0,line.find('#'));
        std::istringstream row(line);std::string kind;row>>kind;
        if(kind=="v" || kind=="vn" || kind=="vt") {
            Vec3 v;
            if(!(row>>v.x>>v.y)) throw std::runtime_error("invalid OBJ vertex");
            if(kind!="vt" && !(row>>v.z)) throw std::runtime_error("invalid OBJ vertex");
            if(kind=="v") vertices.push_back({ox+v.x*scale,oy+v.y*scale,oz-v.z*scale});
            else if(kind=="vn") normals.push_back({v.x,v.y,-v.z});
            else uv.push_back(v);
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
                if(scene.add_triangle(p[0],p[1],p[2],t[0].x,t[0].y,t[1].x,t[1].y,t[2].x,t[2].y,
                    n[0],n[1],n[2],ar,ag,ab,er,eg,eb,metallic,ior,rough,shader)>=0)++added;
            }
        }
    }
    return added;
}
int add_obj_file(Scene& scene,const std::string& path,double ox,double oy,double oz,
 double scale,double ar,double ag,double ab,double er,double eg,double eb,
 double ior,double rough,int shader,double metallic) {
    std::ifstream file(path);if(!file)return -1;
    std::ostringstream data;data<<file.rdbuf();
    return add_obj_tokens(scene,data.str(),ox,oy,oz,scale,ar,ag,ab,er,eg,eb,ior,rough,shader,metallic);
}
}
