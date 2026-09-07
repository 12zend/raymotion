#include <raymotion/runtime.hpp>
#include <fstream>
#include <iostream>
#include <chrono>
using namespace raymotion;
void check(bool b,const char* why) {if(!b) throw std::runtime_error(why);}
int main() {
    auto dir=std::filesystem::temp_directory_path()/("raymotion-obj-"+std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
    std::filesystem::create_directories(dir/"materials");
    struct Cleanup {std::filesystem::path p; ~Cleanup(){std::filesystem::remove_all(p);}} cleanup{dir};
    auto obj=(dir/"model.obj").string(),mtl=(dir/"materials/model.mtl").string();
    std::ofstream(obj)<<"v 0 0 0\nv 1 0 0\nv 0 1 0\nvt 0 0\nvt 1 0\nvt 0 1\nusemtl painted\nf 1/1 2/2 3/3\nusemtl light\nf -3 -2 -1\n";
    std::ofstream(mtl)<<"newmtl painted\nKd .5 .8 1\nPr .2\nPm .7\nmap_Kd color image.ppm\nnewmtl light\nKe 2 1 0\nNi 1.5\n";
    { std::ofstream image(dir/"materials/color image.ppm",std::ios::binary);image<<"P6\n2 2\n255\n"; const unsigned char pixels[]={255,0,0, 0,255,0, 0,0,255, 128,128,128};image.write(reinterpret_cast<const char*>(pixels),12); }
    Objects objects(dir.string(),8,8,1,true);
    auto plain=objects.init(obj),model=objects.init(obj,mtl);
    check(plain!=model && model==objects.init(obj,mtl),"cache must include MTL");
    check(model->tris.size()==2 && model->light_tri.size()==1,"material assignment / lights");
    const auto& t=model->tris[0];
    check(t.ar==.5 && t.rough==.2 && t.metallic==.7 && t.texture,"MTL fields");
    check(!model->tris[1].texture && model->tris[1].ior==1.5,"material switching");
    check(t.texture->sample(.25,.25).z==1 && t.texture->sample(.25,.75).x==1,"UV orientation");
    check(t.texture->sample(-.75,1.25).z==1,"UV wrapping");
    double gray=t.texture->sample(.75,.25).x;
    check(gray>.215 && gray<.217,"sRGB decoding");
    objects.push(model,{0,0,3});
    objects.push(model,{0,0,3},{},{1,1,1},{1,1,1},{},1,.5,0);
    objects.render();
    objects.push(model,{0,0,3});objects.render();
    check(std::filesystem::exists(dir/"f00000001.ppm"),"textured model across frames");
    bool failed=false;try {objects.init(obj,(dir/"missing.mtl").string());}catch(const std::runtime_error&){failed=true;}
    check(failed,"missing MTL diagnostic");
    std::ofstream(mtl)<<"newmtl painted\nmap_Kd missing.png\n";
    failed=false;try {Scene bad;load_obj_materials(bad,obj,mtl);}catch(const std::runtime_error&){failed=true;}
    check(failed,"missing texture diagnostic");
    std::cout<<"OBJ/MTL tests passed\n";
}
