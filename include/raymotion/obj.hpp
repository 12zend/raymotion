#pragma once
// OBJ loader: triangles and polygon fans, optional UV/normals, negative indices.
#include <string>
#include <vector>

#include "raymotion/scene.hpp"

namespace raymotion {

// 標準 .obj ファイルを Scene に追加する.
// ox,oy,oz,scale は goboscript の x,y,z,scale 引数 (init では 0,0,0,1).
// 戻り値は追加された三角形数.
int add_obj_file(Scene& scene, const std::string& path, double ox, double oy, double oz,
                 double scale, double ar, double ag, double ab, double er, double eg,
                 double eb, double ior, double rough, int shader, double metallic = 0);

int load_obj_materials(Scene& scene, const std::string& obj, const std::string& mtl);

// Parse OBJ text using the same rules as add_obj_file.
int add_obj_tokens(Scene& scene, const std::string& data, double ox, double oy, double oz,
                   double scale, double ar, double ag, double ab, double er, double eg,
                   double eb, double ior, double rough, int shader, double metallic = 0);


} // namespace raymotion
