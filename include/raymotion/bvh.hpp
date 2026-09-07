#pragma once
// bvh.hpp — renderer.gs の generatebvhtree/buildnode/min/max/sort を C++ に移植.
// goboscript は中央値分割・最長軸・BFS キューで BVH を組む.
// C++ では std::sort による等価な中央値分割とする (sort proc の独自
// quicksort+insertion と厳密な比較順は異なるが, 分割基準は同一).
// 走査は intersectionaabb/traverse/intersectiontriangle と等価な反復版.

#include "raymotion/scene.hpp"

namespace raymotion {

// 中央値分割 BVH を構築する. max_leaf_tris は generatebvhtree tri 引数 (init では 4).
void refit_bvh(Scene& scene);
void build_bvh(Scene& scene, int max_leaf_tris = 8);

}  // namespace raymotion
