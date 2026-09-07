#pragma once
// scene.hpp — renderer.gs の三角形リスト群とマテリアルを C++ 構造体に移植.
// goboscript は 1-indexed の list 群 (tri_x0, tri_y0, ... tri_reflection,
// light_tri, light_cdf, bvhtri, node_* ...) でシーンを持つ.
// C++ では 0-indexed の vector<Triangle> + vector<Node> に正規化する.
// フィールド名は goboscript 側と対応付けてある.

#include <memory>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include "raymotion/math.hpp"

namespace raymotion {

// 1 三角形. renderer.gs の addtriangle___... proc が add する全リストを集約.
// 使われない UV (u0..v2) と正規化済み幾何法線 (nnx..nnz) も互換のため保持する.
struct Texture {
    int width=0, height=0;
    std::vector<Vec3> pixels; // linear RGB, top row first
    Vec3 sample(double u, double v) const;
};

struct Triangle {
    std::shared_ptr<const Texture> texture;
    // 頂点
    Vec3 v0, v1, v2;
    // 幾何法線 (非正規化 cross) と平面係数 d. tri_nx/ny/nz, tri_d に対応.
    Vec3 n;  // = (v1-v0) x (v2-v0)
    double d = 0;
    // バリセントリック基底. tri_ux..uz, tri_vx..vz に対応.
    Vec3 U, V;
    // 逆行列分母. denom に対応 (面積0判定用).
    double denom = 0;
    // 正規化済み幾何法線. tri_nnx..nnz に対応 (現行シェーディングでは未使用).
    Vec3 nn;
    // UV. map_Kd のサンプリングに使用.
    double tu0 = 0, tv0 = 0, tu1 = 0, tv1 = 0, tu2 = 0, tv2 = 0;
    // 頂点法線. tri_nx0..nz2 に対応.
    Vec3 n0, n1, n2;
    // AABB. tri_min/max* に対応.
    Vec3 mn, mx;
    // マテリアル. tri_ar/ag/ab, tri_er/eg/eb, tri_ior, tri_rougth, tri_shader に対応.
    double ar = 1, ag = 1, ab = 1;
    double er = 0, eg = 0, eb = 0;
    double ior = 1;      // refraction 引数. 1.0001 超で誘電体扱い
    double alpha = 1;    // surface opacity: 0 transparent, 1 opaque
    double rough = 0.5;  // tri_rougth (綴りは原文ママ)
    int shader = 0;      // 1 で hsv 虹シェーダ (castray_dir_dist の id==1 分岐)
    double metallic = 0;  // reflection 引数. NeRF 的には metallic/specular 重み
};

// BVH ノード. goboscript は node_minx 等を符号別に2重化 (node/node+1) するが,
// C++ では単一ノード + 走査時に符号で min/max を選ぶ等価実装とする.
struct BvhNode {
    Vec3 mn, mx;
    int left = -1;    // 内部ノードのみ
    int right = -1;   // 内部ノードのみ
    int offset = -1;  // 葉のみ: bvhtri 中の開始位置
    int count = 0;    // 葉のみ: 三角形数 (0 なら内部ノード)
    bool is_leaf() const { return count > 0; }
};

// 高速走査用の float SoA キャッシュ (build_bvh の最後に構築。画像は不変).
// 走査・交差判定のみに使い、シェーディングは従来通り double の Triangle を参照する.
// 32B パック (キャッシュライン分割を避ける): internal は a=left,b=right,
// leaf は a=offset,b=count (count>0 で leaf 判定).
struct FastNode {
    float mnx, mny, mnz, mxx, mxy, mxz;
    int a = -1;
    int b = 0;
};

// 4分木 BVH (二分木を孫併合で 4 分岐化). NEON で 4 子を 1 パス判定する.
// child[i]>=0: 内部子. cnt[i]>0: 葉 (fidx の offset..+cnt). cnt[i]==0: 内部子.
// cnt[i]==-1: 空スロット (判定常に失敗).
struct FastNode4 {
    float mnx[4], mxx[4], mny[4], mxy[4], mnz[4], mxz[4];
    int child[4];
    int offset[4];
    int cnt[4];
};
struct FastTri {
    float v0x, v0y, v0z;
    float e1x, e1y, e1z;  // v1-v0
    float e2x, e2y, e2z;  // v2-v0
};

struct Scene {
    std::vector<Triangle> tris;
    // BVH
    std::vector<BvhNode> nodes;
    std::vector<int> bvh_tri;  // 三角形インデックス列 (goboscript の bvhtri に対応, 0-based)
    int max_leaf_tris = 8;     // generatebvhtree tri 引数. init では 4 (高速化のため既定 8)
    // float 走査キャッシュ (build_bvh が充填)
    std::vector<FastNode> fnodes;
    std::vector<FastNode4> fnodes4;
    std::vector<FastTri> ftris;
    std::vector<int> fidx;  // bvh_tri の複製 (走査側の所有でキャッシュ局所性を保つ)

    // エミッシブ CDF. light_tri/light_cdf, pt_light_total に対応.
    std::vector<int> light_tri;  // 発光三角形のインデックス
    std::vector<double> light_cdf;
    double light_total = 0;

    void clear_triangles() {
        tris.clear();
        light_tri.clear();
        light_cdf.clear();
        light_total = 0;
    }
    void clear_bvh() {
        nodes.clear();
        bvh_tri.clear();
        fnodes.clear();
        fnodes4.clear();
        ftris.clear();
        fidx.clear();
    }

    // addtriangle___texcoords_normals___material___shader に対応.
    // 面積0 (abs(denom)<=1e-12) は追加しない. 発光なら CDF に積算する.
    // 戻り値は追加された三角形の index (追加なしは -1).
    int add_triangle(const Vec3& p0, const Vec3& p1, const Vec3& p2, double u0, double v0,
                     double u1, double v1, double u2, double v2, const Vec3& n0,
                     const Vec3& n1, const Vec3& n2, double ar, double ag, double ab,
                     double er, double eg, double eb, double metallic, double ior,
                     double rough, int shader, double alpha = 1);
};


} // namespace raymotion
