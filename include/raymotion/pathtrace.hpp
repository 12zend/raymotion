#pragma once
// pathtrace.hpp — renderer.gs のシェーディング系 proc 群を C++ に移植.
// 対応:
//  rgb2hsv, hsv2rgb, reflect_normal, dielectric_fresnel, normalize,
//  trigonometry, castray_dir_dist, traverse, intersectionaabb,
//  intersectiontriangle, castshadowray_dist, intersectiontriangleshadow,
//  pathtrace, sample_cosine_hemisphere, sample_ggx_half,
//  evaluate_opaque_bsdf, direct_lighting, sample_dielectric,
//  pixel, render, init, onflag
//
// goboscript のグローバル変数群は Camera / HitInfo / PathState に集約する.
// 三角関数は度基準 (math.hpp), random は連続一様分布とする.

#include <cstdint>
#include <functional>
#include <vector>

#include "raymotion/math.hpp"
#include "raymotion/scene.hpp"

namespace raymotion {

struct Camera {
    double x = 0, y = 0, z = 0;  // camerax/y/z (onflag 既定)
    double dirx = 0, diry = 0, dirz = 0;  // cameradir* (度)
    double fov = 60;                      // 度
    double focal = 201.3839114815688;      // focallength. init で再計算
    // 回転行列 m0..m8 (trigonometry)
    double m0 = 1, m1 = 0, m2 = 0;
    double m3 = 0, m4 = 1, m5 = 0;
    double m6 = 0, m7 = 0, m8 = 1;
    void update_trig();   // trigonometry proc
    void update_focal();  // focallength = 240/tan(fov*0.5) (度)
};

// castray/traverse のヒット結果. hitt/hittri/iu/iv/ix/iy/iz/inx/iny/inz 等.
struct HitInfo {
    bool hit = false;
    double t = kFarClip;  // hitt
    int tri = -1;         // hittri (0-based. goboscript は 1-based)
    double u = 0, v = 0;  // iu, iv
    Vec3 p;               // ix,iy,iz
    Vec3 n_interp;        // inx,iny,inz (補間法線, 正規化済み)
    // マテリアル解決値
    double ar = 0, ag = 0, ab = 0;
    double er = 0, eg = 0, eb = 0;
    double ior = 0, rough = 0;
    int shader = 0;  // id
    // 幾何法線 (非正規化, tri_nx/ny/nz)
    Vec3 n_geom;
};

// 未使用だが互換のため残す rgb2hsv (renderer.gs:381). 呼び出し元なし.
void rgb2hsv(double r, double g, double b, double& out_h, double& out_s, double& out_v);
// hsv2rgb (renderer.gs:440). h は goboscript 呼び出し側が 0..1 を渡すバグを再現する.
// (atan(...)%360/360 をそのまま受ける. 分岐は $h<60... のままなので常に赤領域)
void hsv2rgb(double h, double s, double v, double& out_r, double& out_g, double& out_b);
// 未使用だが互換のため残す reflect_normal (renderer.gs:481). 呼び出し元なし.
Vec3 reflect_normal(const Vec3& v, const Vec3& n);
// dielectric_fresnel (renderer.gs:487)
double dielectric_fresnel(double cos_in, double eta_in, double eta_out);

class PathTracer {
public:
    PathTracer(Scene* scene, uint64_t seed = 12345) : scene_(scene), rng_(seed) {}
    void seed(uint64_t s) { rng_.seed(s); }
    Rng& rng() { return rng_; }

    // intersectionaabb node (renderer.gs:1089). t0 を返す. hit なら true.
    bool intersect_aabb(const Vec3& ro, const Vec3& inv_dir, int sdx, int sdy, int sdz,
                        int node_idx, double hitt, double& out_t0);
    // intersectiontriangle node (renderer.gs:1335)
    void intersect_triangles(const Vec3& ro, const Vec3& rd, int node_idx, HitInfo& hit);
    // traverse node (renderer.gs:1198)
    void traverse(const Vec3& ro, const Vec3& rd, const Vec3& inv_dir, int sdx, int sdy,
                  int sdz, HitInfo& hit);
    // intersectiontriangleshadow node (renderer.gs:1475). 遮蔽があれば true.
    bool intersect_triangles_shadow(const Vec3& ro, const Vec3& rd, int node_idx,
                                    double hitt);
    // castray_dir_dist (renderer.gs:825)
    HitInfo cast_ray(const Vec3& ro, const Vec3& rd, double dist);
    // castshadowray_dist (renderer.gs:1368). 遮蔽があれば true.
    bool cast_shadow_ray(const Vec3& ro, const Vec3& rd, double dist);

    // sample_cosine_hemisphere (renderer.gs:1691). 結果は pt_wi_* として返す.
    // 高速化のため内部演算は float (分布は等価).
    void sample_cosine_hemisphere(float n_x, float n_y, float n_z, float& ox,
                                  float& oy, float& oz);
    // sample_ggx_half (renderer.gs:1719). 結果は pt_h* として返す.
    void sample_ggx_half(float alpha2, float n_x, float n_y, float n_z, float& hx,
                         float& hy, float& hz);
    // evaluate_opaque_bsdf (renderer.gs:1753)
    struct Bsdf {
        float r = 0, g = 0, b = 0;
        float pdf = 0;
    };
    Bsdf evaluate_opaque_bsdf(float wi_x, float wi_y, float wi_z, float wo_x,
                              float wo_y, float wo_z, float n_x, float n_y, float n_z,
                              float alpha2, float metallic, float spec_prob, float ar,
                              float ag, float ab);
    // direct_lighting (renderer.gs:1807). 寄与を返す. ix/iy/iz 等は引数で受ける.
    void direct_lighting(float p_x, float p_y, float p_z, float n_x, float n_y,
                         float n_z, float wo_x, float wo_y, float wo_z, float rough,
                         float metallic, float spec_prob, float ar, float ag, float ab,
                         float& or_, float& og, float& ob);
    // sample_dielectric (renderer.gs:1876)
    struct DielectricSample {
        bool valid = true;
        float n_x, n_y, n_z;
        float eta_scale = 1;
    };
    DielectricSample sample_dielectric(float wo_x, float wo_y, float wo_z, float n_x,
                                       float n_y, float n_z, float curr_dx,
                                       float curr_dy, float curr_dz, float ior,
                                       bool front_face, float rough, float alpha2);

    // pathtrace (renderer.gs:1505). max_bounces は stage.gs の maxbounces (既定 5).
    Vec3 pathtrace(const Vec3& ro, const Vec3& rd, int max_bounces);

    // pixel 1サンプル分の primary ray 生成 + pathtrace.
    // goboscript の pixel proc は px=$x+random(...), py=$y+random(...) から
    // 方向を組み立てる. C++ では全画素ループ用に (sx,sy) センサ座標で受ける.
    Vec3 sample_sensor(double sx, double sy, const Camera& cam, double focal2,
                       int max_bounces);

private:
    Scene* scene_;
    Rng rng_;
};

// render/onflag に対応する全画素ループ.
// goboscript の render は `pixel resx,resy` の1回呼びに見えるが (WIP のため),
// C++/GLSL 移植では resx*resy の全画素ループに一般化する.
// センサ座標は中央原点: sx = x+jitter - W/2, sy = H/2 - (y+jitter).
// (PPM/PNG/mp4 先頭行=画面上、Scratch 同様 Y+が上)
// 色は Reinhard (c/(c+1)) 後に 8bit 化する (pixel proc の color 計算と同一).
struct RenderConfig {
    int width = 640;   // resx
    int height = 360;  // resy
    int spp = 1;       // sample (onflag では 1). 適応時は上限になる
    int max_bounces = 5;
    double resolution = 1;  // pen size 由来. jitter 幅に使う
    // firefly クランプ (1 サンプルあたりの放射輝度上限). 0 で無効.
    // 発光cube (7) の直接視認や誘電体経由の正当中継 (~10) は保持し,
    // 稀な高輝度パスを打ち切って分散を下げる (ノイズ優先の既定. --clamp で変更).
    double clamp = 16;
    // 適応サンプリング (1-pass, 画素内分散が閾値以下で早期終了。不偏性を保つ).
    // 0 で無効化 (均一 spp)。spp が小さい時は自動で無効 (min >= spp なら均一と等価).
    int adapt_min = 16;      // 最低サンプル数
    int adapt_step = 8;      // 判定間隔
    double adapt_rel = 0.02;  // 相対閾値 (std < rel*mean + abs で終了)
    double adapt_abs = 0.002;
};

std::vector<uint8_t> render_image(Scene& scene, const Camera& cam,
                                  const RenderConfig& cfg, uint64_t seed = 12345);

// 並列版. 行単位で分割し、行シード = seed ^ (y * 0x9E3779B9) で
// スレッド数に依らず決定的になる. num_threads<=0 で自動.
std::vector<uint8_t> render_image_parallel(
    Scene& scene, const Camera& cam, const RenderConfig& cfg,
    uint64_t seed = 12345, int num_threads = 0,
    const std::function<void(int done, int total)>& progress = {});

// Reinhard + pack (pixel proc の color 計算). テスト用に公開.
inline void tonemap_pack(double r, double g, double b, uint8_t& R, uint8_t& G,
                         uint8_t& B) {
    auto f = [](double c) {
        double v = c / (c + 1.0) * 255.0;
        long q = (long)std::floor(v);
        if (q < 0) q = 0;
        if (q > 255) q = 255;
        return (uint8_t)q;
    };
    R = f(r);
    G = f(g);
    B = f(b);
}

}  // namespace raymotion
