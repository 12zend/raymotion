// pathtrace.cpp — シェーディング系 proc 群の移植.
#include "raymotion/pathtrace.hpp"

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <limits>
#include <thread>

#if defined(__aarch64__)
#include <arm_neon.h>
#endif

namespace raymotion {

inline void fast_sincos(double a, double& s, double& c) {
// 1回の呼出しで sin/cos を両方得る (個別呼出しの約半分).
#if defined(__APPLE__)
    __sincos(a, &s, &c);
#elif defined(__GLIBC__) || defined(__linux__)
    ::sincos(a, &s, &c);
#else
    s = std::sin(a);
    c = std::cos(a);
#endif
}


// ---------- float 高速走査 (BVH・三角交差のみ。画像は不変) ----------
// Scene::fnodes/ftris/fidx を使う。シェーディングは double の Triangle 参照のまま.

inline bool fast_aabb(const FastNode* nds, float rox, float roy, float roz,
                      float ivx, float ivy, float ivz, int ni, float hitt,
                      float& t0) {
    // branchless slab (fmin/fmax セレクトのみ. rd は呼び出し側で 0 クランプ済み).
    const FastNode& nd = nds[ni];
    float tmin = 0.0f, tmax = hitt;
    float ax = (nd.mnx - rox) * ivx, bx = (nd.mxx - rox) * ivx;
    float lo = ax < bx ? ax : bx, hi = ax < bx ? bx : ax;
    if (lo > tmin) tmin = lo;
    if (hi < tmax) tmax = hi;
    float ay = (nd.mny - roy) * ivy, by = (nd.mxy - roy) * ivy;
    lo = ay < by ? ay : by;
    hi = ay < by ? by : ay;
    if (lo > tmin) tmin = lo;
    if (hi < tmax) tmax = hi;
    float az = (nd.mnz - roz) * ivz, bz = (nd.mxz - roz) * ivz;
    lo = az < bz ? az : bz;
    hi = az < bz ? bz : az;
    if (lo > tmin) tmin = lo;
    if (hi < tmax) tmax = hi;
    t0 = tmin;
    return tmin <= tmax;
}

// Möller–Trumbore (両面)。triple product は原文の平面法線 den と同スケール.
inline void fast_tris_closest(const FastTri* fts, const int* fidx, float rox,
                              float roy, float roz, float rdx, float rdy,
                              float rdz, int offset, int count, float& best_t,
                              int& best_tri, float& best_u, float& best_v) {
    for (int k = 0; k < count; ++k) {
        int j = fidx[offset + k];
        const FastTri& t = fts[j];
        float px = rdy * t.e2z - rdz * t.e2y;
        float py = rdz * t.e2x - rdx * t.e2z;
        float pz = rdx * t.e2y - rdy * t.e2x;
        float det = t.e1x * px + t.e1y * py + t.e1z * pz;
        if (det > -1e-12f && det < 1e-12f) continue;
        float inv = 1.0f / det;
        float tx = rox - t.v0x, ty = roy - t.v0y, tz = roz - t.v0z;
        float u = (tx * px + ty * py + tz * pz) * inv;
        if (u < 0.0f || u > 1.0f) continue;
        float qx = ty * t.e1z - tz * t.e1y;
        float qy = tz * t.e1x - tx * t.e1z;
        float qz = tx * t.e1y - ty * t.e1x;
        float v = (rdx * qx + rdy * qy + rdz * qz) * inv;
        if (v < 0.0f || u + v > 1.0f) continue;
        float tt = (t.e2x * qx + t.e2y * qy + t.e2z * qz) * inv;
        if (tt > 1e-8f && tt < best_t) {
            best_t = tt;
            best_tri = j;
            best_u = u;
            best_v = v;
        }
    }
}

inline bool fast_tris_occluded(const FastTri* fts, const int* fidx, float rox,
                               float roy, float roz, float rdx, float rdy,
                               float rdz, int offset, int count, float hitt) {
    for (int k = 0; k < count; ++k) {
        int j = fidx[offset + k];
        const FastTri& t = fts[j];
        float px = rdy * t.e2z - rdz * t.e2y;
        float py = rdz * t.e2x - rdx * t.e2z;
        float pz = rdx * t.e2y - rdy * t.e2x;
        float det = t.e1x * px + t.e1y * py + t.e1z * pz;
        if (det > -1e-12f && det < 1e-12f) continue;
        float inv = 1.0f / det;
        float tx = rox - t.v0x, ty = roy - t.v0y, tz = roz - t.v0z;
        float u = (tx * px + ty * py + tz * pz) * inv;
        if (u < 0.0f || u > 1.0f) continue;
        float qx = ty * t.e1z - tz * t.e1y;
        float qy = tz * t.e1x - tx * t.e1z;
        float qz = tx * t.e1y - ty * t.e1x;
        float v = (rdx * qx + rdy * qy + rdz * qz) * inv;
        if (v < 0.0f || u + v > 1.0f) continue;
        float tt = (t.e2x * qx + t.e2y * qy + t.e2z * qz) * inv;
        if (tt > 1e-8f && tt < hitt) return true;
    }
    return false;
}

// ---------- Camera ----------

void Camera::update_focal() {
    // init proc: focallength = 240 / tan(fov * ".5") (度)
    focal = 240.0 / tan_deg(fov * 0.5);
}

void Camera::update_trig() {
    // trigonometry proc (renderer.gs:808)
    double cx = cos_deg(dirx), sx = sin_deg(dirx);
    double cy = cos_deg(diry), sy = sin_deg(diry);
    double cz = cos_deg(dirz), sz = sin_deg(dirz);
    m0 = cy * cz + sy * (sx * sz);
    m1 = -cy * sz + sy * (sx * cz);
    m2 = sy * cx;
    m3 = cx * sz;
    m4 = cx * cz;
    m5 = -sx;
    m6 = -sy * cz + cy * (sx * sz);
    m7 = sy * sz + cy * (sx * cz);
    m8 = cy * cx;
}

// ---------- 小物 ----------

void rgb2hsv(double r, double g, double b, double& out_h, double& out_s,
             double& out_v) {
    // renderer.gs:381 (呼び出し元なし, 互換のため残す)
    double mx, mn;
    const char* mxid;
    const char* mnid;
    if (r < g) {
        if (g < b) {
            mx = b / 255;
            mxid = "b";
        } else {
            mx = g / 255;
            mxid = "g";
        }
    } else if (r < b) {
        mx = b / 255;
        mxid = "b";
    } else {
        mx = r / 255;
        mxid = "r";
    }
    if (r > g) {
        if (g > b) {
            mn = b / 255;
            mnid = "b";
        } else {
            mn = g / 255;
            mnid = "g";
        }
    } else if (r > b) {
        mn = b / 255;
        mnid = "b";
    } else {
        mn = r / 255;
        mnid = "r";
    }
    (void)mnid;
    double ch;
    if (mx - mn == 0) {
        ch = 0;
    } else {
        std::string id(mxid);
        if (id == "r") {
            ch = 60 * gobo_mod((g - b) / 255 / (mx - mn), 6);
        } else if (id == "g") {
            ch = 60 * ((b - r) / 255 / (mx - mn)) + 120;
        } else {
            ch = 60 * ((r - g) / 255 / (mx - mn)) + 240;
        }
    }
    double cs = (mx == 0) ? 0 : (mx - mn) / mx;
    out_h = ch;
    out_s = cs;
    out_v = mx;
}

void hsv2rgb(double h, double s, double v, double& out_r, double& out_g,
             double& out_b) {
    // renderer.gs:440
    double c = s * v;
    double x = c * (1 - std::abs(gobo_mod(h / 60.0, 2.0) - 1));
    double m = v - c;
    double cr = 0, cg = 0, cb = 0;
    if (h < 60) {
        cr = c;
        cg = x;
        cb = 0;
    } else if (h < 120) {
        cr = x;
        cg = c;
        cb = 0;
    } else if (h < 180) {
        cr = 0;
        cg = c;
        cb = x;
    } else if (h < 240) {
        cr = 0;
        cg = x;
        cb = c;
    } else if (h < 300) {
        cr = x;
        cg = 0;
        cb = c;
    } else if (h < 360) {
        cr = c;
        cg = 0;
        cb = x;
    }
    cr += m;
    cg += m;
    cb += m;
    out_r = cr * 255;
    out_g = cg * 255;
    out_b = cb * 255;
}

Vec3 reflect_normal(const Vec3& v, const Vec3& n) {
    // renderer.gs:481 (呼び出し元なし)
    double d = v.x * n.x + (v.y * n.y + v.z * n.z);
    return {v.x - n.x * (2 * d), v.y - n.y * (2 * d), v.z - n.z * (2 * d)};
}

double dielectric_fresnel(double cos_in, double eta_in, double eta_out) {
    // renderer.gs:487
    double ci = cos_in;
    if (ci < 0) ci = 0;
    if (ci > 1) ci = 1;
    double eta = eta_in / eta_out;
    double s2 = eta * eta * (1 - ci * ci);
    if (s2 >= 1) return 1;
    double ct = std::sqrt(1 - s2);
    double rp = (eta_out * ci - eta_in * ct) / (eta_out * ci + eta_in * ct);
    double rs = (eta_in * ci - eta_out * ct) / (eta_in * ci + eta_out * ct);
    return 0.5 * (rp * rp + rs * rs);
}

// ---------- 交差 ----------

bool PathTracer::intersect_aabb(const Vec3& ro, const Vec3& inv_dir, int sdx,
                                int sdy, int sdz, int node_idx, double hitt,
                                double& out_t0) {
    // renderer.gs:1089 intersectionaabb と等価な堅牢版.
    // goboscript は node_minx[node+sign] の2重化で符号分岐を消すが,
    // ここでは inv の符号で min/max を選ぶ (結果は同一).
    // 高速化: 軸ごとの分岐・ラムダを排し branchless slab にする.
    (void)sdx;
    (void)sdy;
    (void)sdz;
    const BvhNode& nd = scene_->nodes[(size_t)node_idx];
    double t0 = 0.0;
    double t1 = hitt;
    // X
    {
        double a = (nd.mn.x - ro.x) * inv_dir.x;
        double b = (nd.mx.x - ro.x) * inv_dir.x;
        double mn = a < b ? a : b;
        double mx = a < b ? b : a;
        // inv==inf (rd==0) の場合は a,b が ±inf/nan になる。平行時は原点内外判定に帰着.
        // inf*0=nana 対策: rd==0 は稀なので分岐で逃がす (予測ほぼ不要).
        if (inv_dir.x == std::numeric_limits<double>::infinity() ||
            inv_dir.x == -std::numeric_limits<double>::infinity()) {
            if (ro.x < nd.mn.x || ro.x > nd.mx.x) return false;
        } else {
            if (mn > t0) t0 = mn;
            if (mx < t1) t1 = mx;
            if (t0 > t1) return false;
        }
    }
    // Y
    {
        double a = (nd.mn.y - ro.y) * inv_dir.y;
        double b = (nd.mx.y - ro.y) * inv_dir.y;
        double mn = a < b ? a : b;
        double mx = a < b ? b : a;
        if (inv_dir.y == std::numeric_limits<double>::infinity() ||
            inv_dir.y == -std::numeric_limits<double>::infinity()) {
            if (ro.y < nd.mn.y || ro.y > nd.mx.y) return false;
        } else {
            if (mn > t0) t0 = mn;
            if (mx < t1) t1 = mx;
            if (t0 > t1) return false;
        }
    }
    // Z
    {
        double a = (nd.mn.z - ro.z) * inv_dir.z;
        double b = (nd.mx.z - ro.z) * inv_dir.z;
        double mn = a < b ? a : b;
        double mx = a < b ? b : a;
        if (inv_dir.z == std::numeric_limits<double>::infinity() ||
            inv_dir.z == -std::numeric_limits<double>::infinity()) {
            if (ro.z < nd.mn.z || ro.z > nd.mx.z) return false;
        } else {
            if (mn > t0) t0 = mn;
            if (mx < t1) t1 = mx;
            if (t0 > t1) return false;
        }
    }
    out_t0 = t0;
    return true;
}

void PathTracer::intersect_triangles(const Vec3& ro, const Vec3& rd,
                                     int node_idx, HitInfo& hit) {
    // renderer.gs:1335 intersectiontriangle
    const BvhNode& nd = scene_->nodes[(size_t)node_idx];
    for (int k = 0; k < nd.count; ++k) {
        int j = scene_->bvh_tri[(size_t)(nd.offset + k)];
        const Triangle& t = scene_->tris[(size_t)j];
        double den = t.n.x * rd.x + (t.n.y * rd.y + t.n.z * rd.z);
        if (std::abs(den) < 1e-30) continue;  // goboscript は素通しだが結果は等価
        double tt = -(t.n.x * ro.x + (t.n.y * ro.y + (t.n.z * ro.z + t.d))) / den;
        if (tt < hit.t && tt > 1e-8) {
            Vec3 p{ro.x + rd.x * tt, ro.y + rd.y * tt, ro.z + rd.z * tt};
            Vec3 d0{p.x - t.v0.x, p.y - t.v0.y, p.z - t.v0.z};
            double u = d0.x * t.U.x + (d0.y * t.U.y + d0.z * t.U.z);
            if (!(u < 0)) {
                double v = d0.x * t.V.x + (d0.y * t.V.y + d0.z * t.V.z);
                if (!(v < 0 || u + v > 1)) {
                    hit.t = tt;
                    hit.tri = j;
                    hit.hit = true;
                    hit.p = p;
                    hit.u = u;
                    hit.v = v;
                }
            }
        }
    }
}

// 4 分木ノード判定: 4 子の slab を一括計算 (NEON 4-wide / スカラ 4 ループ).
// okout[i]=0xFFFFFFFF は 「子 i が有効かつ [t0,t1] が重なる」. t0out[i]=入り t.
static inline void test_node4(const FastNode4& nd, float rox, float roy, float roz,
                              float ivx, float ivy, float ivz, float tmin_in,
                              float tmax_in, float* t0out, uint32_t* okout) {
#if defined(__aarch64__)
    float32x4_t tminv = vdupq_n_f32(tmin_in);
    float32x4_t tmaxv = vdupq_n_f32(tmax_in);
    {
        float32x4_t ro = vdupq_n_f32(rox), iv = vdupq_n_f32(ivx);
        float32x4_t a = vmulq_f32(vsubq_f32(vld1q_f32(nd.mnx), ro), iv);
        float32x4_t b = vmulq_f32(vsubq_f32(vld1q_f32(nd.mxx), ro), iv);
        tminv = vmaxq_f32(tminv, vminq_f32(a, b));
        tmaxv = vminq_f32(tmaxv, vmaxq_f32(a, b));
    }
    {
        float32x4_t ro = vdupq_n_f32(roy), iv = vdupq_n_f32(ivy);
        float32x4_t a = vmulq_f32(vsubq_f32(vld1q_f32(nd.mny), ro), iv);
        float32x4_t b = vmulq_f32(vsubq_f32(vld1q_f32(nd.mxy), ro), iv);
        tminv = vmaxq_f32(tminv, vminq_f32(a, b));
        tmaxv = vminq_f32(tmaxv, vmaxq_f32(a, b));
    }
    {
        float32x4_t ro = vdupq_n_f32(roz), iv = vdupq_n_f32(ivz);
        float32x4_t a = vmulq_f32(vsubq_f32(vld1q_f32(nd.mnz), ro), iv);
        float32x4_t b = vmulq_f32(vsubq_f32(vld1q_f32(nd.mxz), ro), iv);
        tminv = vmaxq_f32(tminv, vminq_f32(a, b));
        tmaxv = vminq_f32(tmaxv, vmaxq_f32(a, b));
    }
    uint32x4_t ok = vandq_u32(vcleq_f32(tminv, tmaxv),
                              vcgeq_s32(vld1q_s32(nd.cnt), vdupq_n_s32(0)));
    vst1q_f32(t0out, tminv);
    vst1q_u32(okout, ok);
#else
    for (int i = 0; i < 4; ++i) {
        float tmin = tmin_in, tmax = tmax_in;
        float ax = (nd.mnx[i] - rox) * ivx, bx = (nd.mxx[i] - rox) * ivx;
        float lo = ax < bx ? ax : bx, hi = ax < bx ? bx : ax;
        if (lo > tmin) tmin = lo;
        if (hi < tmax) tmax = hi;
        float ay = (nd.mny[i] - roy) * ivy, by = (nd.mxy[i] - roy) * ivy;
        lo = ay < by ? ay : by;
        hi = ay < by ? by : ay;
        if (lo > tmin) tmin = lo;
        if (hi < tmax) tmax = hi;
        float az = (nd.mnz[i] - roz) * ivz, bz = (nd.mxz[i] - roz) * ivz;
        lo = az < bz ? az : bz;
        hi = az < bz ? bz : az;
        if (lo > tmin) tmin = lo;
        if (hi < tmax) tmax = hi;
        t0out[i] = tmin;
        okout[i] = (tmin <= tmax && nd.cnt[i] >= 0) ? 0xFFFFFFFFu : 0u;
    }
#endif
}

void PathTracer::traverse(const Vec3& ro, const Vec3& rd, const Vec3& inv_dir,
                          int sdx, int sdy, int sdz, HitInfo& hit) {
    // renderer.gs:1198 traverse と等価な反復・近-first 走査 (4 分木 NEON 版).
    (void)inv_dir;
    (void)sdx;
    (void)sdy;
    (void)sdz;
    if (scene_->fnodes4.empty()) return;
    const FastNode4* nds = scene_->fnodes4.data();
    const FastTri* fts = scene_->ftris.data();
    const int* fidx = scene_->fidx.data();
    float rox = (float)ro.x, roy = (float)ro.y, roz = (float)ro.z;
    float rdx = (float)rd.x, rdy = (float)rd.y, rdz = (float)rd.z;
    // 0 方向は微小値に寄せて inv を有限に保つ (branchless slab の前提).
    if (rdx == 0.0f) rdx = 1e-30f;
    if (rdy == 0.0f) rdy = 1e-30f;
    if (rdz == 0.0f) rdz = 1e-30f;
    float ivx = 1.0f / rdx, ivy = 1.0f / rdy, ivz = 1.0f / rdz;
    float best_t = (float)hit.t;
    int best_tri = -1;
    float best_u = 0, best_v = 0;

    int stack[80];
    int ptr = 0;
    const FastNode4* nd = &nds[0];
    for (;;) {
        float t0s[4];
        uint32_t okm[4];
        test_node4(*nd, rox, roy, roz, ivx, ivy, ivz, 0.0f, best_t, t0s, okm);
        // ヒット子を t0 昇順に整列 (最大 4 個, 挿入ソート)
        int order[4];
        int nk = 0;
        for (int i = 0; i < 4; ++i) {
            if (!okm[i]) continue;
            int k = nk++;
            while (k > 0 && t0s[order[k - 1]] > t0s[i]) {
                order[k] = order[k - 1];
                --k;
            }
            order[k] = i;
        }
        // 葉を近い順に即時処理 (closest 更新で後続が枝刈りされる)
        for (int i = 0; i < nk; ++i) {
            int ci = order[i];
            if (nd->cnt[ci] > 0) {
                fast_tris_closest(fts, fidx, rox, roy, roz, rdx, rdy, rdz,
                                  nd->offset[ci], nd->cnt[ci], best_t, best_tri,
                                  best_u, best_v);
            }
        }
        // 内部子: 最も近いものへ降り, 残りを遠い順に積む (LIFO で近い方が先に戻る)
        int descend = -1;
        for (int i = nk - 1; i >= 0; --i) {
            int ci = order[i];
            if (nd->cnt[ci] != 0) continue;  // 葉は処理済み
            if (descend < 0) {
                descend = nd->child[ci];
            } else if (ptr < 80) {
                stack[ptr++] = nd->child[ci];
            }
        }
        if (descend < 0) {
            if (ptr == 0) break;
            nd = &nds[(size_t)stack[--ptr]];
        } else {
            nd = &nds[(size_t)descend];
        }
    }

    if (best_tri < 0) return;
    hit.t = (double)best_t;
    hit.tri = best_tri;
    hit.hit = true;
    hit.p = {ro.x + rd.x * hit.t, ro.y + rd.y * hit.t, ro.z + rd.z * hit.t};
    hit.u = (double)best_u;
    hit.v = (double)best_v;
}

bool PathTracer::intersect_triangles_shadow(const Vec3& ro, const Vec3& rd,
                                            int node_idx, double hitt) {
    // renderer.gs:1475 intersectiontriangleshadow
    const BvhNode& nd = scene_->nodes[(size_t)node_idx];
    for (int k = 0; k < nd.count; ++k) {
        int j = scene_->bvh_tri[(size_t)(nd.offset + k)];
        const Triangle& t = scene_->tris[(size_t)j];
        double den = t.n.x * rd.x + (t.n.y * rd.y + t.n.z * rd.z);
        if (std::abs(den) > 1e-12) {
            double tt =
                -(t.n.x * ro.x + (t.n.y * ro.y + (t.n.z * ro.z + t.d))) / den;
            if (tt < hitt && tt > 1e-8) {
                Vec3 p{ro.x + rd.x * tt, ro.y + rd.y * tt, ro.z + rd.z * tt};
                Vec3 d0{p.x - t.v0.x, p.y - t.v0.y, p.z - t.v0.z};
                double u = d0.x * t.U.x + (d0.y * t.U.y + d0.z * t.U.z);
                if (!(u < 0)) {
                    double v = d0.x * t.V.x + (d0.y * t.V.y + d0.z * t.V.z);
                    if (!(v < 0 || u + v > 1)) return true;
                }
            }
        }
    }
    return false;
}

HitInfo PathTracer::cast_ray(const Vec3& ro, const Vec3& rd, double dist) {
    // renderer.gs:825 castray_dir_dist
    HitInfo hit;
    hit.t = dist;
    hit.tri = -1;
    hit.hit = false;
    Vec3 inv{1.0 / rd.x, 1.0 / rd.y, 1.0 / rd.z};
    int sdx = (rd.x < 0) ? 1 : 0;
    int sdy = (rd.y < 0) ? 1 : 0;
    int sdz = (rd.z < 0) ? 1 : 0;
    traverse(ro, rd, inv, sdx, sdy, sdz, hit);
    if (hit.tri >= 0) {
        const Triangle& t = scene_->tris[(size_t)hit.tri];
        double w = 1 - (hit.u + hit.v);
        Vec3 ni{w * t.n0.x + (hit.u * t.n1.x + hit.v * t.n2.x),
                w * t.n0.y + (hit.u * t.n1.y + hit.v * t.n2.y),
                w * t.n0.z + (hit.u * t.n1.z + hit.v * t.n2.z)};
        ni = gobo_normalize(ni.x, ni.y, ni.z);
        hit.n_interp = ni;
        hit.ar = t.ar;
        hit.ag = t.ag;
        hit.ab = t.ab;
        if(t.texture) {
            Vec3 c=t.texture->sample(w*t.tu0+hit.u*t.tu1+hit.v*t.tu2,w*t.tv0+hit.u*t.tv1+hit.v*t.tv2);
            hit.ar*=c.x; hit.ag*=c.y; hit.ab*=c.z;
        }
        hit.er = t.er;
        hit.eg = t.eg;
        hit.eb = t.eb;
        hit.ior = t.ior;
        hit.rough = t.rough;
        hit.shader = t.shader;
        hit.n_geom = t.n;
        if (hit.shader == 1) {
            // hsv2rgb (atan(ix/iz) + not(iz>0)*180) %360 /360, 0.8, 1
            double az = atan_deg(hit.p.x / hit.p.z) + ((hit.p.z > 0) ? 0 : 180);
            double h = gobo_mod(az, 360.0) / 360.0;
            double cr, cg, cb;
            hsv2rgb(h, 0.8, 1, cr, cg, cb);
            hit.ar = cr;
            hit.ag = cg;
            hit.ab = cb;
        }
    }
    return hit;
}

bool PathTracer::cast_shadow_ray(const Vec3& ro, const Vec3& rd, double dist) {
    // renderer.gs:1368 castshadowray_dist (遮蔽の有無を返す, 4 分木 any-hit 版)
    if (scene_->fnodes4.empty()) return false;
    const FastNode4* nds = scene_->fnodes4.data();
    const FastTri* fts = scene_->ftris.data();
    const int* fidx = scene_->fidx.data();
    float rox = (float)ro.x, roy = (float)ro.y, roz = (float)ro.z;
    float rdx = (float)rd.x, rdy = (float)rd.y, rdz = (float)rd.z;
    if (rdx == 0.0f) rdx = 1e-30f;
    if (rdy == 0.0f) rdy = 1e-30f;
    if (rdz == 0.0f) rdz = 1e-30f;
    float ivx = 1.0f / rdx, ivy = 1.0f / rdy, ivz = 1.0f / rdz;
    float hitt = (float)dist;
    int stack[80];
    int ptr = 0;
    const FastNode4* nd = &nds[0];
    for (;;) {
        float t0s[4];
        uint32_t okm[4];
        test_node4(*nd, rox, roy, roz, ivx, ivy, ivz, 0.0f, hitt, t0s, okm);
        // 遮蔽判定は any-hit なので順序は結果に無影響. 空いた子を全て積む.
        for (int i = 0; i < 4; ++i) {
            if (!okm[i]) continue;
            if (nd->cnt[i] > 0) {
                if (fast_tris_occluded(fts, fidx, rox, roy, roz, rdx, rdy, rdz,
                                       nd->offset[i], nd->cnt[i], hitt))
                    return true;
            } else if (ptr < 80) {
                stack[ptr++] = nd->child[i];
            }
        }
        if (ptr == 0) return false;
        nd = &nds[(size_t)stack[--ptr]];
    }
}

// ---------- サンプリング (float 高速化版。分布は等価) ----------

static inline void make_tangent_f(float nx, float ny, float nz, float& tx,
                                  float& ty, float& tz, float& bx, float& by,
                                  float& bz) {
    // sample_* proc の接線基底 (abs(nx)>0.1 分岐) と同一
    if (std::fabs(nx) > 0.1f) {
        tx = nz; ty = 0.0f; tz = -nx;
    } else {
        tx = 0.0f; ty = -nz; tz = ny;
    }
    float l = 1.0f / std::sqrt(tx * tx + ty * ty + tz * tz);
    tx *= l; ty *= l; tz *= l;
    bx = ny * tz - nz * ty;
    by = nz * tx - nx * tz;
    bz = nx * ty - ny * tx;
}

inline float fast_sincos_f(float a, float& c) {
#if defined(__APPLE__)
    float s;
    ::__sincosf(a, &s, &c);
    return s;
#elif defined(__GLIBC__) || defined(__linux__)
    float s;
    ::sincosf(a, &s, &c);
    return s;
#else
    s = std::sin(a);
    c = std::cos(a);
    return s;
#endif
}

void PathTracer::sample_cosine_hemisphere(float nx, float ny, float nz, float& ox,
                                          float& oy, float& oz) {
    // renderer.gs:1691 (度→ラジアン直生成 + sincos 1回化)
    float ru = (float)rng_.uniform01();
    float phi = (float)rng_.uniform01() * (float)(2.0 * kPi);
    float c;
    float sp = fast_sincos_f(phi, c);
    float rr = std::sqrt(ru);
    float lx = rr * c;
    float ly = rr * sp;
    float lz = std::sqrt(std::max(0.0f, 1.0f - ru));
    float tx, ty, tz, bx, by, bz;
    make_tangent_f(nx, ny, nz, tx, ty, tz, bx, by, bz);
    ox = tx * lx + (bx * ly + nx * lz);
    oy = ty * lx + (by * ly + ny * lz);
    oz = tz * lx + (bz * ly + nz * lz);
}

void PathTracer::sample_ggx_half(float alpha2, float nx, float ny, float nz,
                                 float& hx, float& hy, float& hz) {
    // renderer.gs:1719
    // pt_rand_u は生成されるが使用されない (原文ママ). 互換のため生成のみ行う.
    float ru_unused = (float)rng_.uniform01();
    (void)ru_unused;
    float rv = (float)rng_.uniform01();
    float phi = (float)rng_.uniform01() * (float)(2.0 * kPi);
    float c;
    float sp = fast_sincos_f(phi, c);
    float cs = std::sqrt((1.0f - rv) / (1.0f + (alpha2 - 1.0f) * rv));
    float sn = std::sqrt(std::max(0.0f, 1.0f - cs * cs));
    float lx = sn * c;
    float ly = sn * sp;
    float lz = cs;
    float tx, ty, tz, bx, by, bz;
    make_tangent_f(nx, ny, nz, tx, ty, tz, bx, by, bz);
    hx = tx * lx + (bx * ly + nx * lz);
    hy = ty * lx + (by * ly + ny * lz);
    hz = tz * lx + (bz * ly + nz * lz);
    float d = 1.0f / std::sqrt(hx * hx + hy * hy + hz * hz);
    hx *= d; hy *= d; hz *= d;
}

static inline float dielectric_fresnel_f(float ci_in, float eta_in, float eta_out) {
    float ci = ci_in;
    if (ci < 0) ci = 0;
    if (ci > 1) ci = 1;
    float eta = eta_in / eta_out;
    float s2 = eta * eta * (1.0f - ci * ci);
    if (s2 >= 1) return 1;
    float ct = std::sqrt(1.0f - s2);
    float rp = (eta_out * ci - eta_in * ct) / (eta_out * ci + eta_in * ct);
    float rs = (eta_in * ci - eta_out * ct) / (eta_in * ci + eta_out * ct);
    return 0.5f * (rp * rp + rs * rs);
}

PathTracer::Bsdf PathTracer::evaluate_opaque_bsdf(float wi_x, float wi_y, float wi_z,
                                                  float wo_x, float wo_y, float wo_z,
                                                  float n_x, float n_y, float n_z,
                                                  float alpha2, float metallic,
                                                  float spec_prob, float ar,
                                                  float ag, float ab) {
    // renderer.gs:1753 evaluate_opaque_bsdf
    Bsdf out;
    float cos_i = wi_x * n_x + (wi_y * n_y + wi_z * n_z);
    float cos_o = wo_x * n_x + (wo_y * n_y + wo_z * n_z);
    if (!(cos_i > 1e-8f && cos_o > 1e-8f)) return out;
    float hx = wi_x + wo_x, hy = wi_y + wo_y, hz = wi_z + wo_z;
    float hl = std::sqrt(hx * hx + hy * hy + hz * hz);
    if (!(hl > 1e-8f)) return out;
    float inv = 1.0f / hl;
    hx *= inv; hy *= inv; hz *= inv;
    float cos_h = hx * n_x + (hy * n_y + hz * n_z);
    float vo_h = wo_x * hx + (wo_y * hy + wo_z * hz);
    if (cos_h < 0) {
        hx = -hx; hy = -hy; hz = -hz;
        cos_h = -cos_h;
        vo_h = -vo_h;
    }
    if (!(vo_h > 1e-8f)) return out;
    float dd = cos_h * cos_h * (alpha2 - 1.0f) + 1.0f;
    float D = alpha2 / ((float)kPi * dd * dd);
    float g1 = 2.0f * cos_o /
               (cos_o + std::sqrt(cos_o * cos_o + alpha2 * (1.0f - cos_o * cos_o)));
    float g2 = 2.0f * cos_i /
               (cos_i + std::sqrt(cos_i * cos_i + alpha2 * (1.0f - cos_i * cos_i)));
    float G = g1 * g2;
    float om = 1.0f - vo_h;
    om *= om;
    om *= om;
    om *= 1.0f - vo_h;
    float di = 1.0f - metallic;
    float f0r = 0.04f * di + ar * metallic;
    float f0g = 0.04f * di + ag * metallic;
    float f0b = 0.04f * di + ab * metallic;
    float fr = f0r + (1.0f - f0r) * om;
    float fg = f0g + (1.0f - f0g) * om;
    float fb = f0b + (1.0f - f0b) * om;
    float kdr = ar * di * (1.0f - fr);
    float kdg = ag * di * (1.0f - fg);
    float kdb = ab * di * (1.0f - fb);
    float base = D * G / (4.0f * cos_i * cos_o);
    out.r = kdr / (float)kPi + base * fr;
    out.g = kdg / (float)kPi + base * fg;
    out.b = kdb / (float)kPi + base * fb;
    float pdf_d = cos_i / (float)kPi;
    float pdf_s = D * cos_h / (4.0f * vo_h);
    out.pdf = (1.0f - spec_prob) * pdf_d + spec_prob * pdf_s;
    return out;
}

void PathTracer::direct_lighting(float p_x, float p_y, float p_z, float n_x,
                                 float n_y, float n_z, float wo_x, float wo_y,
                                 float wo_z, float rough, float metallic,
                                 float spec_prob, float ar, float ag, float ab,
                                 float& or_, float& og, float& ob) {
    // renderer.gs:1807 direct_lighting
    or_ = og = ob = 0;
    if (scene_->light_tri.empty() || !(scene_->light_total > 1e-12)) return;
    float pick =
        (float)rng_.uniform01() * (float)scene_->light_total;
    // 二分探索 (until lo>=hi)
    int lo = 0, hi = (int)scene_->light_cdf.size() - 1;
    while (lo < hi) {
        int mid = (lo + hi) / 2;
        if (scene_->light_cdf[(size_t)mid] >= pick)
            hi = mid;
        else
            lo = mid + 1;
    }
    int li = scene_->light_tri[(size_t)lo];
    const Triangle& t = scene_->tris[(size_t)li];
    float lu = (float)rng_.uniform01();
    float lv = (float)rng_.uniform01();
    if (lu + lv > 1.0f) {
        lu = 1.0f - lu;
        lv = 1.0f - lv;
    }
    float lw = 1.0f - lu - lv;
    float ldx = lw * t.v0.x + (lu * t.v1.x + lv * t.v2.x) - p_x;
    float ldy = lw * t.v0.y + (lu * t.v1.y + lv * t.v2.y) - p_y;
    float ldz = lw * t.v0.z + (lu * t.v1.z + lv * t.v2.z) - p_z;
    float dist = std::sqrt(ldx * ldx + ldy * ldy + ldz * ldz);
    if (!(dist > 0.003f)) return;
    float inv = 1.0f / dist;
    ldx *= inv; ldy *= inv; ldz *= inv;
    // t.nn は正規化済み幾何法線 (毎回の sqrt を省くため事前計算値を使う)
    float lcos = -(t.nn.x * ldx + (t.nn.y * ldy + t.nn.z * ldz));
    float scos = n_x * ldx + (n_y * ldy + n_z * ldz);
    if (!(lcos > 1e-8f && scos > 1e-8f)) return;
    // pdf = P(tri)×pdf_i(d). CDF は発光重み (area×lum) なので lum 因子が要る.
    float lt_lum =
        0.2126f * (float)t.er + 0.7152f * (float)t.eg + 0.0722f * (float)t.eb;
    if (lt_lum < 1e-6f) lt_lum = 1e-6f;
    float pdf =
        dist * dist * lt_lum / (lcos * (float)scene_->light_total);
    float sox = p_x + n_x * 0.002f, soy = p_y + n_y * 0.002f, soz = p_z + n_z * 0.002f;
    if (cast_alpha_ray({sox, soy, soz}, {ldx, ldy, ldz}, dist - 0.003f).tri >= 0) return;
    float sp = spec_prob;
    float r2 = rough;
    if (r2 < 0) r2 = 0;
    if (r2 > 1) r2 = 1;
    if (r2 < 0.02f) r2 = 0.02f;
    float alpha2 = (r2 * r2) * (r2 * r2);
    Bsdf e = evaluate_opaque_bsdf(ldx, ldy, ldz, wo_x, wo_y, wo_z, n_x, n_y, n_z,
                                  alpha2, metallic, sp, ar, ag, ab);
    if (!(pdf > 1e-12f)) return;
    // MIS: NEE 側の重み pl/(pl+pb). pb は混合 pdf (e.pdf). 発光重み CDF 用の pdf.
    float wnee = 1.0f;
    static const bool nomis = std::getenv("UOW2_NOMIS") != nullptr;
    if (!nomis && e.pdf > 1e-20f) wnee = pdf / (pdf + e.pdf);
    float k = scos * wnee * (float)t.alpha / pdf;
    or_ = (float)t.er * e.r * k;
    og = (float)t.eg * e.g * k;
    ob = (float)t.eb * e.b * k;
}

PathTracer::DielectricSample PathTracer::sample_dielectric(
    float wo_x, float wo_y, float wo_z, float n_x, float n_y, float n_z,
    float curr_dx, float curr_dy, float curr_dz, float ior, bool front_face,
    float rough, float alpha2) {
    // renderer.gs:1876 sample_dielectric
    DielectricSample out;
    out.valid = true;
    float eta_i = 1, eta_t = ior;
    if (!front_face) {
        eta_i = ior;
        eta_t = 1;
    }
    float hx, hy, hz;
    if (rough <= 0.0201f) {
        hx = n_x; hy = n_y; hz = n_z;
    } else {
        sample_ggx_half(alpha2, n_x, n_y, n_z, hx, hy, hz);
        float md = wo_x * hx + (wo_y * hy + wo_z * hz);
        if (!(md > 1e-8f)) out.valid = false;
    }
    float cos_m = wo_x * hx + (wo_y * hy + wo_z * hz);
    if (cos_m < 0) {
        hx = -hx; hy = -hy; hz = -hz;
        cos_m = -cos_m;
    }
    if (cos_m > 1) cos_m = 1;
    float fres = dielectric_fresnel_f(cos_m, eta_i, eta_t);
    float eta = eta_i / eta_t;
    float s2 = eta * eta * (1.0f - cos_m * cos_m);
    if (s2 >= 1.0f || (float)rng_.uniform01() < fres) {
        out.n_x = 2.0f * cos_m * hx - wo_x;
        out.n_y = 2.0f * cos_m * hy - wo_y;
        out.n_z = 2.0f * cos_m * hz - wo_z;
        out.eta_scale = 1;
    } else {
        float cos_t = std::sqrt(1.0f - s2);
        out.n_x = eta * curr_dx + hx * (eta * cos_m - cos_t);
        out.n_y = eta * curr_dy + hy * (eta * cos_m - cos_t);
        out.n_z = eta * curr_dz + hz * (eta * cos_m - cos_t);
        out.eta_scale = eta * eta;
    }
    {
        float d = 1.0f / std::sqrt(out.n_x * out.n_x + out.n_y * out.n_y +
                                   out.n_z * out.n_z);
        out.n_x *= d; out.n_y *= d; out.n_z *= d;
    }
    if (rough > 0.0201f) {
        float cos_i = std::fabs(out.n_x * n_x + (out.n_y * n_y + out.n_z * n_z));
        float cos_o = std::fabs(wo_x * n_x + (wo_y * n_y + wo_z * n_z));
        float cos_h = std::fabs(hx * n_x + (hy * n_y + hz * n_z));
        float vo_h = std::fabs(wo_x * hx + (wo_y * hy + wo_z * hz));
        if (cos_i > 1e-8f && cos_o > 1e-8f && cos_h > 1e-8f && vo_h > 1e-8f) {
            float g1 = 2.0f * cos_o /
                       (cos_o + std::sqrt(cos_o * cos_o + alpha2 * (1.0f - cos_o * cos_o)));
            float g2 = 2.0f * cos_i /
                       (cos_i + std::sqrt(cos_i * cos_i + alpha2 * (1.0f - cos_i * cos_i)));
            // Smith 補正重み. cos_h -> 0 で発散し firefly の温床になるため上限を張る
            // (正当中継の見た目は保持. 上限は 1/cos_h^2 型のスパイクのみを抑制).
            float w = g1 * g2 * vo_h / (cos_o * cos_h);
            if (w > 10.0f) w = 10.0f;
            out.eta_scale *= w;
        } else {
            out.valid = false;
        }
    }
    return out;
}

// Stochastic surface coverage. Transparent crossings do not consume bounces.
HitInfo PathTracer::cast_alpha_ray(const Vec3& ro, const Vec3& rd, double dist) {
    double traveled = 0;
    while (traveled < dist) {
        HitInfo hit = cast_ray(ro + rd * traveled, rd, dist - traveled);
        if (hit.tri < 0) return hit;
        float opacity = (float)scene_->tris[(size_t)hit.tri].alpha;
        if (opacity >= 1 || (opacity > 0 && rng_.uniform01() < opacity)) {
            hit.t += traveled;
            return hit;
        }
        traveled += hit.t + 0.0001f;
    }
    return HitInfo{};
}

Vec3 PathTracer::pathtrace(const Vec3& ro, const Vec3& rd, int max_bounces) {
    // renderer.gs:1505 pathtrace (float 内部版)
    float ox = (float)ro.x, oy = (float)ro.y, oz = (float)ro.z;
    float dx = (float)rd.x, dy = (float)rd.y, dz = (float)rd.z;
    float accr = 0, accg = 0, accb = 0;
    float thr_r = 1, thr_g = 1, thr_b = 1;
    int bounce = 0;
    int prev_delta = 1;
    float last_pdf = 0;  // 直前の BSDF 混合 pdf (MIS 用)
    for (int b = 0; b < max_bounces; ++b) {
        HitInfo hit = cast_alpha_ray({ox, oy, oz}, {dx, dy, dz}, kFarClip);
        if (hit.tri < 0) break;
        // 補間法線を幾何法線と同半球に (pt_geom_dot)
        float pnx = (float)hit.n_interp.x, pny = (float)hit.n_interp.y,
              pnz = (float)hit.n_interp.z;
        double geom = hit.n_geom.x * pnx + (hit.n_geom.y * pny + hit.n_geom.z * pnz);
        if (geom < 0) {
            pnx = -pnx; pny = -pny; pnz = -pnz;
        }
        float ndot = dx * pnx + (dy * pny + dz * pnz);
        bool front = true;
        if (ndot > 0) {
            pnx = -pnx; pny = -pny; pnz = -pnz;
            front = false;
        }
        float wox = -dx, woy = -dy, woz = -dz;
        float rough = (float)hit.rough;
        if (rough < 0) rough = 0;
        if (rough > 1) rough = 1;
        if (rough < 0.02f) rough = 0.02f;
        float alpha = rough * rough;
        float alpha2 = alpha * alpha;
        float metallic = (float)scene_->tris[(size_t)hit.tri].metallic;
        if (metallic < 0) metallic = 0;
        if (metallic > 1) metallic = 1;

        if (bounce == 0 || prev_delta == 1) {
            accr += (float)hit.er * thr_r;
            accg += (float)hit.eg * thr_g;
            accb += (float)hit.eb * thr_b;
        } else {
            // MIS (BSDF サンプル側): 直前の方向 pdf (pb) と, 前頂点からこの光点を
            // NEE で選ぶ確率 (pl) の電力比で配分. 発光重み CDF と整合.
            const Triangle& lt = scene_->tris[(size_t)hit.tri];
            float lx = (float)hit.p.x - ox, ly = (float)hit.p.y - oy,
                  lz = (float)hit.p.z - oz;
            float dist2 = lx * lx + ly * ly + lz * lz;
            float dist = std::sqrt(dist2);
            float pl = 1e30f;
            if (dist > 1e-20f && last_pdf > 1e-20f) {
                float inv = 1.0f / dist;
                lx *= inv; ly *= inv; lz *= inv;
                float cos_l = -(lt.nn.x * lx + (lt.nn.y * ly + lt.nn.z * lz));
                if (cos_l > 1e-8f) {
                    float lum = 0.2126f * (float)lt.er + 0.7152f * (float)lt.eg +
                                0.0722f * (float)lt.eb;
                    if (lum < 1e-6f) lum = 1e-6f;
                    pl = dist2 * lum / (cos_l * (float)scene_->light_total);
                }
            }
            static const bool nomis2 = std::getenv("UOW2_NOMIS") != nullptr;
            float w = nomis2 ? 0.0f
                             : ((last_pdf + pl > 1e-20f) ? last_pdf / (last_pdf + pl)
                                                          : 0.0f);
            accr += (float)hit.er * thr_r * w;
            accg += (float)hit.eg * thr_g * w;
            accb += (float)hit.eb * thr_b * w;
        }

        if (hit.ior > 1.0001) {
            DielectricSample ds = sample_dielectric(wox, woy, woz, pnx, pny, pnz,
                                                    dx, dy, dz, (float)hit.ior,
                                                    front, rough, alpha2);
            if (!ds.valid) break;
            thr_r *= ds.eta_scale;
            thr_g *= ds.eta_scale;
            thr_b *= ds.eta_scale;
            float s = 1.0f;
            if (ds.n_x * pnx + (ds.n_y * pny + ds.n_z * pnz) < 0) s = -1.0f;
            ox = (float)hit.p.x + pnx * 0.001f * s;
            oy = (float)hit.p.y + pny * 0.001f * s;
            oz = (float)hit.p.z + pnz * 0.001f * s;
            dx = ds.n_x; dy = ds.n_y; dz = ds.n_z;
            prev_delta = 1;
        } else {
            float spec_prob = 0.05f + 0.95f * metallic;
            if (spec_prob > 0.98f) spec_prob = 0.98f;
            if (metallic >= 0.999f) spec_prob = 1.0f;
            // direct_lighting (renderer.gs:1807)
            {
                float dlr, dlg, dlb;
                direct_lighting((float)hit.p.x, (float)hit.p.y, (float)hit.p.z,
                                pnx, pny, pnz, wox, woy, woz, rough, metallic,
                                spec_prob, (float)hit.ar, (float)hit.ag,
                                (float)hit.ab, dlr, dlg, dlb);
                accr += dlr * thr_r;
                accg += dlg * thr_g;
                accb += dlb * thr_b;
            }
            float nx2, ny2, nz2;
            if ((float)rng_.uniform01() < spec_prob) {
                float hx, hy, hz;
                sample_ggx_half(alpha2, pnx, pny, pnz, hx, hy, hz);
                float md = wox * hx + (woy * hy + woz * hz);
                if (!(md > 1e-8f)) break;
                nx2 = 2.0f * md * hx - wox;
                ny2 = 2.0f * md * hy - woy;
                nz2 = 2.0f * md * hz - woz;
                prev_delta = (rough <= 0.0201f) ? 1 : 0;
            } else {
                sample_cosine_hemisphere(pnx, pny, pnz, nx2, ny2, nz2);
                prev_delta = 0;
            }
            float ncos = nx2 * pnx + (ny2 * pny + nz2 * pnz);
            if (!(ncos > 1e-8f)) break;
            {
                float d = 1.0f / std::sqrt(nx2 * nx2 + ny2 * ny2 + nz2 * nz2);
                nx2 *= d; ny2 *= d; nz2 *= d;
            }
            Bsdf e = evaluate_opaque_bsdf(nx2, ny2, nz2, wox, woy, woz, pnx, pny,
                                          pnz, alpha2, metallic, spec_prob,
                                          (float)hit.ar, (float)hit.ag,
                                          (float)hit.ab);
            if (!(e.pdf > 1e-12f)) break;
            last_pdf = e.pdf;
            float cos_i = nx2 * pnx + (ny2 * pny + nz2 * pnz);
            thr_r *= e.r * cos_i / e.pdf;
            thr_g *= e.g * cos_i / e.pdf;
            thr_b *= e.b * cos_i / e.pdf;
            ox = (float)hit.p.x + pnx * 0.001f;
            oy = (float)hit.p.y + pny * 0.001f;
            oz = (float)hit.p.z + pnz * 0.001f;
            dx = nx2; dy = ny2; dz = nz2;
        }

        if (thr_r < 0) thr_r = 0;
        if (thr_g < 0) thr_g = 0;
        if (thr_b < 0) thr_b = 0;
        float mx = std::max(thr_r, thr_g);
        mx = std::max(mx, thr_b);
        if (!(mx >= 1e-12f)) break;
        // 不偏 Russian Roulette (打ち切り+重み補正で期待値は不変)。
        // 深いバウンスほど1ray単価が高い (非干渉性) ため bounce>=1 から開始し、
        // 高sppではノイズ増は不可視。元の bounce>=2 より深部を刈る。
        if (bounce >= 1) {
            float rr = mx;
            if (rr < 0.05f) rr = 0.05f;
            if (rr > 0.9f) rr = 0.9f;
            if (!((float)rng_.uniform01() <= rr)) break;
            float ir = 1.0f / rr;
            thr_r *= ir;
            thr_g *= ir;
            thr_b *= ir;
        }
        bounce++;
    }
    return {accr, accg, accb};
}

Vec3 PathTracer::sample_sensor(double sx, double sy, const Camera& cam,
                               double focal2, int max_bounces) {
    // pixel proc の方向生成部 (renderer.gs:1288)
    double dist = 1.0 / std::sqrt(sx * sx + (sy * sy + focal2));
    double tx = sx * dist, ty = sy * dist, tz = cam.focal * dist;
    Vec3 rd{tx * cam.m0 + (ty * cam.m1 + tz * cam.m2),
            tx * cam.m3 + (ty * cam.m4 + tz * cam.m5),
            tx * cam.m6 + (ty * cam.m7 + tz * cam.m8)};
    return pathtrace({cam.x, cam.y, cam.z}, rd, max_bounces);
}

std::vector<uint8_t> render_image(Scene& scene, const Camera& cam,
                                  const RenderConfig& cfg, uint64_t seed) {
    // 単純版 (テスト用)。並列版と同一の画素ストリーム・適応判定を使う.
    PathTracer pt(&scene, seed);
    double focal2 = cam.focal * cam.focal;
    const double w_half = cfg.width * 0.5;
    const double h_half = cfg.height * 0.5;
    const double jlo = 1e-12;
    const double jhi = cfg.resolution - 1e-12;
    const double jspan = jhi - jlo;
    const bool use_adapt =
        cfg.adapt_min > 0 && cfg.adapt_min < cfg.spp && !std::getenv("UOW2_NOADAPT");
    const int adapt_min = use_adapt ? cfg.adapt_min : cfg.spp + 1;
    const int adapt_step = cfg.adapt_step > 0 ? cfg.adapt_step : 8;
    const float clamp_max = (float)cfg.clamp;
    std::vector<uint8_t> img((size_t)cfg.width * cfg.height * 3);
    for (int y = 0; y < cfg.height; ++y) {
        uint64_t row_seed = seed ^ ((uint64_t)(y + 1) * 0x9E3779B97F4A7C15ULL);
        for (int x = 0; x < cfg.width; ++x) {
            pt.rng().seed(row_seed ^ ((uint64_t)(x + 1) * 0xBF58476D1CE4E5B9ULL));
            double rr = 0, gg = 0, bb = 0;
            double sum_l = 0, sum_l2 = 0;
            int s = 0;
            for (; s < cfg.spp; ++s) {
                // goboscript: px=$x+random(1e-12,res-1e-12)
                double jx = jlo + jspan * pt.rng().uniform01();
                double jy = jlo + jspan * pt.rng().uniform01();
                // 全画素化: 中央原点のセンサ座標
                double sx = (x + jx) - w_half;
                // PPM/PNG/mp4 は先頭行が画面上。Scratch 同様 Y+ を上にするため
                // 0行目が +sy(上向き) に対応するよう反転する。
                double sy = h_half - (y + jy);
                Vec3 c = pt.sample_sensor(sx, sy, cam, focal2, cfg.max_bounces);
                if (clamp_max > 0) {
                    if (c.x > clamp_max) c.x = clamp_max;
                    if (c.y > clamp_max) c.y = clamp_max;
                    if (c.z > clamp_max) c.z = clamp_max;
                }
                rr += c.x;
                gg += c.y;
                bb += c.z;
                if (use_adapt) {
                    double lum = 0.2126 * c.x + 0.7152 * c.y + 0.0722 * c.z;
                    sum_l += lum;
                    sum_l2 += lum * lum;
                    int n = s + 1;
                    if (n >= adapt_min && (n % adapt_step) == 0) {
                        double mean = sum_l / n;
                        double var = (sum_l2 - sum_l * mean) / (n - 1);
                        if (var < 0) var = 0;
                        double thr = cfg.adapt_rel * mean + cfg.adapt_abs;
                        if (var < thr * thr) break;
                    }
                }
            }
            int n = (use_adapt && s < cfg.spp) ? s + 1 : cfg.spp;
            double inv_n = 1.0 / n;
            uint8_t R, G, B;
            tonemap_pack(rr * inv_n, gg * inv_n, bb * inv_n, R, G, B);
            size_t o = ((size_t)y * cfg.width + x) * 3;
            img[o] = R;
            img[o + 1] = G;
            img[o + 2] = B;
        }
    }
    return img;
}

std::vector<uint8_t> render_image_parallel(
    Scene& scene, const Camera& cam, const RenderConfig& cfg, uint64_t seed,
    int num_threads, const std::function<void(int done, int total)>& progress) {
    if (num_threads <= 0) {
        num_threads = (int)std::thread::hardware_concurrency();
        if (num_threads <= 0) num_threads = 4;
    }
    num_threads = std::min(num_threads, cfg.height);
    std::vector<uint8_t> img((size_t)cfg.width * cfg.height * 3);
    double focal2 = cam.focal * cam.focal;
    const double w_half = cfg.width * 0.5;
    const double h_half = cfg.height * 0.5;
    const double jlo = 1e-12;
    const double jhi = cfg.resolution - 1e-12;
    const double jspan = jhi - jlo;
    std::atomic<int> next_row{0};
    std::atomic<int> done{0};
    std::atomic<long> total_samples{0};
    std::vector<std::thread> workers;
    workers.reserve((size_t)num_threads);
    // 動的スケジューリング: 行コストが不均一 (空/幾何) かつ P/E 非対称のため、
    // 静的等分割より atomic カウンタで1行ずつ払い出す方が速い。
    // 画素ストリームは (y,x) のみで決まるためスレッド数非依存の決定性は維持される.
    const bool use_adapt =
        cfg.adapt_min > 0 && cfg.adapt_min < cfg.spp && !std::getenv("UOW2_NOADAPT");
    const int adapt_min = use_adapt ? cfg.adapt_min : cfg.spp + 1;
    const int adapt_step = cfg.adapt_step > 0 ? cfg.adapt_step : 8;
    const float clamp_max = (float)cfg.clamp;
    for (int t = 0; t < num_threads; ++t) {
        workers.emplace_back([&] {
            // 行ごとに PathTracer を作り直す (RNG 状態が行 y のみで決まる)
            long local_samples = 0;
            for (;;) {
                int y = next_row.fetch_add(1, std::memory_order_relaxed);
                if (y >= cfg.height) break;
                uint64_t row_seed =
                    seed ^ ((uint64_t)(y + 1) * 0x9E3779B97F4A7C15ULL);
                PathTracer pt(&scene, row_seed);
                uint8_t* row = img.data() + (size_t)y * cfg.width * 3;
                for (int x = 0; x < cfg.width; ++x) {
                    // 画素独立ストリーム (適応の有無に関わらず同一。早期終了しても
                    // 他画素に影響しない)
                    pt.rng().seed(row_seed ^ ((uint64_t)(x + 1) * 0xBF58476D1CE4E5B9ULL));
                    double rr = 0, gg = 0, bb = 0;
                    double sum_l = 0, sum_l2 = 0;
                    int s = 0;
                    for (; s < cfg.spp; ++s) {
                        double jx = jlo + jspan * pt.rng().uniform01();
                        double jy = jlo + jspan * pt.rng().uniform01();
                        double sx = (x + jx) - w_half;
                        // 同上 (PPM先頭行=画面上、Y+が上)
                        double sy = h_half - (y + jy);
                        Vec3 c = pt.sample_sensor(sx, sy, cam, focal2, cfg.max_bounces);
                        if (clamp_max > 0) {
                            if (c.x > clamp_max) c.x = clamp_max;
                            if (c.y > clamp_max) c.y = clamp_max;
                            if (c.z > clamp_max) c.z = clamp_max;
                        }
                        rr += c.x;
                        gg += c.y;
                        bb += c.z;
                        if (use_adapt) {
                            double lum = 0.2126 * c.x + 0.7152 * c.y + 0.0722 * c.z;
                            sum_l += lum;
                            sum_l2 += lum * lum;
                            int n = s + 1;
                            if (n >= adapt_min && (n % adapt_step) == 0) {
                                double mean = sum_l / n;
                                double var =
                                    (sum_l2 - sum_l * mean) / (n - 1);
                                if (var < 0) var = 0;
                                double thr = cfg.adapt_rel * mean + cfg.adapt_abs;
                                if (var < thr * thr) break;
                            }
                        }
                    }
                    int n = s + (use_adapt && s < cfg.spp ? 1 : 0);
                    // n は実行サンプル数 (break 時は s が 0-based のため +1)
                    if (!use_adapt) n = cfg.spp;
                    local_samples += n;
                    double inv_n = 1.0 / n;
                    uint8_t R, G, B;
                    tonemap_pack(rr * inv_n, gg * inv_n, bb * inv_n, R, G, B);
                    size_t o = (size_t)x * 3;
                    row[o] = R;
                    row[o + 1] = G;
                    row[o + 2] = B;
                }
                int d = ++done;
                if (progress) progress(d, cfg.height);
            }
            total_samples += local_samples;
        });
    }
    for (auto& th : workers) th.join();
    if (std::getenv("UOW2_STATS"))
        std::fprintf(stderr, "[stats] avg_spp=%.1f\n",
                     (double)total_samples / (cfg.width * cfg.height));
    return img;
}

}  // namespace raymotion
