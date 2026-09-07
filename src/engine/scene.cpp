// scene.cpp — addtriangle proc の移植.
#include "raymotion/scene.hpp"

#include <algorithm>
#include <cmath>

namespace raymotion {

int Scene::add_triangle(const Vec3& p0, const Vec3& p1, const Vec3& p2, double u0,
                        double v0, double u1, double v1, double u2, double v2,
                        const Vec3& n0, const Vec3& n1, const Vec3& n2, double ar,
                        double ag, double ab, double er, double eg, double eb,
                        double metallic, double ior, double rough, int shader) {
    Vec3 e1 = p1 - p0;
    Vec3 e2 = p2 - p0;
    Vec3 n = cross(e1, e2);
    double d = -(n.x * p0.x + (n.y * p0.y + n.z * p0.z));
    double d00 = dot(e1, e1);
    double d01 = dot(e1, e2);
    double d11 = dot(e2, e2);
    double denom = d00 * d11 - d01 * d01;
    if (std::abs(denom) <= 1e-12) return -1;
    double inv = 1.0 / denom;
    Vec3 U{(e1.x * d11 - e2.x * d01) * inv, (e1.y * d11 - e2.y * d01) * inv,
           (e1.z * d11 - e2.z * d01) * inv};
    Vec3 V{(e2.x * d00 - e1.x * d01) * inv, (e2.y * d00 - e1.y * d01) * inv,
           (e2.z * d00 - e1.z * d01) * inv};

    Triangle t;
    t.v0 = p0;
    t.v1 = p1;
    t.v2 = p2;
    t.n = n;
    t.d = d;
    t.U = U;
    t.V = V;
    t.denom = denom;
    t.nn = normalize(n);
    t.tu0 = u0;
    t.tv0 = v0;
    t.tu1 = u1;
    t.tv1 = v1;
    t.tu2 = u2;
    t.tv2 = v2;
    t.n0 = n0;
    t.n1 = n1;
    t.n2 = n2;
    t.mn = {std::min({p0.x, p1.x, p2.x}), std::min({p0.y, p1.y, p2.y}),
            std::min({p0.z, p1.z, p2.z})};
    t.mx = {std::max({p0.x, p1.x, p2.x}), std::max({p0.y, p1.y, p2.y}),
            std::max({p0.z, p1.z, p2.z})};
    t.ar = ar;
    t.ag = ag;
    t.ab = ab;
    t.er = er;
    t.eg = eg;
    t.eb = eb;
    t.ior = ior;
    t.rough = rough;
    t.shader = shader;
    t.metallic = metallic;

    int idx = (int)tris.size();
    tris.push_back(t);
    if (er + (eg + eb) > 0) {
        double area2 = d00 * d11 - d01 * d01;
        if (area2 < 0) area2 = 0;
        double area = 0.5 * std::sqrt(area2);
        if (area > 1e-12) {
            // 発光強度重み CDF (分散低減。pdf 式と整合するため寄与計算は不変).
            double lum = 0.2126 * er + 0.7152 * eg + 0.0722 * eb;
            if (lum < 1e-6) lum = 1e-6;
            light_total += area * lum;
            light_tri.push_back(idx);
            light_cdf.push_back(light_total);
        }
    }
    return idx;
}

}  // namespace raymotion
