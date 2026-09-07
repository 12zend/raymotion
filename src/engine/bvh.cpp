// bvh.cpp — generatebvhtree/buildnode/sort/min/max の移植.
// 分割は SAH ビニング (goboscript の中央値分割より走査が速い。画像は不変).
#include "raymotion/bvh.hpp"

#include <algorithm>
#include <limits>
#include <numeric>
#include <queue>
#include <tuple>

namespace raymotion {
namespace {

inline double centroid_axis(const Triangle& t, int axis) {
    if (axis == 0) return t.v0.x + t.v1.x + t.v2.x;
    if (axis == 1) return t.v0.y + t.v1.y + t.v2.y;
    return t.v0.z + t.v1.z + t.v2.z;
}

inline double area_of(const Vec3& mn, const Vec3& mx) {
    double dx = mx.x - mn.x, dy = mx.y - mn.y, dz = mx.z - mn.z;
    return 2.0 * (dx * dy + dy * dz + dz * dx);
}

}  // namespace



void build_bvh(Scene& scene, int max_leaf_tris) {
    max_leaf_tris = std::max(1, max_leaf_tris);
    scene.clear_bvh();
    size_t n = scene.tris.size();
    if (n == 0) return;
    scene.max_leaf_tris = max_leaf_tris;
    scene.bvh_tri.resize(n);
    std::iota(scene.bvh_tri.begin(), scene.bvh_tri.end(), 0);

    struct Job {
        int offset;
        int count;
        int node;
    };
    // goboscript は明示 stack リストで BFS する. ここでは queue で等価に組む.
    // ノードは事前に作らず, 親から子へ割り当てる.
    scene.nodes.reserve(2 * n);
    scene.nodes.push_back(BvhNode{});
    std::queue<Job> q;
    q.push({0, (int)n, 0});

    constexpr int kBins = 16;
    // ビニング用一時バッファ (再利用して確保を避ける)
    int bin_count[kBins];
    Vec3 bin_mn[kBins], bin_mx[kBins];

    while (!q.empty()) {
        Job job = q.front();
        q.pop();
        // AABB 計算 (min/max proc 相当) + 重心境界
        Vec3 mn{std::numeric_limits<double>::infinity(),
                std::numeric_limits<double>::infinity(),
                std::numeric_limits<double>::infinity()};
        Vec3 mx{-std::numeric_limits<double>::infinity(),
                -std::numeric_limits<double>::infinity(),
                -std::numeric_limits<double>::infinity()};
        Vec3 cmn{std::numeric_limits<double>::infinity(),
                 std::numeric_limits<double>::infinity(),
                 std::numeric_limits<double>::infinity()};
        Vec3 cmx{-std::numeric_limits<double>::infinity(),
                 -std::numeric_limits<double>::infinity(),
                 -std::numeric_limits<double>::infinity()};
        int* base = scene.bvh_tri.data() + job.offset;
        for (int k = 0; k < job.count; ++k) {
            const Triangle& t = scene.tris[(size_t)base[k]];
            if (t.mn.x < mn.x) mn.x = t.mn.x;
            if (t.mn.y < mn.y) mn.y = t.mn.y;
            if (t.mn.z < mn.z) mn.z = t.mn.z;
            if (t.mx.x > mx.x) mx.x = t.mx.x;
            if (t.mx.y > mx.y) mx.y = t.mx.y;
            if (t.mx.z > mx.z) mx.z = t.mx.z;
            double cx = t.v0.x + t.v1.x + t.v2.x;
            double cy = t.v0.y + t.v1.y + t.v2.y;
            double cz = t.v0.z + t.v1.z + t.v2.z;
            if (cx < cmn.x) cmn.x = cx;
            if (cy < cmn.y) cmn.y = cy;
            if (cz < cmn.z) cmn.z = cz;
            if (cx > cmx.x) cmx.x = cx;
            if (cy > cmx.y) cmx.y = cy;
            if (cz > cmx.z) cmx.z = cz;
        }
        scene.nodes[(size_t)job.node].mn = mn;
        scene.nodes[(size_t)job.node].mx = mx;

        if (job.count <= max_leaf_tris) {
            scene.nodes[(size_t)job.node].offset = job.offset;
            scene.nodes[(size_t)job.node].count = job.count;
            scene.nodes[(size_t)job.node].left =
                scene.nodes[(size_t)job.node].right = -1;
            continue;
        }

        // ---- SAH ビニングで最良の (軸, 分割) を探す ----
        double parent_area = area_of(mn, mx);
        if (!(parent_area > 0)) {
            // 縮退 (全て同一点等): 中央分割にフォールバック
            int mid = job.count / 2;
            int left = (int)scene.nodes.size();
            scene.nodes.push_back(BvhNode{});
            int right = (int)scene.nodes.size();
            scene.nodes.push_back(BvhNode{});
            scene.nodes[(size_t)job.node].left = left;
            scene.nodes[(size_t)job.node].right = right;
            scene.nodes[(size_t)job.node].offset = -1;
            scene.nodes[(size_t)job.node].count = 0;
            // 重心ソートなしの等分割では偏るため軸ソートしてから割る
            Vec3 sz{mx.x - mn.x, mx.y - mn.y, mx.z - mn.z};
            int ax = (sz.x > sz.y) ? ((sz.x > sz.z) ? 0 : 2) : ((sz.y > sz.z) ? 1 : 2);
            std::sort(base, base + job.count, [&](int a, int b) {
                return centroid_axis(scene.tris[(size_t)a], ax) <
                       centroid_axis(scene.tris[(size_t)b], ax);
            });
            q.push({job.offset, mid, left});
            q.push({job.offset + mid, job.count - mid, right});
            continue;
        }
        double best_cost = (double)job.count;  // リーフコスト (Ct=1, Ci=1 正規化)
        int best_axis = -1, best_bin = -1;
        for (int axis = 0; axis < 3; ++axis) {
            double c0 = (axis == 0) ? cmn.x : (axis == 1 ? cmn.y : cmn.z);
            double c1 = (axis == 0) ? cmx.x : (axis == 1 ? cmx.y : cmx.z);
            double span = c1 - c0;
            if (!(span > 0)) continue;
            for (int b = 0; b < kBins; ++b) {
                bin_count[b] = 0;
                bin_mn[b] = Vec3{std::numeric_limits<double>::infinity(),
                                 std::numeric_limits<double>::infinity(),
                                 std::numeric_limits<double>::infinity()};
                bin_mx[b] = Vec3{-std::numeric_limits<double>::infinity(),
                                 -std::numeric_limits<double>::infinity(),
                                 -std::numeric_limits<double>::infinity()};
            }
            double inv = kBins / span;
            for (int k = 0; k < job.count; ++k) {
                const Triangle& t = scene.tris[(size_t)base[k]];
                double c = centroid_axis(t, axis);
                int b = (int)((c - c0) * inv);
                if (b < 0) b = 0;
                if (b >= kBins) b = kBins - 1;
                bin_count[b]++;
                if (t.mn.x < bin_mn[b].x) bin_mn[b].x = t.mn.x;
                if (t.mn.y < bin_mn[b].y) bin_mn[b].y = t.mn.y;
                if (t.mn.z < bin_mn[b].z) bin_mn[b].z = t.mn.z;
                if (t.mx.x > bin_mx[b].x) bin_mx[b].x = t.mx.x;
                if (t.mx.y > bin_mx[b].y) bin_mx[b].y = t.mx.y;
                if (t.mx.z > bin_mx[b].z) bin_mx[b].z = t.mx.z;
            }
            // 右側の累積 (sweep)
            int rcount[kBins];
            Vec3 rmn[kBins], rmx[kBins];
            int acc = 0;
            Vec3 amn{std::numeric_limits<double>::infinity(),
                     std::numeric_limits<double>::infinity(),
                     std::numeric_limits<double>::infinity()};
            Vec3 amx{-std::numeric_limits<double>::infinity(),
                     -std::numeric_limits<double>::infinity(),
                     -std::numeric_limits<double>::infinity()};
            for (int b = kBins - 1; b >= 0; --b) {
                acc += bin_count[b];
                rcount[b] = acc;
                if (bin_count[b] > 0) {
                    if (bin_mn[b].x < amn.x) amn.x = bin_mn[b].x;
                    if (bin_mn[b].y < amn.y) amn.y = bin_mn[b].y;
                    if (bin_mn[b].z < amn.z) amn.z = bin_mn[b].z;
                    if (bin_mx[b].x > amx.x) amx.x = bin_mx[b].x;
                    if (bin_mx[b].y > amx.y) amx.y = bin_mx[b].y;
                    if (bin_mx[b].z > amx.z) amx.z = bin_mx[b].z;
                }
                rmn[b] = amn;
                rmx[b] = amx;
            }
            // 左から sweep して分割コストを評価
            int lcount = 0;
            Vec3 lmn{std::numeric_limits<double>::infinity(),
                     std::numeric_limits<double>::infinity(),
                     std::numeric_limits<double>::infinity()};
            Vec3 lmx{-std::numeric_limits<double>::infinity(),
                     -std::numeric_limits<double>::infinity(),
                     -std::numeric_limits<double>::infinity()};
            for (int b = 0; b < kBins - 1; ++b) {
                lcount += bin_count[b];
                if (bin_count[b] > 0) {
                    if (bin_mn[b].x < lmn.x) lmn.x = bin_mn[b].x;
                    if (bin_mn[b].y < lmn.y) lmn.y = bin_mn[b].y;
                    if (bin_mn[b].z < lmn.z) lmn.z = bin_mn[b].z;
                    if (bin_mx[b].x > lmx.x) lmx.x = bin_mx[b].x;
                    if (bin_mx[b].y > lmx.y) lmx.y = bin_mx[b].y;
                    if (bin_mx[b].z > lmx.z) lmx.z = bin_mx[b].z;
                }
                int rc = rcount[b + 1];
                if (lcount == 0 || rc == 0) continue;
                double cost =
                    (area_of(lmn, lmx) * lcount + area_of(rmn[b + 1], rmx[b + 1]) * rc) /
                    parent_area;
                if (cost < best_cost) {
                    best_cost = cost;
                    best_axis = axis;
                    best_bin = b;
                }
            }
        }

        int mid = -1;
        if (best_axis >= 0) {
            // 最良ビン境界で in-place 分割
            double c0 = (best_axis == 0) ? cmn.x : (best_axis == 1 ? cmn.y : cmn.z);
            double c1 = (best_axis == 0) ? cmx.x : (best_axis == 1 ? cmx.y : cmx.z);
            double split = c0 + (c1 - c0) * (best_bin + 1) / (double)kBins;
            int* lo = base;
            int* hi = base + job.count;
            // centroid < split を左へ
            int* m = std::partition(lo, hi, [&](int a) {
                return centroid_axis(scene.tris[(size_t)a], best_axis) < split;
            });
            mid = (int)(m - base);
            if (mid == 0 || mid == job.count) mid = -1;  // 縮退
        }
        if (mid < 0) {
            // フォールバック: 最長軸の中央値分割
            Vec3 sz{mx.x - mn.x, mx.y - mn.y, mx.z - mn.z};
            int ax = (sz.x > sz.y) ? ((sz.x > sz.z) ? 0 : 2) : ((sz.y > sz.z) ? 1 : 2);
            std::sort(base, base + job.count, [&](int a, int b) {
                return centroid_axis(scene.tris[(size_t)a], ax) <
                       centroid_axis(scene.tris[(size_t)b], ax);
            });
            mid = job.count / 2;
        }
        int left = (int)scene.nodes.size();
        scene.nodes.push_back(BvhNode{});
        int right = (int)scene.nodes.size();
        scene.nodes.push_back(BvhNode{});
        scene.nodes[(size_t)job.node].left = left;
        scene.nodes[(size_t)job.node].right = right;
        scene.nodes[(size_t)job.node].offset = -1;
        scene.nodes[(size_t)job.node].count = 0;
        q.push({job.offset, mid, left});
        q.push({job.offset + mid, job.count - mid, right});
    }

}

void refit_bvh(Scene& scene) {
    if (scene.bvh_tri.size() != scene.tris.size() || scene.nodes.empty()) {
        build_bvh(scene); return;
    }
    for (auto it = scene.nodes.rbegin(); it != scene.nodes.rend(); ++it) {
        auto& node = *it;
        Vec3 mn{INFINITY, INFINITY, INFINITY}, mx{-INFINITY, -INFINITY, -INFINITY};
        auto expand = [&](Vec3 a, Vec3 b) {
            mn = {std::min(mn.x,a.x),std::min(mn.y,a.y),std::min(mn.z,a.z)};
            mx = {std::max(mx.x,b.x),std::max(mx.y,b.y),std::max(mx.z,b.z)};
        };
        if (node.is_leaf()) {
            for (int i=0;i<node.count;++i) {
                const auto& t=scene.tris[scene.bvh_tri[node.offset+i]];
                expand(t.mn,t.mx);
            }
        } else {
            expand(scene.nodes[node.left].mn,scene.nodes[node.left].mx);
            expand(scene.nodes[node.right].mn,scene.nodes[node.right].mx);
        }
        node.mn=mn; node.mx=mx;
    }
}

}  // namespace raymotion
