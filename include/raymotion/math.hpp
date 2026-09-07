#pragma once
// math.hpp — goboscript (Scratch) の数値セマンティクスを C++ に移植.
// 対応: renderer.gs の cos/sin/tan/atan/sqrt/abs/floor/ceil/%/random
// Scratch の三角関数は「度」基準. C++ 標準はラジアンなのでラッパで合わせる.

#include <cmath>
#include <cstdint>
#include <random>

namespace raymotion {

inline constexpr double kPi = 3.141592653589793;
inline constexpr double kFarClip = 1e30;
inline constexpr double kEps = 1e-8;
inline constexpr double kDenomEps = 1e-12;

inline double deg2rad(double d) { return d * kPi / 180.0; }
inline double rad2deg(double r) { return r * 180.0 / kPi; }

// Scratch/goboscript 互換の三角関数 (度)
inline double cos_deg(double d) { return std::cos(deg2rad(d)); }
inline double sin_deg(double d) { return std::sin(deg2rad(d)); }
inline double tan_deg(double d) { return std::tan(deg2rad(d)); }
// 単引数 atan (度, -90..90). renderer.gs は atan(ix/iz) + (iz<=0?180:0) で方位角を作る.
inline double atan_deg(double v) { return rad2deg(std::atan(v)); }

// Scratch の mod: 除数が正なら結果は [0,b). C++ fmod とは負数の扱いが違う.
inline double gobo_mod(double a, double b) {
    if (b == 0) return 0;
    double r = a - std::floor(a / b) * b;
    return r;
}

struct Vec3 {
    double x = 0, y = 0, z = 0;
    Vec3() = default;
    Vec3(double x_, double y_, double z_) : x(x_), y(y_), z(z_) {}
    Vec3 operator+(const Vec3& o) const { return {x + o.x, y + o.y, z + o.z}; }
    Vec3 operator-(const Vec3& o) const { return {x - o.x, y - o.y, z - o.z}; }
    Vec3 operator*(double s) const { return {x * s, y * s, z * s}; }
    Vec3 operator/(double s) const { return {x / s, y / s, z / s}; }
    Vec3& operator+=(const Vec3& o) {
        x += o.x; y += o.y; z += o.z; return *this;
    }
};

inline double dot(const Vec3& a, const Vec3& b) {
    return a.x * b.x + a.y * b.y + a.z * b.z;
}
inline Vec3 cross(const Vec3& a, const Vec3& b) {
    return {
        a.y * b.z - a.z * b.y,
        a.z * b.x - a.x * b.z,
        a.x * b.y - a.y * b.x,
    };
}
inline double length(const Vec3& v) { return std::sqrt(dot(v, v)); }
inline Vec3 normalize(const Vec3& v) {
    double l = length(v);
    if (l < 1e-30) return {0, 0, 0};
    return v / l;
}

// goboscript の normalize x,y,z proc に対応 (tmpx/tmpy/tmpz に相当)
inline Vec3 gobo_normalize(double x, double y, double z) {
    double d = 1.0 / std::sqrt(x * x + (y * y + z * z));
    return {x * d, y * d, z * d};
}

// goboscript の random(a,b) に対応.
// Scratch は整数同士だと整数を返すが, このレンダラの用途
// (random(".",1), random(".",360), random(1e-12,res-1e-12) 等) は
// すべて連続一様乱数を期待しているため, 常に double 一様分布とする.
// "." は数値変換で 0 として扱う (Scratch の文字列→数値変換に相当).
// 高速化: std::mt19937_64 + uniform_real_distribution は 1呼出し数十nsと重いため,
// xorshift64star (2ns級) + 上位53bit→double 変換に置換. 分布は等価 (列は不一致で可).
class Rng {
public:
    explicit Rng(uint64_t seed = 12345) { this->seed(seed); }
    void seed(uint64_t s) {
        // splitmix64 で 0 回避 + 拡散して初期化
        uint64_t z = s + 0x9E3779B97F4A7C15ULL;
        z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9ULL;
        z = (z ^ (z >> 27)) * 0x94D049BB133111EBULL;
        state_ = (z ^ (z >> 31)) | 1ULL;  // 0 を避ける
        if (state_ == 0) state_ = 0x2545F4914F6CDD1DULL;
    }
    // [0,1) の一様分布 (上位53bit 使用)
    inline double uniform01() {
        uint64_t x = state_;
        x ^= x >> 12;
        x ^= x << 25;
        x ^= x >> 27;
        state_ = x;
        uint64_t r = x * 0x2545F4914F6CDD1DULL;
        return (double)(r >> 11) * 1.1102230246251565e-16;
    }
    // [lo, hi] の一様分布 (Scratch の random は両端含む. double では差は無視できる)
    inline double uniform(double lo, double hi) {
        double u = uniform01();
        // lo>hi の呼出しは現行コードに無いが互換のため残す (分岐予測はほぼ不要)
        if (lo > hi) {
            double t = lo;
            lo = hi;
            hi = t;
        }
        return lo + (hi - lo) * u;
    }

private:
    uint64_t state_ = 1;
};

}  // namespace raymotion
