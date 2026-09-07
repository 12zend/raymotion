#pragma once
// Camera and Metal rendering settings.
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
    struct VectorView {
        const double &x, &y, &z;
        operator Vec3() const { return {x, y, z}; }
    };
    struct View {
        VectorView position, rotation;
        const double& fov;
    };
    const View get{{x, y, z}, {dirx, diry, dirz}, fov};

    Camera() = default;
    // Views must refer to this camera even when copied for interpolation.
    Camera(const Camera& other) : Camera() { *this = other; }
    Camera& operator=(const Camera& other) {
        x = other.x; y = other.y; z = other.z;
        dirx = other.dirx; diry = other.diry; dirz = other.dirz;
        fov = other.fov; focal = other.focal;
        m0 = other.m0; m1 = other.m1; m2 = other.m2;
        m3 = other.m3; m4 = other.m4; m5 = other.m5;
        m6 = other.m6; m7 = other.m7; m8 = other.m8;
        return *this;
    }
    void update_trig();   // trigonometry proc
    void update_focal();  // focallength = 240/tan(fov*0.5) (度)
    void set(Vec3 position = {}, Vec3 rotation = {}, double field_of_view = 60) {
        x = position.x; y = position.y; z = position.z;
        dirx = rotation.x; diry = rotation.y; dirz = rotation.z;
        fov = field_of_view;
        update_trig();
        update_focal();
    }
};

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
    // ReSTIR DI (reservoir-based spatiotemporal importance resampling).
    // 直接照明の分散を下げ、低 spp でも滑らかにする。不偏性を保つ。
    // candidates: 初期 RIS 候補数 (1 で従来の単一 NEE と等価)。8 前後を推奨。
    // spatial: 空間再利用する近傍数 (0 で無効)。3〜5 前後を推奨。
    // radius: 空間再利用の探索半径 (px)。8〜16 前後を推奨。
    // mcap: reservoir の M 上限 (時間的過信の防止)。128 前後を推奨。
    // 0 にすると該当機能を無効化する (candidates<=1 && spatial<=0 で完全に従来動作)。
    int restir_candidates = 8;
    int restir_spatial = 4;
    double restir_radius = 16;
    int restir_mcap = 128;
};

} // namespace raymotion
