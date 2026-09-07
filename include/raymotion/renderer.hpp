#pragma once
#include <cstdint>
#include <functional>
#include <string>
#include <vector>

#include "raymotion/pathtrace.hpp"
#include "raymotion/scene.hpp"

namespace raymotion {

// progress: (done, total). render 単体では行数、sequence ではフレーム数.
using ProgressFn = std::function<void(int done, int total)>;

struct VideoOptions {
    int frames = 60;
    int fps = 30;
    uint64_t seed = 12345;  // 各フレーム seed + frame_index
    std::string frames_dir;  // 空なら一時ディレクトリ相当 (out.mp4 + ".frames")
    bool keep_frames = false;
    int threads = 0;  // 0 = auto
};

// カメラ補間ヘルパー
Camera lerp_camera(const Camera& a, const Camera& b, double t);
// 水平旋回カメラ: center を注視し、半径 radius・高さ height・方位角 angle_deg.
Camera orbit_camera(const Camera& base, const Vec3& center, double radius,
                    double height, double angle_deg);

class Renderer {
public:
    Scene scene;
    Camera camera;
    RenderConfig config;
    int threads = 0;  // 0 = hardware_concurrency

    Renderer();

    // --- シーン構築 ---
    // BVH 構築 (generatebvhtree 対応)
    void build(int max_leaf_tris = 8);

    // --- 設定 ---
    void configure(int width, int height, int spp = -1, int bounces = -1);
    void set_camera(const Camera& cam) { camera = cam; }
    int effective_threads() const;

    // --- 静止画 ---
    std::vector<uint8_t> render(uint64_t seed = 12345, ProgressFn progress = {});
    bool render_to_ppm(const std::string& path, uint64_t seed = 12345,
                       ProgressFn progress = {});
    bool render_to_png(const std::string& path, uint64_t seed = 12345,
                       ProgressFn progress = {});
    // PNG 保存には外部コマンド不要の最小エンコーダを使う (依存なし)

    // --- 動画 ---
    // cameras.size() == frames の各カメラで連番 PPM を out_dir に書く
    bool render_sequence(const std::vector<Camera>& cameras, const std::string& out_dir,
                         const std::string& basename = "f", uint64_t seed = 12345,
                         ProgressFn progress = {});
    // A→B の線形補間動画
    bool render_lerp_video(const Camera& a, const Camera& b, const std::string& out_mp4,
                           const VideoOptions& opt = {}, ProgressFn progress = {});
    // 旋回動画 (ffmpeg で mp4 化まで行う)
    bool render_orbit_video(const std::string& out_mp4, const Vec3& center, double radius,
                            double height, double start_deg, double end_deg,
                            const VideoOptions& opt = {}, ProgressFn progress = {});
};

// 連番 PPM (out_dir/basename%04d.ppm) を ffmpeg で mp4 化する.
// ffmpeg が無ければ false を返す.
bool encode_mp4(const std::string& frames_dir, const std::string& basename,
                const std::string& out_mp4, int fps, std::string* err = nullptr);
bool have_ffmpeg();

// PPM バッファ保存・読込ヘルパー
bool save_ppm(const std::string& path, int w, int h, const std::vector<uint8_t>& rgb,
              std::string* err = nullptr);
bool save_png(const std::string& path, int w, int h, const std::vector<uint8_t>& rgb,
              std::string* err = nullptr);

}  // namespace raymotion
