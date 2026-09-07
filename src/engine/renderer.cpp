// renderer.cpp — 高レベル API の実装.
#include "raymotion/renderer.hpp"

#include <atomic>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <mutex>
#include <sstream>

#include "raymotion/bvh.hpp"
#include "raymotion/obj.hpp"
#include "raymotion/metal.hpp"

namespace raymotion {
namespace fs = std::filesystem;

Renderer::Renderer() {
    camera.update_focal();
    camera.update_trig();
}

void Renderer::build(int max_leaf_tris) { build_bvh(scene, max_leaf_tris); }

void Renderer::configure(int width, int height, int spp, int bounces) {
    config.width = width;
    config.height = height;
    if (spp > 0) config.spp = spp;
    if (bounces > 0) config.max_bounces = bounces;
}

std::vector<uint8_t> Renderer::render(uint64_t seed, ProgressFn progress) {
    std::vector<uint8_t> image;
    std::string error;
    if (!render_image_metal(scene, camera, config, seed, image, error))
        throw std::runtime_error("Metal: " + error);
    if (progress) progress(config.height, config.height);
    return image;
}

bool Renderer::render_to_ppm(const std::string& path, uint64_t seed,
                             ProgressFn progress) {
    auto img = render(seed, std::move(progress));
    return save_ppm(path, config.width, config.height, img);
}

bool Renderer::render_to_png(const std::string& path, uint64_t seed,
                             ProgressFn progress) {
    auto img = render(seed, std::move(progress));
    return save_png(path, config.width, config.height, img);
}

bool Renderer::render_sequence(const std::vector<Camera>& cameras,
                               const std::string& out_dir, const std::string& basename,
                               uint64_t seed, ProgressFn progress) {
    std::error_code ec;
    fs::create_directories(out_dir, ec);
    if (ec) return false;
    std::atomic<int> done{0};
    std::mutex io_mu;
    bool ok = true;
    // Render each frame on Metal.
    for (size_t f = 0; f < cameras.size(); ++f) {
        char name[64];
        std::snprintf(name, sizeof(name), "%s%04d.ppm", basename.c_str(), (int)f);
        std::vector<uint8_t> img;
        std::string error;
        if (!render_image_metal(scene, cameras[f], config, seed + (uint64_t)f, img, error))
            throw std::runtime_error("Metal: " + error);
        std::string err;
        if (!save_ppm((fs::path(out_dir) / name).string(), config.width,
                      config.height, img, &err)) {
            std::lock_guard<std::mutex> lk(io_mu);
            std::cerr << "frame " << f << ": " << err << "\n";
            ok = false;
            break;
        }
        int d = ++done;
        if (progress) progress(d, (int)cameras.size());
    }
    return ok;
}

bool Renderer::render_lerp_video(const Camera& a, const Camera& b,
                                 const std::string& out_mp4, const VideoOptions& opt,
                                 ProgressFn progress) {
    std::vector<Camera> cams;
    cams.reserve((size_t)opt.frames);
    for (int f = 0; f < opt.frames; ++f) {
        double t = opt.frames == 1 ? 0 : (double)f / (opt.frames - 1);
        cams.push_back(lerp_camera(a, b, t));
    }
    std::string dir = opt.frames_dir.empty() ? out_mp4 + ".frames" : opt.frames_dir;
    bool ok = render_sequence(cams, dir, "f", opt.seed, progress);
    if (!ok) return false;
    std::string err;
    if (!encode_mp4(dir, "f", out_mp4, opt.fps, &err)) {
        std::cerr << err << "\n";
        return false;
    }
    if (!opt.keep_frames) {
        std::error_code ec;
        fs::remove_all(dir, ec);
    }
    return true;
}

bool Renderer::render_orbit_video(const std::string& out_mp4, const Vec3& center,
                                  double radius, double height, double start_deg,
                                  double end_deg, const VideoOptions& opt,
                                  ProgressFn progress) {
    std::vector<Camera> cams;
    cams.reserve((size_t)opt.frames);
    for (int f = 0; f < opt.frames; ++f) {
        double t = opt.frames == 1 ? 0 : (double)f / (opt.frames - 1);
        double ang = start_deg + (end_deg - start_deg) * t;
        cams.push_back(orbit_camera(camera, center, radius, height, ang));
    }
    std::string dir = opt.frames_dir.empty() ? out_mp4 + ".frames" : opt.frames_dir;
    bool ok = render_sequence(cams, dir, "f", opt.seed, progress);
    if (!ok) return false;
    std::string err;
    if (!encode_mp4(dir, "f", out_mp4, opt.fps, &err)) {
        std::cerr << err << "\n";
        return false;
    }
    if (!opt.keep_frames) {
        std::error_code ec;
        fs::remove_all(dir, ec);
    }
    return true;
}

// --- カメラヘルパー ---

Camera lerp_camera(const Camera& a, const Camera& b, double t) {
    Camera c = a;
    c.x = a.x + (b.x - a.x) * t;
    c.y = a.y + (b.y - a.y) * t;
    c.z = a.z + (b.z - a.z) * t;
    c.dirx = a.dirx + (b.dirx - a.dirx) * t;
    c.diry = a.diry + (b.diry - a.diry) * t;
    c.dirz = a.dirz + (b.dirz - a.dirz) * t;
    c.fov = a.fov + (b.fov - a.fov) * t;
    c.update_focal();
    c.update_trig();
    return c;
}

Camera orbit_camera(const Camera& base, const Vec3& center, double radius,
                    double height, double angle_deg) {
    Camera c = base;
    double r = deg2rad(angle_deg);
    c.x = center.x + radius * std::cos(r);
    c.z = center.z + radius * std::sin(r);
    c.y = height;
    // 注視点 center に向くピッチ・ヨーを度で求める (goboscript の回転順と等価な近似:
    // diry(ヨー) → dirx(ピッチ) の順で合わせる. dirz は base のまま)
    double dx = center.x - c.x;
    double dy = center.y - c.y;
    double dz = center.z - c.z;
    c.diry = rad2deg(std::atan2(dx, dz));
    double horiz = std::sqrt(dx * dx + dz * dz);
    c.dirx = -rad2deg(std::atan2(dy, horiz));
    c.update_focal();
    c.update_trig();
    return c;
}

// --- 保存・ffmpeg ---

bool save_ppm(const std::string& path, int w, int h, const std::vector<uint8_t>& rgb,
              std::string* err) {
    if ((int)rgb.size() != w * h * 3) {
        if (err) *err = "bad buffer size";
        return false;
    }
    std::ofstream f(path, std::ios::binary);
    if (!f) {
        if (err) *err = "cannot open " + path;
        return false;
    }
    f << "P6\n" << w << " " << h << "\n255\n";
    f.write((const char*)rgb.data(), (std::streamsize)rgb.size());
    return (bool)f;
}

// CRC32 (PNG 用, 依存なし)
static uint32_t crc32_tab[256];
static bool crc32_init = false;
static void crc32_make() {
    for (uint32_t i = 0; i < 256; ++i) {
        uint32_t c = i;
        for (int k = 0; k < 8; ++k) c = (c & 1) ? (0xEDB88320u ^ (c >> 1)) : (c >> 1);
        crc32_tab[i] = c;
    }
    crc32_init = true;
}
static uint32_t crc32_calc(const uint8_t* p, size_t n) {
    if (!crc32_init) crc32_make();
    uint32_t c = 0xFFFFFFFFu;
    for (size_t i = 0; i < n; ++i) c = crc32_tab[(c ^ p[i]) & 0xFF] ^ (c >> 8);
    return c ^ 0xFFFFFFFFu;
}
static void put_u32(std::vector<uint8_t>& o, uint32_t v) {
    o.push_back((uint8_t)(v >> 24));
    o.push_back((uint8_t)(v >> 16));
    o.push_back((uint8_t)(v >> 8));
    o.push_back((uint8_t)v);
}
static void png_chunk(std::vector<uint8_t>& o, const char* type,
                      const uint8_t* data, size_t len) {
    put_u32(o, (uint32_t)len);
    size_t t0 = o.size();
    for (int i = 0; i < 4; ++i) o.push_back((uint8_t)type[i]);
    if (len) o.insert(o.end(), data, data + len);
    uint32_t c = crc32_calc(o.data() + t0, 4 + len);
    put_u32(o, c);
}

bool save_png(const std::string& path, int w, int h, const std::vector<uint8_t>& rgb,
              std::string* err) {
    if ((int)rgb.size() != w * h * 3) {
        if (err) *err = "bad buffer size";
        return false;
    }
    // 生スキャンライン (filter 0 + RGB)
    std::vector<uint8_t> raw;
    raw.reserve((size_t)h * ((size_t)w * 3 + 1));
    for (int y = 0; y < h; ++y) {
        raw.push_back(0);
        raw.insert(raw.end(), rgb.begin() + (size_t)y * w * 3,
                   rgb.begin() + (size_t)(y + 1) * w * 3);
    }
    // zlib ストリーム (stored blocks, 依存なし)
    std::vector<uint8_t> z;
    z.push_back(0x78);
    z.push_back(0x01);  // zlib header (FCHECK 付き)
    size_t pos = 0;
    while (pos < raw.size()) {
        size_t n = std::min<size_t>(65535, raw.size() - pos);
        bool last = (pos + n == raw.size());
        z.push_back(last ? 0x01 : 0x00);  // BFINAL + BTYPE=00
        z.push_back((uint8_t)(n & 0xFF));
        z.push_back((uint8_t)((n >> 8) & 0xFF));
        z.push_back((uint8_t)(~n & 0xFF));
        z.push_back((uint8_t)((~n >> 8) & 0xFF));
        z.insert(z.end(), raw.begin() + (ssize_t)pos, raw.begin() + (ssize_t)(pos + n));
        pos += n;
    }
    uint32_t ad1 = 1, ad2 = 0;  // Adler-32
    for (uint8_t b : raw) {
        ad1 = (ad1 + b) % 65521;
        ad2 = (ad2 + ad1) % 65521;
    }
    put_u32(z, (ad2 << 16) | ad1);

    std::vector<uint8_t> png;
    const uint8_t sig[8] = {137, 80, 78, 71, 13, 10, 26, 10};
    png.insert(png.end(), sig, sig + 8);
    uint8_t ihdr[13];
    ihdr[0] = (uint8_t)(w >> 24);
    ihdr[1] = (uint8_t)(w >> 16);
    ihdr[2] = (uint8_t)(w >> 8);
    ihdr[3] = (uint8_t)w;
    ihdr[4] = (uint8_t)(h >> 24);
    ihdr[5] = (uint8_t)(h >> 16);
    ihdr[6] = (uint8_t)(h >> 8);
    ihdr[7] = (uint8_t)h;
    ihdr[8] = 8;
    ihdr[9] = 2;
    ihdr[10] = 0;
    ihdr[11] = 0;
    ihdr[12] = 0;
    png_chunk(png, "IHDR", ihdr, 13);
    png_chunk(png, "IDAT", z.data(), z.size());
    png_chunk(png, "IEND", nullptr, 0);

    std::ofstream f(path, std::ios::binary);
    if (!f) {
        if (err) *err = "cannot open " + path;
        return false;
    }
    f.write((const char*)png.data(), (std::streamsize)png.size());
    return (bool)f;
}

bool have_ffmpeg() { return std::system("ffmpeg -hide_banner -loglevel error -version"
                                        " >/dev/null 2>&1") == 0; }

bool encode_mp4(const std::string& frames_dir, const std::string& basename,
                const std::string& out_mp4, int fps, std::string* err) {
    if (!have_ffmpeg()) {
        if (err) *err = "ffmpeg not found in PATH";
        return false;
    }
    // 入力は PPM 連番. yuv420p + faststart で互換性を確保.
    // 奇数サイズは H.264 が扱えないため偶数に寄せる.
    std::ostringstream cmd;
    cmd << "ffmpeg -hide_banner -loglevel error -y -framerate " << fps << " -i \""
        << (fs::path(frames_dir) / (basename + "%04d.ppm")).string()
        << "\" -vf \"scale=trunc(iw/2)*2:trunc(ih/2)*2\""
           " -c:v libx264 -pix_fmt yuv420p -movflags +faststart \"" << out_mp4
        << "\"";
    int rc = std::system(cmd.str().c_str());
    if (rc != 0) {
        if (err) *err = "ffmpeg failed";
        return false;
    }
    return true;
}

}  // namespace raymotion
