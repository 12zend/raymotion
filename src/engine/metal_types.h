#pragma once
// Shared CPU/MSL ABI: scalar triples deliberately avoid float3's 16-byte alignment.
namespace metal_data {
struct MetalVec3 { float x, y, z; };
struct Triangle {
    MetalVec3 v0, v1, v2, n, nn, n0, n1, n2;
    float ar, ag, ab, er, eg, eb, ior, rough, metallic, alpha;
    int shader;
    float tu0,tv0,tu1,tv1,tu2,tv2;
    int texture_offset,texture_width,texture_height;
};
struct Camera {
    float x, y, z, focal;
    float m0, m1, m2, m3, m4, m5, m6, m7, m8;
};
// Preorder escape link in right; internal nodes have count == 0.
struct BvhNode {
    MetalVec3 mn, mx;
    int right, offset, count;
};
struct Params {
    Camera camera;
    int width, height, spp, bounces, nodes, lights, adapt_min, adapt_step, nomis, opaque;
    float light_total, resolution, clamp_value, adapt_rel, adapt_abs;
    int restir_candidates, restir_spatial, restir_mcap;
    float restir_radius;
    // Previous frame camera for temporal reservoir reuse (valid iff hasHistory).
    Camera prevCamera;
    int hasHistory;
};
// ReSTIR DI primary G-buffer (1 pixel = 1 primary hit).
struct RestirGBuffer {
    MetalVec3 pos;
    MetalVec3 normal;
    MetalVec3 albedo;
    MetalVec3 wo;
    MetalVec3 emission;
    float rough;
    float metallic;
    float ior;
    int kind; // 0 miss, 1 opaque, 3 dielectric
};
// ReSTIR DI reservoir (screen-space reuse for primary direct lighting).
// Dual weight sums: sumWs includes visibility (for the contribution weight W),
// sumWu excludes it (for the MIS sampling-pdf estimate).
struct RestirReservoir {
    MetalVec3 lightPos;
    MetalVec3 lightNormal;
    MetalVec3 emission;
    float alpha;
    float p_hat;
    float sumWs;
    float sumWu;
    unsigned M;
    unsigned valid;
};
}
