#pragma once
// Shared CPU/MSL ABI: scalar triples deliberately avoid float3's 16-byte alignment.
namespace metal_data {
struct MetalVec3 { float x, y, z; };
struct Triangle {
    MetalVec3 v0, v1, v2, n, nn, n0, n1, n2;
    float ar, ag, ab, er, eg, eb, ior, rough, metallic;
    int shader;
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
    int width, height, spp, bounces, nodes, lights, adapt_min, adapt_step, nomis;
    float light_total, resolution, clamp_value, adapt_rel, adapt_abs;
};
}
