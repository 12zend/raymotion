// Metal float port of pathtrace.cpp; keep BSDF, MIS and sampling changes in sync.
#include <metal_stdlib>
using namespace metal;
#if USE_METAL_RT
#include <metal_raytracing>
using namespace raytracing;
#endif
#include "metal_types.h"
using namespace metal_data;
constant float kPi=3.141592653589793f, kFarClip=1e30f;
struct HitInfo {
    bool hit = false;
    float t = kFarClip;
    int tri = -1;
    float u = 0, v = 0;
    float3 p;
    float3 n_interp;
    float ar = 0, ag = 0, ab = 0;
    float er = 0, eg = 0, eb = 0;
    float ior = 0, rough = 0;
    int shader = 0;
    float3 n_geom;
};
float3 gobo_normalize(float x,float y,float z) { return normalize(float3(x,y,z)); }
float atan_deg(float x) { return atan(x)*180.0f/kPi; }
float gobo_mod(float a,float b) { return a-floor(a/b)*b; }
void hsv2rgb(float h, float s, float v, thread float& out_r, thread float& out_g,
             thread float& out_b) {
    float c = s * v;
    float x = c * (1 - abs(gobo_mod(h / 60.0f, 2.0f) - 1));
    float m = v - c;
    float cr = 0, cg = 0, cb = 0;
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
static inline void make_tangent_f(float nx, float ny, float nz, thread float& tx,
                                  thread float& ty, thread float& tz, thread float& bx, thread float& by,
                                  thread float& bz) {
    if (fabs(nx) > 0.1f) {
        tx = nz; ty = 0.0f; tz = -nx;
    } else {
        tx = 0.0f; ty = -nz; tz = ny;
    }
    float l = 1.0f / sqrt(tx * tx + ty * ty + tz * tz);
    tx *= l; ty *= l; tz *= l;
    bx = ny * tz - nz * ty;
    by = nz * tx - nx * tz;
    bz = nx * ty - ny * tx;
}
float fast_sincos_f(float a, thread float& c) { c=cos(a); return sin(a); }
static inline float dielectric_fresnel_f(float ci_in, float eta_in, float eta_out) {
    float ci = ci_in;
    if (ci < 0) ci = 0;
    if (ci > 1) ci = 1;
    float eta = eta_in / eta_out;
    float s2 = eta * eta * (1.0f - ci * ci);
    if (s2 >= 1) return 1;
    float ct = sqrt(1.0f - s2);
    float rp = (eta_out * ci - eta_in * ct) / (eta_out * ci + eta_in * ct);
    float rs = (eta_in * ci - eta_out * ct) / (eta_in * ci + eta_out * ct);
    return 0.5f * (rp * rp + rs * rs);
}
// PCG RXS-M-XS: 32-bit state avoids emulated 64-bit multiplies per GPU sample.
struct Rng {
 uint state;
 void seed(ulong s) { ulong z=s+0x9E3779B97F4A7C15UL; z=(z^(z>>30))*0xBF58476D1CE4E5B9UL; z=(z^(z>>27))*0x94D049BB133111EBUL; state=uint(z^(z>>31)); }
 float uniform01() {
  state=state*747796405u+2891336453u;
  uint word=((state>>((state>>28u)+4u))^state)*277803737u;
  return float(((word>>22u)^word)>>8u)*0x1p-24f;
 }
};
struct PathTracer {
#if USE_METAL_RT
 primitive_acceleration_structure acceleration;
#endif
 device const Triangle* tris;
 device const MetalVec3* textures;
 device const BvhNode* nodes;
 device const int* indices;
 device const int* lights;
 device const float* cdf;
 Params params;
 Rng rng_;
 struct Bsdf { float r=0,g=0,b=0,pdf=0; };
 struct DielectricSample { bool valid=true; float n_x,n_y,n_z,eta_scale=1; };
 HitInfo intersect(float3 ro,float3 rd,float dist, bool shadow) {
   HitInfo hit; hit.t=dist;
   if(params.nodes==0) return hit;
#if USE_METAL_RT
   ray r;
   r.origin=ro; r.direction=rd; r.min_distance=1e-8f; r.max_distance=dist;
   intersector<triangle_data> query;
   query.assume_geometry_type(geometry_type::triangle);
   query.force_opacity(forced_opacity::opaque);
   query.accept_any_intersection(shadow);
   auto result=query.intersect(r,acceleration);
   if(result.type!=intersection_type::none) {
      hit.hit=true; hit.tri=int(result.primitive_id); hit.t=result.distance;
      hit.u=result.triangle_barycentric_coord.x; hit.v=result.triangle_barycentric_coord.y;
      hit.p=ro+rd*hit.t;
   }
   return hit;
#else
   float3 safe_rd=select(rd,float3(1e-30f),rd==0.0f);
   float3 inv=1.0f/safe_rd;
   int ni=0;
   while(ni<params.nodes) {
     const device BvhNode& n=nodes[ni];
     float3 mn=float3(n.mn.x,n.mn.y,n.mn.z), mx=float3(n.mx.x,n.mx.y,n.mx.z);
     float3 a=(mn-ro)*inv, b=(mx-ro)*inv;
     float3 low=min(a,b), high=max(a,b);
     float near_t=max(0.0f,max(low.x,max(low.y,low.z)));
     float far_t=min(hit.t,min(high.x,min(high.y,high.z)));
     if(near_t>far_t) { ni=n.right; continue; }
     ++ni;
     if(n.count==0) continue;
     for(int k=0;k<n.count;++k) {
       int j=indices[n.offset+k]; const device Triangle& t=tris[j];
       float3 v0=float3(t.v0.x,t.v0.y,t.v0.z);
       float3 e1=float3(t.v1.x,t.v1.y,t.v1.z)-v0, e2=float3(t.v2.x,t.v2.y,t.v2.z)-v0;
       float3 p=cross(rd,e2); float det=dot(e1,p); if(abs(det)<1e-12f) continue;
       float3 tv=ro-v0; float u=dot(tv,p)/det; if(u<0 || u>1) continue;
       float3 q=cross(tv,e1); float v=dot(rd,q)/det; if(v<0 || u+v>1) continue;
       float d=dot(e2,q)/det;
       if(d>1e-8f && d<hit.t) { hit.t=d; hit.tri=j; hit.u=u; hit.v=v; hit.hit=true; if(shadow) return hit; }
     }
   }
   hit.p=ro+rd*hit.t; return hit;
#endif
 }
 bool cast_shadow_ray(float3 ro,float3 rd,float dist) { return intersect(ro,rd,dist,true).hit; }
HitInfo cast_ray(float3 ro, float3 rd, float dist) {
    HitInfo hit=intersect(ro,rd,dist,false);
    if (hit.tri >= 0) {
        const device Triangle& t = tris[(size_t)hit.tri];
        float w = 1 - (hit.u + hit.v);
        float3 ni{w * t.n0.x + (hit.u * t.n1.x + hit.v * t.n2.x),
                w * t.n0.y + (hit.u * t.n1.y + hit.v * t.n2.y),
                w * t.n0.z + (hit.u * t.n1.z + hit.v * t.n2.z)};
        ni = gobo_normalize(ni.x, ni.y, ni.z);
        hit.n_interp = ni;
        hit.ar = t.ar;
        hit.ag = t.ag;
        hit.ab = t.ab;
        if(t.texture_offset>=0) {
            float u=fract(w*t.tu0+hit.u*t.tu1+hit.v*t.tu2);
            float v=fract(w*t.tv0+hit.u*t.tv1+hit.v*t.tv2);
            int x=min(int(u*t.texture_width),t.texture_width-1);
            int y=t.texture_height-1-min(int(v*t.texture_height),t.texture_height-1);
            MetalVec3 c=textures[t.texture_offset+y*t.texture_width+x];
            hit.ar*=c.x; hit.ag*=c.y; hit.ab*=c.z;
        }
        hit.er = t.er;
        hit.eg = t.eg;
        hit.eb = t.eb;
        hit.ior = t.ior;
        hit.rough = t.rough;
        hit.shader = t.shader;
        hit.n_geom = float3(t.n.x,t.n.y,t.n.z);
        if (hit.shader == 1) {
            float az = atan_deg(hit.p.x / hit.p.z) + ((hit.p.z > 0) ? 0 : 180);
            float h = gobo_mod(az, 360.0f) / 360.0f;
            float cr, cg, cb;
            hsv2rgb(h, 0.8f, 1, cr, cg, cb);
            hit.ar = cr;
            hit.ag = cg;
            hit.ab = cb;
        }
    }
    return hit;
}
void sample_cosine_hemisphere(float nx, float ny, float nz, thread float& ox,
                                          thread float& oy, thread float& oz) {
    float ru = (float)rng_.uniform01();
    float phi = (float)rng_.uniform01() * (float)(2.0f * kPi);
    float c;
    float sp = fast_sincos_f(phi, c);
    float rr = sqrt(ru);
    float lx = rr * c;
    float ly = rr * sp;
    float lz = sqrt(max(0.0f, 1.0f - ru));
    float tx, ty, tz, bx, by, bz;
    make_tangent_f(nx, ny, nz, tx, ty, tz, bx, by, bz);
    ox = tx * lx + (bx * ly + nx * lz);
    oy = ty * lx + (by * ly + ny * lz);
    oz = tz * lx + (bz * ly + nz * lz);
}
void sample_ggx_half(float alpha2, float nx, float ny, float nz,
                                 thread float& hx, thread float& hy, thread float& hz) {

    float ru_unused = (float)rng_.uniform01();
    (void)ru_unused;
    float rv = (float)rng_.uniform01();
    float phi = (float)rng_.uniform01() * (float)(2.0f * kPi);
    float c;
    float sp = fast_sincos_f(phi, c);
    float cs = sqrt((1.0f - rv) / (1.0f + (alpha2 - 1.0f) * rv));
    float sn = sqrt(max(0.0f, 1.0f - cs * cs));
    float lx = sn * c;
    float ly = sn * sp;
    float lz = cs;
    float tx, ty, tz, bx, by, bz;
    make_tangent_f(nx, ny, nz, tx, ty, tz, bx, by, bz);
    hx = tx * lx + (bx * ly + nx * lz);
    hy = ty * lx + (by * ly + ny * lz);
    hz = tz * lx + (bz * ly + nz * lz);
    float d = 1.0f / sqrt(hx * hx + hy * hy + hz * hz);
    hx *= d; hy *= d; hz *= d;
}
Bsdf evaluate_opaque_bsdf(float wi_x, float wi_y, float wi_z,
                                                  float wo_x, float wo_y, float wo_z,
                                                  float n_x, float n_y, float n_z,
                                                  float alpha2, float metallic,
                                                  float spec_prob, float ar,
                                                  float ag, float ab) {
    Bsdf out;
    float cos_i = wi_x * n_x + (wi_y * n_y + wi_z * n_z);
    float cos_o = wo_x * n_x + (wo_y * n_y + wo_z * n_z);
    if (!(cos_i > 1e-8f && cos_o > 1e-8f)) return out;
    float hx = wi_x + wo_x, hy = wi_y + wo_y, hz = wi_z + wo_z;
    float hl = sqrt(hx * hx + hy * hy + hz * hz);
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
               (cos_o + sqrt(cos_o * cos_o + alpha2 * (1.0f - cos_o * cos_o)));
    float g2 = 2.0f * cos_i /
               (cos_i + sqrt(cos_i * cos_i + alpha2 * (1.0f - cos_i * cos_i)));
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
void direct_lighting(float p_x, float p_y, float p_z, float n_x,
                                 float n_y, float n_z, float wo_x, float wo_y,
                                 float wo_z, float rough, float metallic,
                                 float spec_prob, float ar, float ag, float ab,
                                 thread float& or_, thread float& og, thread float& ob) {
    or_ = og = ob = 0;
    if ((params.lights == 0) || !(params.light_total > 1e-12f)) return;
    float pick =
        (float)rng_.uniform01() * (float)params.light_total;
    int lo = 0, hi = (int)params.lights - 1;
    while (lo < hi) {
        int mid = (lo + hi) / 2;
        if (cdf[(size_t)mid] >= pick)
            hi = mid;
        else
            lo = mid + 1;
    }
    int li = lights[(size_t)lo];
    const device Triangle& t = tris[(size_t)li];
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
    float dist = sqrt(ldx * ldx + ldy * ldy + ldz * ldz);
    if (!(dist > 0.003f)) return;
    float inv = 1.0f / dist;
    ldx *= inv; ldy *= inv; ldz *= inv;
    float lcos = -(t.nn.x * ldx + (t.nn.y * ldy + t.nn.z * ldz));
    float scos = n_x * ldx + (n_y * ldy + n_z * ldz);
    if (!(lcos > 1e-8f && scos > 1e-8f)) return;
    float lt_lum =
        0.2126f * (float)t.er + 0.7152f * (float)t.eg + 0.0722f * (float)t.eb;
    if (lt_lum < 1e-6f) lt_lum = 1e-6f;
    float pdf =
        dist * dist * lt_lum / (lcos * (float)params.light_total);
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
    float wnee = 1.0f;
    bool nomis = params.nomis;
    if (!nomis && e.pdf > 1e-20f) wnee = pdf / (pdf + e.pdf);
    float k = scos * wnee * (float)t.alpha / pdf;
    or_ = (float)t.er * e.r * k;
    og = (float)t.eg * e.g * k;
    ob = (float)t.eb * e.b * k;
}
DielectricSample sample_dielectric(
    float wo_x, float wo_y, float wo_z, float n_x, float n_y, float n_z,
    float curr_dx, float curr_dy, float curr_dz, float ior, bool front_face,
    float rough, float alpha2) {
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
        float cos_t = sqrt(1.0f - s2);
        out.n_x = eta * curr_dx + hx * (eta * cos_m - cos_t);
        out.n_y = eta * curr_dy + hy * (eta * cos_m - cos_t);
        out.n_z = eta * curr_dz + hz * (eta * cos_m - cos_t);
        out.eta_scale = eta * eta;
    }
    {
        float d = 1.0f / sqrt(out.n_x * out.n_x + out.n_y * out.n_y +
                                   out.n_z * out.n_z);
        out.n_x *= d; out.n_y *= d; out.n_z *= d;
    }
    if (rough > 0.0201f) {
        float cos_i = fabs(out.n_x * n_x + (out.n_y * n_y + out.n_z * n_z));
        float cos_o = fabs(wo_x * n_x + (wo_y * n_y + wo_z * n_z));
        float cos_h = fabs(hx * n_x + (hy * n_y + hz * n_z));
        float vo_h = fabs(wo_x * hx + (wo_y * hy + wo_z * hz));
        if (cos_i > 1e-8f && cos_o > 1e-8f && cos_h > 1e-8f && vo_h > 1e-8f) {
            float g1 = 2.0f * cos_o /
                       (cos_o + sqrt(cos_o * cos_o + alpha2 * (1.0f - cos_o * cos_o)));
            float g2 = 2.0f * cos_i /
                       (cos_i + sqrt(cos_i * cos_i + alpha2 * (1.0f - cos_i * cos_i)));

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
HitInfo cast_alpha_ray(float3 ro, float3 rd, float dist) {
    float traveled = 0;
    while (traveled < dist) {
        HitInfo hit = cast_ray(ro + rd * traveled, rd, dist - traveled);
        if (hit.tri < 0) return hit;
        float opacity = (float)tris[(size_t)hit.tri].alpha;
        if (opacity >= 1 || (opacity > 0 && rng_.uniform01() < opacity)) {
            hit.t += traveled;
            return hit;
        }
        traveled += hit.t + 0.0001f;
    }
    return HitInfo{};
}

float3 pathtrace(float3 ro, float3 rd, int max_bounces) {
    float ox = (float)ro.x, oy = (float)ro.y, oz = (float)ro.z;
    float dx = (float)rd.x, dy = (float)rd.y, dz = (float)rd.z;
    float accr = 0, accg = 0, accb = 0;
    float thr_r = 1, thr_g = 1, thr_b = 1;
    int bounce = 0;
    int prev_delta = 1;
    float last_pdf = 0;
    for (int b = 0; b < max_bounces; ++b) {
        HitInfo hit = cast_alpha_ray({ox, oy, oz}, {dx, dy, dz}, kFarClip);
        if (hit.tri < 0) break;
        float pnx = (float)hit.n_interp.x, pny = (float)hit.n_interp.y,
              pnz = (float)hit.n_interp.z;
        float geom = hit.n_geom.x * pnx + (hit.n_geom.y * pny + hit.n_geom.z * pnz);
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
        float metallic = (float)tris[(size_t)hit.tri].metallic;
        if (metallic < 0) metallic = 0;
        if (metallic > 1) metallic = 1;
        if (bounce == 0 || prev_delta == 1) {
            accr += (float)hit.er * thr_r;
            accg += (float)hit.eg * thr_g;
            accb += (float)hit.eb * thr_b;
        } else {

            const device Triangle& lt = tris[(size_t)hit.tri];
            float lx = (float)hit.p.x - ox, ly = (float)hit.p.y - oy,
                  lz = (float)hit.p.z - oz;
            float dist2 = lx * lx + ly * ly + lz * lz;
            float dist = sqrt(dist2);
            float pl = 1e30f;
            if (dist > 1e-20f && last_pdf > 1e-20f) {
                float inv = 1.0f / dist;
                lx *= inv; ly *= inv; lz *= inv;
                float cos_l = -(lt.nn.x * lx + (lt.nn.y * ly + lt.nn.z * lz));
                if (cos_l > 1e-8f) {
                    float lum = 0.2126f * (float)lt.er + 0.7152f * (float)lt.eg +
                                0.0722f * (float)lt.eb;
                    if (lum < 1e-6f) lum = 1e-6f;
                    pl = dist2 * lum / (cos_l * (float)params.light_total);
                }
            }
            bool nomis2 = params.nomis;
            float w = nomis2 ? 0.0f
                             : ((last_pdf + pl > 1e-20f) ? last_pdf / (last_pdf + pl)
                                                          : 0.0f);
            accr += (float)hit.er * thr_r * w;
            accg += (float)hit.eg * thr_g * w;
            accb += (float)hit.eb * thr_b * w;
        }
        if (hit.ior > 1.0001f) {
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
                float d = 1.0f / sqrt(nx2 * nx2 + ny2 * ny2 + nz2 * nz2);
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
        float mx = max(thr_r, thr_g);
        mx = max(mx, thr_b);
        if (!(mx >= 1e-12f)) break;

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
float3 sample_sensor(float sx, float sy, Camera cam,
                               float focal2, int max_bounces) {
    float dist = 1.0f / sqrt(sx * sx + (sy * sy + focal2));
    float tx = sx * dist, ty = sy * dist, tz = cam.focal * dist;
    float3 rd{tx * cam.m0 + (ty * cam.m1 + tz * cam.m2),
            tx * cam.m3 + (ty * cam.m4 + tz * cam.m5),
            tx * cam.m6 + (ty * cam.m7 + tz * cam.m8)};
    return pathtrace({cam.x, cam.y, cam.z}, rd, max_bounces);
}
};
kernel void render_paths(device const Triangle* tris [[buffer(0)]], device const BvhNode* nodes [[buffer(1)]], device const int* indices [[buffer(2)]], device const int* lights [[buffer(3)]], device const float* cdf [[buffer(4)]], constant Params& params [[buffer(5)]], constant ulong& seed [[buffer(6)]], device uchar* output [[buffer(7)]], device const MetalVec3* textures [[buffer(9)]], uint id [[thread_position_in_grid]]
#if USE_METAL_RT
 , primitive_acceleration_structure acceleration [[buffer(8)]]
#endif
) {
 if(id>=uint(params.width)*uint(params.height)) return;
 uint x=id%params.width,y=id/params.width;
 PathTracer pt;
#if USE_METAL_RT
 pt.acceleration=acceleration;
#endif
 pt.textures=textures; pt.tris=tris; pt.nodes=nodes; pt.indices=indices; pt.lights=lights; pt.cdf=cdf; pt.params=params;
 pt.rng_.seed(seed^(ulong(y+1)*0x9E3779B97F4A7C15UL)^(ulong(x+1)*0xBF58476D1CE4E5B9UL));
 float3 sum=0; float sum_l=0,sum_l2=0; int n=0;
 for(int s=0;s<params.spp;++s) {
  float sx=x+1e-12f+(params.resolution-2e-12f)*pt.rng_.uniform01()-params.width*0.5f;
  float sy=params.height*0.5f-(y+1e-12f+(params.resolution-2e-12f)*pt.rng_.uniform01());
  float3 c=pt.sample_sensor(sx,sy,params.camera,params.camera.focal*params.camera.focal,params.bounces);
  if(params.clamp_value>0) c=min(c,float3(params.clamp_value));
  sum+=c; ++n;
  if(params.adapt_min>0) {
   float lum=dot(c,float3(0.2126f,0.7152f,0.0722f)); sum_l+=lum; sum_l2+=lum*lum;
   if(n>=params.adapt_min && n>1 && n%params.adapt_step==0) {
    float mean=sum_l/n, var=max(0.0f,(sum_l2-sum_l*mean)/(n-1));
    float threshold=params.adapt_rel*mean+params.adapt_abs; if(var<threshold*threshold) break;
   }
  }
 }
 float3 c=sum/float(n); c=clamp(floor(c/(c+1)*255),0.0f,255.0f);
 output[id*3]=uchar(c.x); output[id*3+1]=uchar(c.y); output[id*3+2]=uchar(c.z);
}

// Specialized to 8 lanes for adaptive sampling, 32 for uniform high-spp work.
// XOR reductions never cross a pixel's subgroup.
constant uint pixel_lanes [[function_constant(0)]];
kernel void render_paths_simd(device const Triangle* tris [[buffer(0)]], device const BvhNode* nodes [[buffer(1)]], device const int* indices [[buffer(2)]], device const int* lights [[buffer(3)]], device const float* cdf [[buffer(4)]], constant Params& params [[buffer(5)]], constant ulong& seed [[buffer(6)]], device uchar* output [[buffer(7)]], device const MetalVec3* textures [[buffer(9)]], uint id [[thread_position_in_grid]]
#if USE_METAL_RT
 , primitive_acceleration_structure acceleration [[buffer(8)]]
#endif
) {
 uint pixel=id/pixel_lanes, lane=id%pixel_lanes;
 uint x=pixel%params.width,y=pixel/params.width;
 PathTracer pt;
#if USE_METAL_RT
 pt.acceleration=acceleration;
#endif
 pt.textures=textures; pt.tris=tris; pt.nodes=nodes; pt.indices=indices; pt.lights=lights; pt.cdf=cdf; pt.params=params;

 pt.rng_.seed(seed^(ulong(y+1)*0x9E3779B97F4A7C15UL)^(ulong(x+1)*0xBF58476D1CE4E5B9UL)^(ulong(lane)*0x94D049BB133111EBUL));
 float3 sum=0; float sum_l=0,sum_l2=0; int n=0;
 for(uint base=0;base<uint(params.spp);base+=pixel_lanes) {
  float3 c=0;
  if(base+lane<uint(params.spp) && pixel<uint(params.width)*uint(params.height)) {
   float sx=x+1e-12f+(params.resolution-2e-12f)*pt.rng_.uniform01()-params.width*.5f;
   float sy=params.height*.5f-(y+1e-12f+(params.resolution-2e-12f)*pt.rng_.uniform01());
   c=pt.sample_sensor(sx,sy,params.camera,params.camera.focal*params.camera.focal,params.bounces);
   if(params.clamp_value>0) c=min(c,float3(params.clamp_value));
  }
  float lum=dot(c,float3(.2126f,.7152f,.0722f));
  float lum2=lum*lum;
  for(ushort mask=1;mask<pixel_lanes;mask*=2) {
   c+=simd_shuffle_xor(c,mask);
   lum+=simd_shuffle_xor(lum,mask);
   lum2+=simd_shuffle_xor(lum2,mask);
  }
  sum+=c; sum_l+=lum; sum_l2+=lum2; n=int(min(base+pixel_lanes,uint(params.spp)));
  if(params.adapt_min>0 && n>=params.adapt_min && n>1 && n%params.adapt_step==0) {
   float mean=sum_l/n, var=max(0.0f,(sum_l2-sum_l*mean)/(n-1));
   float threshold=params.adapt_rel*mean+params.adapt_abs;
   if(var<threshold*threshold) break;
  }
 }
 if(lane==0 && pixel<uint(params.width)*uint(params.height)) {
  float3 c=sum/float(n); c=clamp(floor(c/(c+1)*255),0.0f,255.0f);
  output[pixel*3]=uchar(c.x); output[pixel*3+1]=uchar(c.y); output[pixel*3+2]=uchar(c.z);
 }
}
