// Metal path tracing, BSDF, MIS and adaptive sampling.
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
inline float lum3(float r, float g, float b) {
 return 0.2126f * r + 0.7152f * g + 0.0722f * b;
}
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
float lum3(float r, float g, float b) {
    return 0.2126f * r + 0.7152f * g + 0.0722f * b;
}
void direct_lighting(float p_x, float p_y, float p_z, float n_x,
                                 float n_y, float n_z, float wo_x, float wo_y,
                                 float wo_z, float rough, float metallic,
                                 float spec_prob, float ar, float ag, float ab,
                                 thread float& or_, thread float& og, thread float& ob,
                                 thread float& out_sumW, thread uint& out_M) {
    or_ = og = ob = 0;
    out_sumW = 0;
    out_M = 0;
    if ((params.lights == 0) || !(params.light_total > 1e-12f)) return;
    int Mc = params.restir_candidates;
    if (Mc < 1) Mc = 1;
    if (Mc > 32) Mc = 32;
    float sp = spec_prob;
    float r2 = rough;
    if (r2 < 0) r2 = 0;
    if (r2 > 1) r2 = 1;
    if (r2 < 0.02f) r2 = 0.02f;
    float alpha2 = (r2 * r2) * (r2 * r2);
    bool nomis = params.nomis;
    if (Mc <= 1) {
        // Legacy single-sample NEE (exact RNG order preserved).
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
        if (!(dist > 0.003f)) { out_M = 1; return; }
        float inv = 1.0f / dist;
        ldx *= inv; ldy *= inv; ldz *= inv;
        float lcos = -(t.nn.x * ldx + (t.nn.y * ldy + t.nn.z * ldz));
        float scos = n_x * ldx + (n_y * ldy + n_z * ldz);
        if (!(lcos > 1e-8f && scos > 1e-8f)) { out_M = 1; return; }
        float lt_lum = lum3((float)t.er, (float)t.eg, (float)t.eb);
        if (lt_lum < 1e-6f) lt_lum = 1e-6f;
        float pdf =
            dist * dist * lt_lum / (lcos * (float)params.light_total);
        float sox = p_x + n_x * 0.002f, soy = p_y + n_y * 0.002f, soz = p_z + n_z * 0.002f;
        if (params.opaque ? cast_shadow_ray({sox, soy, soz}, {ldx, ldy, ldz}, dist - 0.003f)
                          : cast_alpha_ray({sox, soy, soz}, {ldx, ldy, ldz}, dist - 0.003f).tri >= 0) {
            // Occluded: still report sumW/M for MIS consistency (target was valid).
            PathTracer::Bsdf eo = evaluate_opaque_bsdf(ldx, ldy, ldz, wo_x, wo_y, wo_z, n_x, n_y, n_z,
                                           alpha2, metallic, sp, ar, ag, ab);
            float Fr0 = (float)t.er * eo.r * scos * (float)t.alpha;
            float Fg0 = (float)t.eg * eo.g * scos * (float)t.alpha;
            float Fb0 = (float)t.eb * eo.b * scos * (float)t.alpha;
            float ph0 = lum3(Fr0, Fg0, Fb0);
            out_sumW = (ph0 > 0 && pdf > 1e-12f) ? ph0 / pdf : 0;
            out_M = 1;
            return;
        }
        Bsdf e = evaluate_opaque_bsdf(ldx, ldy, ldz, wo_x, wo_y, wo_z, n_x, n_y, n_z,
                                      alpha2, metallic, sp, ar, ag, ab);
        if (!(pdf > 1e-12f)) { out_M = 1; return; }
        float Fr = (float)t.er * e.r * scos * (float)t.alpha;
        float Fg = (float)t.eg * e.g * scos * (float)t.alpha;
        float Fb = (float)t.eb * e.b * scos * (float)t.alpha;
        float ph = lum3(Fr, Fg, Fb);
        out_sumW = (ph > 0) ? ph / pdf : 0;
        out_M = 1;
        float wnee = 1.0f;
        if (!nomis && e.pdf > 1e-20f) wnee = pdf / (pdf + e.pdf);
        float k = scos * wnee * (float)t.alpha / pdf;
        or_ = (float)t.er * e.r * k;
        og = (float)t.eg * e.g * k;
        ob = (float)t.eb * e.b * k;
        return;
    }
    // ReSTIR DI initial sampling (RIS): M candidates, 1 shadow ray.
    // p_hat = lum(Le * f * cos * alpha), q = light solid-angle pdf, w = p_hat/q.
    float sumW = 0;
    float sel_lx = 0, sel_ly = 0, sel_lz = 0;
    float sel_er = 0, sel_eg = 0, sel_eb = 0, sel_alpha = 1;
    float sel_Fr = 0, sel_Fg = 0, sel_Fb = 0;
    float sel_phat = 0, sel_epdf = 0;
    bool have = false;
    for (int j = 0; j < Mc; ++j) {
        float pick = (float)rng_.uniform01() * (float)params.light_total;
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
        float qx = lw * t.v0.x + (lu * t.v1.x + lv * t.v2.x);
        float qy = lw * t.v0.y + (lu * t.v1.y + lv * t.v2.y);
        float qz = lw * t.v0.z + (lu * t.v1.z + lv * t.v2.z);
        float ldx = qx - p_x, ldy = qy - p_y, ldz = qz - p_z;
        float dist = sqrt(ldx * ldx + ldy * ldy + ldz * ldz);
        float w = 0, ph = 0, epdf = 0, Fr = 0, Fg = 0, Fb = 0;
        if (dist > 0.003f) {
            float inv = 1.0f / dist;
            float dxn = ldx * inv, dyn = ldy * inv, dzn = ldz * inv;
            float lcos = -(t.nn.x * dxn + (t.nn.y * dyn + t.nn.z * dzn));
            float scos = n_x * dxn + (n_y * dyn + n_z * dzn);
            if (lcos > 1e-8f && scos > 1e-8f) {
                float lt_lum = lum3((float)t.er, (float)t.eg, (float)t.eb);
                if (lt_lum < 1e-6f) lt_lum = 1e-6f;
                float q = dist * dist * lt_lum / (lcos * (float)params.light_total);
                if (q > 1e-12f) {
                    Bsdf e = evaluate_opaque_bsdf(dxn, dyn, dzn, wo_x, wo_y, wo_z,
                                                  n_x, n_y, n_z, alpha2, metallic,
                                                  sp, ar, ag, ab);
                    epdf = e.pdf;
                    Fr = (float)t.er * e.r * scos * (float)t.alpha;
                    Fg = (float)t.eg * e.g * scos * (float)t.alpha;
                    Fb = (float)t.eb * e.b * scos * (float)t.alpha;
                    ph = lum3(Fr, Fg, Fb);
                    if (ph > 0) w = ph / q;
                }
            }
        }
        sumW += w;
        float r = (float)rng_.uniform01();
        if (w > 0 && r * sumW < w) {
            have = true;
            sel_lx = qx; sel_ly = qy; sel_lz = qz;
            sel_er = (float)t.er; sel_eg = (float)t.eg; sel_eb = (float)t.eb;
            sel_alpha = (float)t.alpha;
            sel_Fr = Fr; sel_Fg = Fg; sel_Fb = Fb;
            sel_phat = ph; sel_epdf = epdf;
        }
    }
    out_sumW = sumW;
    out_M = (uint)Mc;
    if (!have || !(sumW > 0) || !(sel_phat > 0)) return;
    float pdf_ris = float(Mc) * sel_phat / sumW;
    float wnee = 1.0f;
    if (!nomis && sel_epdf > 1e-20f) wnee = pdf_ris / (pdf_ris + sel_epdf);
    float vdx = sel_lx - p_x, vdy = sel_ly - p_y, vdz = sel_lz - p_z;
    float vdist = sqrt(vdx * vdx + vdy * vdy + vdz * vdz);
    if (!(vdist > 0.003f)) return;
    float vinv = 1.0f / vdist;
    vdx *= vinv; vdy *= vinv; vdz *= vinv;
    float sox = p_x + n_x * 0.002f, soy = p_y + n_y * 0.002f, soz = p_z + n_z * 0.002f;
    if (params.opaque ? cast_shadow_ray({sox, soy, soz}, {vdx, vdy, vdz}, vdist - 0.003f)
                      : cast_alpha_ray({sox, soy, soz}, {vdx, vdy, vdz}, vdist - 0.003f).tri >= 0) return;
    float W = sumW / (float(Mc) * sel_phat);
    or_ = sel_Fr * W * wnee;
    og = sel_Fg * W * wnee;
    ob = sel_Fb * W * wnee;
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
    if (params.opaque) return cast_ray(ro, rd, dist);
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
    // ReSTIR MIS carry: previous vertex reservoir (sumW/M) + BSDF for sampled dir.
    float prev_sumW = 0;
    uint prev_M = 0;
    float prev_br = 0, prev_bg = 0, prev_bb = 0, prev_cos = 0;
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
            // ReSTIR-aware MIS: pdf of the previous vertex reservoir for this
            // sampled direction (pdf_ris = M * p_hat / sumW). For M==1 this
            // equals the legacy light pdf, preserving old behavior.
            float pl = 1e30f;
            if (dist > 1e-20f && last_pdf > 1e-20f && prev_M > 0 && prev_sumW > 1e-20f) {
                float ph = lum3((float)hit.er * prev_br * prev_cos * (float)lt.alpha,
                                (float)hit.eg * prev_bg * prev_cos * (float)lt.alpha,
                                (float)hit.eb * prev_bb * prev_cos * (float)lt.alpha);
                if (ph > 1e-20f) pl = float(prev_M) * ph / prev_sumW;
                else {
                    // Fallback to legacy light pdf when target is zero.
                    float inv = 1.0f / dist;
                    float lxn = lx * inv, lyn = ly * inv, lzn = lz * inv;
                    float cos_l = -(lt.nn.x * lxn + (lt.nn.y * lyn + lt.nn.z * lzn));
                    if (cos_l > 1e-8f) {
                        float lum = lum3((float)lt.er, (float)lt.eg, (float)lt.eb);
                        if (lum < 1e-6f) lum = 1e-6f;
                        pl = dist2 * lum / (cos_l * (float)params.light_total);
                    }
                }
            } else if (dist > 1e-20f && last_pdf > 1e-20f) {
                float inv = 1.0f / dist;
                lx *= inv; ly *= inv; lz *= inv;
                float cos_l = -(lt.nn.x * lx + (lt.nn.y * ly + lt.nn.z * lz));
                if (cos_l > 1e-8f) {
                    float lum = lum3((float)lt.er, (float)lt.eg, (float)lt.eb);
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
            prev_sumW = 0; prev_M = 0;
        } else {
            float spec_prob = 0.05f + 0.95f * metallic;
            if (spec_prob > 0.98f) spec_prob = 0.98f;
            if (metallic >= 0.999f) spec_prob = 1.0f;
            float cur_sumW = 0;
            uint cur_M = 0;
            {
                float dlr, dlg, dlb;
                direct_lighting((float)hit.p.x, (float)hit.p.y, (float)hit.p.z,
                                pnx, pny, pnz, wox, woy, woz, rough, metallic,
                                spec_prob, (float)hit.ar, (float)hit.ag,
                                (float)hit.ab, dlr, dlg, dlb, cur_sumW, cur_M);
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
            // Carry reservoir + BSDF for next-vertex ReSTIR MIS.
            prev_sumW = cur_sumW; prev_M = cur_M;
            prev_br = e.r; prev_bg = e.g; prev_bb = e.b; prev_cos = cos_i;
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
// ReSTIR DI: primary G-buffer + initial reservoir (RIS, no shadow yet).
kernel void restir_generate(device const Triangle* tris [[buffer(0)]], device const BvhNode* nodes [[buffer(1)]], device const int* indices [[buffer(2)]], device const int* lights [[buffer(3)]], device const float* cdf [[buffer(4)]], constant Params& params [[buffer(5)]], constant ulong& seed [[buffer(6)]], device uchar* output [[buffer(7)]], device const MetalVec3* textures [[buffer(9)]], device RestirGBuffer* gbuf [[buffer(10)]], device RestirReservoir* rinit [[buffer(11)]], uint id [[thread_position_in_grid]]
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
 gbuf[id].kind=0; rinit[id].valid=0;
 float sx=x+1e-12f+(params.resolution-2e-12f)*pt.rng_.uniform01()-params.width*0.5f;
 float sy=params.height*0.5f-(y+1e-12f+(params.resolution-2e-12f)*pt.rng_.uniform01());
 float f2=params.camera.focal*params.camera.focal;
 float dinv=1.0f/sqrt(sx*sx+sy*sy+f2);
 float tx=sx*dinv, ty=sy*dinv, tz=params.camera.focal*dinv;
 float3 rd{tx*params.camera.m0+(ty*params.camera.m1+tz*params.camera.m2),
           tx*params.camera.m3+(ty*params.camera.m4+tz*params.camera.m5),
           tx*params.camera.m6+(ty*params.camera.m7+tz*params.camera.m8)};
 float3 ro{params.camera.x,params.camera.y,params.camera.z};
 HitInfo hit=pt.cast_alpha_ray(ro,rd,kFarClip);
 if(hit.tri<0) return;
 float pnx=hit.n_interp.x,pny=hit.n_interp.y,pnz=hit.n_interp.z;
 float geom=hit.n_geom.x*pnx+(hit.n_geom.y*pny+hit.n_geom.z*pnz);
 if(geom<0){pnx=-pnx;pny=-pny;pnz=-pnz;}
 float ndot=rd.x*pnx+(rd.y*pny+rd.z*pnz);
 if(ndot>0){pnx=-pnx;pny=-pny;pnz=-pnz;}
 float wox=-rd.x,woy=-rd.y,woz=-rd.z;
 float rough=(float)hit.rough; if(rough<0)rough=0; if(rough>1)rough=1;
 float metallic=(float)tris[(size_t)hit.tri].metallic; if(metallic<0)metallic=0; if(metallic>1)metallic=1;
 if(hit.ior>1.0001f) {
  gbuf[id].pos={hit.p.x,hit.p.y,hit.p.z}; gbuf[id].normal={pnx,pny,pnz};
  gbuf[id].albedo={hit.ar,hit.ag,hit.ab}; gbuf[id].wo={wox,woy,woz};
  gbuf[id].emission={hit.er,hit.eg,hit.eb};
  gbuf[id].rough=rough; gbuf[id].metallic=metallic; gbuf[id].ior=(float)hit.ior;
  gbuf[id].kind=3; return;
 }
 gbuf[id].pos={hit.p.x,hit.p.y,hit.p.z}; gbuf[id].normal={pnx,pny,pnz};
 gbuf[id].albedo={hit.ar,hit.ag,hit.ab}; gbuf[id].wo={wox,woy,woz};
 gbuf[id].emission={hit.er,hit.eg,hit.eb};
 gbuf[id].rough=rough; gbuf[id].metallic=metallic; gbuf[id].ior=1;
 gbuf[id].kind=1;
 if(params.lights==0 || !(params.light_total>1e-12f)) return;
 int Mc=params.restir_candidates; if(Mc<1)Mc=1; if(Mc>32)Mc=32;
 float r2=rough; if(r2<0.02f)r2=0.02f; float alpha2=(r2*r2)*(r2*r2);
 float spec_prob=0.05f+0.95f*metallic; if(spec_prob>0.98f)spec_prob=0.98f;
 if(metallic>=0.999f)spec_prob=1.0f;
 float px=hit.p.x,py=hit.p.y,pz=hit.p.z;
 // Visibility reuse: every initial candidate is shadow-checked once per pixel
 // (amortized over spp). sumWu excludes V (MIS pdf), sumWs includes it (weight W).
 float sumWs=0, sumWu=0, sel_lx=0,sel_ly=0,sel_lz=0, sel_nx=0,sel_ny=0,sel_nz=0;
 float sel_er=0,sel_eg=0,sel_eb=0,sel_alpha=1,sel_ph=0;
 bool have=false;
 float sox=px+pnx*0.002f,soy=py+pny*0.002f,soz=pz+pnz*0.002f;
 for(int j=0;j<Mc;++j){
  float pick=(float)pt.rng_.uniform01()*(float)params.light_total;
  int lo=0,hi=(int)params.lights-1;
  while(lo<hi){int mid=(lo+hi)/2; if(cdf[(size_t)mid]>=pick)hi=mid; else lo=mid+1;}
  int li=lights[(size_t)lo]; const device Triangle& t=tris[(size_t)li];
  float lu=(float)pt.rng_.uniform01(), lv=(float)pt.rng_.uniform01();
  if(lu+lv>1.0f){lu=1.0f-lu;lv=1.0f-lv;}
  float lw=1.0f-lu-lv;
  float qx=lw*t.v0.x+(lu*t.v1.x+lv*t.v2.x);
  float qy=lw*t.v0.y+(lu*t.v1.y+lv*t.v2.y);
  float qz=lw*t.v0.z+(lu*t.v1.z+lv*t.v2.z);
  float ldx=qx-px,ldy=qy-py,ldz=qz-pz;
  float dist=sqrt(ldx*ldx+ldy*ldy+ldz*ldz);
  float wu=0, ws=0, ph=0;
  float Fr=0,Fg=0,Fb=0;
  if(dist>0.003f){
   float inv=1.0f/dist; float dxn=ldx*inv,dyn=ldy*inv,dzn=ldz*inv;
   float lcos=-(t.nn.x*dxn+(t.nn.y*dyn+t.nn.z*dzn));
   float scos=pnx*dxn+(pny*dyn+pnz*dzn);
   if(lcos>1e-8f&&scos>1e-8f){
    float ll=lum3((float)t.er,(float)t.eg,(float)t.eb); if(ll<1e-6f)ll=1e-6f;
    float q=dist*dist*ll/(lcos*(float)params.light_total);
    if(q>1e-12f){
     PathTracer::Bsdf e=pt.evaluate_opaque_bsdf(dxn,dyn,dzn,wox,woy,woz,pnx,pny,pnz,alpha2,metallic,spec_prob,(float)hit.ar,(float)hit.ag,(float)hit.ab);
     Fr=(float)t.er*e.r*scos*(float)t.alpha; Fg=(float)t.eg*e.g*scos*(float)t.alpha; Fb=(float)t.eb*e.b*scos*(float)t.alpha;
     ph=lum3(Fr,Fg,Fb);
     if(ph>0){
      wu=ph/q;
      bool occ=params.opaque ? pt.cast_shadow_ray({sox,soy,soz},{dxn,dyn,dzn},dist-0.003f)
                             : pt.cast_alpha_ray({sox,soy,soz},{dxn,dyn,dzn},dist-0.003f).tri>=0;
      if(!occ) ws=wu;
     }
    }
   }
  }
  sumWu+=wu; sumWs+=ws;
  float r=(float)pt.rng_.uniform01();
  if(ws>0&&r*sumWs<ws){have=true; sel_lx=qx;sel_ly=qy;sel_lz=qz; sel_nx=t.nn.x;sel_ny=t.nn.y;sel_nz=t.nn.z; sel_er=(float)t.er;sel_eg=(float)t.eg;sel_eb=(float)t.eb; sel_alpha=(float)t.alpha; sel_ph=ph;}
 }
 if(!have||!(sumWs>0)||!(sumWu>0)||!(sel_ph>0)) return;
 rinit[id].lightPos={sel_lx,sel_ly,sel_lz}; rinit[id].lightNormal={sel_nx,sel_ny,sel_nz};
 rinit[id].emission={sel_er,sel_eg,sel_eb}; rinit[id].alpha=sel_alpha;
 rinit[id].p_hat=sel_ph; rinit[id].sumWs=sumWs; rinit[id].sumWu=sumWu; rinit[id].M=(uint)Mc; rinit[id].valid=1;
}
// ReSTIR DI spatial reuse (visibility reuse, pairwise MIS, normal rejection).
kernel void restir_spatial(device const Triangle* tris [[buffer(0)]], device const BvhNode* nodes [[buffer(1)]], device const int* indices [[buffer(2)]], device const int* lights [[buffer(3)]], device const float* cdf [[buffer(4)]], constant Params& params [[buffer(5)]], constant ulong& seed [[buffer(6)]], device uchar* output [[buffer(7)]], device const MetalVec3* textures [[buffer(9)]], device const RestirGBuffer* gbuf [[buffer(10)]], device const RestirReservoir* rinit [[buffer(11)]], device RestirReservoir* rout [[buffer(12)]], device const RestirGBuffer* histGbuf [[buffer(13)]], device const RestirReservoir* histRes [[buffer(14)]], uint id [[thread_position_in_grid]]
#if USE_METAL_RT
 , primitive_acceleration_structure acceleration [[buffer(8)]]
#endif
) {
 if(id>=uint(params.width)*uint(params.height)) return;
 uint x=id%params.width,y=id/params.width;
 rout[id]=rinit[id];
 if(gbuf[id].kind!=1||rinit[id].valid==0) return;
 int S=params.restir_spatial; if(S<0)S=0; if(S>8)S=8;
 if(S==0) return;
 float radius=params.restir_radius; if(!(radius>0))radius=16;
 int mcap=params.restir_mcap; if(mcap<1)mcap=1;
 PathTracer pt;
#if USE_METAL_RT
 pt.acceleration=acceleration;
#endif
 pt.textures=textures; pt.tris=tris; pt.nodes=nodes; pt.indices=indices; pt.lights=lights; pt.cdf=cdf; pt.params=params;
 pt.rng_.seed(seed^(ulong(id+1)*0x9E3779B97F4A7C15UL)^0xD1B54A35A6F31B458UL);
 float3 cpos{gbuf[id].pos.x,gbuf[id].pos.y,gbuf[id].pos.z};
 float3 cnor{gbuf[id].normal.x,gbuf[id].normal.y,gbuf[id].normal.z};
 float3 cwo{gbuf[id].wo.x,gbuf[id].wo.y,gbuf[id].wo.z};
 float3 calb{gbuf[id].albedo.x,gbuf[id].albedo.y,gbuf[id].albedo.z};
 float rough=gbuf[id].rough; if(rough<0)rough=0; if(rough>1)rough=1;
 float metallic=gbuf[id].metallic; if(metallic<0)metallic=0; if(metallic>1)metallic=1;
 float r2=rough; if(r2<0.02f)r2=0.02f; float alpha2=(r2*r2)*(r2*r2);
 float spec_prob=0.05f+0.95f*metallic; if(spec_prob>0.98f)spec_prob=0.98f;
 if(metallic>=0.999f)spec_prob=1.0f;
 float cur_sumWs=rinit[id].sumWs, cur_sumWu=rinit[id].sumWu;
 uint cur_M=rinit[id].M; float cur_ph=rinit[id].p_hat;
 float3 cur_lp{rinit[id].lightPos.x,rinit[id].lightPos.y,rinit[id].lightPos.z};
 float3 cur_ln{rinit[id].lightNormal.x,rinit[id].lightNormal.y,rinit[id].lightNormal.z};
 float3 cur_le{rinit[id].emission.x,rinit[id].emission.y,rinit[id].emission.z};
 float cur_alpha=rinit[id].alpha;
 // Temporal reuse: reproject into the previous frame, verify geometry, merge
 // its reservoir (visibility re-checked from here). Static content accumulates
 // candidates up to mcap across frames; moved content is rejected geometrically.
 if(params.hasHistory!=0){
  float pdx=cpos.x-params.prevCamera.x, pdy=cpos.y-params.prevCamera.y, pdz=cpos.z-params.prevCamera.z;
  float ccx=params.prevCamera.m0*pdx+params.prevCamera.m3*pdy+params.prevCamera.m6*pdz;
  float ccy=params.prevCamera.m1*pdx+params.prevCamera.m4*pdy+params.prevCamera.m7*pdz;
  float ccz=params.prevCamera.m2*pdx+params.prevCamera.m5*pdy+params.prevCamera.m8*pdz;
  if(ccz>1e-6f){
   float ppx=params.prevCamera.focal*ccx/ccz+float(params.width)*0.5f;
   float ppy=float(params.height)*0.5f-params.prevCamera.focal*ccy/ccz;
   int hx=int(floor(ppx)), hy=int(floor(ppy));
   if(hx>=0&&hy>=0&&hx<params.width&&hy<params.height){
    uint hidx=(uint)hy*(uint)params.width+(uint)hx;
    if(histGbuf[hidx].kind==1&&histRes[hidx].valid!=0){
     float3 hp{histGbuf[hidx].pos.x,histGbuf[hidx].pos.y,histGbuf[hidx].pos.z};
     float3 hn{histGbuf[hidx].normal.x,histGbuf[hidx].normal.y,histGbuf[hidx].normal.z};
     float ddx=cpos.x-hp.x, ddy=cpos.y-hp.y, ddz=cpos.z-hp.z;
     float dd=sqrt(ddx*ddx+ddy*ddy+ddz*ddz);
     float dcx=cpos.x-params.camera.x, dcy=cpos.y-params.camera.y, dcz=cpos.z-params.camera.z;
     float dc=sqrt(dcx*dcx+dcy*dcy+dcz*dcz);
     float hsumWs=histRes[hidx].sumWs, hsumWu=histRes[hidx].sumWu;
     uint hM=histRes[hidx].M; float hph=histRes[hidx].p_hat;
     if(dd<0.02f*dc+1e-3f&&(cnor.x*hn.x+(cnor.y*hn.y+cnor.z*hn.z))>0.9f&&hsumWu>0&&hM>0&&hph>0){
      float3 hlp{histRes[hidx].lightPos.x,histRes[hidx].lightPos.y,histRes[hidx].lightPos.z};
      float3 hln{histRes[hidx].lightNormal.x,histRes[hidx].lightNormal.y,histRes[hidx].lightNormal.z};
      float3 hle{histRes[hidx].emission.x,histRes[hidx].emission.y,histRes[hidx].emission.z};
      float halpha=histRes[hidx].alpha;
      float ldx=hlp.x-cpos.x,ldy=hlp.y-cpos.y,ldz=hlp.z-cpos.z;
      float dist=sqrt(ldx*ldx+ldy*ldy+ldz*ldz);
      if(dist>0.003f){
       float inv=1.0f/dist; float dxn=ldx*inv,dyn=ldy*inv,dzn=ldz*inv;
       float lcos=-(hln.x*dxn+(hln.y*dyn+hln.z*dzn));
       float scos=cnor.x*dxn+(cnor.y*dyn+cnor.z*dzn);
       if(lcos>1e-8f&&scos>1e-8f){
        PathTracer::Bsdf e=pt.evaluate_opaque_bsdf(dxn,dyn,dzn,cwo.x,cwo.y,cwo.z,cnor.x,cnor.y,cnor.z,alpha2,metallic,spec_prob,calb.x,calb.y,calb.z);
        float Fr=hle.x*e.r*scos*halpha, Fg=hle.y*e.g*scos*halpha, Fb=hle.z*e.b*scos*halpha;
        float ph=lum3(Fr,Fg,Fb);
        if(ph>0){
         float wu=ph*hsumWu/hph;
         float sox=cpos.x+cnor.x*0.002f,soy=cpos.y+cnor.y*0.002f,soz=cpos.z+cnor.z*0.002f;
         bool occ=params.opaque ? pt.cast_shadow_ray({sox,soy,soz},{dxn,dyn,dzn},dist-0.003f)
                                : pt.cast_alpha_ray({sox,soy,soz},{dxn,dyn,dzn},dist-0.003f).tri>=0;
         float ws=occ?0.0f:ph*hsumWs/hph;
         float sum_s=cur_sumWs+ws, sum_u=cur_sumWu+wu;
         uint newM=cur_M+hM; if(newM>(uint)mcap)newM=(uint)mcap;
         float r=(float)pt.rng_.uniform01();
         if(sum_s>0&&r*sum_s<ws){cur_lp=hlp;cur_ln=hln;cur_le=hle;cur_alpha=halpha;cur_ph=ph;}
         cur_sumWs=sum_s; cur_sumWu=sum_u; cur_M=newM;
        }
       }
      }
     }
    }
   }
  }
 }
 for(int s=0;s<S;++s){
  float rx=(float)pt.rng_.uniform01()*2.0f-1.0f, ry=(float)pt.rng_.uniform01()*2.0f-1.0f;
  int nx=int(float(x)+rx*radius+0.5f), ny=int(float(y)+ry*radius+0.5f);
  if(nx<0||ny<0||nx>=params.width||ny>=params.height) continue;
  if(nx==(int)x&&ny==(int)y) continue;
  uint nidx=(uint)ny*(uint)params.width+(uint)nx;
  if(gbuf[nidx].kind!=1||rinit[nidx].valid==0) continue;
  float3 nnor{gbuf[nidx].normal.x,gbuf[nidx].normal.y,gbuf[nidx].normal.z};
  if(cnor.x*nnor.x+(cnor.y*nnor.y+cnor.z*nnor.z)<0.9f) continue;
  float nbr_sumWs=rinit[nidx].sumWs, nbr_sumWu=rinit[nidx].sumWu;
  uint nbr_M=rinit[nidx].M; float nbr_ph=rinit[nidx].p_hat;
  if(!(nbr_sumWu>0)||nbr_M==0||!(nbr_ph>0)) continue;
  float3 nlp{rinit[nidx].lightPos.x,rinit[nidx].lightPos.y,rinit[nidx].lightPos.z};
  float3 nln{rinit[nidx].lightNormal.x,rinit[nidx].lightNormal.y,rinit[nidx].lightNormal.z};
  float3 nle{rinit[nidx].emission.x,rinit[nidx].emission.y,rinit[nidx].emission.z};
  float nalpha=rinit[nidx].alpha;
  float ldx=nlp.x-cpos.x,ldy=nlp.y-cpos.y,ldz=nlp.z-cpos.z;
  float dist=sqrt(ldx*ldx+ldy*ldy+ldz*ldz);
  if(!(dist>0.003f)) continue;
  float inv=1.0f/dist; float dxn=ldx*inv,dyn=ldy*inv,dzn=ldz*inv;
  float lcos=-(nln.x*dxn+(nln.y*dyn+nln.z*dzn));
  float scos=cnor.x*dxn+(cnor.y*dyn+cnor.z*dzn);
  if(!(lcos>1e-8f&&scos>1e-8f)) continue;
  PathTracer::Bsdf e=pt.evaluate_opaque_bsdf(dxn,dyn,dzn,cwo.x,cwo.y,cwo.z,cnor.x,cnor.y,cnor.z,alpha2,metallic,spec_prob,calb.x,calb.y,calb.z);
  float Fr=nle.x*e.r*scos*nalpha, Fg=nle.y*e.g*scos*nalpha, Fb=nle.z*e.b*scos*nalpha;
  float ph_cur=lum3(Fr,Fg,Fb);
  if(!(ph_cur>0)) continue;
  float w_nbr_u=ph_cur*nbr_sumWu/nbr_ph;
  if(!(w_nbr_u>0)) continue;
  float sox=cpos.x+cnor.x*0.002f,soy=cpos.y+cnor.y*0.002f,soz=cpos.z+cnor.z*0.002f;
  bool occ=params.opaque ? pt.cast_shadow_ray({sox,soy,soz},{dxn,dyn,dzn},dist-0.003f)
                         : pt.cast_alpha_ray({sox,soy,soz},{dxn,dyn,dzn},dist-0.003f).tri>=0;
  float w_nbr_s=occ ? 0.0f : ph_cur*nbr_sumWs/nbr_ph;
  float w_self_s=cur_sumWs, w_self_u=cur_sumWu;
  float sum_s=w_self_s+w_nbr_s, sum_u=w_self_u+w_nbr_u;
  if(!(sum_u>0)||!(sum_s>=0)) continue;
  float r=(float)pt.rng_.uniform01();
  uint newM=cur_M+nbr_M; if(newM>(uint)mcap)newM=(uint)mcap;
  if(sum_s>0&&r*sum_s<w_nbr_s){cur_lp=nlp;cur_ln=nln;cur_le=nle;cur_alpha=nalpha;cur_ph=ph_cur;}
  cur_sumWs=sum_s; cur_sumWu=sum_u; cur_M=newM;
 }
 rout[id].lightPos={cur_lp.x,cur_lp.y,cur_lp.z}; rout[id].lightNormal={cur_ln.x,cur_ln.y,cur_ln.z};
 rout[id].emission={cur_le.x,cur_le.y,cur_le.z}; rout[id].alpha=cur_alpha;
 rout[id].p_hat=cur_ph; rout[id].sumWs=cur_sumWs; rout[id].sumWu=cur_sumWu; rout[id].M=cur_M; rout[id].valid=1;
}
// ReSTIR DI final shade: primary direct from spatial reservoir + spp-averaged indirect.
kernel void restir_shade(device const Triangle* tris [[buffer(0)]], device const BvhNode* nodes [[buffer(1)]], device const int* indices [[buffer(2)]], device const int* lights [[buffer(3)]], device const float* cdf [[buffer(4)]], constant Params& params [[buffer(5)]], constant ulong& seed [[buffer(6)]], device const MetalVec3* textures [[buffer(9)]], device const RestirGBuffer* gbuf [[buffer(10)]], device const RestirReservoir* rmerged [[buffer(12)]], device float* hdrOut [[buffer(15)]], uint id [[thread_position_in_grid]]
#if USE_METAL_RT
 , primitive_acceleration_structure acceleration [[buffer(8)]]
#endif
) {
 if(id>=uint(params.width)*uint(params.height)) return;
 uint x=id%params.width,y=id/params.width;
 if(gbuf[id].kind==0) { hdrOut[id*3]=0; hdrOut[id*3+1]=0; hdrOut[id*3+2]=0; return; }
 PathTracer pt;
#if USE_METAL_RT
 pt.acceleration=acceleration;
#endif
 pt.textures=textures; pt.tris=tris; pt.nodes=nodes; pt.indices=indices; pt.lights=lights; pt.cdf=cdf; pt.params=params;
 pt.rng_.seed(seed^(ulong(id+1)*0xBF58476D1CE4E5B9UL)^0x94D049BB133111EBUL);
 if(gbuf[id].kind==3) {
  // Dielectric primary: independent standard path tracing (unbiased, keeps AA).
  float3 sum=0; int n=0;
  for(int s=0;s<params.spp;++s){
   float sx=x+1e-12f+(params.resolution-2e-12f)*pt.rng_.uniform01()-params.width*0.5f;
   float sy=params.height*0.5f-(y+1e-12f+(params.resolution-2e-12f)*pt.rng_.uniform01());
   float3 c=pt.sample_sensor(sx,sy,params.camera,params.camera.focal*params.camera.focal,params.bounces);
   if(params.clamp_value>0) c=min(c,float3(params.clamp_value));
   sum+=c; ++n;
  }
   float3 c=sum/float(max(n,1));
   if(params.clamp_value>0) c=min(c,float3(params.clamp_value));
   hdrOut[id*3]=c.x; hdrOut[id*3+1]=c.y; hdrOut[id*3+2]=c.z;
   return;
  }
  // Opaque primary (kind==1).
 float3 gpos{gbuf[id].pos.x,gbuf[id].pos.y,gbuf[id].pos.z};
 float3 gnor{gbuf[id].normal.x,gbuf[id].normal.y,gbuf[id].normal.z};
 float3 galb{gbuf[id].albedo.x,gbuf[id].albedo.y,gbuf[id].albedo.z};
 float3 gwo{gbuf[id].wo.x,gbuf[id].wo.y,gbuf[id].wo.z};
 float3 gemi{gbuf[id].emission.x,gbuf[id].emission.y,gbuf[id].emission.z};
 float rough=gbuf[id].rough; if(rough<0)rough=0; if(rough>1)rough=1;
 float metallic=gbuf[id].metallic; if(metallic<0)metallic=0; if(metallic>1)metallic=1;
 float r2=rough; if(r2<0.02f)r2=0.02f; float alpha2=(r2*r2)*(r2*r2);
 float spec_prob=0.05f+0.95f*metallic; if(spec_prob>0.98f)spec_prob=0.98f;
 if(metallic>=0.999f)spec_prob=1.0f;
 bool nomis=params.nomis;
  // Primary direct from merged reservoir. Visibility was already evaluated for
  // every merged candidate (visibility reuse), so no final shadow ray is needed:
  // the selected sample is known-visible and W carries the averaged visibility.
  float3 prim_direct=0;
  float m_sumWu=0; uint m_M=0;
  if(rmerged[id].valid!=0 && rmerged[id].M>0 && rmerged[id].sumWu>0 && rmerged[id].sumWs>0 && rmerged[id].p_hat>0) {
   float3 lp{rmerged[id].lightPos.x,rmerged[id].lightPos.y,rmerged[id].lightPos.z};
   float3 ln{rmerged[id].lightNormal.x,rmerged[id].lightNormal.y,rmerged[id].lightNormal.z};
   float3 le{rmerged[id].emission.x,rmerged[id].emission.y,rmerged[id].emission.z};
   float lalpha=rmerged[id].alpha;
   m_sumWu=rmerged[id].sumWu; m_M=rmerged[id].M;
   float m_sumWs=rmerged[id].sumWs;
   float ldx=lp.x-gpos.x,ldy=lp.y-gpos.y,ldz=lp.z-gpos.z;
   float dist=sqrt(ldx*ldx+ldy*ldy+ldz*ldz);
   if(dist>0.003f){
    float inv=1.0f/dist; float dxn=ldx*inv,dyn=ldy*inv,dzn=ldz*inv;
    float lcos=-(ln.x*dxn+(ln.y*dyn+ln.z*dzn));
    float scos=gnor.x*dxn+(gnor.y*dyn+gnor.z*dzn);
    if(lcos>1e-8f&&scos>1e-8f){
     PathTracer::Bsdf e=pt.evaluate_opaque_bsdf(dxn,dyn,dzn,gwo.x,gwo.y,gwo.z,gnor.x,gnor.y,gnor.z,alpha2,metallic,spec_prob,galb.x,galb.y,galb.z);
     float Fr=le.x*e.r*scos*lalpha, Fg=le.y*e.g*scos*lalpha, Fb=le.z*e.b*scos*lalpha;
     float ph=lum3(Fr,Fg,Fb);
     if(ph>0){
      float pdf_ris=float(m_M)*ph/m_sumWu;
      float wnee=1.0f;
      if(!nomis&&e.pdf>1e-20f) wnee=pdf_ris/(pdf_ris+e.pdf);
      float W=m_sumWs/(float(m_M)*ph);
      prim_direct={Fr*W*wnee,Fg*W*wnee,Fb*W*wnee};
     }
    }
   }
  }
  if(params.bounces<=1 || params.spp<=0) {
   float3 c=gemi+prim_direct;
   if(params.clamp_value>0) c=min(c,float3(params.clamp_value));
   hdrOut[id*3]=c.x; hdrOut[id*3+1]=c.y; hdrOut[id*3+2]=c.z;
   return;
  }
 // spp-averaged indirect (each sample: BSDF at primary + full path from bounce 1).
 float3 sum=0; int n=0;
 for(int s=0;s<params.spp;++s){
  // Sample BSDF at primary.
  float nx2,ny2,nz2; int pdelta=0;
  if((float)pt.rng_.uniform01()<spec_prob){
   float hx,hy,hz; pt.sample_ggx_half(alpha2,gnor.x,gnor.y,gnor.z,hx,hy,hz);
   float md=gwo.x*hx+(gwo.y*hy+gwo.z*hz);
   if(!(md>1e-8f)) {
    float3 full=gemi+prim_direct;
    if(params.clamp_value>0) full=min(full,float3(params.clamp_value));
    sum+=full; ++n; continue;
   }
   nx2=2.0f*md*hx-gwo.x; ny2=2.0f*md*hy-gwo.y; nz2=2.0f*md*hz-gwo.z;
   pdelta=(rough<=0.0201f)?1:0;
  } else {
   pt.sample_cosine_hemisphere(gnor.x,gnor.y,gnor.z,nx2,ny2,nz2);
   pdelta=0;
  }
  float ncos=nx2*gnor.x+(ny2*gnor.y+nz2*gnor.z);
  if(!(ncos>1e-8f)) {
   float3 full=gemi+prim_direct;
   if(params.clamp_value>0) full=min(full,float3(params.clamp_value));
   sum+=full; ++n; continue;
  }
  {float d=1.0f/sqrt(nx2*nx2+ny2*ny2+nz2*nz2); nx2*=d;ny2*=d;nz2*=d;}
  PathTracer::Bsdf e0=pt.evaluate_opaque_bsdf(nx2,ny2,nz2,gwo.x,gwo.y,gwo.z,gnor.x,gnor.y,gnor.z,alpha2,metallic,spec_prob,galb.x,galb.y,galb.z);
  if(!(e0.pdf>1e-12f)) {
   float3 full=gemi+prim_direct;
   if(params.clamp_value>0) full=min(full,float3(params.clamp_value));
   sum+=full; ++n; continue;
  }
  float cos0=nx2*gnor.x+(ny2*gnor.y+nz2*gnor.z);
  float3 thr{e0.r*cos0/e0.pdf, e0.g*cos0/e0.pdf, e0.b*cos0/e0.pdf};
  float ox=gpos.x+gnor.x*0.001f, oy=gpos.y+gnor.y*0.001f, oz=gpos.z+gnor.z*0.001f;
  float dx=nx2,dy=ny2,dz=nz2;
  float last_pdf=e0.pdf;
  float pb_r=e0.r,pb_g=e0.g,pb_b=e0.b,pb_cos=cos0;
  float psumW=m_sumWu; uint pM=m_M; int pdel=pdelta;
  float3 acc=0;
  for(int b=1;b<params.bounces;++b){
   HitInfo hit=pt.cast_alpha_ray({ox,oy,oz},{dx,dy,dz},kFarClip);
   if(hit.tri<0) break;
   float pnx=hit.n_interp.x,pny=hit.n_interp.y,pnz=hit.n_interp.z;
   float gg=hit.n_geom.x*pnx+(hit.n_geom.y*pny+hit.n_geom.z*pnz);
   if(gg<0){pnx=-pnx;pny=-pny;pnz=-pnz;}
   float nd=dx*pnx+(dy*pny+dz*pnz); bool front=true;
   if(nd>0){pnx=-pnx;pny=-pny;pnz=-pnz; front=false;}
   float wox=-dx,woy=-dy,woz=-dz;
   float rh=(float)hit.rough; if(rh<0)rh=0; if(rh>1)rh=1; if(rh<0.02f)rh=0.02f;
   float al=rh*rh; float a2=al*al;
   float met=(float)tris[(size_t)hit.tri].metallic; if(met<0)met=0; if(met>1)met=1;
   // Emission with ReSTIR MIS (pdel holds previous-vertex delta).
   if(pdel==1){
    acc.x+=(float)hit.er*thr.x; acc.y+=(float)hit.eg*thr.y; acc.z+=(float)hit.eb*thr.z;
   } else {
    const device Triangle& lt=tris[(size_t)hit.tri];
    float lx=(float)hit.p.x-ox,ly=(float)hit.p.y-oy,lz=(float)hit.p.z-oz;
    float d2=lx*lx+ly*ly+lz*lz; float dd=sqrt(d2);
    float pl=1e30f;
    if(dd>1e-20f&&last_pdf>1e-20f&&pM>0&&psumW>1e-20f){
     float ph=lum3((float)hit.er*pb_r*pb_cos*(float)lt.alpha,(float)hit.eg*pb_g*pb_cos*(float)lt.alpha,(float)hit.eb*pb_b*pb_cos*(float)lt.alpha);
     if(ph>1e-20f) pl=float(pM)*ph/psumW;
     else {
      float inv=1.0f/dd; float xn=lx*inv,yn=ly*inv,zn=lz*inv;
      float cl=-(lt.nn.x*xn+(lt.nn.y*yn+lt.nn.z*zn));
      if(cl>1e-8f){float lum=lum3((float)lt.er,(float)lt.eg,(float)lt.eb); if(lum<1e-6f)lum=1e-6f; pl=d2*lum/(cl*(float)params.light_total);}
     }
    } else if(dd>1e-20f&&last_pdf>1e-20f){
     float inv=1.0f/dd; lx*=inv;ly*=inv;lz*=inv;
     float cl=-(lt.nn.x*lx+(lt.nn.y*ly+lt.nn.z*lz));
     if(cl>1e-8f){float lum=lum3((float)lt.er,(float)lt.eg,(float)lt.eb); if(lum<1e-6f)lum=1e-6f; pl=d2*lum/(cl*(float)params.light_total);}
    }
    float w=nomis?0.0f:((last_pdf+pl>1e-20f)?last_pdf/(last_pdf+pl):0.0f);
    acc.x+=(float)hit.er*thr.x*w; acc.y+=(float)hit.eg*thr.y*w; acc.z+=(float)hit.eb*thr.z*w;
   }
   if(hit.ior>1.0001f){
    PathTracer::DielectricSample ds=pt.sample_dielectric(wox,woy,woz,pnx,pny,pnz,dx,dy,dz,(float)hit.ior,front,rh,a2);
    if(!ds.valid) break;
    thr.x*=ds.eta_scale; thr.y*=ds.eta_scale; thr.z*=ds.eta_scale;
    float sgn=1.0f; if(ds.n_x*pnx+(ds.n_y*pny+ds.n_z*pnz)<0) sgn=-1.0f;
    ox=(float)hit.p.x+pnx*0.001f*sgn; oy=(float)hit.p.y+pny*0.001f*sgn; oz=(float)hit.p.z+pnz*0.001f*sgn;
    dx=ds.n_x; dy=ds.n_y; dz=ds.n_z; pdel=1;
    psumW=0; pM=0;
   } else {
    float sp2=0.05f+0.95f*met; if(sp2>0.98f)sp2=0.98f; if(met>=0.999f)sp2=1.0f;
    float dlr=0,dlg=0,dlb=0, csum=0; uint cM=0;
    pt.direct_lighting((float)hit.p.x,(float)hit.p.y,(float)hit.p.z,pnx,pny,pnz,wox,woy,woz,rh,met,sp2,(float)hit.ar,(float)hit.ag,(float)hit.ab,dlr,dlg,dlb,csum,cM);
    acc.x+=dlr*thr.x; acc.y+=dlg*thr.y; acc.z+=dlb*thr.z;
    float qx2,qy2,qz2;
    if((float)pt.rng_.uniform01()<sp2){
     float hx,hy,hz; pt.sample_ggx_half(a2,pnx,pny,pnz,hx,hy,hz);
     float md=wox*hx+(woy*hy+woz*hz);
     if(!(md>1e-8f)) break;
     qx2=2.0f*md*hx-wox; qy2=2.0f*md*hy-woy; qz2=2.0f*md*hz-woz;
     pdel=(rh<=0.0201f)?1:0;
    } else { pt.sample_cosine_hemisphere(pnx,pny,pnz,qx2,qy2,qz2); pdel=0; }
    float nc=qx2*pnx+(qy2*pny+qz2*pnz);
    if(!(nc>1e-8f)) break;
    {float d=1.0f/sqrt(qx2*qx2+qy2*qy2+qz2*qz2); qx2*=d;qy2*=d;qz2*=d;}
    PathTracer::Bsdf ee=pt.evaluate_opaque_bsdf(qx2,qy2,qz2,wox,woy,woz,pnx,pny,pnz,a2,met,sp2,(float)hit.ar,(float)hit.ag,(float)hit.ab);
    if(!(ee.pdf>1e-12f)) break;
    last_pdf=ee.pdf;
    float ci=qx2*pnx+(qy2*pny+qz2*pnz);
    thr.x*=ee.r*ci/ee.pdf; thr.y*=ee.g*ci/ee.pdf; thr.z*=ee.b*ci/ee.pdf;
    psumW=csum; pM=cM; pb_r=ee.r; pb_g=ee.g; pb_b=ee.b; pb_cos=ci;
    ox=(float)hit.p.x+pnx*0.001f; oy=(float)hit.p.y+pny*0.001f; oz=(float)hit.p.z+pnz*0.001f;
    dx=qx2; dy=qy2; dz=qz2;
   }
   float mx=max(thr.x,thr.y); mx=max(mx,thr.z);
   if(!(mx>=1e-12f)) break;
   if(b>=2){
    float rr=mx; if(rr<0.05f)rr=0.05f; if(rr>0.9f)rr=0.9f;
    if(!((float)pt.rng_.uniform01()<=rr)) break;
    float ir=1.0f/rr; thr.x*=ir; thr.y*=ir; thr.z*=ir;
   }
  }
  float3 full=gemi+prim_direct+acc;
  if(params.clamp_value>0) full=min(full,float3(params.clamp_value));
  sum+=full; ++n;
 }
  float3 c=sum/float(max(n,1));
  hdrOut[id*3]=c.x; hdrOut[id*3+1]=c.y; hdrOut[id*3+2]=c.z;
}
// Edge-guided spatiotemporal denoiser (linear HDR in, linear HDR out).
// Temporal: reprojected history with TAA clamp + geometric disocclusion reject.
// Spatial: 2x 5x5 a-trous with normal/depth/luma/albedo edge weights.
inline float denoise_luma(float3 c) { return 0.2126f*c.x+0.7152f*c.y+0.0722f*c.z; }
kernel void denoise_temporal(device const RestirGBuffer* gbuf [[buffer(10)]],
                             device const RestirGBuffer* histGbuf [[buffer(13)]],
                             device const float* hdrIn [[buffer(15)]],
                             device const float* histCol [[buffer(16)]],
                             device float* denOut [[buffer(17)]],
                             constant Params& params [[buffer(5)]],
                             uint id [[thread_position_in_grid]]) {
 if(id>=uint(params.width)*uint(params.height)) return;
 uint x=id%params.width, y=id/params.width;
 float3 cur{hdrIn[id*3],hdrIn[id*3+1],hdrIn[id*3+2]};
 // Specular (dielectric) pixels bypass temporal blending: their shaded color
 // depends on reflected/refracted content, not the surface point, so reprojected
 // history would smear as if glued to the object. Spatial filtering still applies.
 if(params.hasHistory==0||gbuf[id].kind!=1){ denOut[id*3]=cur.x; denOut[id*3+1]=cur.y; denOut[id*3+2]=cur.z; return; }
 float3 mn=cur, mx=cur;
 for(int dy=-1;dy<=1;++dy) for(int dx=-1;dx<=1;++dx){
  int nx=int(x)+dx, ny=int(y)+dy;
  if(nx<0||ny<0||nx>=params.width||ny>=params.height) continue;
  uint nidx=(uint)ny*(uint)params.width+(uint)nx;
  float3 v{hdrIn[nidx*3],hdrIn[nidx*3+1],hdrIn[nidx*3+2]};
  mn=min(mn,v); mx=max(mx,v);
 }
 float3 cpos{gbuf[id].pos.x,gbuf[id].pos.y,gbuf[id].pos.z};
 float3 cnor{gbuf[id].normal.x,gbuf[id].normal.y,gbuf[id].normal.z};
 float pdx=cpos.x-params.prevCamera.x, pdy=cpos.y-params.prevCamera.y, pdz=cpos.z-params.prevCamera.z;
 float ccx=params.prevCamera.m0*pdx+params.prevCamera.m3*pdy+params.prevCamera.m6*pdz;
 float ccy=params.prevCamera.m1*pdx+params.prevCamera.m4*pdy+params.prevCamera.m7*pdz;
 float ccz=params.prevCamera.m2*pdx+params.prevCamera.m5*pdy+params.prevCamera.m8*pdz;
 float3 outc=cur;
 if(ccz>1e-6f){
  float ppx=params.prevCamera.focal*ccx/ccz+float(params.width)*0.5f;
  float ppy=float(params.height)*0.5f-params.prevCamera.focal*ccy/ccz;
  int hx=int(floor(ppx)), hy=int(floor(ppy));
  if(hx>=0&&hy>=0&&hx<params.width&&hy<params.height){
   uint hidx=(uint)hy*(uint)params.width+(uint)hx;
   if(histGbuf[hidx].kind==1){
    float3 hp{histGbuf[hidx].pos.x,histGbuf[hidx].pos.y,histGbuf[hidx].pos.z};
    float3 hn{histGbuf[hidx].normal.x,histGbuf[hidx].normal.y,histGbuf[hidx].normal.z};
    float ddx=cpos.x-hp.x, ddy=cpos.y-hp.y, ddz=cpos.z-hp.z;
    float dd=sqrt(ddx*ddx+ddy*ddy+ddz*ddz);
    float dcx=cpos.x-params.camera.x, dcy=cpos.y-params.camera.y, dcz=cpos.z-params.camera.z;
    float dc=sqrt(dcx*dcx+dcy*dcy+dcz*dcz);
    if(dd<0.02f*dc+1e-3f&&(cnor.x*hn.x+(cnor.y*hn.y+cnor.z*hn.z))>0.9f){
     float3 h{histCol[hidx*3],histCol[hidx*3+1],histCol[hidx*3+2]};
     h=clamp(h,mn,mx);
     outc=mix(h,cur,0.2f);
    }
   }
  }
 }
 denOut[id*3]=outc.x; denOut[id*3+1]=outc.y; denOut[id*3+2]=outc.z;
}
kernel void denoise_atrous(device const RestirGBuffer* gbuf [[buffer(10)]],
                           device const float* colorIn [[buffer(18)]],
                           device float* colorOut [[buffer(19)]],
                           constant Params& params [[buffer(5)]],
                           constant int& step [[buffer(20)]],
                           uint id [[thread_position_in_grid]]) {
 if(id>=uint(params.width)*uint(params.height)) return;
 uint x=id%params.width, y=id/params.width;
 if(gbuf[id].kind==0){ colorOut[id*3]=0; colorOut[id*3+1]=0; colorOut[id*3+2]=0; return; }
 float3 c0{colorIn[id*3],colorIn[id*3+1],colorIn[id*3+2]};
 float3 n0{gbuf[id].normal.x,gbuf[id].normal.y,gbuf[id].normal.z};
 float3 p0{gbuf[id].pos.x,gbuf[id].pos.y,gbuf[id].pos.z};
 float3 a0{gbuf[id].albedo.x,gbuf[id].albedo.y,gbuf[id].albedo.z};
 float l0=denoise_luma(c0);
 float dcx=p0.x-params.camera.x, dcy=p0.y-params.camera.y, dcz=p0.z-params.camera.z;
 float dc=sqrt(dcx*dcx+dcy*dcy+dcz*dcz);
 int stride=1<<step;
 float3 sum=0; float wsum=0;
 for(int dy=-2;dy<=2;++dy) for(int dx=-2;dx<=2;++dx){
  int nx=int(x)+dx*stride, ny=int(y)+dy*stride;
  if(nx<0||ny<0||nx>=params.width||ny>=params.height) continue;
  uint nidx=(uint)ny*(uint)params.width+(uint)nx;
  if(gbuf[nidx].kind==0) continue;
  float3 c{colorIn[nidx*3],colorIn[nidx*3+1],colorIn[nidx*3+2]};
  float3 n{gbuf[nidx].normal.x,gbuf[nidx].normal.y,gbuf[nidx].normal.z};
  float3 pp{gbuf[nidx].pos.x,gbuf[nidx].pos.y,gbuf[nidx].pos.z};
  float3 aa{gbuf[nidx].albedo.x,gbuf[nidx].albedo.y,gbuf[nidx].albedo.z};
  float nd=n.x*n0.x+(n.y*n0.y+n.z*n0.z);
  float wn=pow(max(nd,0.0f),16.0f);
  float ex=pp.x-p0.x, ey=pp.y-p0.y, ez=pp.z-p0.z;
  float wz=exp(-sqrt(ex*ex+ey*ey+ez*ez)/(0.1f*dc+1e-4f));
  float wl=exp(-fabs(denoise_luma(c)-l0)/(0.3f*max(l0,0.5f)+1e-3f));
  float ax=aa.x-a0.x, ay=aa.y-a0.y, az=aa.z-a0.z;
  float wa=exp(-sqrt(ax*ax+ay*ay+az*az)/0.25f);
  float w=wn*wz*wl*wa;
  sum+=w*c; wsum+=w;
 }
 float3 outc=wsum>1e-12f?sum/wsum:c0;
 colorOut[id*3]=outc.x; colorOut[id*3+1]=outc.y; colorOut[id*3+2]=outc.z;
}
kernel void denoise_tonemap(device const float* colorIn [[buffer(18)]],
                            device uchar* output [[buffer(7)]],
                            constant Params& params [[buffer(5)]],
                            uint id [[thread_position_in_grid]]) {
 if(id>=uint(params.width)*uint(params.height)) return;
 float3 c{colorIn[id*3],colorIn[id*3+1],colorIn[id*3+2]};
 c=clamp(floor(c/(c+1)*255),0.0f,255.0f);
 output[id*3]=uchar(c.x); output[id*3+1]=uchar(c.y); output[id*3+2]=uchar(c.z);
}
