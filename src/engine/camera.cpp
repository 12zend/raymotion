#include "raymotion/pathtrace.hpp"
namespace raymotion {
void Camera::update_focal() {
    // init proc: focallength = 240 / tan(fov * ".5") (度)
    focal = 240.0 / tan_deg(fov * 0.5);
}

void Camera::update_trig() {
    // trigonometry proc (renderer.gs:808)
    double cx = cos_deg(dirx), sx = sin_deg(dirx);
    double cy = cos_deg(diry), sy = sin_deg(diry);
    double cz = cos_deg(dirz), sz = sin_deg(dirz);
    m0 = cy * cz + sy * (sx * sz);
    m1 = -cy * sz + sy * (sx * cz);
    m2 = sy * cx;
    m3 = cx * sz;
    m4 = cx * cz;
    m5 = -sx;
    m6 = -sy * cz + cy * (sx * sz);
    m7 = sy * sz + cy * (sx * cz);
    m8 = cy * cx;
}

}
