#include "raymotion/metal.hpp"
namespace raymotion {
bool render_image_metal(const Scene&, const Camera&, const RenderConfig&, uint64_t,
                        std::vector<uint8_t>&, std::string& error) {
    error = "Metal backend is not available in this build";
    return false;
}
}
