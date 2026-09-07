#pragma once
#include "raymotion/pathtrace.hpp"
#include <string>
namespace raymotion {
// Returns false with a diagnostic when Metal cannot complete the frame.
bool render_image_metal(const Scene&, const Camera&, const RenderConfig&, uint64_t,
                        std::vector<uint8_t>&, std::string&);
}
