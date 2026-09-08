#pragma once
#include "raymotion/runtime.hpp"
#include <cstdio>
#include <unistd.h>

namespace raymotion {
// Dedicated descriptor 3 keeps user stdout separate. One byte on stdin grants
// the next frame. Blocking here keeps loops, timer and model cache alive on pause.
class PreviewStream {
    FILE* stream;
public:
    PreviewStream() : stream(fdopen(dup(3), "wb")) {
        if(!stream) throw std::runtime_error("preview frame channel unavailable");
    }
    ~PreviewStream() { if(stream) std::fclose(stream); }
    PreviewStream(const PreviewStream&) = delete;
    PreviewStream& operator=(const PreviewStream&) = delete;
    FrameSink sink() {
        return [this](const std::vector<uint8_t>& rgb, int w, int h, int frame) {
            if(std::fprintf(stream, "RAY1 %d %d %d %zu\n",w,h,frame,rgb.size()) < 0 ||
               std::fwrite(rgb.data(),1,rgb.size(),stream)!=rgb.size() ||
               std::fflush(stream)!=0) throw std::runtime_error("preview channel closed");
            if(std::getchar()==EOF) throw RenderComplete{};
        };
    }
};
}
