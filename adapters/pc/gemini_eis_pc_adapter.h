#pragma once

#include <vector>
#include "api/gemini_eis_session.hpp"

namespace gemini {

class PcAdapter {
public:
    explicit PcAdapter(const GeminiEisConfig& config) : session_(config) {}
    int setViewport(const GeminiEisViewport& v) { return session_.setViewport(v); }
    int pushGyro(const std::vector<GeminiEisGyroSample>& v) {
        return v.empty() ? GEMINI_EIS_EINVAL : session_.pushGyro(v.data(), static_cast<int>(v.size()));
    }
    int pushVisual(const GeminiEisVisualMotion& v) { return session_.pushVisual(v); }
    int process(const GeminiEisFrameInfo& f, GeminiEisResult* r) { return session_.process(f, r); }
    int processLiveYPlane(const GeminiEisFrameInfo& f,
                          const uint8_t* yPlane, int yWidth, int yHeight, int yRowStride,
                          GeminiEisResult* r) {
        return session_.processLiveYPlane(f, yPlane, yWidth, yHeight, yRowStride, r);
    }
private:
    Session session_;
};

}  // namespace gemini
