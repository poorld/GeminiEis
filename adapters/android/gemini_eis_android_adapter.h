#pragma once

#include "api/gemini_eis_session.hpp"

namespace gemini {

class AndroidAdapter {
public:
    explicit AndroidAdapter(const GeminiEisConfig& config) : session_(config) {}

    int setViewport(const GeminiEisViewport& v) { return session_.setViewport(v); }
    int pushGyro(const GeminiEisGyroSample* samples, int count) {
        return session_.pushGyro(samples, count);
    }
    int pushVisual(const GeminiEisVisualMotion& v) { return session_.pushVisual(v); }
    int process(const GeminiEisFrameInfo& f, GeminiEisResult* r) { return session_.process(f, r); }
    int processLiveYPlane(const GeminiEisFrameInfo& f,
                          const uint8_t* yPlane, int yWidth, int yHeight, int yRowStride,
                          GeminiEisResult* r) {
        return session_.processLiveYPlane(f, yPlane, yWidth, yHeight, yRowStride, r);
    }
    int processLiveYPlaneToGlesMesh(const GeminiEisFrameInfo& f,
                                    const uint8_t* yPlane, int yWidth, int yHeight, int yRowStride,
                                    const float* stMatrix4x4, int orientationDeg,
                                    GeminiEisResult* outResult,
                                    float* outInterleavedXyuv2640) {
        int rc = session_.processLiveYPlane(f, yPlane, yWidth, yHeight, yRowStride, outResult);
        if (rc != GEMINI_EIS_OK) return rc;
        return GeminiEisExportGlesMesh(outResult, stMatrix4x4, orientationDeg, outInterleavedXyuv2640);
    }
    static int buildGlesIndices(uint16_t* outIndices3648) {
        return GeminiEisBuildGlesIndices(outIndices3648);
    }

private:
    Session session_;
};

}  // namespace gemini
