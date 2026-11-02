#pragma once

#include <stdexcept>
#include "api/gemini_eis_c_api.h"

namespace gemini {

class Session {
public:
    Session() = default;
    explicit Session(const GeminiEisConfig& config) { init(config); }
    ~Session() { resetHandle(); }

    Session(const Session&) = delete;
    Session& operator=(const Session&) = delete;

    Session(Session&& other) noexcept : handle_(other.handle_) {
        other.handle_ = nullptr;
    }

    Session& operator=(Session&& other) noexcept {
        if (this != &other) {
            resetHandle();
            handle_ = other.handle_;
            other.handle_ = nullptr;
        }
        return *this;
    }

    int init(const GeminiEisConfig& config) {
        resetHandle();
        return GeminiEisInit(&handle_, &config);
    }

    bool valid() const { return handle_ != nullptr; }
    int reset() { return GeminiEisReset(&handle_); }
    int setViewport(const GeminiEisViewport& viewport) {
        return GeminiEisSetViewport(&handle_, &viewport);
    }
    int pushGyro(const GeminiEisGyroSample* samples, int count) {
        return GeminiEisPushGyro(&handle_, samples, count);
    }
    int pushVisual(const GeminiEisVisualMotion& visual) {
        return GeminiEisPushVisual(&handle_, &visual);
    }
    int process(const GeminiEisFrameInfo& frame, GeminiEisResult* result) {
        return GeminiEisProcessFrame(&handle_, &frame, result);
    }
    int processLiveYPlane(const GeminiEisFrameInfo& frame,
                          const uint8_t* yPlane, int yWidth, int yHeight, int yRowStride,
                          GeminiEisResult* result) {
        return GeminiEisProcessLiveYPlane(&handle_, &frame, yPlane, yWidth, yHeight, yRowStride, result);
    }
    int processCausalFeatureMatches(const GeminiEisFrameInfo& frame,
                                    int64_t refSensorTsNs, int64_t refExposureNs, int64_t refSkewNs,
                                    int featW, int featH,
                                    const GeminiEisFeaturePointPair* matches, int matchCount,
                                    GeminiEisResult* result) {
        return GeminiEisProcessCausalFeatureMatches(
            &handle_, &frame, refSensorTsNs, refExposureNs, refSkewNs,
            featW, featH, matches, matchCount, result);
    }

    GeminiEisHandle raw() const { return handle_; }

private:
    void resetHandle() {
        if (handle_) GeminiEisUninit(&handle_);
    }
    GeminiEisHandle handle_ = nullptr;
};

}  // namespace gemini
