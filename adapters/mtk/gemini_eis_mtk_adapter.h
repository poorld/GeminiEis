#pragma once

#include <vector>
#include "api/gemini_eis_session.hpp"

namespace gemini {

struct MtkSensorProviderGyro {
    int64_t bootTimestampNs = 0;
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
};

struct MtkFrameTiming {
    int64_t p1FrameStartBootNs = 0;
    int64_t exposureNs = 0;
    int64_t rollingShutterSkewNs = 0;
    double zoom = 0.0;
};

struct MtkWpeWarpMapOut {
    uint32_t sensorWidth = 2560;
    uint32_t sensorHeight = 1440;
    uint32_t strideElements = 33;
    int fixedPointShift = 8; // 8 for MTK WPE s24.8, 5 for s24.5
    int32_t* wpeMapXFixed = nullptr;
    int32_t* wpeMapYFixed = nullptr;
    float* wpeMapXFloat = nullptr;
    float* wpeMapYFloat = nullptr;
};

class MtkAdapter {
public:
    explicit MtkAdapter(const GeminiEisConfig& config) : config_(config), session_(config) {}

    int setRuntimeViewport(uint32_t bufferW, uint32_t bufferH,
                           uint32_t outputW, uint32_t outputH,
                           double roiX, double roiY, double roiW, double roiH) {
        GeminiEisViewport v{};
        v.buffer_width = bufferW;
        v.buffer_height = bufferH;
        v.output_width = outputW;
        v.output_height = outputH;
        v.roi_x = roiX;
        v.roi_y = roiY;
        v.roi_width = roiW;
        v.roi_height = roiH;
        return session_.setViewport(v);
    }

    int feedSensorProviderGyro(const std::vector<MtkSensorProviderGyro>& input) {
        if (input.empty()) return GEMINI_EIS_EINVAL;
        std::vector<GeminiEisGyroSample> samples;
        samples.reserve(input.size());
        for (const auto& s : input) {
            samples.push_back({s.bootTimestampNs, s.x, s.y, s.z});
        }
        return session_.pushGyro(samples.data(), static_cast<int>(samples.size()));
    }

    int pushVisual(const GeminiEisVisualMotion& visual) {
        return session_.pushVisual(visual);
    }

    int process(const MtkFrameTiming& timing, GeminiEisResult* result) {
        GeminiEisFrameInfo f{};
        f.sensor_timestamp_ns = timing.p1FrameStartBootNs;
        f.exposure_time_ns = timing.exposureNs;
        f.rolling_shutter_skew_ns = timing.rollingShutterSkewNs;
        f.zoom = timing.zoom;
        return session_.process(f, result);
    }

    int processLiveYPlaneToWpe(const MtkFrameTiming& timing,
                               const uint8_t* rrzoYPlane, int rrzoW, int rrzoH, int rrzoStride,
                               GeminiEisResult* result,
                               const MtkWpeWarpMapOut& wpeOut) {
        GeminiEisFrameInfo f{};
        f.sensor_timestamp_ns = timing.p1FrameStartBootNs;
        f.exposure_time_ns = timing.exposureNs;
        f.rolling_shutter_skew_ns = timing.rollingShutterSkewNs;
        f.zoom = timing.zoom;
        int rc = session_.processLiveYPlane(f, rrzoYPlane, rrzoW, rrzoH, rrzoStride, result);
        if (rc != GEMINI_EIS_OK) return rc;
        return GeminiEisExportMtkWpeMap(
            result,
            wpeOut.sensorWidth > 0 ? wpeOut.sensorWidth : config_.input_width,
            wpeOut.sensorHeight > 0 ? wpeOut.sensorHeight : config_.input_height,
            wpeOut.strideElements,
            wpeOut.fixedPointShift,
            wpeOut.wpeMapXFixed,
            wpeOut.wpeMapYFixed,
            wpeOut.wpeMapXFloat,
            wpeOut.wpeMapYFloat);
    }

private:
    GeminiEisConfig config_{};
    Session session_;
};

}  // namespace gemini
