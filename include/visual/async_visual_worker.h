#pragma once

#include <vector>
#include "common/ring_buffer.hpp"
#include "common/types.hpp"

namespace gemini {

struct CameraModel {
    double fx = 99.0;
    double fy = 99.0;
    double cx = 96.0;
    double cy = 54.0;
};

struct VisualConfig {
    int ransacIterations = 80;
    double inlierThresholdPx = 1.5;
    double fbThresholdPx = 1.0;
    int targetInliers = 48;
    double exposureGateS = 0.025;      // 25ms motion blur gate threshold
    double motionGateRadS = 0.4;       // 0.4 rad/s angular speed threshold
    double trustFbAlpha = 0.7;
    double isoGate = 800.0;            // ISO noise degradation gate
    double maxIso = 3200.0;            // Maximum ISO normalization
};

class AsyncVisualWorkerMath {
public:
    AsyncVisualWorkerMath(CameraModel camera = {}, VisualConfig config = {});

    std::vector<FeatureMatch> gyroPrewarp(
        const std::vector<FeatureMatch>& matches,
        const FrameMeta& prevMeta,
        const FrameMeta& currMeta,
        const GyroRingBuffer& gyro) const;

    Similarity2D estimateSimilarity(
        const std::vector<FeatureMatch>& matches,
        double exposureS,
        double angularSpeedRadS,
        double iso = 100.0) const;

    const CameraModel& camera() const { return camera_; }
    const VisualConfig& config() const { return config_; }

private:
    CameraModel camera_;
    VisualConfig config_;
};

}  // namespace gemini
