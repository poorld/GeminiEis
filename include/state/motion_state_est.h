#pragma once

#include <deque>
#include "common/ring_buffer.hpp"
#include "common/types.hpp"

namespace gemini {

struct MotionStateConfig {
    double visualTauS = 0.60;
    double visualMaxAgeS = 0.20;
    double baseCropBudgetRad = 0.10;
    double rs19MarginRad = 0.012;
    int64_t gyroBufferHorizonNs = 400000000LL;  // 400ms buffer capacity for historical fusion
};

class MotionStateEstimator {
public:
    explicit MotionStateEstimator(MotionStateConfig config = {});

    void pushGyro(const GyroSample& sample);
    void pushVisual(const VisualResidual& residual);
    MotionState evaluate(int64_t timestampNs);

    const GyroRingBuffer& gyroBuffer() const { return gyro_; }

private:
    MotionStateConfig config_;
    GyroRingBuffer gyro_;
    GyroSample lastGyro_{};
    GyroSample prevGyro_{};
    bool haveLastGyro_ = false;
    bool havePrevGyro_ = false;
    std::deque<VisualResidual> visual_;
};

}  // namespace gemini
