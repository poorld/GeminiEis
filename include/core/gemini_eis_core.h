#pragma once

#include <array>
#include <vector>
#include "common/types.hpp"
#include "control/constrained_mpc.h"
#include "control/intent_scheduler.h"
#include "state/motion_state_est.h"
#include "warp/rs19_one_warp.h"

namespace gemini {

struct CoreConfig {
    MpcConfig mpc{};
    SchedulerConfig scheduler{};
    MotionStateConfig motion{};
    Rs19Config warp{};
};

struct FrameInput {
    int64_t timestampNs = 0;
    Vec3 rawAngleRad{};
    Vec3 angularVelocityRadS{};
    Vec3 angularAccelerationRadS2{};
    Similarity2D visual{};
    std::array<Vec3, 19> rs19{};
    double zoom = 1.0;
    Quat qCenter{0, 0, 0, 1};
    std::array<Quat, 20> rowQuats{};
    bool useQuats = false;
};

struct FrameOutput {
    Vec3 targetAngleRad{};
    Vec3 correctionRad{};
    Vec3 jerkRadS3{};
    double panConfidence = 0.0;
    double oscillationScore = 0.0;
    CropBudget crop{};
    Similarity2D visual{};
    Quat qTarget{0, 0, 0, 1};
    double zoom = 1.12;
    std::vector<WarpVertex> mesh;
};

class GeminiEisCore {
public:
    explicit GeminiEisCore(CoreConfig config = {});
    FrameOutput processFrame(const FrameInput& input);

private:
    CoreConfig config_;
    ConstrainedMpc1D pitchMpc_;
    ConstrainedMpc1D yawMpc_;
    ConstrainedMpc1D rollMpc_;
    IntentScheduler1D pitchScheduler_;
    IntentScheduler1D yawScheduler_;
    IntentScheduler1D rollScheduler_;
    Rs19OneWarp warp_;
    AxisState pitchState_{};
    AxisState yawState_{};
    AxisState rollState_{};

    Quat qTarget_{0, 0, 0, 1};
    Quat qAnchor_{0, 0, 0, 1};
    double currentZoom_ = 1.0;
    int64_t lastTimestampNs_ = 0;
    bool quatInitialized_ = false;
    int frameCount_ = 0;
};

}  // namespace gemini
