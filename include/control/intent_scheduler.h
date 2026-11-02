#pragma once

#include <cstdint>
#include <deque>
#include "common/types.hpp"
#include "control/constrained_mpc.h"

namespace gemini {

enum class IntentState {
    Hold = 0,
    PanFollow = 1,
    StopSettle = 2,
};

struct SchedulerConfig {
    double panOmegaThresholdRadS = 0.25;
    double panExitOmegaThresholdRadS = 0.12;
    double panAccelThresholdRadS2 = 2.5;
    double reversalWindowS = 0.80;
    double holdRefTauS = 0.55;
    double followRefTauS = 0.085;
    double stopOmegaThresholdRadS = 0.08;
    double stopSettleTauS = 0.06;
    double minDwellS = 0.15;
    double cadenceFreqMinHz = 1.4;
    double cadenceFreqMaxHz = 2.6;
    // Dynamic crop envelope follow
    double softLimitRatio = 0.60;
    double envelopeLimitTauS = 0.065;
};

struct SchedulerOutput {
    double referenceAngle = 0.0;
    double referenceVelocity = 0.0;
    double panConfidence = 0.0;
    double oscillationScore = 0.0;
    IntentState state = IntentState::Hold;
    bool isStopped = false;
    MpcWeights weights{};
};

class IntentScheduler1D {
public:
    explicit IntentScheduler1D(SchedulerConfig config = {});

    SchedulerOutput update(int64_t timestampNs,
                           double rawAngle,
                           double angularVelocity,
                           double angularAcceleration,
                           double cropBudgetRad = 0.088);

    void reset();
    const SchedulerConfig& config() const { return config_; }

private:
    SchedulerConfig config_;
    bool initialized_ = false;
    int64_t lastTimestampNs_ = 0;
    double referenceAngle_ = 0.0;
    double referenceVelocity_ = 0.0;
    double panConfidence_ = 0.0;
    IntentState state_ = IntentState::Hold;
    double stateDwellTimerS_ = 0.0;

    struct OmegaRecord {
        int64_t timestampNs = 0;
        double omega = 0.0;
    };
    std::deque<OmegaRecord> omegaHistory_;
};

}  // namespace gemini
