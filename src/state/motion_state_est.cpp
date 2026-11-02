#include "state/motion_state_est.h"

#include <algorithm>
#include <cmath>

namespace gemini {

MotionStateEstimator::MotionStateEstimator(MotionStateConfig config)
    : config_(config), gyro_(config.gyroBufferHorizonNs) {}

void MotionStateEstimator::pushGyro(const GyroSample& sample) {
    gyro_.push(sample);
    if (haveLastGyro_) {
        prevGyro_ = lastGyro_;
        havePrevGyro_ = true;
    }
    lastGyro_ = sample;
    haveLastGyro_ = true;
}

void MotionStateEstimator::pushVisual(const VisualResidual& residual) {
    visual_.push_back(residual);
    const int64_t minTs = residual.timestampNs - config_.gyroBufferHorizonNs;
    while (!visual_.empty() && visual_.front().timestampNs < minTs) {
        visual_.pop_front();
    }
}

MotionState MotionStateEstimator::evaluate(int64_t timestampNs) {
    MotionState out;
    out.timestampNs = timestampNs;

    GyroSample current{};
    gyro_.sample(timestampNs, &current);
    out.raw.pitch.velocity = current.omegaRadS.x;
    out.raw.yaw.velocity = current.omegaRadS.y;
    out.raw.roll.velocity = current.omegaRadS.z;

    if (havePrevGyro_ && haveLastGyro_
            && lastGyro_.timestampNs > prevGyro_.timestampNs) {
        const double dt = (lastGyro_.timestampNs - prevGyro_.timestampNs) * 1e-9;
        out.angularAcceleration = (lastGyro_.omegaRadS - prevGyro_.omegaRadS)
                * (1.0 / std::max(1e-6, dt));
        out.raw.pitch.acceleration = out.angularAcceleration.x;
        out.raw.yaw.acceleration = out.angularAcceleration.y;
        out.raw.roll.acceleration = out.angularAcceleration.z;
    }

    if (!visual_.empty()) {
        const auto& v = visual_.back();
        const double age = (timestampNs - v.timestampNs) * 1e-9;
        if (age >= 0.0 && age <= config_.visualMaxAgeS && v.similarity.trust > 0.01) {
            const double decay = std::exp(-age / std::max(1e-4, config_.visualTauS));
            out.delayedVisual = v.similarity;

            // Forward causal re-propagation from visual capture time to evaluation time:
            if (gyro_.covers(v.timestampNs) && gyro_.covers(timestampNs)) {
                const Vec3 dR = gyro_.integrate(v.timestampNs, timestampNs);
                const double cosR = std::cos(dR.z);
                const double sinR = std::sin(dR.z);
                const double tx = v.similarity.txPx;
                const double ty = v.similarity.tyPx;
                out.delayedVisual.txPx = (cosR * tx - sinR * ty) * decay;
                out.delayedVisual.tyPx = (sinR * tx + cosR * ty) * decay;
            } else {
                out.delayedVisual.txPx *= decay;
                out.delayedVisual.tyPx *= decay;
            }
            out.delayedVisual.rotationRad *= decay;
            out.delayedVisual.scale = 1.0 + (out.delayedVisual.scale - 1.0) * decay;
            out.delayedVisual.trust *= decay;
        }
    }

    const double available = std::max(0.01,
        config_.baseCropBudgetRad - config_.rs19MarginRad);
    out.crop = {available, available, available * 0.85};
    return out;
}

}  // namespace gemini
