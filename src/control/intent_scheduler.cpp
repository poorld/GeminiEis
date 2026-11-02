#include "control/intent_scheduler.h"

#include <algorithm>
#include <cmath>

namespace gemini {

IntentScheduler1D::IntentScheduler1D(SchedulerConfig config)
    : config_(config) {}

void IntentScheduler1D::reset() {
    initialized_ = false;
    lastTimestampNs_ = 0;
    referenceAngle_ = 0.0;
    referenceVelocity_ = 0.0;
    panConfidence_ = 0.0;
    state_ = IntentState::Hold;
    stateDwellTimerS_ = 0.0;
    omegaHistory_.clear();
}

SchedulerOutput IntentScheduler1D::update(int64_t timestampNs,
                                          double rawAngle,
                                          double angularVelocity,
                                          double angularAcceleration,
                                          double cropBudgetRad) {
    SchedulerOutput out;
    if (!initialized_) {
        initialized_ = true;
        lastTimestampNs_ = timestampNs;
        referenceAngle_ = rawAngle;
        referenceVelocity_ = 0.0;
        panConfidence_ = 0.0;
        state_ = IntentState::Hold;
        stateDwellTimerS_ = 0.0;
    }

    double dt = (timestampNs - lastTimestampNs_) * 1e-9;
    dt = clamp(dt, 1e-4, 0.10);
    lastTimestampNs_ = timestampNs;
    stateDwellTimerS_ += dt;

    // 1. Maintain sliding-window angular velocity history
    omegaHistory_.push_back({timestampNs, angularVelocity});
    const int64_t minTs = timestampNs - static_cast<int64_t>(config_.reversalWindowS * 1e9);
    while (!omegaHistory_.empty() && omegaHistory_.front().timestampNs < minTs) {
        omegaHistory_.pop_front();
    }

    // 2. Oscillation V2: Cadence frequency detection (1.4 ~ 2.6 Hz) and directional coherence
    int cadenceHits = 0;
    int64_t lastCrossTs = 0;
    double netDisp = 0.0;
    double absDisp = 0.0;

    for (std::size_t i = 1; i < omegaHistory_.size(); ++i) {
        const double w0 = omegaHistory_[i - 1].omega;
        const double w1 = omegaHistory_[i].omega;
        const double stepDt = (omegaHistory_[i].timestampNs - omegaHistory_[i - 1].timestampNs) * 1e-9;
        if (stepDt <= 0.0 || stepDt > 0.10) continue;

        netDisp += 0.5 * (w0 + w1) * stepDt;
        absDisp += 0.5 * (std::abs(w0) + std::abs(w1)) * stepDt;

        if ((w0 < -0.015 && w1 >= 0.015) || (w0 > 0.015 && w1 <= -0.015)) {
            const int64_t crossTs = omegaHistory_[i].timestampNs;
            if (lastCrossTs > 0) {
                const double intervalS = (crossTs - lastCrossTs) * 1e-9;
                // Half-period interval for 1.4 ~ 2.6 Hz is ~0.19 ~ 0.36 s
                if (intervalS >= (0.5 / config_.cadenceFreqMaxHz)
                        && intervalS <= (0.5 / config_.cadenceFreqMinHz)) {
                    ++cadenceHits;
                }
            }
            lastCrossTs = crossTs;
        }
    }

    const double coherence = absDisp > 1e-4 ? clamp(std::abs(netDisp) / absDisp, 0.0, 1.0) : 1.0;
    double oscScore = 0.0;
    if (absDisp > 0.015) {
        const double freqFactor = clamp(static_cast<double>(cadenceHits) / 2.0, 0.0, 1.0);
        const double coherenceFactor = clamp(1.0 - coherence, 0.0, 1.0);
        const double energyFactor = clamp(absDisp / 0.05, 0.0, 1.0);
        oscScore = clamp(freqFactor * 0.65 + coherenceFactor * energyFactor * 0.35, 0.0, 1.0);
    }

    // 3. Pan intent detection with directional hysteresis
    const double absOmega = std::abs(angularVelocity);
    const double speedScore = clamp(
        (absOmega - config_.panExitOmegaThresholdRadS)
        / std::max(1e-4, config_.panOmegaThresholdRadS - config_.panExitOmegaThresholdRadS),
        0.0, 1.0);
    const double accelPenalty = clamp(
        std::abs(angularAcceleration) / std::max(1e-4, config_.panAccelThresholdRadS2),
        0.0, 1.0);

    // 4. Emergency Stop State Machine: only enter StopSettle from a high-coherence intentional pan
    // (Never trigger on periodic oscillation zero-crossings where coherence is low)
    const bool wasPanning = (panConfidence_ > 0.35 && coherence > 0.45);
    const bool isBraking = (angularVelocity * angularAcceleration < -0.05)
                        || (absOmega < config_.stopOmegaThresholdRadS && wasPanning);

    if (wasPanning && isBraking && absOmega < config_.stopOmegaThresholdRadS) {
        state_ = IntentState::StopSettle;
        stateDwellTimerS_ = 0.0;
    } else if (state_ == IntentState::StopSettle) {
        if (stateDwellTimerS_ >= config_.minDwellS) {
            state_ = (speedScore > 0.5 && coherence > 0.6)
                ? IntentState::PanFollow : IntentState::Hold;
            stateDwellTimerS_ = 0.0;
        }
    }

    // Smooth pan confidence blending
    double targetPan = speedScore * coherence * (1.0 - 0.85 * oscScore) * (1.0 - 0.40 * accelPenalty);
    if (state_ == IntentState::StopSettle) {
        targetPan = 0.0;
    }
    targetPan = clamp(targetPan, 0.0, 1.0);

    const double panTau = (targetPan > panConfidence_) ? 0.06 : 0.16;
    panConfidence_ += (dt / std::max(1e-4, panTau)) * (targetPan - panConfidence_);
    panConfidence_ = clamp(panConfidence_, 0.0, 1.0);

    if (state_ != IntentState::StopSettle) {
        state_ = (panConfidence_ > 0.35) ? IntentState::PanFollow : IntentState::Hold;
    }

    // 5. Dynamic Crop Envelope Follow & Two-Tier LFGO
    const double errA = referenceAngle_ - rawAngle;
    const double absErrA = std::abs(errA);
    const double softLimit = cropBudgetRad * config_.softLimitRatio;

    double envelopeBlend = 0.0;
    if (absErrA > softLimit) {
        const double u = clamp((absErrA - softLimit) / std::max(1e-4, cropBudgetRad - softLimit), 0.0, 1.0);
        envelopeBlend = u * u * (3.0 - 2.0 * u);  // smoothstep
    }

    if (state_ == IntentState::StopSettle && envelopeBlend < 0.2) {
        // Critical damping to zero velocity to avoid overshoot
        referenceVelocity_ *= std::exp(-dt / std::max(1e-4, config_.stopSettleTauS));
        if (std::abs(referenceVelocity_) < 1e-4) referenceVelocity_ = 0.0;

        referenceAngle_ += referenceVelocity_ * dt;
        // Anti-overshoot clamping: never overshoot past the stopped rawAngle
        const double errAfter = referenceAngle_ - rawAngle;
        if (referenceVelocity_ * errAfter > 0.0) {
            referenceVelocity_ = 0.0;
            referenceAngle_ = rawAngle;
        }
    } else {
        // If envelope pressure is active, smoothly shorten time constants
        const double baseTauOmega = config_.holdRefTauS * (1.0 - panConfidence_)
                                  + config_.followRefTauS * panConfidence_;
        const double tauOmega = baseTauOmega * (1.0 - envelopeBlend) + config_.envelopeLimitTauS * envelopeBlend;

        const double baseTauTheta = 0.60 * (1.0 - panConfidence_) + 0.085 * panConfidence_;
        const double tauTheta = baseTauTheta * (1.0 - envelopeBlend) + config_.envelopeLimitTauS * envelopeBlend;

        referenceVelocity_ += (dt / std::max(1e-4, tauOmega)) * (angularVelocity - referenceVelocity_);
        referenceAngle_ += referenceVelocity_ * dt
                         + (dt / std::max(1e-4, tauTheta)) * (rawAngle - referenceAngle_);

        // Hard reference crop envelope ceiling: never allow ref to exceed 90% of budget
        const double maxRefCorr = cropBudgetRad * 0.90;
        if (std::abs(referenceAngle_ - rawAngle) > maxRefCorr) {
            const double sgn = (referenceAngle_ > rawAngle) ? 1.0 : -1.0;
            referenceAngle_ = rawAngle + sgn * maxRefCorr;
            referenceVelocity_ = angularVelocity;
        }
    }

    // 6. Output packaging and time-varying MPC weights
    out.referenceAngle = referenceAngle_;
    out.referenceVelocity = referenceVelocity_;
    out.panConfidence = panConfidence_;
    out.oscillationScore = oscScore;
    out.state = state_;
    out.isStopped = (state_ == IntentState::StopSettle);

    out.weights.position = 16.0 + 22.0 * oscScore + (out.isStopped ? 10.0 : 0.0);
    out.weights.velocity = 2.0 + 3.0 * (1.0 - panConfidence_) + (out.isStopped ? 5.0 : 0.0);
    out.weights.acceleration = 0.25 + 0.40 * oscScore;
    out.weights.jerk = 0.02 + 0.07 * oscScore;
    out.weights.slack = 140.0 + 80.0 * (1.0 - panConfidence_);

    return out;
}

}  // namespace gemini
