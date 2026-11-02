#pragma once

#include <algorithm>
#include <cstdint>
#include <deque>
#include <stdexcept>
#include "common/types.hpp"

namespace gemini {

struct GyroPoseSample {
    int64_t timestampNs = 0;
    Vec3 omegaRadS{};
    Quat pose{0, 0, 0, 1};
};

class GyroRingBuffer {
public:
    static constexpr int64_t kMaxCausalTailExtrapNs = 35000000LL; // 35ms causal tail extrapolation for RS19 bottom rows
    static constexpr int64_t kMaxHeadToleranceNs = 15000000LL;    // 15ms head tolerance

    explicit GyroRingBuffer(int64_t horizonNs = 5000000000LL)
        : horizonNs_(horizonNs) {}

    void push(const GyroSample& sample) {
        if (!samples_.empty() && sample.timestampNs <= samples_.back().timestampNs) {
            // Ignore duplicate or out-of-order sensor hub timestamps instead of throwing in real-time
            return;
        }
        if (samples_.empty()) {
            biasEstimate_ = {};
            stationaryCount_ = 0;
            samples_.push_back({sample.timestampNs, sample.omegaRadS, Quat::identity()});
            return;
        }

        Vec3 omegaCorr = sample.omegaRadS - biasEstimate_;
        if (sample.omegaRadS.norm() < 0.038 && omegaCorr.norm() < 0.038) {
            stationaryCount_ = std::min(stationaryCount_ + 1, 10000);
            if (stationaryCount_ >= 20) {
                biasEstimate_ = biasEstimate_ * 0.992 + sample.omegaRadS * 0.008;
                omegaCorr = sample.omegaRadS - biasEstimate_;
                // Deadband stationary/handheld MEMS white noise (< 0.0065 rad/s ~ 0.37 deg/s) to zero
                if (omegaCorr.norm() < 0.0065) {
                    omegaCorr = {};
                }
            }
        } else {
            stationaryCount_ = std::max(0, stationaryCount_ - 2);
        }

        const auto& last = samples_.back();
        const int64_t dtNs = sample.timestampNs - last.timestampNs;
        Quat newPose = last.pose;
        if (dtNs > 0 && dtNs < 50000000LL) {
            const double dtS = static_cast<double>(dtNs) * 1e-9;
            const Vec3 omegaMid = (last.omegaRadS + omegaCorr) * 0.5;
            const Quat delta = Quat::fromRotationVector(omegaMid * dtS);
            newPose = (last.pose * delta).normalized();
        }

        samples_.push_back({sample.timestampNs, omegaCorr, newPose});

        const int64_t minTs = sample.timestampNs - horizonNs_;
        while (samples_.size() > 4 && samples_[1].timestampNs < minTs) {
            samples_.pop_front();
        }
    }

    bool empty() const { return samples_.empty(); }
    std::size_t size() const { return samples_.size(); }
    int64_t firstTimestampNs() const { return samples_.empty() ? 0 : samples_.front().timestampNs; }
    int64_t lastTimestampNs() const { return samples_.empty() ? 0 : samples_.back().timestampNs; }

    bool covers(int64_t timestampNs) const {
        return !samples_.empty()
            && timestampNs >= (samples_.front().timestampNs - kMaxHeadToleranceNs)
            && timestampNs <= (samples_.back().timestampNs + kMaxCausalTailExtrapNs);
    }

    bool sample(int64_t timestampNs, GyroSample* out) const {
        if (!out || samples_.empty()) return false;
        if (timestampNs <= samples_.front().timestampNs) {
            out->timestampNs = timestampNs;
            out->omegaRadS = samples_.front().omegaRadS;
            return true;
        }
        if (timestampNs >= samples_.back().timestampNs) {
            out->timestampNs = timestampNs;
            out->omegaRadS = samples_.back().omegaRadS;
            return true;
        }
        auto it = std::lower_bound(samples_.begin(), samples_.end(), timestampNs,
            [](const GyroPoseSample& s, int64_t t) { return s.timestampNs < t; });
        const auto& b = *it;
        const auto& a = *(it - 1);
        const double denom = static_cast<double>(b.timestampNs - a.timestampNs);
        const double q = denom > 0.0 ? static_cast<double>(timestampNs - a.timestampNs) / denom : 0.0;
        out->timestampNs = timestampNs;
        out->omegaRadS = a.omegaRadS * (1.0 - q) + b.omegaRadS * q;
        return true;
    }

    bool orientationAt(int64_t timestampNs, Quat* out) const {
        if (!out || samples_.empty()) return false;
        if (timestampNs <= samples_.front().timestampNs) {
            *out = samples_.front().pose;
            return true;
        }
        if (timestampNs >= samples_.back().timestampNs) {
            const auto& last = samples_.back();
            const double dtExtrapS = std::min(0.025, std::max(0.0, static_cast<double>(timestampNs - last.timestampNs) * 1e-9));
            *out = (last.pose * Quat::fromRotationVector(last.omegaRadS * dtExtrapS)).normalized();
            return true;
        }
        auto it = std::lower_bound(samples_.begin(), samples_.end(), timestampNs,
            [](const GyroPoseSample& s, int64_t t) { return s.timestampNs < t; });
        const auto& b = *it;
        const auto& a = *(it - 1);
        const double denom = static_cast<double>(b.timestampNs - a.timestampNs);
        const double q = denom > 0.0 ? static_cast<double>(timestampNs - a.timestampNs) / denom : 0.0;
        *out = Quat::slerp(a.pose, b.pose, q);
        return true;
    }

    Vec3 integrate(int64_t beginNs, int64_t endNs) const {
        if (samples_.empty() || endNs <= beginNs) return {};
        Quat qa{}, qb{};
        if (orientationAt(beginNs, &qa) && orientationAt(endNs, &qb)) {
            const Quat qDelta = (qa.inverse() * qb).normalized();
            return Vec3{2.0 * qDelta.x, 2.0 * qDelta.y, 2.0 * qDelta.z};
        }
        return {};
    }

private:
    int64_t horizonNs_;
    std::deque<GyroPoseSample> samples_;
    Vec3 biasEstimate_{};
    int stationaryCount_ = 0;
};

}  // namespace gemini
