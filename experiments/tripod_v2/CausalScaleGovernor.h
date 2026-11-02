#pragma once
// Gemini Tripod V2: a portable strictly CAUSAL Scale state machine.
// This component alone is causal; its upstream PC V1 full-sequence
// visual_tripod_*.f64 / oracle_matrices.f64 are explicitly NON-CAUSAL.
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>

namespace gemini::tripod_v2 {
struct ScaleGovernorConfig {
    double max_log_step_per_frame = 0.0015;
    double min_scale = 0.1;
    double max_scale = 10.0;
};
class CausalScaleGovernor {
public:
    explicit CausalScaleGovernor(ScaleGovernorConfig config={}) noexcept: config_(config) {}
    void reset() noexcept { have_previous_=false; prev_log_scale_=0.0; last_sensor_ts_ns_=0; }
    bool hasValue() const noexcept { return have_previous_; }
    double update(double measured_scale,std::int64_t sensor_ts_ns) noexcept {
        if (!std::isfinite(measured_scale) ||
            measured_scale < config_.min_scale ||
            measured_scale > config_.max_scale ||
            sensor_ts_ns<=0 ||
            (have_previous_ && sensor_ts_ns<=last_sensor_ts_ns_)) {
            return have_previous_ ? std::exp(prev_log_scale_) : 1.0;
        }
        const double candidate=std::log(measured_scale);
        const double max_step=std::max(1e-8,config_.max_log_step_per_frame);
        if (have_previous_) {
            prev_log_scale_+=std::clamp(candidate-prev_log_scale_,-max_step,max_step);
        } else {
            prev_log_scale_=candidate;
            have_previous_=true;
        }
        last_sensor_ts_ns_=sensor_ts_ns;
        return std::exp(prev_log_scale_);
    }
private:
    ScaleGovernorConfig config_{};
    bool have_previous_=false;
    double prev_log_scale_=0;
    std::int64_t last_sensor_ts_ns_=0;
};
}  // namespace gemini::tripod_v2
