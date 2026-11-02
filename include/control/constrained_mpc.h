#pragma once

#include <vector>
#include "common/types.hpp"

namespace gemini {

struct MpcWeights {
    double position = 18.0;
    double velocity = 2.0;
    double acceleration = 0.25;
    double jerk = 0.02;
    double slack = 120.0;
};

struct MpcConfig {
    double dtS = 1.0 / 30.0;
    int horizon = 10;
    int iterations = 15;
    double stepSize = 0.0008;
    double maxJerkRadS3 = 35.0;
};

struct MpcRequest {
    AxisState initial{};
    std::vector<double> referenceAngle;
    std::vector<double> referenceVelocity;
    std::vector<double> rawAngle;
    double cropBudgetRad = 0.10;
    MpcWeights weights{};
};

struct MpcResult {
    bool valid = false;
    double jerkCommand = 0.0;
    AxisState next{};
    double objective = 0.0;
    double maxConstraintViolationRad = 0.0;
    std::vector<AxisState> predicted;
};

class ConstrainedMpc1D {
public:
    explicit ConstrainedMpc1D(MpcConfig config = {});
    const MpcConfig& config() const { return config_; }
    MpcResult solve(const MpcRequest& request) const;

private:
    MpcConfig config_;
};

}  // namespace gemini
