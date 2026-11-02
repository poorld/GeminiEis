#include "control/constrained_mpc.h"

#include <algorithm>
#include <array>
#include <cmath>

namespace gemini {
namespace {

constexpr int kMaxHorizon = 32;

inline AxisState stepState(const AxisState& x, double jerk, double dt) {
    AxisState y;
    const double dt2 = dt * dt;
    const double dt3 = dt2 * dt;
    y.angle = x.angle + x.velocity * dt + 0.5 * x.acceleration * dt2
            + (1.0 / 6.0) * jerk * dt3;
    y.velocity = x.velocity + x.acceleration * dt + 0.5 * jerk * dt2;
    y.acceleration = x.acceleration + jerk * dt;
    return y;
}

inline double valueAt(const std::vector<double>& v, int i, double fallback) {
    if (v.empty()) return fallback;
    return v[std::min<int>(i, static_cast<int>(v.size()) - 1)];
}

}  // namespace

ConstrainedMpc1D::ConstrainedMpc1D(MpcConfig config) : config_(config) {
    config_.horizon = std::clamp(config_.horizon, 2, kMaxHorizon);
    config_.iterations = std::max(1, config_.iterations);
    config_.dtS = std::max(1e-4, config_.dtS);
    config_.stepSize = std::max(1e-8, config_.stepSize);
    config_.maxJerkRadS3 = std::max(0.1, config_.maxJerkRadS3);
}

MpcResult ConstrainedMpc1D::solve(const MpcRequest& request) const {
    MpcResult result;
    const int n = std::min(config_.horizon, kMaxHorizon);
    const double dt = config_.dtS;
    const double dt2 = dt * dt;
    const double dt3 = dt2 * dt;

    // Precompute sensitivity coefficients S[k][j] for 0 <= j <= k < n:
    // S_theta[k][j] = dt^3 * (3*p^2 + 3*p + 1) / 6
    // S_omega[k][j] = dt^2 * (2*p + 1) / 2
    // S_alpha[k][j] = dt
    // where p = k - j.
    std::array<std::array<double, kMaxHorizon>, kMaxHorizon> sTheta{};
    std::array<std::array<double, kMaxHorizon>, kMaxHorizon> sOmega{};
    std::array<std::array<double, kMaxHorizon>, kMaxHorizon> sAlpha{};
    for (int k = 0; k < n; ++k) {
        for (int j = 0; j <= k; ++j) {
            const int p = k - j;
            sTheta[k][j] = dt3 * (3.0 * p * p + 3.0 * p + 1.0) / 6.0;
            sOmega[k][j] = dt2 * (2.0 * p + 1.0) / 2.0;
            sAlpha[k][j] = dt;
        }
    }

    std::array<double, kMaxHorizon> u{};
    std::array<AxisState, kMaxHorizon> traj{};
    std::array<double, kMaxHorizon> lambdaTheta{};
    std::array<double, kMaxHorizon> lambdaOmega{};
    std::array<double, kMaxHorizon> lambdaAlpha{};
    std::array<double, kMaxHorizon> grad{};
    std::array<double, kMaxHorizon> diagH{};

    const double freeTheta1 = request.initial.angle + request.initial.velocity * dt
                            + 0.5 * request.initial.acceleration * dt2;
    const double b0 = (1.0 / 6.0) * dt3;
    const double raw0 = valueAt(request.rawAngle, 0, request.initial.angle);

    for (int iter = 0; iter < config_.iterations; ++iter) {
        // 1. Forward rollout with current control inputs u
        AxisState x = request.initial;
        for (int k = 0; k < n; ++k) {
            x = stepState(x, u[k], dt);
            traj[k] = x;
        }

        // 2. Analytical adjoint gradients for state objectives
        for (int k = 0; k < n; ++k) {
            const double refA = valueAt(request.referenceAngle, k, request.initial.angle);
            const double refV = valueAt(request.referenceVelocity, k, 0.0);
            const double rawA = valueAt(request.rawAngle, k, refA);

            const double posE = traj[k].angle - refA;
            const double velE = traj[k].velocity - refV;
            const double accE = traj[k].acceleration;

            const double corr = traj[k].angle - rawA;
            const double absCorr = std::abs(corr);
            const double violation = std::max(0.0, absCorr - request.cropBudgetRad);

            double slackGrad = 0.0;
            if (violation > 0.0) {
                const double sgn = (corr >= 0.0) ? 1.0 : -1.0;
                slackGrad = 2.0 * request.weights.slack * violation * sgn;
            }

            lambdaTheta[k] = 2.0 * request.weights.position * posE + slackGrad;
            lambdaOmega[k] = 2.0 * request.weights.velocity * velE;
            lambdaAlpha[k] = 2.0 * request.weights.acceleration * accE;
        }

        // 3. Exact analytical gradient and diagonal Hessian
        for (int j = 0; j < n; ++j) {
            double g = 2.0 * request.weights.jerk * u[j];
            double h = 2.0 * request.weights.jerk;

            for (int k = j; k < n; ++k) {
                const double st = sTheta[k][j];
                const double so = sOmega[k][j];
                const double sa = sAlpha[k][j];

                g += lambdaTheta[k] * st + lambdaOmega[k] * so + lambdaAlpha[k] * sa;

                const double rawA = valueAt(request.rawAngle, k, request.initial.angle);
                const double absCorr = std::abs(traj[k].angle - rawA);
                const double wSlackEff = (absCorr > request.cropBudgetRad) ? (2.0 * request.weights.slack) : 0.0;
                const double wPosEff = 2.0 * request.weights.position + wSlackEff;

                h += wPosEff * st * st
                   + 2.0 * request.weights.velocity * so * so
                   + 2.0 * request.weights.acceleration * sa * sa;
            }
            grad[j] = g;
            diagH[j] = std::max(h, 1e-4);
        }

        // 4. Scaled Projected Gradient Descent
        const double gamma = 0.80 / (1.0 + 0.03 * iter);
        for (int j = 0; j < n; ++j) {
            const double delta = gamma * (grad[j] / diagH[j]);
            u[j] = std::clamp(u[j] - delta, -config_.maxJerkRadS3, config_.maxJerkRadS3);
        }

        // 5. Hard crop safety projection on u[0]
        if (b0 > 1e-12) {
            const double u0MinCrop = (raw0 - request.cropBudgetRad - freeTheta1) / b0;
            const double u0MaxCrop = (raw0 + request.cropBudgetRad - freeTheta1) / b0;
            const double u0Low = std::max(-config_.maxJerkRadS3, u0MinCrop);
            const double u0High = std::min(config_.maxJerkRadS3, u0MaxCrop);
            if (u0Low <= u0High) {
                u[0] = std::clamp(u[0], u0Low, u0High);
            } else {
                u[0] = (freeTheta1 > raw0) ? -config_.maxJerkRadS3 : config_.maxJerkRadS3;
            }
        }
    }

    // 6. Final evaluation and result packaging
    AxisState x = request.initial;
    double cost = 0.0;
    double maxV = 0.0;
    result.predicted.clear();
    result.predicted.reserve(n);

    for (int k = 0; k < n; ++k) {
        x = stepState(x, u[k], dt);
        result.predicted.push_back(x);

        const double refA = valueAt(request.referenceAngle, k, request.initial.angle);
        const double refV = valueAt(request.referenceVelocity, k, 0.0);
        const double rawA = valueAt(request.rawAngle, k, refA);

        const double posE = x.angle - refA;
        const double velE = x.velocity - refV;
        const double absCorr = std::abs(x.angle - rawA);
        const double violation = std::max(0.0, absCorr - request.cropBudgetRad);
        maxV = std::max(maxV, violation);

        cost += request.weights.position * posE * posE
              + request.weights.velocity * velE * velE
              + request.weights.acceleration * x.acceleration * x.acceleration
              + request.weights.jerk * u[k] * u[k]
              + request.weights.slack * violation * violation;
    }

    result.objective = cost;
    result.maxConstraintViolationRad = maxV;
    result.jerkCommand = u.front();
    result.next = stepState(request.initial, result.jerkCommand, dt);
    result.valid = std::isfinite(result.objective)
            && std::isfinite(result.next.angle)
            && (result.maxConstraintViolationRad < std::max(0.05, request.cropBudgetRad)
                || std::abs(result.next.angle - raw0) <= request.cropBudgetRad + 1e-4);

    return result;
}

}  // namespace gemini
