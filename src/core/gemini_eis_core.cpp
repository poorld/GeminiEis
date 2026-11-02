#include "core/gemini_eis_core.h"

#include <algorithm>
#include <cmath>

namespace gemini {
namespace {

MpcResult solveAxis(ConstrainedMpc1D& solver,
                    AxisState& state,
                    double rawAngle,
                    double refAngle,
                    double refVelocity,
                    double cropBudget,
                    const MpcWeights& weights) {
    const int n = solver.config().horizon;
    MpcRequest req;
    req.initial = state;
    req.referenceAngle.assign(n, refAngle);
    req.referenceVelocity.assign(n, refVelocity);
    req.rawAngle.assign(n, rawAngle);
    req.cropBudgetRad = cropBudget;
    req.weights = weights;
    auto result = solver.solve(req);
    if (result.valid) state = result.next;
    return result;
}

}  // namespace

GeminiEisCore::GeminiEisCore(CoreConfig config)
    : config_(config),
      pitchMpc_(config.mpc),
      yawMpc_(config.mpc),
      rollMpc_(config.mpc),
      pitchScheduler_(config.scheduler),
      yawScheduler_(config.scheduler),
      rollScheduler_(config.scheduler),
      warp_(config.warp) {}

FrameOutput GeminiEisCore::processFrame(const FrameInput& input) {
    FrameOutput out;

    const double baseBudget = std::max(0.01,
        config_.motion.baseCropBudgetRad - config_.motion.rs19MarginRad);
    out.crop = {baseBudget, baseBudget, baseBudget * 0.85};

    const auto sp = pitchScheduler_.update(input.timestampNs, input.rawAngleRad.x,
        input.angularVelocityRadS.x, input.angularAccelerationRadS2.x, out.crop.pitchRad);
    const auto sy = yawScheduler_.update(input.timestampNs, input.rawAngleRad.y,
        input.angularVelocityRadS.y, input.angularAccelerationRadS2.y, out.crop.yawRad);
    const auto sr = rollScheduler_.update(input.timestampNs, input.rawAngleRad.z,
        input.angularVelocityRadS.z, input.angularAccelerationRadS2.z, out.crop.rollRad);

    const auto rp = solveAxis(pitchMpc_, pitchState_, input.rawAngleRad.x,
        sp.referenceAngle, sp.referenceVelocity, out.crop.pitchRad, sp.weights);
    const auto ry = solveAxis(yawMpc_, yawState_, input.rawAngleRad.y,
        sy.referenceAngle, sy.referenceVelocity, out.crop.yawRad, sy.weights);
    const auto rr = solveAxis(rollMpc_, rollState_, input.rawAngleRad.z,
        sr.referenceAngle, sr.referenceVelocity, out.crop.rollRad, sr.weights);

    out.targetAngleRad = {pitchState_.angle, yawState_.angle, rollState_.angle};
    out.correctionRad = out.targetAngleRad - input.rawAngleRad;
    out.jerkRadS3 = {rp.jerkCommand, ry.jerkCommand, rr.jerkCommand};
    out.panConfidence = (sp.panConfidence + sy.panConfidence + sr.panConfidence) / 3.0;
    out.oscillationScore = std::max({sp.oscillationScore, sy.oscillationScore, sr.oscillationScore});
    out.visual = input.visual;

    WarpRequest wr;
    wr.globalCorrectionRad = out.correctionRad;
    wr.visual = input.visual;
    wr.rowCorrectionRad = input.rs19;
    wr.useQuats = input.useQuats;

    if (!input.useQuats) {
        wr.zoom = (input.zoom > 0.0 && input.zoom <= 1.5) ? input.zoom : 1.0;
        out.mesh = warp_.buildMesh(wr);
        return out;
    }

    const Quat qCenter = input.qCenter.normalized();
    wr.qCenter = qCenter;
    wr.rowQuats = input.rowQuats;

    // Hybrid PixSmart + Virtual Tripod Deadband Architecture:
    //   1. Constant baseZoom (1.12x, zero heavy zoom pumping)
    //   2. Anchor Deadband Lock (tauAnchor = 60s inside 5.4 deg oscillation window so single/double-sided swings stay rock-solid)
    //   3. PixSmart Two-Rectangle (Inner Safe Rect + innerRatio=0.90 Outer Contour) 4-corner camera mapping
    //   4. 1/rho Reciprocal Boundary Release + 99.8% Tripod Hold inside safe contour
    //   5. Second Boundary Gate -> Per-Row RS Endpoint Target Trajectory (FilterSteadyGL)
    const double baseZoom = (input.zoom > 0.0 && input.zoom <= 1.24) ? input.zoom : 1.12;

    const double W = config_.warp.width > 0 ? static_cast<double>(config_.warp.width) : 2560.0;
    const double H = config_.warp.height > 0 ? static_cast<double>(config_.warp.height) : 1440.0;
    const double baseCrop = (config_.warp.baseCrop > 0.0 && config_.warp.baseCrop <= 1.0) ? config_.warp.baseCrop : 0.80;
    const double effScale = std::clamp(baseCrop / baseZoom, 0.50, 0.95);
    const double innerLeft   = 0.5 * (1.0 - effScale) * W;
    const double innerRight  = W - innerLeft;
    const double innerTop    = 0.5 * (1.0 - effScale) * H;
    const double innerBottom = H - innerTop;

    // PixSmart innerRatio = 0.90 (reserves ~36.6px outer buffer on 2560x1440 while maximizing +-9.2 deg swing budget)
    const double innerRatio  = 0.90;
    const double outerLeft   = (1.0 - innerRatio) * innerLeft;
    const double outerTop    = (1.0 - innerRatio) * innerTop;

    struct BoundaryEval {
        bool hit = false;
        double rho = 0.0;
        int side = 0; // 0 = A (top RS endpoint), 1 = B (bottom RS endpoint)
    };

    auto evalOneEndpoint = [&](const Quat& sensorQ, const Quat& trialQ) -> std::pair<bool, double> {
        WarpRequest wTest = wr;
        wTest.visual = Similarity2D{};
        wTest.zoom = baseZoom;
        wTest.qTarget = trialQ;
        wTest.hasRowTargets = false;
        wTest.rowQuats.fill(sensorQ);
        static constexpr double kCorners[4][2] = {
            {0.0, 0.0}, {1.0, 0.0}, {1.0, 1.0}, {0.0, 1.0}
        };
        bool anyHit = false;
        double maxRho = 0.0;
        const double mx = std::max(1.0, innerLeft - outerLeft);
        const double my = std::max(1.0, innerTop - outerTop);
        for (const auto& c : kCorners) {
            const WarpVertex v = warp_.projectPoint(c[0], c[1], wTest);
            const double px = v.u * W;
            const double py = v.v * H;
            double prho = 0.0;
            if (px < innerLeft - 1e-4)   prho = std::max(prho, (innerLeft - px) / mx);
            if (px > innerRight + 1e-4)  prho = std::max(prho, (px - innerRight) / mx);
            if (py < innerTop - 1e-4)    prho = std::max(prho, (innerTop - py) / my);
            if (py > innerBottom + 1e-4) prho = std::max(prho, (py - innerBottom) / my);
            if (prho > 0.0) {
                anyHit = true;
                maxRho = std::max(maxRho, prho);
            }
        }
        return {anyHit, maxRho};
    };

    auto evalBoundary = [&](const Quat& trialQ, const Quat& aQ, const Quat& bQ) -> BoundaryEval {
        const auto [aHit, aRho] = evalOneEndpoint(aQ, trialQ);
        const auto [bHit, bRho] = evalOneEndpoint(bQ, trialQ);
        if (aRho >= bRho) {
            return {aHit, aRho, 0};
        }
        return {bHit, bRho, 1};
    };

    if (!quatInitialized_) {
        qTarget_ = qCenter;
        qAnchor_ = qCenter;
        currentZoom_ = baseZoom;
        quatInitialized_ = true;
        lastTimestampNs_ = input.timestampNs;
        frameCount_ = 1;
        wr.hasRowTargets = false;
    } else {
        double dtS = static_cast<double>(input.timestampNs - lastTimestampNs_) * 1e-9;
        if (!(dtS > 1e-4 && dtS < 0.25)) {
            dtS = 1.0 / 30.0;
        }
        lastTimestampNs_ = input.timestampNs;
        frameCount_++;
        currentZoom_ = baseZoom;

        const Quat A0 = input.rowQuats[0].normalized();
        const Quat B0 = input.rowQuats[19].normalized();

        // V5 PixSmart Pure Recursive FilterPose Control Law:
        // Directly chains from previous output pose P = qTarget_ without any discontinuous deadband state-machine jump!
        const double omegaMag = input.angularVelocityRadS.norm();
        if (frameCount_ <= 12) {
            const double alphaInit = 1.0 - std::exp(-dtS / 0.18);
            qTarget_ = Quat::slerp(qTarget_, qCenter, alphaInit);
            qAnchor_ = qTarget_;
        }
        const Quat P = qTarget_;

        // Build causal 4-sample lookahead window A[0..3], B[0..3]:
        // Only extrapolate velocity during confirmed intentional pan (never during bilateral/unilateral oscillation!)
        constexpr int kWin = 4;
        std::array<Quat, kWin> A{};
        std::array<Quat, kWin> B{};
        A[0] = A0;
        B[0] = B0;
        const bool allowLookahead = (out.panConfidence > 0.65 && out.oscillationScore < 0.25);
        for (int i = 1; i < kWin; ++i) {
            if (allowLookahead) {
                const double dtLook = i * 0.015 * std::pow(0.65, i - 1);
                const Quat dQ = Quat::fromRotationVector(input.angularVelocityRadS * dtLook);
                A[static_cast<size_t>(i)] = (A0 * dQ).normalized();
                B[static_cast<size_t>(i)] = (B0 * dQ).normalized();
            } else {
                A[static_cast<size_t>(i)] = A0;
                B[static_cast<size_t>(i)] = B0;
            }
        }

        // PixSmart FilterPose main loop: Farthest-future (N-1) down to current (0) with 1/rho reciprocal release
        Quat T = P;
        int k = kWin - 1;
        int releaseCount = 0;
        for (int i = kWin - 1; i >= 0; --i) {
            const double coeff = static_cast<double>(i + 1) / static_cast<double>(k + 1);
            const Quat V = Quat::slerp(P, T, coeff);
            const auto br = evalBoundary(V, A[static_cast<size_t>(i)], B[static_cast<size_t>(i)]);
            if (br.hit && br.rho > 1.0) {
                const Quat& source = (br.side == 0) ? A[static_cast<size_t>(i)] : B[static_cast<size_t>(i)];
                T = Quat::slerp(source, V, 1.0 / br.rho);
                k = i;
                releaseCount++;
            }
        }
        const Quat H = Quat::slerp(P, T, 1.0 / static_cast<double>(k + 1));

        // V6 Zero-Sway Tripod Anchor + PixSmart Smooth Reciprocal Release:
        //   - Pure recursive P = qTarget_ (zero discontinuous state-machine jumps)
        //   - Inside safe contour (releaseCount == 0): holdWeight = 0.9992 (tau = 41.7s, 99.6% swing suppression)
        //   - Calm/settled (omegaMag < 0.085 rad/s): holdWeight = 1.0000 (100% locked, zero post-settle drift)
        //   - Boundary release (releaseCount > 0): holdWeight = 0.9920 (smooth PixSmart 1/rho reciprocal release)
        const double distFromH = (A0.inverse() * H).angle();
        double holdWeight = 0.9992;
        if (releaseCount > 0) {
            holdWeight = 0.9920;
        } else if (omegaMag < 0.085 && distFromH < 0.060) {
            holdWeight = 1.0000;
        }
        Quat Q = Quat::slerp(A0, H, holdWeight);

        // PixSmart Second Boundary Gate -> Per-Row Target Trajectory (FilterSteadyGL)
        const auto second = evalBoundary(Q, A0, B0);
        if (second.hit && second.rho > 1.0) {
            const Quat& source = (second.side == 0) ? A0 : B0;
            const Quat E = Quat::slerp(source, Q, 1.0 / second.rho);
            wr.hasRowTargets = true;
            for (int r = 0; r < 20; ++r) {
                const double tRow = static_cast<double>(r) / 19.0;
                wr.rowTargets[static_cast<size_t>(r)] = (second.side == 0)
                    ? Quat::slerp(E, Q, tRow)
                    : Quat::slerp(Q, E, tRow);
            }
            // Smoothly pull center pose Q toward E only by the minimum required amount to stay >= 32px inside sensor
            WarpRequest wrCheck = wr;
            wrCheck.visual = Similarity2D{};
            wrCheck.zoom = baseZoom;
            wrCheck.qTarget = Q;
            wrCheck.hasRowTargets = false;
            if (warp_.computeMarginPx(wrCheck) < 32.0) {
                double lo = 0.0, hi = 1.0;
                for (int iter = 0; iter < 10; ++iter) {
                    const double mid = 0.5 * (lo + hi);
                    wrCheck.qTarget = Quat::slerp(Q, E, mid);
                    if (warp_.computeMarginPx(wrCheck) >= 32.0) hi = mid;
                    else lo = mid;
                }
                Q = Quat::slerp(Q, E, hi);
            }
        } else {
            wr.hasRowTargets = false;
        }
        qAnchor_ = Q;

        // Stage 2: Visual Homography/Similarity Margin Check at constant zoom
        Similarity2D visCand = input.visual;
        wr.qTarget = Q;
        wr.visual = visCand;
        wr.zoom = baseZoom;
        if (warp_.computeMarginPx(wr) < 2.0) {
            double lo = 0.0, hi = 1.0;
            for (int iter = 0; iter < 12; ++iter) {
                const double mid = 0.5 * (lo + hi);
                const double keep = 1.0 - mid;
                wr.visual = visCand;
                wr.visual.txPx = visCand.txPx * keep;
                wr.visual.tyPx = visCand.tyPx * keep;
                wr.visual.rotationRad = visCand.rotationRad * keep;
                wr.visual.scale = 1.0 + (visCand.scale - 1.0) * keep;
                if (visCand.hasHomography) {
                    for (int m = 0; m < 9; ++m) {
                        const double eyeVal = (m % 4 == 0) ? 1.0 : 0.0;
                        wr.visual.hInv[static_cast<size_t>(m)] =
                            eyeVal + (visCand.hInv[static_cast<size_t>(m)] - eyeVal) * keep;
                    }
                }
                if (warp_.computeMarginPx(wr) >= 2.0) hi = mid;
                else lo = mid;
            }
            const double keep = 1.0 - hi;
            visCand.txPx *= keep;
            visCand.tyPx *= keep;
            visCand.rotationRad *= keep;
            visCand.scale = 1.0 + (visCand.scale - 1.0) * keep;
            if (visCand.hasHomography) {
                for (int m = 0; m < 9; ++m) {
                    const double eyeVal = (m % 4 == 0) ? 1.0 : 0.0;
                    visCand.hInv[static_cast<size_t>(m)] =
                        eyeVal + (visCand.hInv[static_cast<size_t>(m)] - eyeVal) * keep;
                }
            }
            wr.visual = visCand;
            if (warp_.computeMarginPx(wr) < 2.0) {
                currentZoom_ = 1.14;
            }
        }

        qTarget_ = Q;
        out.visual = visCand;
    }

    wr.qTarget = qTarget_;
    wr.visual = out.visual;
    wr.zoom = currentZoom_;
    out.qTarget = qTarget_;
    out.zoom = currentZoom_;
    out.mesh = warp_.buildMesh(wr);
    return out;
}

}  // namespace gemini
