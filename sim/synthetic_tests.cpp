#include <cmath>
#include <iostream>
#include "control/constrained_mpc.h"
#include "core/gemini_eis_core.h"

using namespace gemini;

static bool testBoundary() {
    MpcConfig cfg;
    cfg.horizon = 10;
    cfg.iterations = 20;
    ConstrainedMpc1D mpc(cfg);
    MpcRequest req;
    req.initial = {0.0, 0.0, 0.0};
    req.referenceAngle.assign(cfg.horizon, 0.0);
    req.referenceVelocity.assign(cfg.horizon, 0.0);
    req.rawAngle.assign(cfg.horizon, 0.095);
    req.cropBudgetRad = 0.10;
    const auto r = mpc.solve(req);
    std::cout << "boundary violation=" << r.maxConstraintViolationRad << "\n";
    return r.valid && r.maxConstraintViolationRad < 1e-6;
}

static bool testStepAndReversal() {
    GeminiEisCore core;
    const int64_t dt = 33333333LL;
    int64_t t = 1000000000LL;
    double maxCorrection = 0.0;
    double prevTarget = 0.0;
    double maxTargetStep = 0.0;

    for (int i = 0; i < 120; ++i) {
        double raw = 0.0;
        if (i >= 20 && i < 70) raw = 0.07;
        if (i >= 70) raw = -0.03;
        FrameInput in;
        in.timestampNs = t;
        in.rawAngleRad.y = raw;
        in.angularVelocityRadS.y = (i == 20 ? 0.8 : (i == 70 ? -1.0 : 0.0));
        in.zoom = 1.08;
        const auto out = core.processFrame(in);
        maxCorrection = std::max(maxCorrection, std::abs(out.correctionRad.y));
        maxTargetStep = std::max(maxTargetStep, std::abs(out.targetAngleRad.y - prevTarget));
        prevTarget = out.targetAngleRad.y;
        if (out.mesh.size() != 660) return false;
        t += dt;
    }

    std::cout << "step maxCorrection=" << maxCorrection
              << " maxTargetStep=" << maxTargetStep << "\n";
    return maxCorrection <= 0.11 && maxTargetStep < 0.02;
}

int main() {
    const bool a = testBoundary();
    const bool b = testStepAndReversal();
    const bool ok = a && b;
    std::cout << (ok ? "GEMINI_EIS_SYNTH_PASS" : "GEMINI_EIS_SYNTH_FAIL") << "\n";
    return ok ? 0 : 2;
}
