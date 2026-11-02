#include <chrono>
#include <cmath>
#include <iostream>
#include <string>
#include <vector>

#include "common/ring_buffer.hpp"
#include "control/constrained_mpc.h"
#include "control/intent_scheduler.h"
#include "state/motion_state_est.h"
#include "visual/async_visual_worker.h"
#include "warp/rs19_one_warp.h"

using namespace gemini;

namespace {

bool runMpcSmoke() {
    MpcConfig cfg;
    cfg.dtS = 1.0 / 30.0;
    cfg.horizon = 10;
    cfg.iterations = 15;
    ConstrainedMpc1D mpc(cfg);

    MpcRequest req;
    req.initial = {0.0, 0.0, 0.0};
    req.referenceAngle.assign(cfg.horizon, 0.0);
    req.referenceVelocity.assign(cfg.horizon, 0.0);
    req.rawAngle.assign(cfg.horizon, 0.08);
    req.cropBudgetRad = 0.10;

    const auto t0 = std::chrono::high_resolution_clock::now();
    MpcResult last;
    constexpr int kRuns = 500;
    for (int i = 0; i < kRuns; ++i) last = mpc.solve(req);
    const auto t1 = std::chrono::high_resolution_clock::now();
    const double us = std::chrono::duration<double, std::micro>(t1 - t0).count() / kRuns;

    std::cout << "MPC avg solve: " << us << " us\n";
    std::cout << "MPC next angle: " << last.next.angle
              << " maxViolation: " << last.maxConstraintViolationRad << "\n";

    return last.valid && std::isfinite(us);
}

bool runPipelineSmoke() {
    MotionStateEstimator estimator;
    const int64_t base = 1000000000LL;
    for (int i = 0; i < 80; ++i) {
        estimator.pushGyro({base + i * 2500000LL,
            {0.01 * std::sin(i * 0.1), 0.03, 0.005}});
    }

    VisualResidual vr;
    vr.timestampNs = base + 120000000LL;
    vr.similarity = {0.4, -0.2, 0.001, 1.0005, 0.8, 35};
    estimator.pushVisual(vr);
    const MotionState ms = estimator.evaluate(base + 150000000LL);

    IntentScheduler1D scheduler;
    auto sched = scheduler.update(base + 150000000LL, 0.02,
                                  ms.raw.yaw.velocity,
                                  ms.raw.yaw.acceleration);

    std::vector<FeatureMatch> matches;
    for (int i = 0; i < 20; ++i) {
        FeatureMatch m;
        m.prevPx = {20.0 + (i % 5) * 25.0, 15.0 + (i / 5) * 20.0};
        m.currPx = {m.prevPx.x + 1.2, m.prevPx.y - 0.6};
        m.forwardBackwardErrorPx = 0.2;
        matches.push_back(m);
    }

    AsyncVisualWorkerMath visual;
    Similarity2D sim = visual.estimateSimilarity(matches, 0.010, 0.1);

    Rs19OneWarp warp;
    WarpRequest wr;
    wr.globalCorrectionRad = {0.002, -0.003, 0.001};
    wr.visual = sim;
    wr.zoom = 1.08;
    auto mesh = warp.buildMesh(wr);

    std::cout << "Visual inliers=" << sim.inliers
              << " trust=" << sim.trust
              << " tx=" << sim.txPx
              << " ty=" << sim.tyPx << "\n";
    std::cout << "Scheduler pan=" << sched.panConfidence
              << " oscillation=" << sched.oscillationScore << "\n";
    std::cout << "Mesh vertices=" << mesh.size() << "\n";

    return sim.inliers >= 15 && sim.trust > 0.25 && mesh.size() == 660;
}

}  // namespace

int main(int argc, char** argv) {
    const bool selfTest = argc > 1 && std::string(argv[1]) == "--self-test";
    const bool okMpc = runMpcSmoke();
    const bool okPipeline = runPipelineSmoke();
    const bool ok = okMpc && okPipeline;

    std::cout << (ok ? "GEMINI_EIS_SMOKE_PASS" : "GEMINI_EIS_SMOKE_FAIL") << "\n";
    return (selfTest && !ok) ? 2 : (ok ? 0 : 1);
}
