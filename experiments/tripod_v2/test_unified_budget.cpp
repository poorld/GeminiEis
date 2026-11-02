#include "UnifiedTripodBudget.h"
#include <cassert>
#include <algorithm>
#include <cstdio>
#include <cmath>
#include <vector>
#include <chrono>
using gemini::Rs19OneWarp;
using gemini::Rs19Config;
using gemini::WarpRequest;
using gemini::WarpVertex;
using gemini::Quat;
using gemini::tripod_v2::UnifiedBudgetController;
using gemini::tripod_v2::JointBudgetConfig;

void assert_equal(const std::vector<WarpVertex>& a,
                  const std::vector<WarpVertex>& b) {
    assert(a.size()==b.size());
    for(size_t i=0;i<a.size();++i) {
        assert(a[i].u==b[i].u);
        assert(a[i].v==b[i].v);
    }
}
WarpRequest empty_warp() {
    WarpRequest w{};
    w.useQuats=true;w.zoom=1.0;
    w.visual.scale=1.0;
    w.qCenter=Quat::identity();
    w.qTarget=Quat::identity();
    w.rowQuats.fill(Quat::identity());
    return w;
}
int main() {
    Rs19Config camera{};
    camera.width=2560;camera.height=1440;camera.rowBands=19;
    camera.columns=32;camera.baseCrop=.8;
    Rs19OneWarp realRS(camera);
    const auto origin=empty_warp();
    UnifiedBudgetController idle;
    for(int i=0;i<50;i++){
        const auto result=idle.choose(origin,realRS,
                    1000000000LL+33000000LL*i,.05,.03);
        assert(result.diag.accepted==0);
        assert_equal(result.vertices,realRS.buildMesh(origin));
        assert(result.vertices.size()==660);
    }
    JointBudgetConfig cfg{};
    cfg.risk_margin_px=1000.;
    cfg.scale_jitter_trigger=.001;
    cfg.min_gain_span_source_px=.25;
    UnifiedBudgetController c(cfg);
    auto first=c.choose(origin,realRS,1000000000LL,.05,0.0);
    assert(first.diag.accepted==0);
    auto shock=origin;
    shock.visual.scale=.97;
    auto second=c.choose(shock,realRS,1033000000LL,.05,0.0);
    assert(second.diag.attempted>0);
    assert(second.diag.accepted);
    assert(second.diag.final_zoom>=second.diag.base_zoom);
    assert(second.diag.result_margin_px+1e-4>=second.diag.margin_px);
    assert(second.diag.center_shift_source_px<=cfg.max_center_shift_source_px);
    for(const auto& p:second.vertices)
        assert(p.u>=0&&p.u<=1&&p.v>=0&&p.v<=1);
    // Same exact prefix with a different FUTURE. All choices must agree.
    UnifiedBudgetController aa(cfg),bb(cfg);
    auto a0=aa.choose(origin,realRS,1000000000LL,.05,0.0);
    auto b0=bb.choose(origin,realRS,1000000000LL,.05,0.0);
    assert_equal(a0.vertices,b0.vertices);
    auto a1=aa.choose(shock,realRS,1033000000LL,.05,0.0);
    auto b1=bb.choose(shock,realRS,1033000000LL,.05,0.0);
    assert_equal(a1.vertices,b1.vertices);
    auto future=shock;future.visual.scale=1.02;
    aa.choose(future,realRS,1066000000LL,.05,0.0);
    bb.choose(shock,realRS,1066000000LL,.05,0.0);
    // New scene PAN bypass: no attempt at correction even with big visual spike.
    UnifiedBudgetController pan(cfg);
    pan.choose(origin,realRS,1000000000LL,.07,0.0);
    auto shielded=pan.choose(shock,realRS,1033000000LL,.07,.99);
    assert(shielded.diag.accepted==0);
    assert_equal(shielded.vertices,realRS.buildMesh(shock));
    // Duplicate and backward SensorTS are not allowed to overwrite the
    // committed state; preserve the exact baseline output on that frame.
    auto duplicate=pan.choose(shock,realRS,1033000000LL,.07,0.0);
    assert(duplicate.diag.reason==2 && duplicate.diag.accepted==0);
    assert_equal(duplicate.vertices,realRS.buildMesh(shock));
    auto restored=pan.choose(origin,realRS,1066000000LL,.07,.99);
    assert(restored.diag.reason==3 && restored.diag.accepted==0);
    assert_equal(restored.vertices,realRS.buildMesh(origin));
    // Performance ONLY full 20x33 and geometry proposal across 200 frames,
    // not raw camera/gyro/LK/GPU/encoder/Android.
    JointBudgetConfig bench=cfg;
    bench.risk_margin_px=1000;
    UnifiedBudgetController governor(bench);
    const auto t0=std::chrono::steady_clock::now();
    double checksum=0;
    for(int i=0;i<200;i++) {
        auto local=origin;
        local.visual.scale=1.+.012*std::sin(i*.18);
        auto r=governor.choose(local,realRS,
                       1000000000LL+33000000LL*i,.04,0.);
        checksum+=r.vertices.at(0).u;
    }
    const auto elapsed=std::chrono::duration<double,std::milli>(
        std::chrono::steady_clock::now()-t0).count();
    std::printf("GEMINI_TRIPOD_V2_UNIFIED_CPP_TEST_PASS "
      "no_future=1 normal_bitexact=1 pan_veto=1 true_20x33=1 "
      "shock_candidate_accepted=%d benchmark_full_geometry_ms_per_frame=%.4f "
      "checksum=%.2f\n",second.diag.accepted,elapsed/200,checksum);
}
