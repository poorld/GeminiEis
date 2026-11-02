#pragma once
// Gemini Tripod V2: unified within-frame causal geometric budget governor.
// Works on the same physical RS19 20x33 projection as the final renderer.
// Does not consume future camera/visual samples, untrusted LK, or 8DoF meshes.
// Upstream OFFLINE oracle/visual matrices remain NONCAUSAL until replaced.
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <vector>
#include "common/types.hpp"
#include "warp/rs19_one_warp.h"

namespace gemini::tripod_v2 {
struct JointBudgetConfig {
    double safe_margin_px=2.0;           // Never relax the existing RS physical gate
    double risk_margin_px=78.0;          // Enter only near physical RS19 border
    double emergency_margin_px=15.0;
    double scale_jitter_trigger=.006;    // Relative one-frame scale change
    double scale_step_limit_log=.0015;
    double zoom_step_limit_log=.008;     // Budget stage cannot jump wildly
    double min_gain_span_source_px=2.0;  // No-op unless optical edge benefit
    double max_center_shift_source_px=.18;
    double max_extra_zoom=1.14;
    double pan_veto_confidence=.65;
    int max_candidates=6;
};
enum class DecisionReason : int {
    FirstFrame=1, BadTimestamp=2, Pan=3, Normal=4,
    RiskNoBenefit=5, SafetyVeto=6, Accepted=7
};
struct Diagnostic {
    int reason=0, attempted=0, accepted=0, candidate_count=0;
    double margin_px=0, span_source_px=0;
    double result_margin_px=0, result_span_source_px=0;
    double base_zoom=1., final_zoom=1.;
    double base_scale=1., final_scale=1.;
    double yaw_rad_s=0, center_shift_source_px=0;
    std::int64_t timestamp_ns=0;
};
struct Choice {
    WarpRequest warp{};
    std::vector<WarpVertex> vertices;
    Diagnostic diag{};
};

class UnifiedBudgetController {
public:
    explicit UnifiedBudgetController(JointBudgetConfig c={}) noexcept:cfg_(c){}
    void reset() noexcept {
        ready_=false;last_ts_=0;last_margin_=1e9;last_scale_=1.;last_zoom_=1.;
        last_yaw_=0;last_sign_=0;had_intervention_=false;previous_.clear();
    }
    // Calling this every frame is required. Must be the ONLY output warp
    // selection for this GeminiEisCore instance. Baseline is computed inside
    // the SAME core; if rejected the output remains numerically bit-exact.
    Choice choose(const WarpRequest& baseline,
                  const Rs19OneWarp& physics,
                  const std::int64_t ts,
                  const double yaw_rate_rad_s,
                  const double pan_confidence) {
        Choice res{};
        res.warp=baseline;
        res.vertices=physics.buildMesh(baseline);
        const auto baseline_geometry=measure(res.vertices);
        auto& d=res.diag;
        d.timestamp_ns=ts;d.base_zoom=baseline.zoom;d.final_zoom=baseline.zoom;
        d.base_scale=baseline.visual.scale;
        d.final_scale=baseline.visual.scale;
        d.yaw_rad_s=yaw_rate_rad_s;
        d.margin_px=baseline_geometry.min_margin;
        d.result_margin_px=baseline_geometry.min_margin;
        d.span_source_px=baseline_geometry.span;
        d.result_span_source_px=baseline_geometry.span;

        auto commit=[&](const DecisionReason why) {
            d.reason=int(why);
            previous_=res.vertices;
            last_margin_=d.result_margin_px;
            last_scale_=d.final_scale;
            last_zoom_=d.final_zoom;
            last_ts_=ts;
            last_yaw_=yaw_rate_rad_s;
            if(std::abs(yaw_rate_rad_s)>.03)
                last_sign_=yaw_rate_rad_s>0?1:-1;
            ready_=true;
            had_intervention_=d.accepted!=0;
            return res;
        };
        if(!ready_) return commit(DecisionReason::FirstFrame);
        if(ts<=last_ts_) {
            // Do not corrupt the committed temporal state on repeated or
            // backward SensorTS. Emit unchanged baseline for this frame.
            d.reason=int(DecisionReason::BadTimestamp);
            return res;
        }
        if(ts-last_ts_>250000000) {
            // Long camera gaps end the half-cycle and visual budget episode.
            reset();
            return commit(DecisionReason::BadTimestamp);
        }
        if(pan_confidence>cfg_.pan_veto_confidence)
            return commit(DecisionReason::Pan);
        const bool reversing=std::abs(yaw_rate_rad_s)>.025
            && last_sign_ && ((yaw_rate_rad_s>0?1:-1)!=last_sign_);
        const double scale_delta=std::abs(std::log(
            std::max(.1,baseline.visual.scale)/std::max(.1,last_scale_)));
        const bool near_margin=baseline_geometry.min_margin<cfg_.risk_margin_px;
        const bool high_pressure=baseline_geometry.min_margin<cfg_.emergency_margin_px
                             || (near_margin && last_margin_-baseline_geometry.min_margin>26.)
                             || (near_margin && scale_delta>cfg_.scale_jitter_trigger)
                             || (near_margin && reversing)
                             || (had_intervention_ && baseline_geometry.min_margin<120.);
        if(!high_pressure)
            return commit(DecisionReason::Normal);

        // Perimeter RS19 is tested by the original core; we independently
        // check its COMPLETE 20x33 physical mesh and all mid-cell edges.
        // We do not attempt to make unsafe margins "safe" by changing Crop
        // or the physical max Zoom.
        const auto prev_geometry=measure(previous_);
        const double baseline_step=std::abs(
            baseline_geometry.span-prev_geometry.span);
        double winner_score=baseline_step;
        bool geometry_veto=false;

        const double log_input=std::log(std::max(.1,baseline.visual.scale));
        const double log_prev=std::log(std::max(.1,last_scale_));
        const double clip_log=log_prev+std::clamp(log_input-log_prev,
             -cfg_.scale_step_limit_log,cfg_.scale_step_limit_log);
        const double proposed_scale=std::exp(clip_log);
        // Candidate combinations: constant original, only bounded Scale,
        // 1–2 slow Zoom options, and their combination. No forward lookahead.
        const double wz=std::min(cfg_.max_extra_zoom,
              last_zoom_*std::exp(cfg_.zoom_step_limit_log));
        const double zw=std::min(cfg_.max_extra_zoom,
              baseline.zoom*std::exp(cfg_.zoom_step_limit_log));
        const double zs[3]={baseline.zoom,
            std::max(baseline.zoom,wz),std::max(baseline.zoom,zw)};
        for(int iz=0;iz<3;++iz) for(int is=0;is<2;++is) {
            if(iz==0 && is==0) continue;
            if(d.attempted>=cfg_.max_candidates) break;
            WarpRequest trial=baseline;
            trial.zoom=zs[iz];
            if(is==1) {
                trial.visual.scale=proposed_scale;
                // Preserve exact optical center at input of physical projection:
                // xCenter = -tx / scale, yCenter = -ty / scale.
                const double fac=proposed_scale/baseline.visual.scale;
                trial.visual.txPx*=fac;
                trial.visual.tyPx*=fac;
            }
            if(std::abs(trial.zoom-baseline.zoom)<1e-10 &&
               std::abs(trial.visual.scale-baseline.visual.scale)<1e-10)
                continue;
            ++d.attempted;
            if(physics.computeMarginPx(trial)<cfg_.safe_margin_px) {
                geometry_veto=true;continue;
            }
            std::vector<WarpVertex> mesh=physics.buildMesh(trial);
            auto g=measure(mesh);
            ++d.candidate_count;
            if(g.min_margin<cfg_.safe_margin_px || !g.inside) {
                geometry_veto=true;continue;
            }
            // Never make an already safe baseline UV margin worse; the
            // governing variable must not introduce any geometric debt.
            if(g.min_margin+1e-3<baseline_geometry.min_margin) {
                geometry_veto=true;continue;
            }
            const double center_shift=centerDistance(mesh,res.vertices);
            if(center_shift>cfg_.max_center_shift_source_px) {
                geometry_veto=true;continue;
            }
            // Frame-local causal score: prefer preserving previous edge
            // geometry, while controlling zoom/scale rate and center lock.
            // Hard safety gates above always override the objective.
            double score=std::abs(g.span-prev_geometry.span)
                +.80*std::abs(center_shift)
                +350.*std::abs(std::log(trial.zoom/baseline.zoom))
                +130.*std::abs(std::log(trial.visual.scale/baseline.visual.scale));
            if(score+cfg_.min_gain_span_source_px<winner_score) {
                winner_score=score;
                res.warp=trial;
                res.vertices=std::move(mesh);
                d.accepted=1;
                d.center_shift_source_px=center_shift;
                d.final_zoom=trial.zoom;
                d.final_scale=trial.visual.scale;
                d.result_span_source_px=g.span;
                d.result_margin_px=g.min_margin;
            }
        }
        if(!d.accepted)
            return commit(geometry_veto?DecisionReason::SafetyVeto:
                                             DecisionReason::RiskNoBenefit);
        return commit(DecisionReason::Accepted);
    }
private:
    struct Geometry {
        bool inside=false;
        double min_margin=-1e9;
        double span=0;
    };
    static Geometry measure(const std::vector<WarpVertex>& v) noexcept {
        Geometry g{};
        if(v.size()!=20u*33u)return g;
        g.inside=true;g.min_margin=1e10;
        for(size_t i=0;i<v.size();++i) {
            const auto& p=v[i];
            if(!std::isfinite(p.u)||!std::isfinite(p.v))return Geometry{};
            g.min_margin=std::min(g.min_margin,
                 std::min({p.u*2560.-.5,(1.-p.u)*2560.-.5,
                           p.v*1440.-.5,(1.-p.v)*1440.-.5}));
            if(p.u<0||p.u>1||p.v<0||p.v>1)g.inside=false;
        }
        const auto& l=v[9*33];
        const auto& lb=v[10*33];
        const auto& r=v[9*33+32];
        const auto& rb=v[10*33+32];
        g.span=(r.u+rb.u-l.u-lb.u)*1280.;
        return g;
    }
    static double centerDistance(const std::vector<WarpVertex>& a,
                                 const std::vector<WarpVertex>& b) noexcept {
        if(a.size()!=660||b.size()!=660)return 1e10;
        constexpr size_t i=9*33+16,j=10*33+16;
        const double dx=((a[i].u+a[j].u)-(b[i].u+b[j].u))*1280.;
        const double dy=((a[i].v+a[j].v)-(b[i].v+b[j].v))*720.;
        return std::hypot(dx,dy);
    }
    JointBudgetConfig cfg_{};
    bool ready_=false,had_intervention_=false;
    std::int64_t last_ts_=0;
    double last_margin_=1e9,last_scale_=1,last_zoom_=1,last_yaw_=0;
    int last_sign_=0;
    std::vector<WarpVertex> previous_;
};
} // namespace gemini::tripod_v2
