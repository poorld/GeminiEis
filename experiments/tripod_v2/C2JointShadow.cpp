#include "C2JointShadow.h"
#include "warp/rs19_one_warp.h"
#include "common/types.hpp"
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <vector>
#include <new>

namespace {
using namespace gemini;
constexpr double KWidth=2560., KHeight=1440.;
constexpr double kFx=1202.3,kMaxZoom=1.14,kSafe=2.0,kMaxVisualTrust=1.0;
using Clock=std::chrono::steady_clock;
struct Feature {double dx=0,dy=0,theta=0,lnscale=0,trust=0;bool valid=false;};
struct Physical {
  bool valid=false;
  double margin=-1.e12, span=0, cx=0,cy=0;
};
Quat unmarshal(const double* a) {
  return Quat{a[0],a[1],a[2],a[3]}.normalized();
}
Quat safeMix(Quat a,Quat b,double fraction){
  return Quat::slerp(a,b,std::clamp(fraction,0.,1.));
}
double attitudeDistance(Quat a,Quat b) {
  const auto r=(a.inverse()*b).normalized();
  return r.angle();
}
Physical inspect(const std::vector<WarpVertex>& mesh){
  Physical p{};
  if(mesh.size()!=660)return p;
  p.valid=true;p.margin=1e12;
  for(const auto& v:mesh){
    if(!std::isfinite(v.u)||!std::isfinite(v.v)){
      p.valid=false;return p;
    }
    p.margin=std::min(p.margin,std::min({
        v.u*KWidth-.5,(1.-v.u)*KWidth-.5,
        v.v*KHeight-.5,(1.-v.v)*KHeight-.5}));
    if(v.u<.5/KWidth||v.u>1.-.5/KWidth ||
       v.v<.5/KHeight||v.v>1.-.5/KHeight)p.valid=false;
  }
  const auto& L=mesh[9*33];
  const auto& L2=mesh[10*33];
  const auto& R=mesh[9*33+32];
  const auto& R2=mesh[10*33+32];
  const auto& C=mesh[9*33+16];
  const auto& C2=mesh[10*33+16];
  p.span=((R.u+R2.u)-(L.u+L2.u))*KWidth*.5;
  p.cx=(C.u+C2.u)*KWidth*.5;
  p.cy=(C.v+C2.v)*KHeight*.5;
  return p;
}
struct Candidate {
  WarpRequest warp{};
  std::vector<WarpVertex> mesh;
  Physical geometry{};
  double score=1e100, step=0, residual_feature_px=0;
  int usedVisual=0;
  bool valid=false;
};
}

struct C2Owner {
  explicit C2Owner(const int visual):mode(visual){
    Rs19Config config;
    config.width=2560; config.height=1440;
    config.columns=32;config.rowBands=19;
    config.fxPx=1202.3;config.fyPx=1208.5;
    config.cxPx=1280;config.cyPx=720;
    config.baseCrop=.80;
    warp=Rs19OneWarp(config);
  }
  Rs19OneWarp warp;
  int mode=0;
  bool initialized=false;
  int64_t lastSource=-1,lastTS=0;
  Quat virtualQ{};
  Similarity2D visual{};
  double zoom=1.,lastVel=0.,lastSpan=0.;
  int lastYawSign=0;
  std::vector<WarpVertex> previousMesh;
  double dxPath320=0,dyPath320=0;
#if defined(C3_GLOBAL_PATH)
  // Three temporally distinct state variables under the sole target owner.
  // All are updated ONLY after physically valid RS19 atomic commit.
  double yaw_fast=0.,yaw_slow=0.,previous_target_step_source_px=0.;
  double common_rotation_path=0.,common_dx_path=0.,common_dy_path=0.;
  int signed_halfcycle=0,halfcycle_frames=0,reversal_events=0;
#endif
};

C2Owner* c2_shadow_create(int mode){
  if(mode!=0&&mode!=1)return nullptr;
  try{return new C2Owner(mode);}catch(...){return nullptr;}
}
int c2_shadow_step(C2Owner* state,const C2Input* input,C2Result* output){
  using namespace gemini;
  if(!state||!input||!output)return -1;
  const auto t0=Clock::now();
  *output=C2Result{};
  output->code=-1;
  output->source_id=input->source_id;
  output->sensor_ts_ns=input->sensor_ts_ns;
  if(input->source_id<0||input->sensor_ts_ns<=0)return -2;
  if(state->initialized &&
      (input->source_id!=state->lastSource+1 ||
       input->sensor_ts_ns<=state->lastTS))return -3;
  for(const double x:input->q_center)if(!std::isfinite(x))return -4;
  for(const double x:input->q_rows20)if(!std::isfinite(x))return -4;
  const auto center=unmarshal(input->q_center);
  WarpRequest physical{};
  physical.useQuats=true;
  physical.qCenter=center;
  for(int i=0;i<20;++i)
     physical.rowQuats[static_cast<size_t>(i)]=
         unmarshal(input->q_rows20+4*i);
  const bool fresh=!state->initialized;
  Quat previousQ=fresh?center:state->virtualQ;
  Similarity2D lastVisual=fresh?Similarity2D{}:state->visual;
  double previousZoom=fresh?1.:state->zoom;
  if(fresh){state->dxPath320=state->dyPath320=0;}
#if defined(C3_GLOBAL_PATH)
  // Scene-independent causal yaw halfcycle and velocity field. This is a
  // one-step kinematic prediction from current/past gyro, NEVER future gyro.
  const double dt=fresh?1./29.605:std::clamp(
       (input->sensor_ts_ns-state->lastTS)*1.e-9,.002,.12);
  const double fastAlpha=1.-std::exp(-dt/.055);
  const double slowAlpha=1.-std::exp(-dt/.23);
  const double nowYawFast=state->yaw_fast+
       fastAlpha*(input->yaw_rad_s-state->yaw_fast);
  const double nowYawSlow=state->yaw_slow+
       slowAlpha*(input->yaw_rad_s-state->yaw_slow);
  const int yawSign=nowYawFast>.075?1:nowYawFast<-.075?-1:0;
  const bool reversal=!fresh && state->signed_halfcycle!=0 &&
     yawSign!=0 && yawSign!=state->signed_halfcycle &&
     state->halfcycle_frames>=3;
  const int nowHalfcycleFrames=(yawSign==0?0:
      (yawSign==state->signed_halfcycle?state->halfcycle_frames+1:1));
  const double predictedYawRate=std::clamp(
       .70*nowYawFast+.30*input->yaw_rad_s,-1.8,1.8);
  (void)predictedYawRate; // no-boundary ablation still tracks yaw phase
#if defined(C3_FORECAST_BOUNDARY) || defined(C3_FORECAST_FOLLOW_ONLY)
  // Horizon is algorithmic prediction, NOT an additional frame of delay.
  // Immediately after a reversal use a shorter horizon to avoid extending
  // the previous halfcycle into the new direction.
  const double forecastSeconds=reversal?.030:
        (nowHalfcycleFrames>=4?.070:.045);
  const Quat qForecast=Quat::fromRotationVector(
       Vec3{0.,predictedYawRate*forecastSeconds,0.});
  WarpRequest futurePhysical=physical;
  futurePhysical.qCenter=(physical.qCenter*qForecast).normalized();
  for(auto& q:futurePhysical.rowQuats)
       q=(q*qForecast).normalized();
#endif
#endif
  Feature visual{};
  visual.trust=std::clamp(input->visual_trust,0.,kMaxVisualTrust);
  visual.valid=state->mode==1 && input->visual_accepted==1 &&
      input->visual_inliers>=25 && visual.trust>=.30 &&
      std::isfinite(input->visual_dx_320px) &&
      std::isfinite(input->visual_dy_320px) &&
      std::isfinite(input->visual_rotation_rad) &&
      std::isfinite(input->visual_logscale) &&
      std::hypot(input->visual_dx_320px,input->visual_dy_320px)<5. &&
      std::abs(input->visual_logscale)<.012 &&
      std::abs(input->visual_rotation_rad)<.025;
  if(visual.valid){
    visual.dx=input->visual_dx_320px;
    visual.dy=input->visual_dy_320px;
    visual.theta=input->visual_rotation_rad;
    visual.lnscale=input->visual_logscale;
  }
  // The only non-sensor residual is a current-frame global background model;
  // do NOT treat gyro-correlated motion as new physical camera rotation.
  // Visual offset stays a bounded 4DoF similarity *inside* the joint solver.
  // Modes merely include/exclude visual candidates; both use one target state.
  std::vector<Similarity2D> visualOptions;
  visualOptions.push_back(lastVisual);
  if(visual.valid){
    Similarity2D desired=lastVisual;
#if defined(C2_CENTER_CALIBRATED)
    // Optical pixel geometry: each C1 320x180 feature pixel represents
    // 2560/320=8 buffer-native source pixels, NO arbitrary gain and NO
    // gyromotion reweighting. This residual was measured AFTER Gyro+RS19
    // prewarp. Angular turnover does not justify ignoring observed pixel
    // motion, but actual spatial trust and full RS19 budget can veto it.
    const double trust=visual.trust;
    desired.txPx+=std::clamp(-8.*visual.dx,-32.,32.);
    desired.tyPx+=std::clamp(-8.*visual.dy,-32.,32.);
    desired.trust=trust;
    desired.inliers=input->visual_inliers;
    // Candidate A: just translation. Do not invent extra scale/zoom
    // breathing when the observed output problem is translational motion.
    visualOptions.push_back(desired);
    // Global roll is NOT optional decoration: C1 measured it reliably,
    // whereas C2 historically treated it as an expensive candidate with
    // no predicted reward. Evaluate translation+roll independently from
    // scale to avoid paying unnecessary zoom/scale distortion.
    desired.rotationRad+=std::clamp(-visual.theta,-.003,.003);
#if defined(C3_GLOBAL_PATH)
    visualOptions.push_back(desired);
#endif
#if !defined(C3_ROLL_ONLY)
    const double logScale=std::log(std::max(.7,lastVisual.scale))
            +std::clamp(-visual.lnscale,-.0015,.0015);
    desired.scale=std::exp(std::clamp(logScale,-.045,.045));
    visualOptions.push_back(desired);
#endif
#else
    // At 320 feature px one pixel = 8 raw source px. Strong current yaw
    // makes model mismatch likelier, so high-rate gyro motion suppresses
    // extra Visual correction instead of double-adding it to gyro.
    const double yawFactor=1./(1.+std::pow(std::abs(input->yaw_rad_s)/.45,2));
    const double trust=visual.trust*yawFactor;
    const double rawDX=std::clamp(-8.*visual.dx*trust,-12.,12.);
    const double rawDY=std::clamp(-8.*visual.dy*trust,-12.,12.);
    desired.txPx+=rawDX;
    desired.tyPx+=rawDY;
    desired.rotationRad+=std::clamp(-visual.theta*trust,-.003,.003);
    const double logScale=std::log(std::max(.7,lastVisual.scale))
            +std::clamp(-visual.lnscale*trust,-.0015,.0015);
    desired.scale=std::exp(std::clamp(logScale,-.045,.045));
    desired.trust=trust;
    desired.inliers=input->visual_inliers;
    visualOptions.push_back(desired);
#endif
  }
#if defined(C2_EDGE_COHERENT)
  // Temporal geometry is a first-class constraint of the SAME joint solver.
  // 1280px final output has a 640px half-width: an isolated Zoom step
  // moves the peripheral content by approximately 640*|delta log Zoom|.
  // Restrict direct Zoom-induced edge motion to <=2 output px per frame.
  // If this can't satisfy full RS19 coverage, the SAME solver chooses the
  // minimum physically safe qVirtual follow; never append a post-warp Zoom
  // floor, delayed patch or second controller.
  const double logZoomEdgeRate=2.0/640.0;
  const double zoomDown=std::max(1.,previousZoom*std::exp(-logZoomEdgeRate));
  const double zoomUp=std::min(kMaxZoom,previousZoom*std::exp(.5*logZoomEdgeRate));
  const double zoomUp2=std::min(kMaxZoom,previousZoom*std::exp(logZoomEdgeRate));
#else
  const double zoomDown=std::max(1.,previousZoom/1.006);
  const double zoomUp=std::min(kMaxZoom,previousZoom*1.010);
  const double zoomUp2=std::min(kMaxZoom,previousZoom*1.018);
#endif
  const std::array<double,4> zoomOptions={
      zoomDown,previousZoom,zoomUp,zoomUp2};
  Candidate best{};
  int attempts=0,rejected=0;
  for(size_t visualOption=0;visualOption<visualOptions.size();++visualOption)
   for(const double zoom:zoomOptions){
    const auto& v=visualOptions[visualOption];
    if(!std::isfinite(zoom)||zoom<1.||zoom>kMaxZoom)continue;
    WarpRequest req=physical;
    req.visual=v;
    req.zoom=zoom;
    // Bounded HOLD first, then solve the MINIMUM possible causal follow
    // fraction toward the *current* physical camera when border is exhausted.
    auto trial=[&](double fraction)->bool{
      req.qTarget=safeMix(previousQ,center,fraction);
      return state->warp.computeMarginPx(req)>=kSafe;
    };
    double smallest=0.;
    if(!trial(0.)){
      if(!trial(1.)){
        ++rejected;continue;
      }
      double lo=0.,hi=1.;
      for(int i=0;i<12;++i) {
        const double mid=(lo+hi)*.5;
        if(trial(mid))hi=mid;
        else lo=mid;
      }
      smallest=hi;
    }
    // Probe minimal and slightly more-following solutions; no future frames.
    const std::array<double,3> follow={
      smallest, std::min(1.,smallest+.012),std::min(1.,smallest+.035)};
    for(const double alpha:follow){
      ++attempts;
      req.qTarget=safeMix(previousQ,center,alpha);
      if(state->warp.computeMarginPx(req)<kSafe){
        ++rejected;continue;
      }
      auto mesh=state->warp.buildMesh(req); // real 20x33 physical RS19
      auto g=inspect(mesh);
      if(!g.valid||g.margin<kSafe) {
        ++rejected;continue;
      }
      const double angularStep=attitudeDistance(previousQ,req.qTarget);
      const double yawPixelStep=kFx*angularStep;
      const double zoomStep=std::abs(std::log(req.zoom/previousZoom))*1024.;
      const double visualStep=std::hypot(
          req.visual.txPx-lastVisual.txPx,
          req.visual.tyPx-lastVisual.tyPx);
      const double scaleStep=std::abs(
          std::log(req.visual.scale/lastVisual.scale))*1024.;
      const double rotStep=std::abs(
          req.visual.rotationRad-lastVisual.rotationRad)*1000.;
      const double spanStep=fresh?0.:std::abs(g.span-state->lastSpan);
      // Predicted residual assumes dx_feature is previous->current
      // scene drift after physical Gyro prewarp. -tx samples future source
      // location, so predicted residual=measured+delta(tx)/8.
      const double remx=visual.valid?
          (visual.dx+(req.visual.txPx-lastVisual.txPx)/8.):0.;
      const double remy=visual.valid?
          (visual.dy+(req.visual.tyPx-lastVisual.tyPx)/8.):0.;
      const double predicted=std::hypot(remx,remy);
      // ONE objective: preserve center tripod target before spending edge
      // scale and Zoom budget; reduce trusted residual without late patch.
      // Penalties are in physically approximate optical source pixels.
#if defined(C2_CENTER_CALIBRATED)
      // Native camera-crop optical-pixel objective: predicted 320px
      // residual * (2560/320) is directly comparable to optical source
      // pixels. Prior weak candidate spent ~0.53 source px per observed
      // feature px instead of the required 8px, explaining V1 pixel Gate
      // miss. Center residual has higher priority than edge/zoom terms;
      // no post-solve additive Visual correction exists.
      const double centerResidualSourcePx=predicted*8.;
#if defined(C3_GLOBAL_PATH)
      // C3 physical 4DoF global output path observer. Center translation
      // already uses camera 8:1 source/feature conversion. Rotation and
      // logScale MUST be optimized in the SAME time domain: apply this
      // frame's current global similarity plus this frame's candidate
      // relative to LAST committed Visual state. Output half-width=640px.
      const double residualRotation=visual.valid?
         visual.theta+(req.visual.rotationRad-lastVisual.rotationRad):0.;
      const double rollEdgePx=640.*std::abs(residualRotation);
#if defined(C3_ROLL_ONLY)
      // Ablation: native C1 logScale is tracked but NOT actuated until
      // independent output evidence shows scale is causal of artifact.
      // This isolates valid scene roll from scale/zoom overreaction.
      const double scaleEdgePx=0.;
#else
      const double residualScale=visual.valid?
         visual.lnscale+std::log(req.visual.scale/lastVisual.scale)
                 +std::log(req.zoom/previousZoom):0.;
      const double scaleEdgePx=640.*std::abs(residualScale);
#endif
      const double cameraStepJerk=std::abs(
         yawPixelStep-state->previous_target_step_source_px);
      double predictedMarginPressure=0.;
#if defined(C3_FORECAST_BOUNDARY) || defined(C3_FORECAST_FOLLOW_ONLY)
      // Predict future physically plausible RS row poses at the next few
      // tens of milliseconds without accessing any future sensor sample.
      WarpRequest projected=futurePhysical;
#if defined(C3_FORECAST_FOLLOW_ONLY)
      // Ablation for the old failed forecast: predicted headroom is
      // explicitly NOT purchasable with a new Zoom or Visual action.
      // Forecast affects qVirtual target timing ONLY, so it cannot cause
      // forecast-induced Zoom rising to 1.14 from baseline 1.02.
      projected.visual=lastVisual;
      projected.zoom=previousZoom;
#else
      projected.visual=req.visual;
      projected.zoom=req.zoom;
#endif
      projected.qTarget=req.qTarget;
      const double futureMargin=state->warp.computeMarginPx(projected);
      if(!std::isfinite(futureMargin)){++rejected;continue;}
      // Soft reserve is physically meaningful: risk in SOURCE pixels, not
      // an artificial zoom preset; if budget runs out, qVirtual may follow.
#if defined(C3_FORECAST_FOLLOW_ONLY)
      // When halfcycle is unconfirmed or just reversed, don't infer the
      // direction of future Camera from only 1-2 unstable gyro samples.
      const bool stableHalfcycle=nowHalfcycleFrames>=4 &&
           std::abs(nowYawFast)>.20 && !reversal;
      predictedMarginPressure=stableHalfcycle?
          .65*std::max(0.,18.-futureMargin):0.;
#else
      predictedMarginPressure=.90*std::max(0.,22.-futureMargin);
#endif
#endif
      const double cost=2.8*yawPixelStep +.92*zoomStep+
          .65*visualStep +1.15*scaleStep+.45*rotStep+
          .35*spanStep + .12*cameraStepJerk+
          (visual.valid?
             (6.0*visual.trust*centerResidualSourcePx+
              7.0*visual.trust*rollEdgePx+
              1.8*visual.trust*scaleEdgePx):0.)+
          predictedMarginPressure+(g.margin<8.?8.-g.margin:0.);
#else
      const double cost=2.8*yawPixelStep +.92*zoomStep+
          .65*visualStep +1.2*scaleStep+.7*rotStep+
          .35*spanStep + (visual.valid?
                   6.0*visual.trust*centerResidualSourcePx:0.)+
          (g.margin<8.?8.-g.margin:0.);
#endif
#else
      const double cost=2.8*yawPixelStep +.92*zoomStep+
          .65*visualStep +1.2*scaleStep+.7*rotStep+
          .35*spanStep + (visual.valid?12.*visual.trust*predicted:0.)+
          (g.margin<8.?8.-g.margin:0.);
#endif
      if(cost<best.score){
        best.warp=req;
        best.mesh=std::move(mesh);
        best.geometry=g;
        best.score=cost;
        best.step=angularStep;
        best.residual_feature_px=predicted;
        best.usedVisual=int(visualOption!=0);
        best.valid=true;
      }
    }
  }
  if(!best.valid){
    // Hard fail closed: at worst choose camera orientation and no external
    // visual. Never publish UV-invalid mesh nor poison committed state.
    WarpRequest fallback=physical;
    fallback.qTarget=center;
    fallback.zoom=1.0;
    fallback.visual=Similarity2D{};
    auto mesh=state->warp.buildMesh(fallback);
    const auto g=inspect(mesh);
    if(!g.valid||g.margin<kSafe)return -6;
    best.warp=fallback;
    best.mesh=std::move(mesh);
    best.geometry=g;
    best.step=attitudeDistance(previousQ,center);
    best.valid=true;
    output->reason=3; // safety pressure emergency
  } else {
    output->reason=fresh?1:(best.step>.0001?2:0);
  }
  output->code=0;
  output->visual_used=best.usedVisual;
  output->alternatives_tested=attempts;
  output->rejected_safety=rejected;
  const auto& q=best.warp.qTarget;
  output->target_xyzw[0]=q.x;output->target_xyzw[1]=q.y;
  output->target_xyzw[2]=q.z;output->target_xyzw[3]=q.w;
  output->zoom=best.warp.zoom;
  output->visual_tx_source_px=best.warp.visual.txPx;
  output->visual_ty_source_px=best.warp.visual.tyPx;
  output->visual_rotation_rad=best.warp.visual.rotationRad;
  output->visual_scale=best.warp.visual.scale;
  output->min_margin_source_px=best.geometry.margin;
  output->center_x_source_px=best.geometry.cx;
  output->center_y_source_px=best.geometry.cy;
  output->span_source_px=best.geometry.span;
  output->target_step_rad=best.step;
  output->previous_mesh_span_step_px=fresh?0.:
      best.geometry.span-state->lastSpan;
  output->predicted_remaining_visual_px320=best.residual_feature_px;
  for(size_t i=0;i<660;++i) {
    output->mesh_uv[2*i]=best.mesh[i].u;
    output->mesh_uv[2*i+1]=best.mesh[i].v;
  }
  output->solve_ms=std::chrono::duration<double,std::milli>(
      Clock::now()-t0).count();
  // Atomic commit: no post-decision Zoom write, hidden second controller,
  // or intermittent qTarget/Visual state mismatch.
  state->initialized=true;
  state->lastSource=input->source_id;
  state->lastTS=input->sensor_ts_ns;
  state->virtualQ=q;
  state->visual=best.warp.visual;
  state->zoom=best.warp.zoom;
  state->lastSpan=best.geometry.span;
  state->previousMesh=std::move(best.mesh);
#if defined(C3_GLOBAL_PATH)
  // Commit causal observer, reversal phase and target path together with
  // only the accepted real RS19 geometry; cannot leak future to the past.
  state->yaw_fast=nowYawFast;
  state->yaw_slow=nowYawSlow;
  if(yawSign){
     state->signed_halfcycle=yawSign;
     state->halfcycle_frames=nowHalfcycleFrames;
  }else state->halfcycle_frames=0;
  if(reversal)++state->reversal_events;
  if(visual.valid){
      state->common_dx_path=std::clamp(
          state->common_dx_path+visual.dx,-65.,65.);
      state->common_dy_path=std::clamp(
          state->common_dy_path+visual.dy,-65.,65.);
      state->common_rotation_path=std::clamp(
          state->common_rotation_path+visual.theta,-.12,.12);
  }
  state->previous_target_step_source_px=kFx*best.step;
#endif
  return 0;
}
void c2_shadow_destroy(C2Owner** state){
  if(!state||!*state)return;
  delete *state;
  *state=nullptr;
}
