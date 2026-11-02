#pragma once
// V2C C2 isolated SHADOW: ONE causal qVirtual + Visual + Zoom Target owner.
// Physical RS19 20x33 full perimeter precommit and atomic state update.
// Does NOT read V1 Golden meshes, offline oracle, future video or Gemini Core.
// Every final full-resolution warp is represented by precisely ONE mesh.
#include <cstdint>
#if defined(_WIN32)
# define C2_API extern "C" __declspec(dllexport)
#else
# define C2_API extern "C" __attribute__((visibility("default")))
#endif
struct C2Owner;
struct C2Input {
    std::int64_t source_id, sensor_ts_ns;
    double q_center[4], q_rows20[80];  // 20 physical XYZW rows
    double yaw_rad_s;
    double visual_dx_320px, visual_dy_320px;
    double visual_rotation_rad, visual_logscale, visual_trust;
    int visual_inliers, visual_accepted;
};
struct C2Result {
    int code, reason, visual_used, alternatives_tested, rejected_safety;
    std::int64_t source_id, sensor_ts_ns;
    double target_xyzw[4], zoom, visual_tx_source_px,
      visual_ty_source_px, visual_rotation_rad, visual_scale;
    double mesh_uv[1320]; // 660 RS19 20x33 doubles (u,v)
    double min_margin_source_px, center_x_source_px, center_y_source_px;
    double span_source_px, target_step_rad, previous_mesh_span_step_px;
    double predicted_remaining_visual_px320, solve_ms;
};
// mode:0=joint geometry with visual candidate vetoed;
//      1=causal trusted center-weighted C1 visual global residual candidates.
// Both are separate A/B instances of the SAME unique Target owner.
C2_API C2Owner* c2_shadow_create(int visual_mode);
C2_API int c2_shadow_step(C2Owner*, const C2Input*, C2Result*);
C2_API void c2_shadow_destroy(C2Owner**);
