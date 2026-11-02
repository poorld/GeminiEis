# GeminiEis — 原始防抖流程与实施建议（设计基准）

## 1. 原始端到端流程
Camera YUV -> 192x108 Gray -> Async Visual Worker -> Gyro-seeded bidirectional LK -> coordinate-space gyro prewarp (row timestamp) -> 4DoF RANSAC Similarity -> Exposure-Aware Adaptive Trust -> delayed visual residual.
400Hz gyro -> historical state ring -> delayed-state fusion -> realtime motion state (angular velocity, acceleration, reversal, oscillation, visual path, crop budget).
Upper tier Intent Scheduler: Hold/Follow + Oscillation V2 + LFGO history -> reference trajectory and time-varying Q/R weights; scheduler MUST NOT directly step virtual pose.
Lower tier: 3-axis decoupled constrained MPC + minimum jerk -> Unified Target under soft/hard crop budget.
Unified Target + gyro pose + visual residual + camera K + RS19 row rotations -> ONE-WARP 33x20 UV mesh -> PC GPU / App GLES / MTK warp renderer.

## 2. 附件五项关键建议（不得丢失）
1. Coordinate-space gyro prewarp: LK on raw 192x108 gray; per-feature row time t(y)=t_sof+(y/108)*t_readout+0.5*t_exp; unrotate normalized camera coordinates before 4DoF RANSAC, not a second pixel warp.
2. Delayed-state fusion: align asynchronous visual residual at its historical time node in a 100ms gyro/state buffer, then propagate to current state; do not simply add stale residual to the latest frame.
3. Two-tier causal virtual camera: Hold/Follow, Oscillation V2, LFGO only schedule references and Q/R; one MPC owns target, minimum-jerk and crop constraints.
4. Exposure-aware trust: combine SENSOR_EXPOSURE_TIME, SENSOR_SENSITIVITY (ISO), RANSAC inliers and forward-backward LK error; reject low-light motion-blur wobble.
5. OIS/RS19 frequency decoupling: model OIS mechanical response (approximately 1-15Hz discussion range) before gyro-driven row correction; validate experimentally, never enable an arbitrary notch/high-pass by default.

## 3. MPC 设计公式
State x=[angle, angular_velocity, angular_acceleration]; control u=jerk.
Objective J=sum(w_pos*(angle-ref)^2 + w_vel*(velocity-ref_velocity)^2 + w_acc*acceleration^2 + w_jerk*jerk^2 + w_slack*slack^2).
Crop: abs(angle-raw_angle) <= available_budget + slack, slack >= 0. Implement hard feasibility/safety independently of soft optimization.

## 4. 四阶段 Gate（附件目标，不代表已达成）
Phase 1: PC replay + 1D ADMM/projected-gradient MPC; three-axis solve <0.2ms; no hard crop clipping on step/stop datasets.
Phase 2: async coordinate prewarp + LK/RANSAC + 100ms delayed fusion; occlusion and dark-scene gating; no low-frequency rebound.
Phase 3: Hold/Follow, Oscillation V2, LFGO and multi-dataset sweep; 1.5-2.5Hz walking ITF >15dB, pan phase delay <45ms, stop overshoot <0.5deg.
Phase 4: RS19 ONE-WARP + Android NDK/JNI + Camera2 metadata; CPU <8% single core, processing <1.5ms/frame, 4K60 continuous without dropped frames.

## 5. 三端共库要求
Only one C++ core and C ABI: PC adapter, Android JNI adapter, MTK Streaming adapter. Gyro axis mapping, timestamp offset, camera K/ROI, visual delay, MPC, crop and mesh belong to core. Adapters may acquire buffers, metadata and render output, not implement another algorithm.

## 6. 实施状态与差距（2026-10-22）
Host synthetic build, three-adapter parity, ARM64/ARMv7 library/JNI compilation were reported PASS. This does NOT establish real Android/MTK parity or visual quality.
Incomplete: real bidirectional LK frontend, precise historical delayed-state fusion, full Oscillation V2/LFGO trajectory model, ISO-aware trust, calibrated OIS response, real distortion/focus model, MTK HAL build and device gates.
Current implementation uses a 400ms gyro buffer in C ABI, while original design calls for 100ms historical fusion; buffer capacity and historical alignment are distinct and must be reconciled by evidence.
Do not mark any Phase as complete based only on smoke tests. Reuse existing Y1 datasets 024605/024618/024630/024647 and compare PC V3/Golden before device port.

## 7. Source fidelity
This document records the user-provided GeminiEis architecture and recommendations, plus explicitly labeled implementation gaps. Targets are acceptance criteria, not measured results. Refer to ARCHITECTURE_PLAN.md for current code map.
