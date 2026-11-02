# GeminiEis Architecture & Progress

Target:
Zero-lookahead causal hybrid EIS with camera/gyro -> delayed visual residual -> realtime motion state -> two-tier virtual camera -> RS19 -> ONE-WARP.

Implemented:
- shared common types and gyro ring buffer
- constrained minimum-jerk MPC
- intent scheduler
- delayed-state motion estimator
- coordinate-space gyro prewarp + 4DoF similarity RANSAC math
- RS19 ONE-WARP mesh generator
- PC deterministic smoke test

Gate order:
1. Build + smoke PASS.
2. MPC step / reversal / boundary synthetic tests.
3. Replay Y1 real gyro + feature tracks.
4. Compare against PC V3 / PC-exact golden.
5. Add Android JNI using same shared core.
6. Profile, then tune.


# GeminiEis: Causal Hybrid EIS Architecture & Implementation Plan

## 1. Architecture Overview (Zero-Lookahead Hybrid EIS)

```text
Camera Frame (YUV) ──┬─→ Downsample (192×108 Gray) ──→ [Async Visual Worker]
                     │                                  ├─ Gyro-Seeded Bidirectional LK
                     │                                  ├─ 2D Coord Gyro Prewarp (Row-Timestamp RS Comp)
                     │                                  ├─ 4-DoF RANSAC Similarity
                     │                                  └─ Exposure-Aware Adaptive Trust (Shutter/ISO Gate)
                     │                                                │
                     │                                                │ Delayed Visual Residual (t - Δt)
                     │                                                ↓
                     │    Gyro 400Hz + OIS Filter ────→ [100ms State Ring Buffer & Delayed Fusion]
                     │                                                │
                     │                                                ↓
                     │                                  [Realtime Motion State]
                     │                                  ├─ Angular Velocity & Acceleration
                     │                                  ├─ Reversal History & Oscillation State (1.5~2.5Hz)
                     │                                  ├─ Delay-Compensated Visual Residual Trajectory
                     │                                  └─ Dynamic Crop Budget (Net of RS19 Margin)
                     │                                                │
                     │                                                ↓
                     │                                  [Causal Virtual Camera (Two-Tier Decoupled)]
                     │                                  ┌─ Upper Tier: Intent & Weight Scheduler ────┐
                     │                                  │  • Hold / Follow State Machine             │
                     │                                  │  • Oscillation V2 & LFGO History           │
                     │                                  │  → Outputs Reference θ_ref & Weights (Q,R) │
                     │                                  ├─ Lower Tier: Unified Execution Solver ─────┤
                     │                                  │  • 3-Axis Decoupled 1D Constrained MPC     │
                     │                                  │  • Embedded Minimum-Jerk (3rd Derivative)  │
                     │                                  │  • Hard/Soft Crop Boundary Constraints     │
                     │                                  └────────────────────────────────────────────┘
                     │                                                │
                     │                                                ↓
                     │                                         Unified Target
                     │                                                │
                     │                                         Crop Constraint
                     │                                                │
                     └──────────────────────────────────────→  RS19 → ONE-WARP (Single-Pass Mesh Warp)
```