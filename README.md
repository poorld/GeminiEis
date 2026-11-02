# GeminiEis

Causal Hybrid EIS shared-core prototype.

Current implementation status:
- C++17 zero-third-party core build.
- 150ms gyro ring buffer with interpolation/integration.
- 1D constrained MPC with minimum-jerk control and crop soft constraints.
- Hold/Follow + reversal/oscillation-aware weight scheduler.
- Delayed visual residual state propagation.
- Coordinate-space gyro prewarp math and deterministic 4-DoF similarity RANSAC.
- Exposure/motion-aware visual trust.
- RS19 + global + visual residual ONE-WARP mesh composition.
- PC smoke/simulation executable.

Pending:
- Real LK image frontend.
- Real camera calibration / distortion model.
- Dataset replay and HTML comparison integration.
- Android Camera2 metadata/JNI/runtime glue.
- Device performance and visual-effect gates.

Build:
cmake -S . -B build
cmake --build build --config Release
ctest --test-dir build -C Release --output-on-failure
