# GeminiEis - Golden 1492bd5 (Source Snapshot)

This repository is a lightweight source-only export of the **golden baseline** commit `1492bd51da5b7cc16658ed5d6a88f514e8620635` from the original local GeminiEis project. The original working tree and Git history were not modified.

## Algorithm

- Y1 / IMX476 Android real-time EIS, gyroscope ~400Hz
- Camera capture 2560x1440; 1920x1080 output
- Center-lock quaternion virtual camera, 19 rolling-shutter intervals, 20x33 one-warp mesh
- Camera calibration baseline: fx=972, fy=976, k1=-0.015, camera-to-gyro offset +4ms
- Android entry point: `android_app/app/src/main/java/com/yuy/y3realtime/GeminiCamera2EisController.java`
- Core: `src/core/gemini_eis_core.cpp`
- Warp: `src/warp/rs19_one_warp.cpp`
- Native API: `src/api/gemini_eis_c_api.cpp`

## Unresolved challenge

Strong center stabilization during left/right yaw creates visible peripheral stretching, compression or wobble. Investigate true optical/ISP calibration, RS timing, center-locked virtual-camera projective geometry, source-boundary clamp and per-vertex residual smoothing. Propose a causal, real-time edge-distortion-aware algorithm with one final resample. Inverse local scaling is a hypothesis, not a validated solution. Unavailable source pixels cannot be recovered by reverse scaling.

## Exclusions

Generated videos, mesh binary dumps, native binaries, Android build caches, bulky archived test results and original Git history are intentionally excluded. This is for source review and algorithm research; it does not preserve the complete experiment archive. The original source project is `D:\hongda\app\GeminiEis-1492bd5`.

## Build and development

See `README.md`, `CMakeLists.txt` and `android_app/`. No APK or build results are included. Calibration constants are implementation values, not a guarantee of an independently verified post-ISP camera model.
