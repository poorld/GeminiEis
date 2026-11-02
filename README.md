# GeminiEis: Golden V7 source snapshot (`1492bd5`)

This is a source-only, independent-history export of the 2026 golden baseline (`1492bd51da5b7cc16658ed5d6a88f514e8620635`). The original golden working directory was not modified. Source and build assets are preserved; generated recordings and large experimental outputs are omitted.

**For automated code reviewers / Claude Opus (do not trust older README descriptions):**

- **[Opus source review with line-numbered code excerpts](OPUS_SOURCE_REVIEW.md)** — self-contained critical functions, including boundary gates, per-row target, zoom selection, RS19 physical warp, KLT residual, breathing and clamps.
- [Raw Opus review document](https://raw.githubusercontent.com/poorld/GeminiEis/main/OPUS_SOURCE_REVIEW.md)
- [Raw C++ virtual camera controller](https://raw.githubusercontent.com/poorld/GeminiEis/main/src/core/gemini_eis_core.cpp)
- [Raw RS19 ONE-WARP implementation](https://raw.githubusercontent.com/poorld/GeminiEis/main/src/warp/rs19_one_warp.cpp)
- [Raw visual residual & C API](https://raw.githubusercontent.com/poorld/GeminiEis/main/src/api/gemini_eis_c_api.cpp)
- [Raw Camera2 Android controller](https://raw.githubusercontent.com/poorld/GeminiEis/main/android_app/app/src/main/java/com/yuy/y3realtime/GeminiCamera2EisController.java)

## Actual golden implementation

- IMX476 Y1: 2560x1440 capture, 1920x1080 output; ~400 Hz gyroscope; KLT at 320x180.
- Camera intrinsics in code: fx=972, fy=976, cx=1280, cy=720, k1=-0.015; camera-to-gyro offset +4 ms. These are implementation values, **not independently validated post-ISP calibration ground truth**.
- Real-time quaternion virtual camera; RS19 uses a row-dependent sensor pose and 20x33 one-warp sampling mesh.
- Crop factor 0.80, nominal zoom 1.12 with possible boundary-triggered change to 1.14; per-row virtual targets are possible under second boundary gate.
- Visual correction includes bounded per-vertex UV residual with smoothing, breathScale and boundary clamp. Some affine/projective/RBF degrees are deliberately zeroed; nonrigid output can nevertheless emerge from later per-vertex operations.

## Outstanding problem

The center is strongly stabilized during oscillating left/right yaw, but the periphery can visibly stretch, shrink, or wobble. Do not simply reverse-scale the original physical warp Jacobian: first isolate artificial strain from correct projection and rolling-shutter effects, and account for finite FOV, parallax and missing pixels.

For an actual causal attribution experiment, replay four identical un-stabilized Yaw sequences with the actual same qTarget history and compare independently: zoom/breathScale, per-row target, visual mesh residual and its clamp, and calibration timing/model perturbations. The golden code does not currently export all requisite per-frame internal diagnostics.

## Build

See `CMakeLists.txt`, `android_app/` and `GITHUB_SOURCE_README.md`. Do not interpret this as a full replay archive: recordings, original Git history and binary artifacts are excluded.
