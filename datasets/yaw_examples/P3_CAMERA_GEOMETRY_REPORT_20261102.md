# GeminiEis P3 — post-ISP projection and rolling-shutter calibration investigation
**2026-11-02 | experimental-only | golden commit 1492bd5 preserved**

## Executive verdict

**P3-0 through P3-2: data-driven projection mismatch evidence obtained. P3-5: first same-qTarget real-pixel test done; visible edge-deformation Gate NOT PASSED.**

The original golden center stabilization remains the reference. A model jointly adjusting post-ISP radial curvature, shared focal scale and effective RS row times predicts image feature motion better than the golden model, including on previously unseen Yaw sequences. But these fitted parameters are NOT independently measured camera intrinsics, and the first calibrated four-way pixel replay does not uniformly improve differential edge deformation. **Do not deploy the candidate to Android or silently change the golden baseline.**

## Experiment isolation

- Golden original: `D:\hongda\app\GeminiEis-1492bd5` at `1492bd5` (not edited).
- GitHub `poorld/GeminiEis` main remains separate/unmodified.
- Actual experiments: `D:\hongda\app\GeminiEis-1492bd5-yaw-experiment`, branch `experiment/yaw-p0-p2`.
- Original source video and IMU: `D:\Yuy\doc\EIS\Yaw\Movies` and `rec_diagnostics\20261017_<case>_*`.
- Output directory: `D:\Yuy\doc\EIS\Yaw\GeminiEIS_P3_20261102`.
- Full data: `p3_fit_debiased_s1.json`, `p3_validation_cases.json`, per-case `*_raw_tracks.npz` and `*_extract_summary.json`.

## P3-0: Real EIS-OFF feature observation

Source video native resolution 2560x1440. For each consecutive pair, independent OpenCV LK forward/backward checking with homography RANSAC selected static-looking tracks distributed across the whole image. Optical-flow and RANSAC rejection are imperfect: they cannot fully eliminate parallax, texture motion, exposure blur or moving subjects.

| Recording | Matched image feature pairs | Fraction from outer 25% horizontal regions | Valid adjacent frame pairs |
|---|---:|---:|---:|
| 024605 | 50,116 | 45.2% | 293 |
| 024618 | 48,686 | 45.3% | 293 |
| 024630 | 44,529 | 42.7% | 293 |
| 024647 | 44,970 | 49.1% | 293 |
| **Total** | **188,301** | — | **1,172** |

A smaller set of 10,032 feature pairs, stratified by image regions and whole frame indices, trains the model. A completely separate 2,508 feature pairs from non-overlapping frame indices provides within-sequence validation. Other validation withholds entire recordings.

- Exact recorded Camera2 `sensor_timestamp_ns`, `exposure_ns=29996000`, `rolling_shutter_skew_ns=29192380`.
- Gyro-to-camera orientation matrix from actual app: `[[0,1,0],[1,0,0],[0,0,-1]]`.
- Uncalibrated gyro bias columns are subtracted as Java does. Reverse gyro sign was directly rejected (whole-data median reprojection error ~41px, compared to ~1.25px golden signs).
- First recorded frame has no exact metadata; calibration tracking excludes early frame pairs affected by initial IMU coverage.
- Model: input Brown radial inverse to ray, rotation between per-feature real source row times, Brown radial forward projection into next-frame source coordinates.

## P3-1/P3-2: Model fitting (held-out frame reprojection)

All errors below are per-feature input-image reprojection P90 in **2560x1440 source pixels**, not final stabilized-frame pixel flow.

| Candidate | Phase / effective RS | fx multiplier | Brown k1 | Held-out all P90 | Held-out left P90 | Held-out right P90 |
|---|---|---:|---:|---:|---:|---:|
| Golden original | +4ms / 1.0000 | 1.0000 | -0.01500 | 6.002 | 9.368 | 8.167 |
| Derived 16:9 crop geometry only | +7.626ms / 0.75155 | 1.0000 | -0.01500 | 6.182 | 9.325 | 7.821 |
| Golden readout fixed; fitted radial/focal | +4ms / 1.0000 | 1.02247 | -0.05730 | 2.954 | 3.391 | 3.063 |
| Physical crop timing, fitted radial/focal | +7.626ms / 0.75155 | 1.02175 | -0.05690 | 2.672 | 3.142 | 2.706 |
| Unrestricted joint model | +7.330ms / 0.71735 | 1.02173 | -0.05685 | **2.619** | **2.994** | **2.705** |

**Result:** Changing readout alone does not cure the observable projection error. Most edge reprojection benefit comes from radial/focal model changes. Readout/phase refinements offer additional improvement but are partly correlated.

## Independent video-held-out validation

Each sequence below was COMPLETELY excluded from calibration fitting, then tested on its held-out frames.

| Excluded test recording | Golden P90 | Joint model P90 | Physical crop constrained P90 |
|---|---:|---:|---:|
| 024605 | 4.061 | 1.238 | 1.241 |
| 024618 | 6.778 | 2.200 | 2.241 |
| 024630 | 8.610 | 3.617 | 3.825 |
| 024647 | 5.485 | 2.955 | 2.979 |

This supports a repeatable image-motion/model mismatch across scenes, not a one-clip overfit.

## Camera mode / timing rationale (not a confirmed per-frame ROI)

The original ALPS driver at `Z:\hongda\L1T-6769\alps\kernel-4.14\drivers\misc\mediatek\imgsensor\src\common\v1\imx476_mipi_raw\imx476mipiraw_Sensor.c` declares preview/video output at **2592x1940 (2x2 binning)**.
**Driver-clock validation:** The same IMX476 Mode-D source declares PixelClock=210MHz and LineLength=3160 clocks. Thus each physical row is 3160/210000000 = 15.047619 microseconds; times 1940 physical rows gives 29.192381ms, matching the real Camera2 RS skew 29.192380ms almost exactly. This supports the metadata describing **full-sensor Mode-D timing** rather than directly measuring the cropped 1440 output-row span. The actual post-ISP viewport crop must still be observed.


If the physical source is center-cropped to 16:9 before producing 2560x1440, the active source-row count is approximately 2592*1440/2560 = **1458**, or 1458/1940 = **0.751546** of full readout. Metadata 29.19238ms would then correspond to ~21.94ms of actual image row span; the first image row is ~3.626ms later than the full-frame row-zero timing. The phase adjustment from +4ms to +7.626ms preserves the same approximate center-row timestamp.

This is a **testable coordinate-domain hypothesis**, not evidence of the exact HAL RRZO/ISP timing transformation. The recorded `crop_region` is [0,0,5184,3880] rather than an authoritative RRZO crop/readout map. Prior untracked `YAW_TIMING_ISOLATION_RESULTS.md` in the reference tree also documented that simply scaling readout to ~0.75 failed to settle the distortion problem.

### Identifiability caveats

- Normalized residual response correlation between offset and readout scale is **0.8618**; their fitted individual values are not independently established.
- Single-case readout estimates range ~0.704–0.733; single-case k1 ~-0.0558 to -0.0596, but fitted shared focal scale varies ~1.01 to ~1.085.
- We did **not** independently solve fx versus fy, principal point cx/cy, k2/p1/p2, gyro-to-camera extrinsic error or whether ISP already applies lens correction. These require additional differently excited and/or checkerboard imagery.
- The k1 estimate may be a surrogate for the true processed-pixel mapping, not a physical lens coefficient. We do NOT claim laboratory camera calibration.
- Exposure ~30ms at ~30fps blurs fast yaw; even RANSAC inliers can be noisy.

## P3-5: Same golden qTarget source-pixel A/B

A diagnostic-only C++ API creates calibrated row quaternions using the **original native causal gyro ring**, and sends the SAME recorded golden virtual qTarget into RS19/ONE-WARP. Native original projectPoint with the original k1 and row quaternions was reproduced to ~2.2e-16 normalized UV; the real product/Android default code is not modified.

Visual residual UV is kept **frozen as the golden visual correction** and added to the candidate physical mesh. This isolates geometry, but is **not equivalent** to rerunning the visual optimizer after replacing its physical camera model.

For 024630, a 294-frame real-video four-way 1920x1080 preview was rendered:
- Top left: EIS-off original source.
- Top right: golden 1492bd5.
- Bottom left: physically constrained center-crop RS candidate (plus fitted focal/k1).
- Bottom right: unconstrained data-fit candidate.

### 024630 independent frame-motion stability

Median of frame-by-frame P90 motion, at 960x540 per quadrant:

| Region | Golden | Physical crop candidate | Joint fitted candidate |
|---|---:|---:|---:|
| Center | 2.371 px | 2.457 px | 2.373 px |
| Left edge | 3.390 px | 3.269 px | 3.287 px |
| Right edge | 3.854 px | 2.783 px | 2.686 px |

Max out-of-range sample fraction = **0** for golden and both candidates in that preview. The joint candidate preserves approximate center motion by this metric and improves right-strip motion.


### Four-clip real-pixel validation (294 valid frames each)

Each candidate is reconstructed with identical golden qTarget and frozen golden visual UV residual. Values are independent frame-flow P90 medians at 960x540 per quadrant.

| Recording | Golden center / left / right (px) | Physical crop center / left / right (px) | Data joint center / left / right (px) |
|---|---|---|---|
| 024605 | 0.533 / 1.049 / 0.838 | 0.552 / 0.833 / 0.652 | **0.518 / 0.824 / 0.646** |
| 024618 | 0.983 / 2.042 / 1.727 | 1.019 / 1.435 / 1.233 | **1.015 / 1.460 / 1.236** |
| 024630 | 2.371 / 3.390 / 3.854 | 2.457 / 3.269 / 2.783 | **2.373 / 3.287 / 2.686** |
| 024647 | 3.962 / 1.358 / 1.060 | 4.103 / 1.419 / 1.166 | **4.079 / 1.398 / 1.185** |

**Critical negative control:** 024647 becomes slightly worse in center and both edges. No candidate can pass the required all-clips center/edge Gate. All 1176 candidate frames had no out-of-source sample pixels in the PC preview, and the recalibrated Mesh reconstructed its original coefficients to 0–2.22e-16 normalized UV parity.

### Important countercheck: left/right-relative-center and local affine strain

A **second**, independently tracked set of 293 decoded video transitions measured movement relative to the center of the same output:

| Metric, absolute P95 over frames | Golden | Physical crop | Joint fitted |
|---|---:|---:|---:|
| Left-minus-center horizontal displacement | 1.742px | 2.891px | 2.700px |
| Right-minus-center horizontal displacement | 1.522px | 1.594px | 1.664px |
| Left-minus-center horizontal affine strain proxy | 4.86% | 3.74% | 3.47% |
| Right-minus-center horizontal affine strain proxy | 1.99% | 2.43% | 3.31% |

**This is the decisive limitation:** better projection residual and lower edge-motion P90 do **not** translate into universal reduction of edge-versus-center motion/strain. Some regions get worse. This is NOT a no-rubber-sheet solution. These affine numbers are feature-tracking proxies, still affected by parallax and moving objects, not laboratory physical strain.

Videos: `024630_P3_FOUR_WAY_CROP_JOINT_H264.mp4` and local `index.html`.

## Next controlled gates

1. Confirm actual post-ISP image ROI, distortion-correction state, focal lengths and principal point from HAL/Camera2 logs and a 2560x1440 checkerboard covering edges. Do not turn fitted k1 into a blind product constant.
2. Capture independent short-exposure far-field yaw with different velocities for gyro phase and row mapping, and compare camera-to-gyro extrinsic alternatives. The present 30ms exposure limits precision.
3. Preserve original golden qTarget, RS19 and ONE-WARP; improve edge-versus-center differential and straight-line stability **without regressing center P90 or creating borders**.
4. Only after physical calibration evidence, consider limited-ray-rotation SO(3) residual ablations; do not increase local mesh freedom or simply delete golden visual limits.

**P3 conclusion: scientific evidence of model mismatch + partial output improvement; calibration authenticity NOT PROVEN and final peripheral deformation criterion NOT PASSED.**
