#pragma once
// V2C C1-B portable, scene-agnostic native multizone FB-LK visual observer.
// BUFFER-NATIVE 320x180 feature-space pixels, already Gyro+RS prestabilized.
// No image/gyro future frames, no Oracle, no target/zoom adjustment.
// C ABI uses only fixed POD shapes for Windows/Android portability.
#include <stdint.h>
#ifdef _WIN32
#define C1B_EXPORT extern "C" __declspec(dllexport)
#else
#define C1B_EXPORT extern "C" __attribute__((visibility("default")))
#endif
enum C1BStatus : int32_t {
    C1B_ACCEPT=0, C1B_COVERAGE_LOW=1, C1B_NO_FEATURES=2,
    C1B_TOO_FEW=3, C1B_NO_MODEL=4, C1B_SPATIAL_FAIL=5,
    C1B_FB_UNRELIABLE=6, C1B_NONRIGID=7, C1B_LOW_TRUST=8,
    C1B_MOTION_OUTLIER=9, C1B_INVALID_ARGUMENT=-1
};
struct C1BVisual {
    int32_t status, checked, tracked, inliers, support_tiles, support_cols;
    double trust, center_dx, center_dy, rotation_rad, logscale;
    double fb_p90, fit_p90;
    double matrix[6]; // 2x3 forward similarity of prev feature grid to current
    int32_t tile_checked[9], tile_inliers[9];
    double tile_dx[9], tile_dy[9], tile_fit[9];
    double elapsed_ms;
};
C1B_EXPORT int c1b_visual_pair(
    const uint8_t* prev_gray, const uint8_t* curr_gray,
    const uint8_t* prev_valid, const uint8_t* curr_valid,
    int width, int height, int stride, C1BVisual* out);
