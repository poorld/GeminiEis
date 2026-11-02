#pragma once
// V2C C1-B2: portable Camera Y plane to 320x180 current-available feature.
// Real native 8x8 luma area reduction, supports Android row/pixel stride.
// Native prewarp is strictly SHADOW and never modifies the final ONE-WARP.
#include <stdint.h>
#if defined(_WIN32)
# define C1B2_API extern "C" __declspec(dllexport)
#else
# define C1B2_API extern "C" __attribute__((visibility("default")))
#endif
// Y plane from NV12/NV21/YUV_420_888. Camera supplies source memory and
// its lifetime; output is persistent caller owned 320x180 Gray8.
// Allow pixel_stride>=1 for planar/interleaved vendor layouts.
// -1 invalid arguments, 0 success. Only 2560x1440 input is supported.
C1B2_API int gemini_c1b2_luma_area8(
    const uint8_t* y_plane,int width,int height,
    int y_row_stride,int y_pixel_stride,
    uint8_t* out_gray,int out_row_stride);

// 320x180 feature-space Gyro+physical20Row RS19 prewarp.
// Geometry from the SAME original Gemini C++ native row projection,
// plus bilinear image resampling and physical-validity mask.
// Actual high-res output RS19 ONE-WARP is unchanged.
C1B2_API int gemini_c1b2_prewarp_gray(
    const uint8_t* src_gray,int gray_row_stride,
    double fx,double fy,double cx,double cy,
    const double* q20_rows_xyzw,const double* q_ref_xyzw,
    uint8_t* dst_gray,uint8_t* dst_valid);
