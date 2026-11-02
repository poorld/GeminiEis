#pragma once
// Gemini V2C C1: portable, CPU-only, pixel-accurate current-frame
// 19RS / 20-row Gyro pre-warp MAPPING for low-res Visual LK.
// NO image future, no oracle, no Visual target, no GPU/encoder. C ABI can
// be shared by Windows Host and NDK JNI without OpenCV in this file.
#include <cstdint>
#if defined(_WIN32)
#  define V2C_EXPORT extern "C" __declspec(dllexport)
#else
#  define V2C_EXPORT extern "C" __attribute__((visibility("default")))
#endif

// Input qRow[20][4] and qReference[4] XYZW camera gyro orientation;
// K=low-resolution fx,fy,cx,cy in same buffer-native pixel coordinates.
// Output mapX/mapY OpenCV remap pixel coordinates and valid bit (0/255).
// Returns 0 on success, negative on invalid input/quaternion geometry.
V2C_EXPORT int gemini_v2c_rs19_prewarp_map(
    int width,int height,
    double fx,double fy,double cx,double cy,
    const double* q_rows20_xyzw,const double* q_ref_xyzw,
    float* map_x,float* map_y,std::uint8_t* valid);
