#pragma once

#include <stdint.h>

#if defined(GEMINI_EIS_STATIC)
  #define GEMINI_EIS_API
#elif defined(_WIN32)
  #if defined(GEMINI_EIS_BUILDING_LIBRARY)
    #define GEMINI_EIS_API __declspec(dllexport)
  #else
    #define GEMINI_EIS_API __declspec(dllimport)
  #endif
#elif defined(__GNUC__)
  #define GEMINI_EIS_API __attribute__((visibility("default")))
#else
  #define GEMINI_EIS_API
#endif

#ifdef __cplusplus
extern "C" {
#endif

#define GEMINI_EIS_API_VERSION 1u
#define GEMINI_EIS_MESH_COLUMNS 32u
#define GEMINI_EIS_RS_BANDS 19u
#define GEMINI_EIS_MESH_COLS_VERTS 33u
#define GEMINI_EIS_MESH_ROWS_VERTS 20u
#define GEMINI_EIS_MESH_VERTEX_COUNT 660u
#define GEMINI_EIS_GLES_INDEX_COUNT 3648u /* 32 * 19 * 2 * 3 */

typedef void* GeminiEisHandle;

typedef enum GeminiEisStatus {
    GEMINI_EIS_OK = 0,
    GEMINI_EIS_EINVAL = -1,
    GEMINI_EIS_ESTATE = -2,
    GEMINI_EIS_ECALIBRATION = -3,
    GEMINI_EIS_ECOVERAGE = -4
} GeminiEisStatus;

typedef struct GeminiEisConfig {
    uint32_t struct_size;
    uint32_t api_version;

    uint32_t input_width;
    uint32_t input_height;
    uint32_t output_width;
    uint32_t output_height;

    double fx;
    double fy;
    double cx;
    double cy;

    int64_t default_readout_ns;
    int64_t camera_to_gyro_offset_ns;
    double gyro_to_camera[9];

    double base_crop_budget_rad;
    double rs19_margin_rad;
    double default_zoom;
} GeminiEisConfig;

typedef struct GeminiEisViewport {
    uint32_t buffer_width;
    uint32_t buffer_height;
    uint32_t output_width;
    uint32_t output_height;
    double roi_x;
    double roi_y;
    double roi_width;
    double roi_height;
} GeminiEisViewport;

typedef struct GeminiEisGyroSample {
    int64_t timestamp_ns;
    double x_rad_s;
    double y_rad_s;
    double z_rad_s;
} GeminiEisGyroSample;

typedef struct GeminiEisVisualMotion {
    int64_t timestamp_ns;
    double tx_px;
    double ty_px;
    double rotation_rad;
    double scale;
    double trust;
    int32_t inliers;
    int32_t has_homography;
    double h_inv[9];
} GeminiEisVisualMotion;

/* Causal Feature Correspondences in Raw Sensor / Downsampled Y-Plane Pixels */
typedef struct GeminiEisFeaturePointPair {
    float ref_x_px;     /* Feature X in reference frame [0..feature_width) */
    float ref_y_px;     /* Feature Y in reference frame [0..feature_height) */
    float curr_x_px;    /* Feature X in current frame [0..feature_width) */
    float curr_y_px;    /* Feature Y in current frame [0..feature_height) */
    float weight;       /* Tracking confidence weight (0..1] */
} GeminiEisFeaturePointPair;

typedef struct GeminiEisFrameInfo {
    int64_t sensor_timestamp_ns;
    int64_t exposure_time_ns;
    int64_t rolling_shutter_skew_ns;
    double zoom;
} GeminiEisFrameInfo;

typedef struct GeminiEisWarpVertex {
    double u;
    double v;
} GeminiEisWarpVertex;

typedef struct GeminiEisResult {
    uint32_t valid;
    uint32_t applied;
    uint64_t frame_index;

    double raw_angle_rad[3];
    double target_angle_rad[3];
    double correction_rad[3];
    double jerk_rad_s3[3];

    double pan_confidence;
    double oscillation_score;
    double crop_budget_rad[3];

    double visual_tx_px;
    double visual_ty_px;
    double visual_rotation_rad;
    double visual_scale;
    double visual_trust;

    uint32_t mesh_vertex_count;
    GeminiEisWarpVertex mesh[GEMINI_EIS_MESH_VERTEX_COUNT];
} GeminiEisResult;

/* Core Lifecycle & IMU/Visual Ingestion */
GEMINI_EIS_API void GeminiEisDefaultConfig(GeminiEisConfig* config);
GEMINI_EIS_API int GeminiEisInit(GeminiEisHandle* handle, const GeminiEisConfig* config);
GEMINI_EIS_API void GeminiEisUninit(GeminiEisHandle* handle);
GEMINI_EIS_API int GeminiEisReset(GeminiEisHandle* handle);
GEMINI_EIS_API int GeminiEisSetViewport(GeminiEisHandle* handle, const GeminiEisViewport* viewport);
GEMINI_EIS_API int GeminiEisPushGyro(GeminiEisHandle* handle, const GeminiEisGyroSample* samples, int count);
GEMINI_EIS_API int GeminiEisPushVisual(GeminiEisHandle* handle, const GeminiEisVisualMotion* visual);

/* Causal Real-Time Single-Pass Processing (Android & MTK Unified EntryPoints) */
GEMINI_EIS_API int GeminiEisProcessFrame(
    GeminiEisHandle* handle,
    const GeminiEisFrameInfo* frame,
    GeminiEisResult* result);

/**
 * Causal Real-Time Full-Frame 8-DoF + 20x33 Mesh Lock from Y-Plane:
 * Takes downsampled Y-plane (e.g. 320x180 or 640x360 from Camera2 YUV_420_888 or MTK RRZO),
 * runs causal 3D Gyro RS19 + closed-form sensor-to-virtual feature unwarping +
 * causal past-keyframe ring pool matching + regularized 8-DoF homography +
 * causal 20x33 Gaussian RBF residual fold-in in a single pass.
 */
GEMINI_EIS_API int GeminiEisProcessLiveYPlane(
    GeminiEisHandle* handle,
    const GeminiEisFrameInfo* frame,
    const uint8_t* y_plane,
    int y_width,
    int y_height,
    int y_row_stride,
    GeminiEisResult* result);

/**
 * Causal Real-Time 8-DoF + 20x33 Mesh Lock from Matched Sensor Feature Pairs:
 * Allows external hardware/NEON optical flow (or MTK ME/gyro-guided KLT) to supply
 * raw sensor feature correspondences between a past reference frame (ref_timestamp_ns)
 * and the current frame, unwarping them analytically via 3D Gyro RS19 and folding
 * the 8-DoF + 20x33 RBF lock directly into result->mesh.
 */
GEMINI_EIS_API int GeminiEisProcessCausalFeatureMatches(
    GeminiEisHandle* handle,
    const GeminiEisFrameInfo* frame,
    int64_t ref_sensor_timestamp_ns,
    int64_t ref_exposure_time_ns,
    int64_t ref_rolling_shutter_skew_ns,
    int feature_domain_width,
    int feature_domain_height,
    const GeminiEisFeaturePointPair* matches,
    int match_count,
    GeminiEisResult* result);

/**
 * Exports OpenGL ES 3.0 Single-Pass ONE-WARP Interleaved Vertex Buffer:
 * Writes 660 * 4 floats: [ndc_x, ndc_y, tex_u, tex_v] for each vertex in row-major order (20x33).
 * Optionally applies 4x4 SurfaceTexture st_matrix (column-major, or NULL for identity)
 * and sensor orientation_deg (0, 90, 180, 270).
 */
GEMINI_EIS_API int GeminiEisExportGlesMesh(
    const GeminiEisResult* result,
    const float* st_matrix_4x4,
    int orientation_deg,
    float* out_interleaved_xyuv_2640);

/**
 * Generates the static OpenGL ES 3.0 Element Array Buffer (IBO) of 3648 uint16_t indices
 * (32 * 19 * 2 triangles) matching the exact ONE-WARP diagonal split.
 */
GEMINI_EIS_API int GeminiEisBuildGlesIndices(
    uint16_t* out_indices_3648);

/**
 * Exports MediaTek (MTK) WPE (Warp Engine) Hardware WarpMap:
 * Converts the 33x20 normalized mesh into MTK WPE fixed-point s24.8 (or s24.5) X/Y tables
 * and/or float pixel coordinate tables.
 * @param fixed_point_shift  Fractional bits for MTK WPE integer map (e.g., 8 for s24.8, 5 for s24.5).
 */
GEMINI_EIS_API int GeminiEisExportMtkWpeMap(
    const GeminiEisResult* result,
    uint32_t sensor_width,
    uint32_t sensor_height,
    uint32_t map_stride_elements,
    int fixed_point_shift,
    int32_t* out_wpe_x_fixed,
    int32_t* out_wpe_y_fixed,
    float* out_wpe_x_float,
    float* out_wpe_y_float);

GEMINI_EIS_API const char* GeminiEisGetVersion(void);
GEMINI_EIS_API const char* GeminiEisGetBuildId(void);

#ifdef __cplusplus
}
#endif