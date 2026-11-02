#include "api/gemini_eis_c_api.h"

#include <cstdint>
#include <cstring>
#include <mutex>

/**
 * MediaTek (MTK) Camera HAL3 / FeaturePipe WPE (Warp Engine) Unified Adapter.
 * Uses the exact same C++ causal stabilization core (libgemini_eis_core) as the Android GLES 3.0 pipeline.
 * Converts the 33x20 normalized ONE-WARP mesh directly into MTK WPE hardware WarpMap buffers
 * (supporting both s24.8 / s24.5 fixed-point integer tables and FP32 pixel tables).
 */
namespace mtk_eis_adapter {

struct MtkWpeWarpMapBuffer {
    uint32_t grid_cols = GEMINI_EIS_MESH_COLS_VERTS; // 33
    uint32_t grid_rows = GEMINI_EIS_MESH_ROWS_VERTS; // 20
    uint32_t stride_elements = 33;                   // Row stride in elements (e.g. 33 or 36 for 16-byte align)
    int32_t  fixed_point_shift = 8;                  // 8 for MTK WPE s24.8, 5 for s24.5
    int32_t* wpe_map_x_fixed = nullptr;              // Hardware WPE X plane (int32_t fixed-point)
    int32_t* wpe_map_y_fixed = nullptr;              // Hardware WPE Y plane (int32_t fixed-point)
    float*   wpe_map_x_float = nullptr;              // Optional GPU/CPU float X plane
    float*   wpe_map_y_float = nullptr;              // Optional GPU/CPU float Y plane
};

class GeminiMtkHalSession {
public:
    GeminiMtkHalSession() = default;
    ~GeminiMtkHalSession() { uninit(); }

    int init(uint32_t sensorW, uint32_t sensorH, uint32_t outW, uint32_t outH,
             double fx, double fy, double cx, double cy,
             int64_t readoutNs, int64_t gyroOffsetNs) {
        std::lock_guard<std::mutex> lock(mtx_);
        GeminiEisDefaultConfig(&cfg_);
        cfg_.input_width = sensorW;
        cfg_.input_height = sensorH;
        cfg_.output_width = outW;
        cfg_.output_height = outH;
        cfg_.fx = fx;
        cfg_.fy = fy;
        cfg_.cx = cx;
        cfg_.cy = cy;
        cfg_.default_readout_ns = readoutNs > 0 ? readoutNs : 29000000LL;
        cfg_.camera_to_gyro_offset_ns = gyroOffsetNs;
        cfg_.default_zoom = 1.12; // Constant Tripod Zoom 1.12x
        return GeminiEisInit(&handle_, &cfg_);
    }

    void uninit() {
        std::lock_guard<std::mutex> lock(mtx_);
        GeminiEisUninit(&handle_);
    }

    int pushGyro(const GeminiEisGyroSample* samples, int count) {
        std::lock_guard<std::mutex> lock(mtx_);
        return GeminiEisPushGyro(&handle_, samples, count);
    }

    /**
     * Executes causal real-time stabilization on MTK P1 RRZO/YUVO Y-plane and populates
     * the hardware WPE WarpMap buffer in a single pass.
     */
    int processFrameToWpeMap(
        int64_t sofTimestampNs,
        int64_t exposureNs,
        int64_t readoutNs,
        const uint8_t* rrzoYPlane,
        int rrzoWidth,
        int rrzoHeight,
        int rrzoStride,
        const MtkWpeWarpMapBuffer& wpeOut) {

        std::lock_guard<std::mutex> lock(mtx_);
        GeminiEisFrameInfo fi{};
        fi.sensor_timestamp_ns = sofTimestampNs;
        fi.exposure_time_ns = exposureNs > 0 ? exposureNs : 10000000LL;
        fi.rolling_shutter_skew_ns = readoutNs > 0 ? readoutNs : cfg_.default_readout_ns;
        fi.zoom = 1.12;

        GeminiEisResult res{};
        int rc = GEMINI_EIS_OK;
        if (rrzoYPlane && rrzoWidth > 0 && rrzoHeight > 0) {
            rc = GeminiEisProcessLiveYPlane(&handle_, &fi, rrzoYPlane, rrzoWidth, rrzoHeight, rrzoStride, &res);
        } else {
            rc = GeminiEisProcessFrame(&handle_, &fi, &res);
        }
        if (rc != GEMINI_EIS_OK) return rc;

        return GeminiEisExportMtkWpeMap(
            &res,
            cfg_.input_width,
            cfg_.input_height,
            wpeOut.stride_elements,
            wpeOut.fixed_point_shift,
            wpeOut.wpe_map_x_fixed,
            wpeOut.wpe_map_y_fixed,
            wpeOut.wpe_map_x_float,
            wpeOut.wpe_map_y_float);
    }

private:
    std::mutex mtx_;
    GeminiEisConfig cfg_{};
    GeminiEisHandle handle_ = nullptr;
};

} // namespace mtk_eis_adapter