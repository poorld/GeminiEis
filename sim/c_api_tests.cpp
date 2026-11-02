#include <algorithm>
#include <cmath>
#include <cstring>
#include <iostream>
#include <vector>

#include "api/gemini_eis_c_api.h"
#include "pc/gemini_eis_pc_adapter.h"
#include "android/gemini_eis_android_adapter.h"
#include "mtk/gemini_eis_mtk_adapter.h"

using namespace gemini;

static bool near(double a, double b, double eps = 1e-12) {
    return std::abs(a - b) <= eps;
}

static std::vector<GeminiEisGyroSample> makeGyro() {
    std::vector<GeminiEisGyroSample> out;
    const int64_t start = 950000000LL;
    for (int i = 0; i < 180; ++i) {
        const double phase = i * 0.031;
        out.push_back({
            start + i * 2500000LL,
            0.015 * std::sin(phase),
            0.12 + 0.02 * std::cos(phase * 0.7),
            0.008 * std::sin(phase * 1.3)
        });
    }
    return out;
}

static bool compare(const GeminiEisResult& a, const GeminiEisResult& b) {
    if (a.valid != b.valid || a.applied != b.applied
            || a.mesh_vertex_count != b.mesh_vertex_count) return false;
    for (int k = 0; k < 3; ++k) {
        if (!near(a.raw_angle_rad[k], b.raw_angle_rad[k])
                || !near(a.target_angle_rad[k], b.target_angle_rad[k])
                || !near(a.correction_rad[k], b.correction_rad[k])
                || !near(a.jerk_rad_s3[k], b.jerk_rad_s3[k])) return false;
    }
    for (uint32_t i = 0; i < a.mesh_vertex_count; ++i) {
        if (!near(a.mesh[i].u, b.mesh[i].u)
                || !near(a.mesh[i].v, b.mesh[i].v)) return false;
    }
    return true;
}

static double liveTurnPhysicalYawRate(int frame) {
    if (frame < 30) return 0.0;
    if (frame < 90) return 0.42;
    if (frame < 150) return -0.42;
    return 0.0;
}

static bool testLiveTurnVisualContinuity() {
    GeminiEisConfig cfg{};
    GeminiEisDefaultConfig(&cfg);
    cfg.camera_to_gyro_offset_ns = 0;

    GeminiEisHandle live = nullptr;
    GeminiEisHandle gyroOnly = nullptr;
    if (GeminiEisInit(&live, &cfg) != 0 || GeminiEisInit(&gyroOnly, &cfg) != 0) {
        GeminiEisUninit(&live);
        GeminiEisUninit(&gyroOnly);
        return false;
    }

    const int width = 320;
    const int height = 180;
    std::vector<uint8_t> yPlane(static_cast<size_t>(width * height));
    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            yPlane[static_cast<size_t>(y * width + x)] = static_cast<uint8_t>(
                ((((x / 8) + (y / 8)) & 1) ? 190 : 55) + ((x * 7 + y * 13) % 31));
        }
    }

    constexpr int64_t startNs = 1000000000LL;
    constexpr int64_t frameDtNs = 33333333LL;
    constexpr int64_t gyroDtNs = 2500000LL;
    int64_t gyroTsNs = startNs - 100000000LL;
    double previousVisualRms = 0.0;
    double maxVisualRms = 0.0;
    double maxVisualJump = 0.0;
    bool ok = true;

    for (int frame = 0; frame < 190 && ok; ++frame) {
        const int64_t frameTsNs = startNs + frame * frameDtNs;
        const int64_t neededGyroNs = frameTsNs + 25000000LL;
        std::vector<GeminiEisGyroSample> gyroBatch;
        while (gyroTsNs <= neededGyroNs) {
            const int gyroFrame = static_cast<int>((gyroTsNs - startNs) / frameDtNs);
            // Default Y1 mapping converts physical X gyro into camera-space Yaw.
            gyroBatch.push_back({gyroTsNs, liveTurnPhysicalYawRate(gyroFrame), 0.0, 0.0});
            gyroTsNs += gyroDtNs;
        }
        if (!gyroBatch.empty()) {
            ok = GeminiEisPushGyro(&live, gyroBatch.data(), static_cast<int>(gyroBatch.size())) == 0
                && GeminiEisPushGyro(&gyroOnly, gyroBatch.data(), static_cast<int>(gyroBatch.size())) == 0;
        }
        if (!ok) break;

        GeminiEisFrameInfo fi{};
        fi.sensor_timestamp_ns = frameTsNs;
        fi.exposure_time_ns = 10000000LL;
        fi.rolling_shutter_skew_ns = 19000000LL;
        fi.zoom = 1.12;

        GeminiEisResult liveResult{};
        GeminiEisResult gyroResult{};
        ok = GeminiEisProcessLiveYPlane(
                 &live, &fi, yPlane.data(), width, height, width, &liveResult) == 0
            && GeminiEisProcessFrame(&gyroOnly, &fi, &gyroResult) == 0;
        if (!ok || liveResult.mesh_vertex_count != gyroResult.mesh_vertex_count) {
            ok = false;
            break;
        }

        double squared = 0.0;
        for (uint32_t i = 0; i < liveResult.mesh_vertex_count; ++i) {
            const double du = liveResult.mesh[i].u - gyroResult.mesh[i].u;
            const double dv = liveResult.mesh[i].v - gyroResult.mesh[i].v;
            squared += du * du + dv * dv;
        }
        const double visualRms = liveResult.mesh_vertex_count > 0
            ? std::sqrt(squared / liveResult.mesh_vertex_count) : 0.0;
        maxVisualRms = std::max(maxVisualRms, visualRms);
        maxVisualJump = std::max(maxVisualJump, std::abs(visualRms - previousVisualRms));
        previousVisualRms = visualRms;
    }

    GeminiEisUninit(&live);
    GeminiEisUninit(&gyroOnly);
    std::cout << "live-turn visual maxRms=" << maxVisualRms
              << " maxJump=" << maxVisualJump << "\n";
    return ok && maxVisualRms < 0.0085 && maxVisualJump < 0.0055;
}

int main() {
    GeminiEisConfig cfg{};
    GeminiEisDefaultConfig(&cfg);
    cfg.input_width = 2560;
    cfg.input_height = 1440;
    cfg.output_width = 1920;
    cfg.output_height = 1080;
    cfg.camera_to_gyro_offset_ns = -30000000LL;

    PcAdapter pc(cfg);
    AndroidAdapter app(cfg);
    MtkAdapter mtk(cfg);

    GeminiEisViewport vp{};
    vp.buffer_width = 2560;
    vp.buffer_height = 1440;
    vp.output_width = 1920;
    vp.output_height = 1080;
    vp.roi_x = 0.0;
    vp.roi_y = 160.0;
    vp.roi_width = 2560.0;
    vp.roi_height = 1120.0;

    if (pc.setViewport(vp) != 0 || app.setViewport(vp) != 0
            || mtk.setRuntimeViewport(2560, 1440, 1920, 1080, 0, 160, 2560, 1120) != 0) {
        std::cerr << "viewport failed\n";
        return 2;
    }

    const auto gyro = makeGyro();
    if (pc.pushGyro(gyro) != 0 || app.pushGyro(gyro.data(), static_cast<int>(gyro.size())) != 0) {
        std::cerr << "pc/app gyro failed\n";
        return 3;
    }

    std::vector<MtkSensorProviderGyro> mtkGyro;
    mtkGyro.reserve(gyro.size());
    for (const auto& g : gyro) {
        mtkGyro.push_back({g.timestamp_ns, g.x_rad_s, g.y_rad_s, g.z_rad_s});
    }
    if (mtk.feedSensorProviderGyro(mtkGyro) != 0) {
        std::cerr << "mtk gyro failed\n";
        return 4;
    }

    const int yW = 320, yH = 180;
    std::vector<uint8_t> yPlane(yW * yH, 128);
    for (int y = 0; y < yH; ++y) {
        for (int x = 0; x < yW; ++x) {
            yPlane[y * yW + x] = static_cast<uint8_t>((((x / 10) + (y / 10)) & 1 ? 180 : 60) + ((x * 5 + y * 11) % 35));
        }
    }

    std::vector<float> glesVbo(GEMINI_EIS_MESH_VERTEX_COUNT * 4);
    std::vector<uint16_t> glesIbo(GEMINI_EIS_GLES_INDEX_COUNT);
    AndroidAdapter::buildGlesIndices(glesIbo.data());

    std::vector<int32_t> wpeXFixed(GEMINI_EIS_MESH_VERTEX_COUNT);
    std::vector<int32_t> wpeYFixed(GEMINI_EIS_MESH_VERTEX_COUNT);
    std::vector<float> wpeXFloat(GEMINI_EIS_MESH_VERTEX_COUNT);
    std::vector<float> wpeYFloat(GEMINI_EIS_MESH_VERTEX_COUNT);
    MtkWpeWarpMapOut wpeOut{};
    wpeOut.sensorWidth = 2560;
    wpeOut.sensorHeight = 1440;
    wpeOut.strideElements = 33;
    wpeOut.fixedPointShift = 8;
    wpeOut.wpeMapXFixed = wpeXFixed.data();
    wpeOut.wpeMapYFixed = wpeYFixed.data();
    wpeOut.wpeMapXFloat = wpeXFloat.data();
    wpeOut.wpeMapYFloat = wpeYFloat.data();

    GeminiEisResult rp{}, ra{}, rm{};
    int ep = 0, ea = 0, em = 0;
    for (int step = 0; step < 3; ++step) {
        GeminiEisFrameInfo f{};
        f.sensor_timestamp_ns = 1040000000LL + step * 33333333LL;
        f.exposure_time_ns = 10000000LL;
        f.rolling_shutter_skew_ns = 19000000LL;
        f.zoom = 1.12;

        MtkFrameTiming mf{};
        mf.p1FrameStartBootNs = f.sensor_timestamp_ns;
        mf.exposureNs = f.exposure_time_ns;
        mf.rollingShutterSkewNs = f.rolling_shutter_skew_ns;
        mf.zoom = f.zoom;

        ep = pc.processLiveYPlane(f, yPlane.data(), yW, yH, yW, &rp);
        ea = app.processLiveYPlaneToGlesMesh(f, yPlane.data(), yW, yH, yW, nullptr, 180, &ra, glesVbo.data());
        em = mtk.processLiveYPlaneToWpe(mf, yPlane.data(), yW, yH, yW, &rm, wpeOut);
    }

    std::cout << "version=" << GeminiEisGetVersion()
              << " build=" << GeminiEisGetBuildId() << "\n";
    std::cout << "ret pc/app/mtk=" << ep << "/" << ea << "/" << em << "\n";
    std::cout << "mesh=" << rp.mesh_vertex_count
              << " wpe0=(" << wpeXFixed[0] << "," << wpeYFixed[0] << ")"
              << " gles0=(" << glesVbo[2] << "," << glesVbo[3] << ")\n";

    const bool parity = ep == 0 && ea == 0 && em == 0
        && compare(rp, ra) && compare(rp, rm);
    const bool liveTurnSmooth = testLiveTurnVisualContinuity();
    std::cout << (parity ? "GEMINI_EIS_3END_PARITY_PASS" : "GEMINI_EIS_3END_PARITY_FAIL") << "\n";
    std::cout << (liveTurnSmooth ? "GEMINI_EIS_LIVE_TURN_SMOOTH_PASS" : "GEMINI_EIS_LIVE_TURN_SMOOTH_FAIL") << "\n";
    return parity && liveTurnSmooth ? 0 : 5;
}
