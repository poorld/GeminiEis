#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include "api/gemini_eis_c_api.h"

namespace {

struct FrameMeta {
    int source_index = 0;
    int64_t pts_us = 0;
    int64_t sensor_ts_ns = 0;
    int64_t exposure_ns = 0;
    int64_t skew_ns = 0;
};

struct BundleConfig {
    int source_width = 2560;
    int source_height = 1440;
    int output_width = 1280;
    int output_height = 720;
    double fx = 1202.3;
    double fy = 1208.5;
    double cx = 1280.0;
    double cy = 720.0;
    double base_crop = 0.80;
    int64_t camera_to_gyro_offset_ns = 0;
    int64_t offset_delta_ns = 8000000;
    int orientation_deg = 180;
    std::vector<FrameMeta> frames;
};

bool invert3x3(const double m[9], double invOut[9]) {
    const double det = m[0] * (m[4] * m[8] - m[5] * m[7])
                     - m[1] * (m[3] * m[8] - m[5] * m[6])
                     + m[2] * (m[3] * m[7] - m[4] * m[6]);
    if (std::abs(det) < 1e-12) return false;
    const double invDet = 1.0 / det;
    invOut[0] = (m[4] * m[8] - m[5] * m[7]) * invDet;
    invOut[1] = (m[2] * m[7] - m[1] * m[8]) * invDet;
    invOut[2] = (m[1] * m[5] - m[2] * m[4]) * invDet;
    invOut[3] = (m[5] * m[6] - m[3] * m[8]) * invDet;
    invOut[4] = (m[0] * m[8] - m[2] * m[6]) * invDet;
    invOut[5] = (m[2] * m[3] - m[0] * m[5]) * invDet;
    invOut[6] = (m[3] * m[7] - m[4] * m[6]) * invDet;
    invOut[7] = (m[1] * m[6] - m[0] * m[7]) * invDet;
    invOut[8] = (m[0] * m[4] - m[1] * m[3]) * invDet;
    return true;
}

double extractNumber(const std::string& text, const std::string& key, double defVal) {
    const std::string pat = "\"" + key + "\"";
    auto pos = text.find(pat);
    if (pos == std::string::npos) return defVal;
    pos = text.find(':', pos + pat.size());
    if (pos == std::string::npos) return defVal;
    try {
        return std::stod(text.substr(pos + 1));
    } catch (...) {
        return defVal;
    }
}

int64_t extractInt64(const std::string& text, const std::string& key, int64_t defVal) {
    const std::string pat = "\"" + key + "\"";
    auto pos = text.find(pat);
    if (pos == std::string::npos) return defVal;
    pos = text.find(':', pos + pat.size());
    if (pos == std::string::npos) return defVal;
    try {
        return std::stoll(text.substr(pos + 1));
    } catch (...) {
        return defVal;
    }
}

bool loadBundle(const std::string& path, BundleConfig& cfg) {
    std::ifstream ifs(path);
    if (!ifs.is_open()) return false;
    std::string content((std::istreambuf_iterator<char>(ifs)), std::istreambuf_iterator<char>());

    cfg.source_width = static_cast<int>(extractNumber(content, "source_width", 2560));
    cfg.source_height = static_cast<int>(extractNumber(content, "source_height", 1440));
    cfg.output_width = static_cast<int>(extractNumber(content, "output_width", 1280));
    cfg.output_height = static_cast<int>(extractNumber(content, "output_height", 720));
    cfg.fx = extractNumber(content, "fx", 1202.3);
    cfg.fy = extractNumber(content, "fy", 1208.5);
    cfg.cx = extractNumber(content, "cx", 1280.0);
    cfg.cy = extractNumber(content, "cy", 720.0);
    cfg.base_crop = extractNumber(content, "base_crop", 0.80);
    cfg.camera_to_gyro_offset_ns = extractInt64(content, "camera_to_gyro_offset_ns", 0);
    cfg.offset_delta_ns = extractInt64(content, "offset_delta_ns", 8000000);
    cfg.orientation_deg = static_cast<int>(extractNumber(content, "orientation_deg", 180));

    auto fpos = content.find("\"frames\":");
    if (fpos == std::string::npos) return false;
    size_t cur = content.find('[', fpos);
    while (cur != std::string::npos) {
        auto objStart = content.find('{', cur);
        if (objStart == std::string::npos) break;
        auto objEnd = content.find('}', objStart);
        if (objEnd == std::string::npos) break;
        std::string obj = content.substr(objStart, objEnd - objStart + 1);
        FrameMeta fm{};
        fm.source_index = static_cast<int>(extractInt64(obj, "source_index", 0));
        fm.pts_us = extractInt64(obj, "pts_us", 0);
        fm.sensor_ts_ns = extractInt64(obj, "sensor_ts_ns", 0);
        fm.exposure_ns = extractInt64(obj, "exposure_ns", 10000000);
        fm.skew_ns = extractInt64(obj, "skew_ns", 29000000);
        if (fm.sensor_ts_ns > 0) cfg.frames.push_back(fm);
        cur = objEnd + 1;
    }
    return !cfg.frames.empty();
}

bool loadGyro(const std::string& path, std::vector<GeminiEisGyroSample>& out) {
    std::ifstream ifs(path);
    if (!ifs.is_open()) return false;
    std::string line;
    std::getline(ifs, line);
    while (std::getline(ifs, line)) {
        if (line.empty()) continue;
        std::stringstream ss(line);
        std::string c0, c1, c2, c3;
        if (!std::getline(ss, c0, ',')) continue;
        if (!std::getline(ss, c1, ',')) continue;
        if (!std::getline(ss, c2, ',')) continue;
        if (!std::getline(ss, c3, ',')) continue;
        GeminiEisGyroSample s{};
        s.timestamp_ns = std::stoll(c0);
        s.x_rad_s = std::stod(c1);
        s.y_rad_s = std::stod(c2);
        s.z_rad_s = std::stod(c3);
        out.push_back(s);
    }
    return !out.empty();
}

}  // namespace

int main(int argc, char** argv) {
    std::string bundleDir = "D:/hongda/app/YuySuperEIS/artifacts/b38_restore_bundles_20261020_172849/024605";
    if (argc > 1) {
        std::string arg1 = argv[1];
        if (arg1 == "024605" || arg1 == "024618" || arg1 == "024630" || arg1 == "024647") {
            bundleDir = "D:/hongda/app/YuySuperEIS/artifacts/b38_restore_bundles_20261020_172849/" + arg1;
        } else {
            bundleDir = arg1;
        }
    }

    BundleConfig bundle{};
    if (!loadBundle(bundleDir + "/bundle.json", bundle)) {
        std::cerr << "ERROR: Failed to load bundle.json from " << bundleDir << "\n";
        return 1;
    }

    std::vector<GeminiEisGyroSample> gyro;
    if (!loadGyro(bundleDir + "/gyro_camera.csv", gyro)) {
        std::cerr << "ERROR: Failed to load gyro_camera.csv from " << bundleDir << "\n";
        return 2;
    }

    std::string oraclePath = bundleDir + "/oracle_matrices.f64";
    bool disableVisual = false;
    if (argc > 3) {
        std::string arg3 = argv[3];
        if (arg3 == "NONE") {
            disableVisual = true;
        } else {
            oraclePath = arg3;
        }
    }

    std::vector<double> oracleMats;
    if (!disableVisual) {
        std::ifstream mifs(oraclePath, std::ios::binary | std::ios::ate);
        if (mifs.is_open()) {
            auto sz = static_cast<size_t>(mifs.tellg());
            if (sz >= bundle.frames.size() * 9 * sizeof(double)) {
                oracleMats.resize(bundle.frames.size() * 9);
                mifs.seekg(0, std::ios::beg);
                mifs.read(reinterpret_cast<char*>(oracleMats.data()),
                          static_cast<std::streamsize>(oracleMats.size() * sizeof(double)));
            }
        }
    }

    GeminiEisConfig cfg{};
    GeminiEisDefaultConfig(&cfg);
    cfg.input_width = static_cast<uint32_t>(bundle.source_width);
    cfg.input_height = static_cast<uint32_t>(bundle.source_height);
    cfg.output_width = static_cast<uint32_t>(bundle.output_width);
    cfg.output_height = static_cast<uint32_t>(bundle.output_height);
    cfg.fx = bundle.fx;
    cfg.fy = bundle.fy;
    cfg.cx = bundle.cx;
    cfg.cy = bundle.cy;
    cfg.camera_to_gyro_offset_ns = bundle.camera_to_gyro_offset_ns + bundle.offset_delta_ns;
    for (int i = 0; i < 9; ++i) cfg.gyro_to_camera[i] = (i % 4 == 0) ? 1.0 : 0.0;
    cfg.base_crop_budget_rad = 0.25;
    cfg.rs19_margin_rad = 0.005;
    cfg.default_zoom = 1.12; // Constant Tripod Zoom 1.12x

    GeminiEisHandle handle = nullptr;
    if (GeminiEisInit(&handle, &cfg) != GEMINI_EIS_OK || !handle) {
        std::cerr << "ERROR: Failed to initialize GeminiEis Session!\n";
        return 3;
    }

    std::string meshPath = (argc > 2) ? std::string(argv[2]) : (bundleDir + "/gemini_mesh.f32");
    std::ofstream meshOut(meshPath, std::ios::binary);

    const double W = static_cast<double>(bundle.source_width);
    const double H = static_cast<double>(bundle.source_height);
    const double baseCrop = bundle.base_crop > 0.0 ? bundle.base_crop : 0.80;

    size_t gyroCursor = 0;
    int hardViolations = 0;
    double minU = 1.0, maxU = 0.0, minV = 1.0, maxV = 0.0;

    const auto t0 = std::chrono::high_resolution_clock::now();
    for (size_t i = 0; i < bundle.frames.size(); ++i) {
        const auto& fm = bundle.frames[i];
        const int64_t neededGyroTs = fm.sensor_ts_ns + fm.exposure_ns / 2 + fm.skew_ns + 50000000LL;
        std::vector<GeminiEisGyroSample> batch;
        while (gyroCursor < gyro.size() && gyro[gyroCursor].timestamp_ns <= neededGyroTs) {
            batch.push_back(gyro[gyroCursor++]);
        }
        if (!batch.empty()) {
            GeminiEisPushGyro(&handle, batch.data(), static_cast<int>(batch.size()));
        }

        if (oracleMats.size() >= (i + 1) * 9) {
            const double* m = &oracleMats[i * 9];
            GeminiEisVisualMotion vm{};
            vm.timestamp_ns = fm.sensor_ts_ns;
            vm.tx_px = ((m[0] - 1.0) * 0.5 + m[1] * 0.5 + m[2]) * (W * baseCrop);
            vm.ty_px = (m[3] * 0.5 + (m[4] - 1.0) * 0.5 + m[5]) * (H * baseCrop);
            vm.rotation_rad = std::atan2(m[3] * (H / W), m[0]);
            vm.scale = std::hypot(m[0], m[3] * (H / W));
            vm.trust = 1.0;
            vm.inliers = 100;
            if (invert3x3(m, vm.h_inv)) {
                vm.has_homography = 1;
            }
            GeminiEisPushVisual(&handle, &vm);
        }

        GeminiEisFrameInfo fi{};
        fi.sensor_timestamp_ns = fm.sensor_ts_ns;
        fi.exposure_time_ns = fm.exposure_ns;
        fi.rolling_shutter_skew_ns = fm.skew_ns;
        fi.zoom = 1.12; // Constant Tripod Zoom 1.12x

        GeminiEisResult res{};
        int rc = GeminiEisProcessFrame(&handle, &fi, &res);
        if (rc != GEMINI_EIS_OK) {
            std::cerr << "Frame " << i << " returned error code: " << rc << "\n";
            GeminiEisUninit(&handle);
            return 4;
        }

        for (uint32_t k = 0; k < res.mesh_vertex_count; ++k) {
            const float u = static_cast<float>(res.mesh[k].u);
            const float v = static_cast<float>(res.mesh[k].v);
            if (u < 0.0f || u > 1.0f || v < 0.0f || v > 1.0f) {
                hardViolations++;
            }
            minU = std::min(minU, static_cast<double>(u));
            maxU = std::max(maxU, static_cast<double>(u));
            minV = std::min(minV, static_cast<double>(v));
            maxV = std::max(maxV, static_cast<double>(v));
            meshOut.write(reinterpret_cast<const char*>(&u), sizeof(float));
            meshOut.write(reinterpret_cast<const char*>(&v), sizeof(float));
        }
    }
    const auto t1 = std::chrono::high_resolution_clock::now();
    const double avgUs = std::chrono::duration<double, std::micro>(t1 - t0).count() / bundle.frames.size();

    std::cout << "Average Frame Latency     : " << avgUs << " us (Gate < 1500 us)\n";
    std::cout << "Mesh UV Range             : u=[" << minU << ", " << maxU << "], v=[" << minV << ", " << maxV << "]\n";
    std::cout << "Hard Boundary Violations  : " << hardViolations << " (Gate == 0)\n";
    std::cout << "VERDICT: " << (hardViolations == 0 ? "GEMINI_EIS_REPLAY_GATE_PASS" : "GEMINI_EIS_REPLAY_GATE_FAIL") << "\n";

    GeminiEisUninit(&handle);
    return hardViolations == 0 ? 0 : 5;
}
