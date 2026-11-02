#include "api/gemini_eis_c_api.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <deque>
#include <memory>
#include <new>
#include <vector>

#include "common/ring_buffer.hpp"
#include "core/gemini_eis_core.h"

namespace {

using gemini::FrameInput;
using gemini::GeminiEisCore;
using gemini::GyroRingBuffer;
using gemini::GyroSample;
using gemini::Mat3;
using gemini::Quat;
using gemini::Similarity2D;
using gemini::Vec2;
using gemini::Vec3;
using gemini::WarpRequest;

constexpr const char* kVersion = "2.0.0-causal-live";
constexpr const char* kBuildId = "gemini-eis-unified-cabi-v2";
constexpr int kMeshRows = 20;
constexpr int kMeshCols = 33;
constexpr int kMeshVerts = kMeshRows * kMeshCols; // 660

struct FramePoseSnapshot {
    int64_t sensorTsNs = 0;
    int64_t exposureNs = 0;
    int64_t readoutNs = 0;
    double zoom = 1.12;
    Quat qCenter = Quat::identity();
    Quat qTarget = Quat::identity();
    std::array<Quat, kMeshRows> rowQuats{};
    double omegaNormRadS = 0.0;
};

struct CausalKeyframe {
    FramePoseSnapshot pose{};
    int width = 0;
    int height = 0;
    std::vector<uint8_t> yImg;
    std::vector<uint8_t> yImgPyr1;
    std::vector<Vec2> sensorPts;
    std::vector<Vec2> targetVirt;
};

struct Context {
    GeminiEisConfig config{};
    GeminiEisViewport viewport{};
    std::unique_ptr<GeminiEisCore> core;
    GyroRingBuffer gyro{400000000};
    bool haveLastFrame = false;
    int64_t lastFrameCenterGyroNs = 0;
    Vec3 rawAngle{};
    Vec3 lastOmega{};
    bool haveLastOmega = false;
    Similarity2D visual{};
    int64_t visualTimestampNs = 0;
    uint64_t frameIndex = 0;

    // Causal Real-Time 8-DoF + Keyframe Ring Pool + 20x33 RBF state
    std::deque<FramePoseSnapshot> poseHistory{};
    std::deque<CausalKeyframe> keyframes{};
    std::array<double, 8> smoothedDelta{};
    std::array<double, kMeshVerts> duRbfSmooth{};
    std::array<double, kMeshVerts> dvRbfSmooth{};
    std::array<Vec2, kMeshVerts> lastDeltaM{};
    double lastObsMinusTgtX = 0.0;
    double lastObsMinusTgtY = 0.0;
    std::array<double, 9> lastRigidH{1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
    bool hasDeltaM = false;
    bool retireVisualCorrection = false;
};

Vec3 matrixMul(const double m[9], const Vec3& v) {
    return {
        m[0] * v.x + m[1] * v.y + m[2] * v.z,
        m[3] * v.x + m[4] * v.y + m[5] * v.z,
        m[6] * v.x + m[7] * v.y + m[8] * v.z
    };
}

double determinant3(const double m[9]) {
    return m[0] * (m[4] * m[8] - m[5] * m[7])
         - m[1] * (m[3] * m[8] - m[5] * m[6])
         + m[2] * (m[3] * m[7] - m[4] * m[6]);
}

bool invert3x3(const double m[9], double invOut[9]) {
    const double det = determinant3(m);
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

void mul3x3(const double A[9], const double B[9], double C[9]) {
    double tmp[9]{};
    for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) {
            tmp[r * 3 + c] = A[r * 3 + 0] * B[0 * 3 + c]
                           + A[r * 3 + 1] * B[1 * 3 + c]
                           + A[r * 3 + 2] * B[2 * 3 + c];
        }
    }
    std::memcpy(C, tmp, sizeof(tmp));
}

Vec2 applyHomography(const double H[9], double x, double y) {
    const double w = H[6] * x + H[7] * y + H[8];
    const double invW = std::abs(w) > 1e-8 ? (1.0 / w) : 1.0;
    return {(H[0] * x + H[1] * y + H[2]) * invW,
            (H[3] * x + H[4] * y + H[5]) * invW};
}

bool solve8x8(double A[8][8], double b[8], double x[8]) {
    for (int i = 0; i < 8; ++i) {
        int piv = i;
        double maxVal = std::abs(A[i][i]);
        for (int r = i + 1; r < 8; ++r) {
            if (std::abs(A[r][i]) > maxVal) {
                maxVal = std::abs(A[r][i]);
                piv = r;
            }
        }
        if (maxVal < 1e-12) return false;
        if (piv != i) {
            for (int c = i; c < 8; ++c) std::swap(A[i][c], A[piv][c]);
            std::swap(b[i], b[piv]);
        }
        const double invDiag = 1.0 / A[i][i];
        for (int r = i + 1; r < 8; ++r) {
            const double factor = A[r][i] * invDiag;
            for (int c = i; c < 8; ++c) A[r][c] -= factor * A[i][c];
            b[r] -= factor * b[i];
        }
    }
    for (int i = 7; i >= 0; --i) {
        double sum = b[i];
        for (int c = i + 1; c < 8; ++c) sum -= A[i][c] * x[c];
        x[i] = sum / A[i][i];
    }
    return true;
}

bool validConfig(const GeminiEisConfig& c) {
    return c.api_version == GEMINI_EIS_API_VERSION
        && c.struct_size >= sizeof(GeminiEisConfig)
        && c.input_width > 0 && c.input_height > 0
        && c.output_width > 0 && c.output_height > 0
        && std::isfinite(c.fx) && c.fx > 0.0
        && std::isfinite(c.fy) && c.fy > 0.0
        && std::isfinite(c.cx) && std::isfinite(c.cy)
        && std::abs(determinant3(c.gyro_to_camera)) > 0.5;
}

Vec3 signedIntegrate(const GyroRingBuffer& gyro, int64_t a, int64_t b) {
    if (a == b) return {};
    if (b > a) return gyro.integrate(a, b);
    return gyro.integrate(b, a) * -1.0;
}

bool sameViewport(const GeminiEisViewport& a, const GeminiEisViewport& b) {
    return a.buffer_width == b.buffer_width
        && a.buffer_height == b.buffer_height
        && a.output_width == b.output_width
        && a.output_height == b.output_height
        && std::abs(a.roi_x - b.roi_x) < 1e-6
        && std::abs(a.roi_y - b.roi_y) < 1e-6
        && std::abs(a.roi_width - b.roi_width) < 1e-6
        && std::abs(a.roi_height - b.roi_height) < 1e-6;
}

void resetRuntime(Context& c) {
    gemini::CoreConfig cfg;
    cfg.warp.width = static_cast<int>(c.config.input_width);
    cfg.warp.height = static_cast<int>(c.config.input_height);
    cfg.warp.fxPx = c.config.fx;
    cfg.warp.fyPx = c.config.fy;
    cfg.warp.cxPx = c.config.cx;
    cfg.warp.cyPx = c.config.cy;
    cfg.warp.baseCrop = 0.80;
    cfg.motion.baseCropBudgetRad = c.config.base_crop_budget_rad;
    cfg.motion.rs19MarginRad = c.config.rs19_margin_rad;
    c.core = std::make_unique<GeminiEisCore>(cfg);
    c.gyro = GyroRingBuffer(2000000000LL);
    c.haveLastFrame = false;
    c.lastFrameCenterGyroNs = 0;
    c.rawAngle = {};
    c.lastOmega = {};
    c.haveLastOmega = false;
    c.visual = {};
    c.visualTimestampNs = 0;
    c.frameIndex = 0;
    c.poseHistory.clear();
    c.keyframes.clear();
    c.smoothedDelta.fill(0.0);
    c.duRbfSmooth.fill(0.0);
    c.dvRbfSmooth.fill(0.0);
    c.lastDeltaM.fill({});
    c.lastObsMinusTgtX = 0.0;
    c.lastObsMinusTgtY = 0.0;
    c.lastRigidH = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
    c.hasDeltaM = false;
    c.retireVisualCorrection = false;
}

void applyViewportToMesh(const Context& c, GeminiEisResult* out) {
    if (!out || c.viewport.buffer_width == 0 || c.viewport.buffer_height == 0
            || c.viewport.roi_width <= 0.0 || c.viewport.roi_height <= 0.0) {
        return;
    }

    const double sx = c.viewport.roi_width / c.viewport.buffer_width;
    const double sy = c.viewport.roi_height / c.viewport.buffer_height;
    const double ox = c.viewport.roi_x / c.viewport.buffer_width;
    const double oy = c.viewport.roi_y / c.viewport.buffer_height;
    for (uint32_t i = 0; i < out->mesh_vertex_count; ++i) {
        out->mesh[i].u = ox + out->mesh[i].u * sx;
        out->mesh[i].v = oy + out->mesh[i].v * sy;
    }
}

/* Closed-form Sensor [0,1]^2 -> Virtual Tripod Screen [0,1]^2 (0 iterations) */
Vec2 unwarpSensorPointToVirtual(const GeminiEisConfig& cfg, double u_s, double v_s, const FramePoseSnapshot& pose) {
    const double W = static_cast<double>(cfg.input_width);
    const double H = static_cast<double>(cfg.input_height);
    const double fx = cfg.fx;
    const double fy = cfg.fy;
    const double cx = cfg.cx;
    const double cy = cfg.cy;
    const double baseCrop = 0.80;
    const double zoom = pose.zoom > 0.0 ? pose.zoom : 1.12;

    const double rf = gemini::clamp(v_s, 0.0, 1.0) * 19.0;
    const size_t i0 = std::min<size_t>(18, static_cast<size_t>(rf));
    const double alpha = rf - static_cast<double>(i0);
    const Quat qRow = Quat::slerp(pose.rowQuats[i0], pose.rowQuats[i0 + 1], alpha);

    constexpr double k1 = -0.0150;
    constexpr double k2 =  0.0;
    constexpr double k3 =  0.0;
    constexpr double p1 =  0.0;
    constexpr double p2 =  0.0;
    auto distortBC = [&](double x, double y, double& xd, double& yd) {
        const double r2 = x * x + y * y;
        const double r4 = r2 * r2;
        const double r6 = r4 * r2;
        const double radial = 1.0 + k1 * r2 + k2 * r4 + k3 * r6;
        const double xy2 = 2.0 * x * y;
        xd = x * radial + p1 * xy2 + p2 * (r2 + 2.0 * x * x);
        yd = y * radial + p1 * (r2 + 2.0 * y * y) + p2 * xy2;
    };

    const double sx_d = (u_s * W - cx) / fx;
    const double sy_d = (v_s * H - cy) / fy;
    double sx_u = sx_d, sy_u = sy_d;
    for (int it = 0; it < 4; ++it) {
        double fx0 = 0.0, fy0 = 0.0;
        distortBC(sx_u, sy_u, fx0, fy0);
        sx_u -= (fx0 - sx_d);
        sy_u -= (fy0 - sy_d);
    }
    const Vec3 ray_phys{sx_u, sy_u, 1.0};
    const Mat3 rVirtFromPhys = (pose.qTarget.inverse() * qRow).matrix();
    const Vec3 ray_virt = rVirtFromPhys * ray_phys;
    const double z = std::max(1e-4, ray_virt.z);
    const double vxn = ray_virt.x / z;
    const double vyn = ray_virt.y / z;
    double vxo = vxn, vyo = vyn;
    distortBC(vxn, vyn, vxo, vyo);
    const double x_pix = fx * vxo + cx;
    const double y_pix = fy * vyo + cy;

    const double u_zoom = 0.5 + (x_pix / W - 0.5) / baseCrop;
    const double v_zoom = 0.5 + (y_pix / H - 0.5) / baseCrop;
    return {0.5 + (u_zoom - 0.5) * zoom, 0.5 + (v_zoom - 0.5) * zoom};
}

Vec2 projectVirtualToSensor(const GeminiEisConfig& cfg, double u0, double v0, const FramePoseSnapshot& pose) {
    constexpr double k1 = -0.0150;
    constexpr double k2 =  0.0;
    constexpr double k3 =  0.0;
    constexpr double p1 =  0.0;
    constexpr double p2 =  0.0;
    auto distortBC = [&](double x, double y, double& xd, double& yd) {
        const double r2 = x * x + y * y;
        const double r4 = r2 * r2;
        const double r6 = r4 * r2;
        const double radial = 1.0 + k1 * r2 + k2 * r4 + k3 * r6;
        const double xy2 = 2.0 * x * y;
        xd = x * radial + p1 * xy2 + p2 * (r2 + 2.0 * x * x);
        yd = y * radial + p1 * (r2 + 2.0 * y * y) + p2 * xy2;
    };

    const double W = static_cast<double>(cfg.input_width);
    const double H = static_cast<double>(cfg.input_height);
    const double fx = cfg.fx;
    const double fy = cfg.fy;
    const double cx = cfg.cx;
    const double cy = cfg.cy;
    const double baseCrop = 0.80;
    const double zoom = pose.zoom > 0.0 ? pose.zoom : 1.12;

    const double u_zoom = 0.5 + (u0 - 0.5) / zoom;
    const double v_zoom = 0.5 + (v0 - 0.5) / zoom;
    const double x_pix = (0.5 + (u_zoom - 0.5) * baseCrop) * W;
    const double y_pix = (0.5 + (v_zoom - 0.5) * baseCrop) * H;
    const double rx_d = (x_pix - cx) / fx;
    const double ry_d = (y_pix - cy) / fy;
    double rx_u = rx_d, ry_u = ry_d;
    for (int it = 0; it < 4; ++it) {
        double fx0 = 0.0, fy0 = 0.0;
        distortBC(rx_u, ry_u, fx0, fy0);
        rx_u -= (fx0 - rx_d);
        ry_u -= (fy0 - ry_d);
    }
    const Vec3 ray_virt{rx_u, ry_u, 1.0};

    double x_sensor = x_pix, y_sensor = y_pix;
    double current_row = gemini::clamp(y_pix / H, 0.0, 1.0);
    for (int iter = 0; iter < 6; ++iter) {
        const double rf = gemini::clamp(current_row, 0.0, 1.0) * 19.0;
        const size_t i0 = std::min<size_t>(18, static_cast<size_t>(rf));
        const double u = rf - static_cast<double>(i0);
        const Quat qRow = Quat::slerp(pose.rowQuats[i0], pose.rowQuats[i0 + 1], u);
        const Mat3 rRel = (qRow.inverse() * pose.qTarget).matrix();
        const Vec3 ray_phys = rRel * ray_virt;
        if (ray_phys.z > 1e-4) {
            const double xu = ray_phys.x / ray_phys.z;
            const double yu = ray_phys.y / ray_phys.z;
            double xo = xu, yo = yu;
            distortBC(xu, yu, xo, yo);
            x_sensor = fx * xo + cx;
            y_sensor = fy * yo + cy;
            const double next_row = gemini::clamp(y_sensor / H, 0.0, 1.0);
            if (std::abs(next_row - current_row) < 1e-6) break;
            current_row = next_row;
        } else {
            break;
        }
    }
    return {x_sensor / W, y_sensor / H};
}

float sampleBilinear(const std::vector<uint8_t>& img, int w, int h, double x, double y) {
    x = gemini::clamp(x, 0.0, static_cast<double>(w - 1));
    y = gemini::clamp(y, 0.0, static_cast<double>(h - 1));
    const int x0 = static_cast<int>(x);
    const int y0 = static_cast<int>(y);
    const int x1 = std::min(w - 1, x0 + 1);
    const int y1 = std::min(h - 1, y0 + 1);
    const double fx = x - x0;
    const double fy = y - y0;
    const double v00 = img[static_cast<size_t>(y0 * w + x0)];
    const double v01 = img[static_cast<size_t>(y0 * w + x1)];
    const double v10 = img[static_cast<size_t>(y1 * w + x0)];
    const double v11 = img[static_cast<size_t>(y1 * w + x1)];
    return static_cast<float>((1.0 - fy) * ((1.0 - fx) * v00 + fx * v01) +
                              fy * ((1.0 - fx) * v10 + fx * v11));
}

void buildPyr1(const std::vector<uint8_t>& src, int w, int h, std::vector<uint8_t>& dst) {
    const int w1 = w / 2;
    const int h1 = h / 2;
    dst.resize(static_cast<size_t>(w1 * h1));
    for (int y = 0; y < h1; ++y) {
        const uint8_t* r0 = src.data() + (2 * y) * w;
        const uint8_t* r1 = src.data() + (2 * y + 1) * w;
        for (int x = 0; x < w1; ++x) {
            const int sum = static_cast<int>(r0[2 * x]) + static_cast<int>(r0[2 * x + 1])
                          + static_cast<int>(r1[2 * x]) + static_cast<int>(r1[2 * x + 1]);
            dst[static_cast<size_t>(y * w1 + x)] = static_cast<uint8_t>((sum + 2) >> 2);
        }
    }
}

void extractGridFeatures(const std::vector<uint8_t>& img, int w, int h, std::vector<Vec2>& outPts) {
    outPts.clear();
    const int gx = 8, gy = 5, border = 6;
    const int cw = (w - 2 * border) / gx;
    const int ch = (h - 2 * border) / gy;
    for (int iy = 0; iy < gy; ++iy) {
        for (int ix = 0; ix < gx; ++ix) {
            const int x0 = border + ix * cw;
            const int y0 = border + iy * ch;
            double bestScore1 = 20.0, bestScore2 = 20.0;
            Vec2 bestP1{-1.0, -1.0}, bestP2{-1.0, -1.0};
            for (int y = y0 + 1; y < y0 + ch - 1; y += 2) {
                for (int x = x0 + 1; x < x0 + cw - 1; x += 2) {
                    double sxx = 0.0, syy = 0.0, sxy = 0.0;
                    for (int dy = -1; dy <= 1; ++dy) {
                        for (int dx = -1; dx <= 1; ++dx) {
                            const int idx = (y + dy) * w + (x + dx);
                            const double gxVal = 0.5 * (img[static_cast<size_t>(idx + 1)] - img[static_cast<size_t>(idx - 1)]);
                            const double gyVal = 0.5 * (img[static_cast<size_t>(idx + w)] - img[static_cast<size_t>(idx - w)]);
                            sxx += gxVal * gxVal;
                            syy += gyVal * gyVal;
                            sxy += gxVal * gyVal;
                        }
                    }
                    const double tr = sxx + syy;
                    const double det = sxx * syy - sxy * sxy;
                    const double minEig = 0.5 * (tr - std::sqrt(std::max(0.0, tr * tr - 4.0 * det)));
                    if (minEig > bestScore1) {
                        bestScore2 = bestScore1; bestP2 = bestP1;
                        bestScore1 = minEig; bestP1 = {static_cast<double>(x), static_cast<double>(y)};
                    } else if (minEig > bestScore2 && std::hypot(x - bestP1.x, y - bestP1.y) > 3.5) {
                        bestScore2 = minEig; bestP2 = {static_cast<double>(x), static_cast<double>(y)};
                    }
                }
            }
            if (bestP1.x > 0.0) outPts.push_back(bestP1);
            if (bestP2.x > 0.0) outPts.push_back(bestP2);
        }
    }
}

bool solveLkLevel(
    const std::vector<uint8_t>& refImg,
    const std::vector<uint8_t>& curImg,
    int w, int h,
    double x0, double y0,
    double& ioX1, double& ioY1,
    int maxIters) {
    if (x0 < 3.5 || x0 >= w - 3.5 || y0 < 3.5 || y0 >= h - 3.5) return false;
    if (ioX1 < 3.5 || ioX1 >= w - 3.5 || ioY1 < 3.5 || ioY1 >= h - 3.5) return false;

    double sxx = 0.0, syy = 0.0, sxy = 0.0;
    float tpl[25]{}, gxArr[25]{}, gyArr[25]{};
    int k = 0;
    for (int dy = -2; dy <= 2; ++dy) {
        for (int dx = -2; dx <= 2; ++dx, ++k) {
            const double px = x0 + dx;
            const double py = y0 + dy;
            tpl[k] = sampleBilinear(refImg, w, h, px, py);
            const float gx = 0.5f * (sampleBilinear(refImg, w, h, px + 1.0, py) -
                                     sampleBilinear(refImg, w, h, px - 1.0, py));
            const float gy = 0.5f * (sampleBilinear(refImg, w, h, px, py + 1.0) -
                                     sampleBilinear(refImg, w, h, px, py - 1.0));
            gxArr[k] = gx;
            gyArr[k] = gy;
            sxx += gx * gx;
            syy += gy * gy;
            sxy += gx * gy;
        }
    }
    const double det = sxx * syy - sxy * sxy;
    if (det < 8.0) return false;
    const double invDet = 1.0 / det;

    for (int iter = 0; iter < maxIters; ++iter) {
        double bx = 0.0, by = 0.0;
        int m = 0;
        for (int dy = -2; dy <= 2; ++dy) {
            for (int dx = -2; dx <= 2; ++dx, ++m) {
                const float curVal = sampleBilinear(curImg, w, h, ioX1 + dx, ioY1 + dy);
                const double diff = tpl[m] - curVal;
                bx += gxArr[m] * diff;
                by += gyArr[m] * diff;
            }
        }
        const double stepX = (syy * bx - sxy * by) * invDet;
        const double stepY = (-sxy * bx + sxx * by) * invDet;
        ioX1 += stepX;
        ioY1 += stepY;
        if (ioX1 < 3.0 || ioX1 >= w - 3.0 || ioY1 < 3.0 || ioY1 >= h - 3.0) {
            return false;
        }
        if (stepX * stepX + stepY * stepY < 1e-4) break;
    }
    return true;
}

void trackKeyframeKlt(
    const Context& ctx,
    const CausalKeyframe& kf,
    const FramePoseSnapshot& currPose,
    const std::vector<uint8_t>& curImg,
    const std::vector<uint8_t>& curImgPyr1,
    int w, int h,
    std::vector<Vec2>& ioObsVirt,
    std::vector<Vec2>& ioTgtVirt) {

    const GeminiEisConfig& cfg = ctx.config;
    const int w1 = w / 2;
    const int h1 = h / 2;
    const bool hasPyr1 = (kf.yImgPyr1.size() == static_cast<size_t>(w1 * h1)) &&
                         (curImgPyr1.size() == static_cast<size_t>(w1 * h1));

    // Bounded motion-predicted search initialization (clamped to +-8px in 640x360)
    const double shiftX = gemini::clamp(ctx.lastObsMinusTgtX, -8.0, 8.0);
    const double shiftY = gemini::clamp(ctx.lastObsMinusTgtY, -8.0, 8.0);

    for (size_t idx = 0; idx < kf.sensorPts.size(); ++idx) {
        const Vec2& p0 = kf.sensorPts[idx];
        const Vec2& tgtV = kf.targetVirt[idx];

        const double predVx = gemini::clamp((tgtV.x + shiftX) / 640.0, 0.02, 0.98);
        const double predVy = gemini::clamp((tgtV.y + shiftY) / 360.0, 0.02, 0.98);
        const Vec2 predSens = projectVirtualToSensor(cfg, predVx, predVy, currPose);
        double x1 = predSens.x * w - 0.5;
        double y1 = predSens.y * h - 0.5;
        if (x1 < 4.5 || x1 >= w - 4.5 || y1 < 4.5 || y1 >= h - 4.5) continue;

        // Coarse-to-fine 2-level Pyramidal Lucas-Kanade
        if (hasPyr1) {
            double x1_c = 0.5 * (x1 - 0.5);
            double y1_c = 0.5 * (y1 - 0.5);
            const double x0_c = 0.5 * (p0.x - 0.5);
            const double y0_c = 0.5 * (p0.y - 0.5);
            if (solveLkLevel(kf.yImgPyr1, curImgPyr1, w1, h1, x0_c, y0_c, x1_c, y1_c, 4)) {
                const double candX1 = 2.0 * x1_c + 0.5;
                const double candY1 = 2.0 * y1_c + 0.5;
                if (std::hypot(candX1 - x1, candY1 - y1) < 4.5) {
                    x1 = candX1;
                    y1 = candY1;
                }
            }
        }

        if (!solveLkLevel(kf.yImg, curImg, w, h, p0.x, p0.y, x1, y1, 6)) continue;

        // Forward-Backward consistency check (curImg -> kf.yImg)
        double xBack = p0.x;
        double yBack = p0.y;
        if (!solveLkLevel(curImg, kf.yImg, w, h, x1, y1, xBack, yBack, 4)) continue;
        if (std::hypot(xBack - p0.x, yBack - p0.y) > 0.70) continue;

        const Vec2 curVirt = unwarpSensorPointToVirtual(cfg, (x1 + 0.5) / w, (y1 + 0.5) / h, currPose);
        const Vec2 obsV{curVirt.x * 640.0, curVirt.y * 360.0};
        if (std::hypot(obsV.x - tgtV.x, obsV.y - tgtV.y) < 16.0) {
            ioObsVirt.push_back(obsV);
            ioTgtVirt.push_back(tgtV);
        }
    }
}

void applyDeltaMToMesh(
    const Context& ctx,
    const std::array<Vec2, kMeshVerts>& deltaM,
    GeminiEisResult* ioResult) {
    if (!ioResult) return;
    const double minU = 16.0 / std::max<uint32_t>(640, ctx.config.input_width);
    const double maxU = 1.0 - minU;
    const double minV = 16.0 / std::max<uint32_t>(360, ctx.config.input_height);
    const double maxV = 1.0 - minV;

    const size_t centerIdx = 10 * kMeshCols + 16;
    const double uCenter = gemini::clamp(ioResult->mesh[centerIdx].u, 0.25, 0.75);
    const double vCenter = gemini::clamp(ioResult->mesh[centerIdx].v, 0.25, 0.75);
    auto isSafeWithBreath = [&](double breathScale, double alpha) {
        for (size_t k = 0; k < kMeshVerts; ++k) {
            const double uRaw = ioResult->mesh[k].u + alpha * deltaM[k].x;
            const double vRaw = ioResult->mesh[k].v + alpha * deltaM[k].y;
            const double u = uCenter + breathScale * (uRaw - uCenter);
            const double v = vCenter + breathScale * (vRaw - vCenter);
            if (u < minU || u > maxU || v < minV || v > maxV) return false;
        }
        return true;
    };

    double breathScale = 1.0;
    double alphaSafe = 1.0;
    if (!isSafeWithBreath(1.0, 1.0)) {
        if (isSafeWithBreath(0.992, 1.0)) {
            double blo = 0.992, bhi = 1.0;
            for (int iter = 0; iter < 10; ++iter) {
                const double bmid = 0.5 * (blo + bhi);
                if (isSafeWithBreath(bmid, 1.0)) blo = bmid;
                else bhi = bmid;
            }
            breathScale = blo;
        } else {
            breathScale = 0.992;
            double lo = 0.0, hi = 1.0;
            for (int iter = 0; iter < 14; ++iter) {
                const double mid = 0.5 * (lo + hi);
                if (isSafeWithBreath(breathScale, mid)) lo = mid;
                else hi = mid;
            }
            alphaSafe = lo;
        }
    }

    for (size_t k = 0; k < kMeshVerts; ++k) {
        const double uRaw = ioResult->mesh[k].u + alphaSafe * deltaM[k].x;
        const double vRaw = ioResult->mesh[k].v + alphaSafe * deltaM[k].y;
        ioResult->mesh[k].u = gemini::clamp(uCenter + breathScale * (uRaw - uCenter), minU, maxU);
        ioResult->mesh[k].v = gemini::clamp(vCenter + breathScale * (vRaw - vCenter), minV, maxV);
    }
}

void applyHeldDeltaMToMesh(Context& ctx, GeminiEisResult* ioResult) {
    if (!ctx.hasDeltaM || !ioResult) return;

    const double decay = ctx.retireVisualCorrection ? 0.95 : 0.995;
    double maxMagnitude = 0.0;
    for (size_t k = 0; k < kMeshVerts; ++k) {
        ctx.lastDeltaM[k].x *= decay;
        ctx.lastDeltaM[k].y *= decay;
        maxMagnitude = std::max(maxMagnitude,
            std::hypot(ctx.lastDeltaM[k].x, ctx.lastDeltaM[k].y));
    }

    if (ctx.retireVisualCorrection && maxMagnitude < 1e-6) {
        ctx.lastDeltaM.fill({});
        ctx.hasDeltaM = false;
        ctx.retireVisualCorrection = false;
        return;
    }
    applyDeltaMToMesh(ctx, ctx.lastDeltaM, ioResult);
}

void applyCausal8DofAndRbfToMesh(
    Context& ctx,
    const std::vector<Vec2>& ptsObsIn,
    const std::vector<Vec2>& ptsTgt,
    double omegaNormRadS,
    GeminiEisResult* ioResult) {

    const size_t n = ptsObsIn.size();
    if (n < 12 || !ioResult) {
        applyHeldDeltaMToMesh(ctx, ioResult);
        return;
    }

    // Handheld-aware sub-pixel deadband to suppress 30Hz KLT quantization & sensor ISO noise
    std::vector<Vec2> ptsObs = ptsObsIn;
    if (omegaNormRadS < 0.085) {
        const double calmFactor = 1.0 - gemini::clamp(omegaNormRadS / 0.085, 0.0, 1.0);
        const double deadbandPx = 0.45 + 0.55 * calmFactor; // Up to 1.0px in 640x360 (0.25px in 160x90 KLT grid)
        for (size_t i = 0; i < n; ++i) {
            const double dx = ptsObs[i].x - ptsTgt[i].x;
            const double dy = ptsObs[i].y - ptsTgt[i].y;
            const double mag = std::hypot(dx, dy);
            if (mag <= deadbandPx) {
                ptsObs[i] = ptsTgt[i];
            } else {
                const double scale = (mag - deadbandPx) / mag;
                ptsObs[i].x = ptsTgt[i].x + dx * scale;
                ptsObs[i].y = ptsTgt[i].y + dy * scale;
            }
        }
    }

    // V6 RANSAC + Tukey Biweight Outlier Rejection on KLT feature displacements:
    // Prevents moving foreground objects (hands, pedestrians) or repetitive-texture false matches
    // from dragging the global visual lock!
    std::vector<double> dxList(n), dyList(n);
    for (size_t i = 0; i < n; ++i) {
        dxList[i] = ptsTgt[i].x - ptsObs[i].x;
        dyList[i] = ptsTgt[i].y - ptsObs[i].y;
    }
    std::vector<double> dxSorted = dxList, dySorted = dyList;
    std::nth_element(dxSorted.begin(), dxSorted.begin() + n / 2, dxSorted.end());
    std::nth_element(dySorted.begin(), dySorted.begin() + n / 2, dySorted.end());
    const double medDx = dxSorted[n / 2];
    const double medDy = dySorted[n / 2];

    std::vector<double> priorW(n, 0.0);
    double priorSum = 0.0;
    size_t inlierCount = 0;
    Vec2 cObs{0.0, 0.0}, cTgt{0.0, 0.0};
    constexpr double kTukeyCutPx = 2.0; // Strict 2.0px consensus gate in 640x360 virtual space
    for (size_t i = 0; i < n; ++i) {
        const double errPx = std::hypot(dxList[i] - medDx, dyList[i] - medDy);
        if (errPx >= kTukeyCutPx) continue;
        const double u = errPx / kTukeyCutPx;
        const double tukeyW = (1.0 - u * u) * (1.0 - u * u);
        const double ux = std::abs(ptsTgt[i].x - 320.0) / 320.0;
        const double vy = std::abs(ptsTgt[i].y - 180.0) / 180.0;
        priorW[i] = tukeyW * (1.0 + 2.2 * ux * ux + 0.5 * vy * vy);
        priorSum += priorW[i];
        cObs.x += priorW[i] * ptsObs[i].x; cObs.y += priorW[i] * ptsObs[i].y;
        cTgt.x += priorW[i] * ptsTgt[i].x; cTgt.y += priorW[i] * ptsTgt[i].y;
        inlierCount++;
    }
    if (inlierCount < 10 || priorSum < 1e-6) {
        applyHeldDeltaMToMesh(ctx, ioResult);
        return;
    }
    cObs.x /= priorSum; cObs.y /= priorSum;
    cTgt.x /= priorSum; cTgt.y /= priorSum;

    double num = 0.0, den = 0.0;
    for (size_t i = 0; i < n; ++i) {
        if (priorW[i] <= 0.0) continue;
        const double qx0 = ptsObs[i].x - cObs.x, qy0 = ptsObs[i].y - cObs.y;
        const double qx1 = ptsTgt[i].x - cTgt.x, qy1 = ptsTgt[i].y - cTgt.y;
        num += priorW[i] * (qx0 * qy1 - qy0 * qx1);
        den += priorW[i] * (qx0 * qx1 + qy0 * qy1);
    }
    const double th = gemini::clamp(std::atan2(num, den), -0.008, 0.008);
    const double ca = std::cos(th), sa = std::sin(th);
    const double tx = gemini::clamp(cTgt.x - (ca * cObs.x - sa * cObs.y), -6.5, 6.5);
    const double ty = gemini::clamp(cTgt.y - (sa * cObs.x + ca * cObs.y), -6.5, 6.5);
    const double H_rigid[9] = {ca, -sa, tx, sa, ca, ty, 0.0, 0.0, 1.0};
    ctx.lastObsMinusTgtX = gemini::clamp(cObs.x - cTgt.x, -6.5, 6.5);
    ctx.lastObsMinusTgtY = gemini::clamp(cObs.y - cTgt.y, -6.5, 6.5);
    for (int k = 0; k < 9; ++k) ctx.lastRigidH[static_cast<size_t>(k)] = H_rigid[k];

    const double cx = 320.0, cy = 180.0;
    std::vector<double> weights(n, 1.0);
    double delta[8]{};
    // Rigid-dominant regularization: full translation/rotation lock while strongly suppressing
    // non-rigid affine/keystone border deformation (delta[0,1,3,4,6,7]) that causes left/right edge wobble!
    const double lam_aff = 1.25, lam_trans = 0.015, lam_iso = 2.20, lam_persp = 2.60;
    const double huber = 1.5 / cx;

    for (int iter = 0; iter < 4; ++iter) {
        double ATA[8][8]{};
        double ATb[8]{};
        const double regDiag[8] = {lam_aff, lam_aff, lam_trans, lam_aff, lam_aff, lam_trans, lam_persp, lam_persp};
        for (int d = 0; d < 8; ++d) ATA[d][d] = regDiag[d];
        ATA[0][0] += lam_iso; ATA[0][4] += lam_iso;
        ATA[4][0] += lam_iso; ATA[4][4] += lam_iso;

        for (size_t i = 0; i < n; ++i) {
            const Vec2 pb = applyHomography(H_rigid, ptsObs[i].x, ptsObs[i].y);
            const double x = (pb.x - cx) / cx;
            const double y = (pb.y - cy) / cy;
            const double xp = (ptsTgt[i].x - cx) / cx;
            const double yp = (ptsTgt[i].y - cy) / cy;
            const double w = weights[i] * priorW[i];

            const double rowX[8] = {x, y, 1.0, 0.0, 0.0, 0.0, -x * xp, -y * xp};
            const double rhsX = xp - x;
            const double rowY[8] = {0.0, 0.0, 0.0, x, y, 1.0, -x * yp, -y * yp};
            const double rhsY = yp - y;

            for (int r = 0; r < 8; ++r) {
                ATb[r] += w * (rowX[r] * rhsX + rowY[r] * rhsY);
                for (int c = 0; c < 8; ++c) {
                    ATA[r][c] += w * (rowX[r] * rowX[c] + rowY[r] * rowY[c]);
                }
            }
        }
        if (!solve8x8(ATA, ATb, delta)) break;

        for (size_t i = 0; i < n; ++i) {
            const Vec2 pb = applyHomography(H_rigid, ptsObs[i].x, ptsObs[i].y);
            const double x = (pb.x - cx) / cx;
            const double y = (pb.y - cy) / cy;
            const double xp = (ptsTgt[i].x - cx) / cx;
            const double yp = (ptsTgt[i].y - cy) / cy;
            const double rx = x * delta[0] + y * delta[1] + delta[2] - x * xp * delta[6] - y * xp * delta[7] - (xp - x);
            const double ry = x * delta[3] + y * delta[4] + delta[5] - x * yp * delta[6] - y * yp * delta[7] - (yp - y);
            const double rMag = std::hypot(rx, ry);
            weights[i] = (rMag <= huber) ? 1.0 : (huber / (rMag + 1e-12));
        }
    }

    // V5 Pure Rigid SE(2) Global Lock:
    // Lock non-translation affine/keystone terms to zero so all 660 mesh vertices move as a 100% solid rigid body,
    // completely eliminating wave-like left/right border wobble during yaw swings!
    delta[0] = 0.0;
    delta[1] = 0.0;
    delta[3] = 0.0;
    delta[4] = 0.0;
    delta[6] = 0.0;
    delta[7] = 0.0;
    ctx.smoothedDelta.fill(0.0);

    const double H_res_norm[9] = {
        1.0, 0.0, delta[2],
        0.0, 1.0, delta[5],
        0.0, 0.0, 1.0
    };
    const double T_norm[9] = {1.0 / cx, 0.0, -1.0, 0.0, 1.0 / cy, -1.0, 0.0, 0.0, 1.0};
    const double T_inv[9]  = {cx, 0.0, cx, 0.0, cy, cy, 0.0, 0.0, 1.0};
    double tmp[9]{}, H_res_pix[9]{}, H_p2[9]{}, H_tgt_to_obs[9]{};
    mul3x3(T_inv, H_res_norm, tmp);
    mul3x3(tmp, T_norm, H_res_pix);
    mul3x3(H_res_pix, H_rigid, H_p2);
    if (!invert3x3(H_p2, H_tgt_to_obs)) {
        if (!invert3x3(H_rigid, H_tgt_to_obs)) return;
    }

    std::vector<Vec2> rbfLoc;
    std::vector<Vec2> rbfRes;
    for (size_t i = 0; i < n; ++i) {
        const Vec2 mappedObs = applyHomography(H_tgt_to_obs, ptsTgt[i].x, ptsTgt[i].y);
        const double ru = (ptsObs[i].x - mappedObs.x) / 640.0;
        const double rv = (ptsObs[i].y - mappedObs.y) / 360.0;
        if (std::hypot(ru * 640.0, rv * 360.0) < 4.5) {
            rbfLoc.push_back({ptsTgt[i].x / 640.0, ptsTgt[i].y / 360.0});
            rbfRes.push_back({ru, rv});
        }
    }

    const double inv2Sig2 = 1.0 / (2.0 * 0.25 * 0.25);
    const double lamRbf = 80.0; // Strongly damp non-rigid local patch shimmering to preserve fine texture sharpness
    const double alphaRbf = 0.03;

    std::array<double, kMeshVerts> duTotal{};
    std::array<double, kMeshVerts> dvTotal{};

    for (int r = 0; r < kMeshRows; ++r) {
        const double vg = static_cast<double>(r) / (kMeshRows - 1);
        for (int c = 0; c < kMeshCols; ++c) {
            const double ug = static_cast<double>(c) / (kMeshCols - 1);
            const size_t idx = static_cast<size_t>(r * kMeshCols + c);
            const double gx = ug * 640.0 - 0.5;
            const double gy = vg * 360.0 - 0.5;
            const Vec2 gObs = applyHomography(H_tgt_to_obs, gx, gy);
            const double duHom = (gObs.x - gx) / 640.0;
            const double dvHom = (gObs.y - gy) / 360.0;

            (void)inv2Sig2; (void)lamRbf; (void)alphaRbf;
            ctx.duRbfSmooth[idx] = 0.0;
            ctx.dvRbfSmooth[idx] = 0.0;
            duTotal[idx] = duHom;
            dvTotal[idx] = dvHom;
        }
    }

    std::array<Vec2, kMeshVerts> deltaM{};
    constexpr double kMaxDeltaUv = 0.0042; // Bounded fine visual authority (+-10.7px on 2560x1440)
    for (int r = 0; r < kMeshRows; ++r) {
        const int rPrev = std::max(0, r - 1);
        const int rNext = std::min(kMeshRows - 1, r + 1);
        const double scaleV = (rNext > rPrev) ? (static_cast<double>(kMeshRows - 1) / (rNext - rPrev)) : 0.0;
        for (int c = 0; c < kMeshCols; ++c) {
            const int cPrev = std::max(0, c - 1);
            const int cNext = std::min(kMeshCols - 1, c + 1);
            const double scaleU = (cNext > cPrev) ? (static_cast<double>(kMeshCols - 1) / (cNext - cPrev)) : 0.0;
            const size_t idx = static_cast<size_t>(r * kMeshCols + c);

            const Vec2 dM_du{
                (ioResult->mesh[r * kMeshCols + cNext].u - ioResult->mesh[r * kMeshCols + cPrev].u) * scaleU,
                (ioResult->mesh[r * kMeshCols + cNext].v - ioResult->mesh[r * kMeshCols + cPrev].v) * scaleU
            };
            const Vec2 dM_dv{
                (ioResult->mesh[rNext * kMeshCols + c].u - ioResult->mesh[rPrev * kMeshCols + c].u) * scaleV,
                (ioResult->mesh[rNext * kMeshCols + c].v - ioResult->mesh[rPrev * kMeshCols + c].v) * scaleV
            };
            double dxUv = dM_du.x * duTotal[idx] + dM_dv.x * dvTotal[idx];
            double dyUv = dM_du.y * duTotal[idx] + dM_dv.y * dvTotal[idx];
            const double magUv = std::hypot(dxUv, dyUv);
            if (magUv > kMaxDeltaUv) {
                const double s = kMaxDeltaUv / magUv;
                dxUv *= s;
                dyUv *= s;
            }
            deltaM[idx] = {dxUv, dyUv};
        }
    }

    ctx.retireVisualCorrection = false;
    // 1-Euro Speed-Adaptive Filter:
    //   - When innovation is tiny (< 0.00045 UV ~ 1.1px on 2560x1440), use ultra-low alpha=0.040
    //     to eliminate 30Hz KLT sub-pixel micro-jitter so fine details stay razor-sharp ("不发虚").
    //   - When innovation is large (> 0.0022 UV ~ 5.6px, real wrist translation during swing),
    //     ramp alpha smoothly up to 0.32 for instant parallax lock.
    const size_t centerIdx = 10 * kMeshCols + 16;
    const double innovUv = ctx.hasDeltaM
        ? std::hypot(deltaM[centerIdx].x - ctx.lastDeltaM[centerIdx].x,
                     deltaM[centerIdx].y - ctx.lastDeltaM[centerIdx].y)
        : 0.0015;
    const double speedGain = gemini::clamp((innovUv - 0.00038) / 0.00140, 0.0, 1.0);
    const double swingGain = gemini::clamp((omegaNormRadS - 0.060) / 0.250, 0.0, 1.0);
    // Fast parallax cancellation up to 0.55 rad/s, then gentle motion-blur gate above 0.55 rad/s
    const double blurGate = 1.0 - 0.55 * gemini::clamp((omegaNormRadS - 0.55) / 0.60, 0.0, 1.0);
    const double alphaMesh = (0.040 + 0.340 * std::max(speedGain, swingGain * speedGain)) * blurGate;
    constexpr double kMaxStepUv = 0.0028;
    if (!ctx.hasDeltaM) {
        for (size_t k = 0; k < kMeshVerts; ++k) {
            ctx.lastDeltaM[k].x = alphaMesh * deltaM[k].x;
            ctx.lastDeltaM[k].y = alphaMesh * deltaM[k].y;
        }
        ctx.hasDeltaM = true;
    } else {
        for (size_t k = 0; k < kMeshVerts; ++k) {
            double stepX = alphaMesh * (deltaM[k].x - ctx.lastDeltaM[k].x);
            double stepY = alphaMesh * (deltaM[k].y - ctx.lastDeltaM[k].y);
            const double stepMag = std::hypot(stepX, stepY);
            if (stepMag > kMaxStepUv) {
                const double s = kMaxStepUv / stepMag;
                stepX *= s;
                stepY *= s;
            }
            ctx.lastDeltaM[k].x += stepX;
            ctx.lastDeltaM[k].y += stepY;
        }
    }
    applyDeltaMToMesh(ctx, ctx.lastDeltaM, ioResult);
}

int processFrameInternal(Context& c, const GeminiEisFrameInfo* frame, GeminiEisResult* result, FramePoseSnapshot* outSnap) {
    std::memset(result, 0, sizeof(*result));

    const int64_t readoutNs = frame->rolling_shutter_skew_ns > 0
        ? frame->rolling_shutter_skew_ns : c.config.default_readout_ns;
    const int64_t centerCameraNs = frame->sensor_timestamp_ns
        + frame->exposure_time_ns / 2 + readoutNs / 2;
    const int64_t centerGyroNs = centerCameraNs + c.config.camera_to_gyro_offset_ns;

    const int64_t firstRowGyroNs = frame->sensor_timestamp_ns
        + frame->exposure_time_ns / 2
        + static_cast<int64_t>(readoutNs * (0.5 / GEMINI_EIS_RS_BANDS))
        + c.config.camera_to_gyro_offset_ns;
    const int64_t lastRowGyroNs = frame->sensor_timestamp_ns
        + frame->exposure_time_ns / 2
        + static_cast<int64_t>(readoutNs * ((GEMINI_EIS_RS_BANDS - 0.5)
            / GEMINI_EIS_RS_BANDS))
        + c.config.camera_to_gyro_offset_ns;

    if (!c.gyro.covers(firstRowGyroNs) || !c.gyro.covers(lastRowGyroNs)
            || !c.gyro.covers(centerGyroNs)) {
        return GEMINI_EIS_ECOVERAGE;
    }

    GyroSample center{};
    if (!c.gyro.sample(centerGyroNs, &center)) return GEMINI_EIS_ECOVERAGE;

    double frameDtS = 1.0 / 30.0;
    if (c.haveLastFrame) {
        frameDtS = (centerGyroNs - c.lastFrameCenterGyroNs) * 1e-9;
        if (!(frameDtS > 1e-4 && frameDtS < 0.25)) {
            return GEMINI_EIS_ESTATE;
        }
        c.rawAngle = c.rawAngle + signedIntegrate(
            c.gyro, c.lastFrameCenterGyroNs, centerGyroNs);
    }
    c.lastFrameCenterGyroNs = centerGyroNs;
    c.haveLastFrame = true;

    Vec3 accel{};
    if (c.haveLastOmega) {
        accel = (center.omegaRadS - c.lastOmega) * (1.0 / frameDtS);
    }
    c.lastOmega = center.omegaRadS;
    c.haveLastOmega = true;

    Quat qCenter = Quat::identity();
    c.gyro.orientationAt(centerGyroNs, &qCenter);

    FrameInput in;
    in.timestampNs = frame->sensor_timestamp_ns;
    in.rawAngleRad = c.rawAngle;
    in.angularVelocityRadS = center.omegaRadS;
    in.angularAccelerationRadS2 = accel;
    in.zoom = frame->zoom > 0.0 ? frame->zoom : c.config.default_zoom;
    in.qCenter = qCenter;
    in.useQuats = true;

    if (c.visualTimestampNs > 0 && c.visualTimestampNs <= frame->sensor_timestamp_ns) {
        const double ageS = (frame->sensor_timestamp_ns - c.visualTimestampNs) * 1e-9;
        if (ageS <= 0.20 && c.visual.trust > 0.01) {
            const double decay = std::exp(-ageS / 0.60);
            in.visual = c.visual;

            const int64_t visGyroNs = c.visualTimestampNs + c.config.camera_to_gyro_offset_ns;
            if (c.gyro.covers(visGyroNs) && c.gyro.covers(centerGyroNs)) {
                const Vec3 dR = signedIntegrate(c.gyro, visGyroNs, centerGyroNs);
                const double cosR = std::cos(dR.z);
                const double sinR = std::sin(dR.z);
                const double tx = c.visual.txPx;
                const double ty = c.visual.tyPx;
                in.visual.txPx = (cosR * tx - sinR * ty) * decay;
                in.visual.tyPx = (sinR * tx + cosR * ty) * decay;
            } else {
                in.visual.txPx *= decay;
                in.visual.tyPx *= decay;
            }
            in.visual.rotationRad *= decay;
            in.visual.scale = 1.0 + (in.visual.scale - 1.0) * decay;
            in.visual.trust *= decay;
        }
    }

    for (uint32_t row = 0; row < 20; ++row) {
        const double y = static_cast<double>(row) / 19.0;
        const int64_t rowCameraNs = frame->sensor_timestamp_ns
            + frame->exposure_time_ns / 2
            + static_cast<int64_t>(readoutNs * y);
        const int64_t rowGyroNs = rowCameraNs + c.config.camera_to_gyro_offset_ns;
        Quat qRow = Quat::identity();
        c.gyro.orientationAt(rowGyroNs, &qRow);
        in.rowQuats[row] = qRow;
        if (row < GEMINI_EIS_RS_BANDS) {
            in.rs19[row] = signedIntegrate(c.gyro, centerGyroNs, rowGyroNs);
        }
    }

    const auto out = c.core->processFrame(in);
    result->valid = 1;
    result->applied = 1;
    result->frame_index = ++c.frameIndex;

    result->raw_angle_rad[0] = c.rawAngle.x;
    result->raw_angle_rad[1] = c.rawAngle.y;
    result->raw_angle_rad[2] = c.rawAngle.z;
    result->target_angle_rad[0] = out.targetAngleRad.x;
    result->target_angle_rad[1] = out.targetAngleRad.y;
    result->target_angle_rad[2] = out.targetAngleRad.z;
    result->correction_rad[0] = out.correctionRad.x;
    result->correction_rad[1] = out.correctionRad.y;
    result->correction_rad[2] = out.correctionRad.z;
    result->jerk_rad_s3[0] = out.jerkRadS3.x;
    result->jerk_rad_s3[1] = out.jerkRadS3.y;
    result->jerk_rad_s3[2] = out.jerkRadS3.z;
    result->pan_confidence = out.panConfidence;
    result->oscillation_score = out.oscillationScore;
    result->crop_budget_rad[0] = out.crop.pitchRad;
    result->crop_budget_rad[1] = out.crop.yawRad;
    result->crop_budget_rad[2] = out.crop.rollRad;
    result->visual_tx_px = out.visual.txPx;
    result->visual_ty_px = out.visual.tyPx;
    result->visual_rotation_rad = out.visual.rotationRad;
    result->visual_scale = out.visual.scale;
    result->visual_trust = out.visual.trust;

    result->mesh_vertex_count = static_cast<uint32_t>(
        std::min<std::size_t>(out.mesh.size(), GEMINI_EIS_MESH_VERTEX_COUNT));
    for (uint32_t i = 0; i < result->mesh_vertex_count; ++i) {
        result->mesh[i].u = out.mesh[i].u;
        result->mesh[i].v = out.mesh[i].v;
    }

    FramePoseSnapshot snap{};
    snap.sensorTsNs = frame->sensor_timestamp_ns;
    snap.exposureNs = frame->exposure_time_ns;
    snap.readoutNs = readoutNs;
    snap.zoom = out.zoom > 0.0 ? out.zoom : 1.12;
    snap.qCenter = qCenter;
    snap.qTarget = out.qTarget;
    snap.rowQuats = in.rowQuats;
    snap.omegaNormRadS = center.omegaRadS.norm();
    c.poseHistory.push_back(snap);
    while (c.poseHistory.size() > 180) c.poseHistory.pop_front();
    if (outSnap) *outSnap = snap;
    return GEMINI_EIS_OK;
}

}  // namespace

extern "C" {

void GeminiEisDefaultConfig(GeminiEisConfig* c) {
    if (!c) return;
    std::memset(c, 0, sizeof(*c));
    c->struct_size = sizeof(*c);
    c->api_version = GEMINI_EIS_API_VERSION;
    c->input_width = 2560;
    c->input_height = 1440;
    c->output_width = 1920;
    c->output_height = 1080;
    c->fx = 972.0;
    c->fy = 976.0;
    c->cx = 1280.0;
    c->cy = 720.0;
    c->default_readout_ns = 29000000;
    c->camera_to_gyro_offset_ns = 4000000;
    // Y1 (AR_6769) verified gyro_to_camera = {0, 1, 0,  1, 0, 0,  0, 0, -1}
    c->gyro_to_camera[0] = 0.0; c->gyro_to_camera[1] = 1.0; c->gyro_to_camera[2] = 0.0;
    c->gyro_to_camera[3] = 1.0; c->gyro_to_camera[4] = 0.0; c->gyro_to_camera[5] = 0.0;
    c->gyro_to_camera[6] = 0.0; c->gyro_to_camera[7] = 0.0; c->gyro_to_camera[8] = -1.0;
    c->base_crop_budget_rad = 0.25;
    c->rs19_margin_rad = 0.005;
    c->default_zoom = 1.12;
}

int GeminiEisInit(GeminiEisHandle* handle, const GeminiEisConfig* config) {
    if (!handle || !config) return GEMINI_EIS_EINVAL;
    *handle = nullptr;
    if (!validConfig(*config)) return GEMINI_EIS_ECALIBRATION;

    auto* c = new (std::nothrow) Context();
    if (!c) return GEMINI_EIS_ESTATE;
    c->config = *config;
    c->viewport.buffer_width = config->input_width;
    c->viewport.buffer_height = config->input_height;
    c->viewport.output_width = config->output_width;
    c->viewport.output_height = config->output_height;
    c->viewport.roi_x = 0.0;
    c->viewport.roi_y = 0.0;
    c->viewport.roi_width = config->input_width;
    c->viewport.roi_height = config->input_height;
    resetRuntime(*c);
    *handle = c;
    return GEMINI_EIS_OK;
}

void GeminiEisUninit(GeminiEisHandle* handle) {
    if (!handle || !*handle) return;
    delete static_cast<Context*>(*handle);
    *handle = nullptr;
}

int GeminiEisReset(GeminiEisHandle* handle) {
    if (!handle || !*handle) return GEMINI_EIS_EINVAL;
    resetRuntime(*static_cast<Context*>(*handle));
    return GEMINI_EIS_OK;
}

int GeminiEisSetViewport(GeminiEisHandle* handle, const GeminiEisViewport* viewport) {
    if (!handle || !*handle || !viewport) return GEMINI_EIS_EINVAL;
    if (viewport->buffer_width == 0 || viewport->buffer_height == 0
            || viewport->output_width == 0 || viewport->output_height == 0
            || viewport->roi_width <= 0.0 || viewport->roi_height <= 0.0
            || viewport->roi_x < 0.0 || viewport->roi_y < 0.0
            || viewport->roi_x + viewport->roi_width > viewport->buffer_width + 0.5
            || viewport->roi_y + viewport->roi_height > viewport->buffer_height + 0.5) {
        return GEMINI_EIS_EINVAL;
    }
    auto& c = *static_cast<Context*>(*handle);
    if (!sameViewport(c.viewport, *viewport)) {
        c.viewport = *viewport;
    }
    return GEMINI_EIS_OK;
}

int GeminiEisPushGyro(GeminiEisHandle* handle,
                      const GeminiEisGyroSample* samples,
                      int count) {
    if (!handle || !*handle || !samples || count <= 0) return GEMINI_EIS_EINVAL;
    auto& c = *static_cast<Context*>(*handle);
    try {
        for (int i = 0; i < count; ++i) {
            const Vec3 physical{samples[i].x_rad_s, samples[i].y_rad_s, samples[i].z_rad_s};
            const Vec3 camera = matrixMul(c.config.gyro_to_camera, physical);
            c.gyro.push({samples[i].timestamp_ns, camera});
        }
    } catch (...) {
        return GEMINI_EIS_ESTATE;
    }
    return GEMINI_EIS_OK;
}

int GeminiEisPushVisual(GeminiEisHandle* handle,
                        const GeminiEisVisualMotion* visual) {
    if (!handle || !*handle || !visual) return GEMINI_EIS_EINVAL;
    auto& c = *static_cast<Context*>(*handle);
    c.visualTimestampNs = visual->timestamp_ns;
    c.visual.txPx = visual->tx_px;
    c.visual.tyPx = visual->ty_px;
    c.visual.rotationRad = visual->rotation_rad;
    c.visual.scale = visual->scale > 0.0 ? visual->scale : 1.0;
    c.visual.trust = gemini::clamp(visual->trust, 0.0, 1.0);
    c.visual.inliers = visual->inliers;
    c.visual.hasHomography = (visual->has_homography != 0);
    if (c.visual.hasHomography) {
        for (size_t k = 0; k < 9; ++k) {
            c.visual.hInv[k] = visual->h_inv[k];
        }
    }
    return GEMINI_EIS_OK;
}

int GeminiEisProcessFrame(GeminiEisHandle* handle,
                          const GeminiEisFrameInfo* frame,
                          GeminiEisResult* result) {
    if (!handle || !*handle || !frame || !result) return GEMINI_EIS_EINVAL;
    if (frame->sensor_timestamp_ns <= 0 || frame->exposure_time_ns <= 0) {
        return GEMINI_EIS_EINVAL;
    }
    auto& c = *static_cast<Context*>(*handle);
    int rc = processFrameInternal(c, frame, result, nullptr);
    if (rc != GEMINI_EIS_OK) return rc;
    applyHeldDeltaMToMesh(c, result);
    applyViewportToMesh(c, result);
    return GEMINI_EIS_OK;
}

int GeminiEisProcessLiveYPlane(
    GeminiEisHandle* handle,
    const GeminiEisFrameInfo* frame,
    const uint8_t* y_plane,
    int y_width,
    int y_height,
    int y_row_stride,
    GeminiEisResult* result) {
    if (!handle || !*handle || !frame || !result) return GEMINI_EIS_EINVAL;
    if (frame->sensor_timestamp_ns <= 0 || frame->exposure_time_ns <= 0) {
        return GEMINI_EIS_EINVAL;
    }
    auto& c = *static_cast<Context*>(*handle);
    FramePoseSnapshot currPose{};
    int rc = processFrameInternal(c, frame, result, &currPose);
    if (rc != GEMINI_EIS_OK) return rc;

    if (y_plane && y_width >= 64 && y_height >= 48 && y_row_stride >= y_width) {
        const int wWork = 320;
        const int hWork = 180;
        std::vector<uint8_t> curImg(static_cast<size_t>(wWork * hWork));
        // 2x2 box-filtered downsampling to suppress live sensor photon/ISO noise
        for (int y = 0; y < hWork; ++y) {
            const int sy0 = (y * y_height) / hWork;
            const int sy1 = std::min(y_height - 1, sy0 + 1);
            const uint8_t* row0 = y_plane + sy0 * y_row_stride;
            const uint8_t* row1 = y_plane + sy1 * y_row_stride;
            for (int x = 0; x < wWork; ++x) {
                const int sx0 = (x * y_width) / wWork;
                const int sx1 = std::min(y_width - 1, sx0 + 1);
                const int sum = static_cast<int>(row0[sx0]) + static_cast<int>(row0[sx1])
                              + static_cast<int>(row1[sx0]) + static_cast<int>(row1[sx1]);
                curImg[static_cast<size_t>(y * wWork + x)] = static_cast<uint8_t>((sum + 2) >> 2);
            }
        }

        std::vector<uint8_t> curImgPyr1;
        buildPyr1(curImg, wWork, hWork, curImgPyr1);

        // Only evict stale keyframes when virtual camera qTarget has panned > 6.0 deg (0.105 rad),
        // preventing mid-swing keyframe dropouts and 3-6px visual resets!
        while (!c.keyframes.empty()) {
            const double targetShiftRad = (c.keyframes.front().pose.qTarget.inverse() * currPose.qTarget).angle();
            if (targetShiftRad > 0.105) {
                c.keyframes.pop_front();
                c.retireVisualCorrection = c.keyframes.empty() && c.hasDeltaM;
            } else {
                break;
            }
        }

        std::vector<Vec2> ptsObs;
        std::vector<Vec2> ptsTgt;
        for (const auto& kf : c.keyframes) {
            trackKeyframeKlt(c, kf, currPose, curImg, curImgPyr1, wWork, hWork, ptsObs, ptsTgt);
        }

        if (ptsObs.size() >= 12) {
            applyCausal8DofAndRbfToMesh(c, ptsObs, ptsTgt, currPose.omegaNormRadS, result);
        } else {
            applyHeldDeltaMToMesh(c, result);
        }

        const bool needFirstKf = c.keyframes.empty();
        const bool lowMotionBlur = (currPose.omegaNormRadS < 0.65);
        const int64_t minKfSpacingNs = (c.keyframes.size() < 2) ? 400000000LL : 1200000000LL;
        const bool spacedEnough = c.keyframes.empty() ||
            (currPose.sensorTsNs - c.keyframes.back().pose.sensorTsNs) > minKfSpacingNs;
        if (needFirstKf || (lowMotionBlur && spacedEnough)) {
            CausalKeyframe newKf;
            newKf.pose = currPose;
            newKf.width = wWork;
            newKf.height = hWork;
            newKf.yImg = std::move(curImg);
            newKf.yImgPyr1 = std::move(curImgPyr1);
            extractGridFeatures(newKf.yImg, wWork, hWork, newKf.sensorPts);
            newKf.targetVirt.reserve(newKf.sensorPts.size());
            for (const auto& sp : newKf.sensorPts) {
                const Vec2 v = unwarpSensorPointToVirtual(
                    c.config, (sp.x + 0.5) / wWork, (sp.y + 0.5) / hWork, currPose);
                Vec2 tgtV{v.x * 640.0, v.y * 360.0};
                if (!needFirstKf) {
                    // Anchor subsequent keyframes in the exact same visually-locked virtual coordinate frame as kf[0]
                    // to prevent 1.2s keyframe-replacement target creep!
                    tgtV = applyHomography(c.lastRigidH.data(), tgtV.x, tgtV.y);
                }
                newKf.targetVirt.push_back(tgtV);
            }
            if (newKf.sensorPts.size() >= 16) {
                c.keyframes.push_back(std::move(newKf));
                while (c.keyframes.size() > 3) {
                    c.keyframes.erase(c.keyframes.begin() + 1);
                }
            }
        }
    } else {
        applyHeldDeltaMToMesh(c, result);
    }

    applyViewportToMesh(c, result);
    return GEMINI_EIS_OK;
}

int GeminiEisProcessCausalFeatureMatches(
    GeminiEisHandle* handle,
    const GeminiEisFrameInfo* frame,
    int64_t ref_sensor_timestamp_ns,
    int64_t ref_exposure_time_ns,
    int64_t ref_rolling_shutter_skew_ns,
    int feature_domain_width,
    int feature_domain_height,
    const GeminiEisFeaturePointPair* matches,
    int match_count,
    GeminiEisResult* result) {
    if (!handle || !*handle || !frame || !result) return GEMINI_EIS_EINVAL;
    auto& c = *static_cast<Context*>(*handle);
    FramePoseSnapshot currPose{};
    int rc = processFrameInternal(c, frame, result, &currPose);
    if (rc != GEMINI_EIS_OK) return rc;

    if (matches && match_count >= 12 && feature_domain_width > 0 && feature_domain_height > 0) {
        FramePoseSnapshot refPose = currPose;
        for (auto it = c.poseHistory.rbegin(); it != c.poseHistory.rend(); ++it) {
            if (std::abs(it->sensorTsNs - ref_sensor_timestamp_ns) < 5000000LL) {
                refPose = *it;
                break;
            }
        }
        (void)ref_exposure_time_ns;
        (void)ref_rolling_shutter_skew_ns;

        const double invW = 1.0 / static_cast<double>(feature_domain_width);
        const double invH = 1.0 / static_cast<double>(feature_domain_height);
        std::vector<Vec2> ptsTgt;
        std::vector<Vec2> ptsObs;
        for (int i = 0; i < match_count; ++i) {
            const Vec2 refV = unwarpSensorPointToVirtual(
                c.config, (matches[i].ref_x_px + 0.5) * invW, (matches[i].ref_y_px + 0.5) * invH, refPose);
            const Vec2 curV = unwarpSensorPointToVirtual(
                c.config, (matches[i].curr_x_px + 0.5) * invW, (matches[i].curr_y_px + 0.5) * invH, currPose);
            ptsTgt.push_back({refV.x * 640.0, refV.y * 360.0});
            ptsObs.push_back({curV.x * 640.0, curV.y * 360.0});
        }
        applyCausal8DofAndRbfToMesh(c, ptsObs, ptsTgt, currPose.omegaNormRadS, result);
    }

    applyViewportToMesh(c, result);
    return GEMINI_EIS_OK;
}

int GeminiEisExportGlesMesh(
    const GeminiEisResult* result,
    const float* st_matrix_4x4,
    int orientation_deg,
    float* out_interleaved_xyuv_2640) {
    if (!result || !out_interleaved_xyuv_2640 || result->mesh_vertex_count < GEMINI_EIS_MESH_VERTEX_COUNT) {
        return GEMINI_EIS_EINVAL;
    }

    for (int r = 0; r < kMeshRows; ++r) {
        const float v_screen = static_cast<float>(r) / (kMeshRows - 1);
        for (int c = 0; c < kMeshCols; ++c) {
            const float u_screen = static_cast<float>(c) / (kMeshCols - 1);
            float ndc_x = u_screen * 2.0f - 1.0f;
            float ndc_y = 1.0f - v_screen * 2.0f;
            if (orientation_deg == 180) {
                ndc_x = -ndc_x;
                ndc_y = -ndc_y;
            } else if (orientation_deg == 90) {
                const float tx = ndc_x;
                ndc_x = ndc_y;
                ndc_y = -tx;
            } else if (orientation_deg == 270) {
                const float tx = ndc_x;
                ndc_x = -ndc_y;
                ndc_y = tx;
            }

            const int idx = r * kMeshCols + c;
            float u_tex = static_cast<float>(result->mesh[idx].u);
            float v_tex = static_cast<float>(result->mesh[idx].v);

            if (st_matrix_4x4) {
                const float tu = st_matrix_4x4[0] * u_tex + st_matrix_4x4[4] * v_tex + st_matrix_4x4[12];
                const float tv = st_matrix_4x4[1] * u_tex + st_matrix_4x4[5] * v_tex + st_matrix_4x4[13];
                u_tex = tu;
                v_tex = tv;
            }

            float* dst = out_interleaved_xyuv_2640 + idx * 4;
            dst[0] = ndc_x;
            dst[1] = ndc_y;
            dst[2] = u_tex;
            dst[3] = v_tex;
        }
    }
    return GEMINI_EIS_OK;
}

int GeminiEisBuildGlesIndices(uint16_t* out_indices_3648) {
    if (!out_indices_3648) return GEMINI_EIS_EINVAL;
    int ptr = 0;
    for (int r = 0; r < kMeshRows - 1; ++r) {
        for (int c = 0; c < kMeshCols - 1; ++c) {
            const uint16_t tl = static_cast<uint16_t>(r * kMeshCols + c);
            const uint16_t tr = static_cast<uint16_t>(tl + 1);
            const uint16_t bl = static_cast<uint16_t>(tl + kMeshCols);
            const uint16_t br = static_cast<uint16_t>(bl + 1);
            out_indices_3648[ptr++] = tl;
            out_indices_3648[ptr++] = bl;
            out_indices_3648[ptr++] = tr;
            out_indices_3648[ptr++] = tr;
            out_indices_3648[ptr++] = bl;
            out_indices_3648[ptr++] = br;
        }
    }
    return GEMINI_EIS_OK;
}

int GeminiEisExportMtkWpeMap(
    const GeminiEisResult* result,
    uint32_t sensor_width,
    uint32_t sensor_height,
    uint32_t map_stride_elements,
    int fixed_point_shift,
    int32_t* out_wpe_x_fixed,
    int32_t* out_wpe_y_fixed,
    float* out_wpe_x_float,
    float* out_wpe_y_float) {
    if (!result || sensor_width == 0 || sensor_height == 0 || result->mesh_vertex_count < GEMINI_EIS_MESH_VERTEX_COUNT) {
        return GEMINI_EIS_EINVAL;
    }
    const uint32_t stride = map_stride_elements >= GEMINI_EIS_MESH_COLS_VERTS
        ? map_stride_elements : GEMINI_EIS_MESH_COLS_VERTS;
    const double scaleFixed = static_cast<double>(1 << gemini::clamp(fixed_point_shift, 0, 12));
    const double maxX = static_cast<double>(sensor_width - 1);
    const double maxY = static_cast<double>(sensor_height - 1);

    for (uint32_t r = 0; r < GEMINI_EIS_MESH_ROWS_VERTS; ++r) {
        for (uint32_t c = 0; c < GEMINI_EIS_MESH_COLS_VERTS; ++c) {
            const uint32_t srcIdx = r * GEMINI_EIS_MESH_COLS_VERTS + c;
            const uint32_t dstIdx = r * stride + c;
            const double px = gemini::clamp(result->mesh[srcIdx].u * sensor_width - 0.5, 0.0, maxX);
            const double py = gemini::clamp(result->mesh[srcIdx].v * sensor_height - 0.5, 0.0, maxY);

            if (out_wpe_x_fixed) out_wpe_x_fixed[dstIdx] = static_cast<int32_t>(std::llround(px * scaleFixed));
            if (out_wpe_y_fixed) out_wpe_y_fixed[dstIdx] = static_cast<int32_t>(std::llround(py * scaleFixed));
            if (out_wpe_x_float) out_wpe_x_float[dstIdx] = static_cast<float>(px);
            if (out_wpe_y_float) out_wpe_y_float[dstIdx] = static_cast<float>(py);
        }
    }
    return GEMINI_EIS_OK;
}

const char* GeminiEisGetVersion(void) {
    return kVersion;
}

const char* GeminiEisGetBuildId(void) {
    return kBuildId;
}

}  // extern "C"
