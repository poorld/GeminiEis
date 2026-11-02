#include "warp/rs19_one_warp.h"

#include <algorithm>
#include <cmath>

namespace gemini {

Rs19OneWarp::Rs19OneWarp(Rs19Config config) : config_(config) {}

WarpVertex Rs19OneWarp::projectPoint(double u0, double v0, const WarpRequest& request) const {
    const double W = config_.width > 0 ? static_cast<double>(config_.width) : 2560.0;
    const double H = config_.height > 0 ? static_cast<double>(config_.height) : 1440.0;
    const double fx = config_.fxPx > 0.0 ? config_.fxPx : 1202.3;
    const double fy = config_.fyPx > 0.0 ? config_.fyPx : 1208.5;
    const double cx = config_.cxPx > 0.0 ? config_.cxPx : W * 0.5;
    const double cy = config_.cyPx > 0.0 ? config_.cyPx : H * 0.5;
    const double baseCrop = (config_.baseCrop > 0.0 && config_.baseCrop <= 1.0) ? config_.baseCrop : 0.80;
    const double zoom = request.zoom > 0.0 ? request.zoom : 1.0;

    const double u_zoom = 0.5 + (u0 - 0.5) / zoom;
    const double v_zoom = 0.5 + (v0 - 0.5) / zoom;

    double x_pix = 0.0;
    double y_pix = 0.0;

    if (request.visual.hasHomography) {
        const auto& h = request.visual.hInv;
        const double denom = std::max(1e-6, h[6] * u_zoom + h[7] * v_zoom + h[8]);
        const double u_corr = (h[0] * u_zoom + h[1] * v_zoom + h[2]) / denom;
        const double v_corr = (h[3] * u_zoom + h[4] * v_zoom + h[5]) / denom;
        x_pix = (0.5 + (u_corr - 0.5) * baseCrop) * W;
        y_pix = (0.5 + (v_corr - 0.5) * baseCrop) * H;
    } else {
        const double x_crop_px = (u_zoom - 0.5) * (W * baseCrop) - request.visual.txPx;
        const double y_crop_px = (v_zoom - 0.5) * (H * baseCrop) - request.visual.tyPx;
        const double cosR = std::cos(-request.visual.rotationRad);
        const double sinR = std::sin(-request.visual.rotationRad);
        const double invScale = request.visual.scale > 1e-4 ? (1.0 / request.visual.scale) : 1.0;
        const double x_rot_px = invScale * (cosR * x_crop_px - sinR * y_crop_px);
        const double y_rot_px = invScale * (sinR * x_crop_px + cosR * y_crop_px);
        x_pix = 0.5 * W + x_rot_px;
        y_pix = 0.5 * H + y_rot_px;
    }

    const double v_crop = y_pix / H;

    // Calibrated Y1 (IMX476) symmetric radial Brown-Conrady lens model (undistortNewton -> 3D rotate -> redistort)
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

    double x_sensor = x_pix;
    double y_sensor = y_pix;

    if (request.useQuats) {
        double current_row = std::clamp(v_crop, 0.0, 1.0);
        for (int iter = 0; iter < 6; ++iter) {
            const double rf = std::clamp(current_row, 0.0, 1.0) * 19.0;
            const size_t i0 = std::min<size_t>(18, static_cast<size_t>(rf));
            const double u = rf - static_cast<double>(i0);
            const Quat qRow = Quat::slerp(request.rowQuats[i0], request.rowQuats[i0 + 1], u);
            const Quat qTgt = request.hasRowTargets
                ? Quat::slerp(request.rowTargets[i0], request.rowTargets[i0 + 1], u)
                : request.qTarget;
            const Mat3 rRel = (qRow.inverse() * qTgt).matrix();
            const Vec3 ray_phys = rRel * ray_virt;
            if (ray_phys.z > 1e-4) {
                const double xu = ray_phys.x / ray_phys.z;
                const double yu = ray_phys.y / ray_phys.z;
                double xo = xu, yo = yu;
                distortBC(xu, yu, xo, yo);
                x_sensor = fx * xo + cx;
                y_sensor = fy * yo + cy;
                const double next_row = std::clamp(y_sensor / H, 0.0, 1.0);
                if (std::abs(next_row - current_row) < 1e-6) {
                    break;
                }
                current_row = next_row;
            } else {
                break;
            }
        }
    } else {
        const int band = std::min(config_.rowBands - 1, static_cast<int>(std::clamp(v0, 0.0, 0.9999) * config_.rowBands));
        const Vec3 rr = request.rowCorrectionRad[static_cast<size_t>(band)];
        const Vec3 rot{
            -request.globalCorrectionRad.x + rr.x,
            -request.globalCorrectionRad.y + rr.y,
            -request.globalCorrectionRad.z + rr.z - request.visual.rotationRad
        };
        const Quat qRel = Quat::fromRotationVector(rot);
        const Mat3 rRel = qRel.matrix();
        const Vec3 ray_phys = rRel * ray_virt;
        if (ray_phys.z > 1e-4) {
            x_sensor = fx * (ray_phys.x / ray_phys.z) + cx;
            y_sensor = fy * (ray_phys.y / ray_phys.z) + cy;
        }
    }

    return {x_sensor / W, y_sensor / H};
}

double Rs19OneWarp::computeMarginPx(const WarpRequest& request) const {
    const double W = config_.width > 0 ? static_cast<double>(config_.width) : 2560.0;
    const double H = config_.height > 0 ? static_cast<double>(config_.height) : 1440.0;
    static constexpr double kPerimeter[16][2] = {
        {0.0, 0.0}, {0.25, 0.0}, {0.5, 0.0}, {0.75, 0.0}, {1.0, 0.0},
        {0.0, 0.25}, {1.0, 0.25},
        {0.0, 0.5},  {1.0, 0.5},
        {0.0, 0.75}, {1.0, 0.75},
        {0.0, 1.0}, {0.25, 1.0}, {0.5, 1.0}, {0.75, 1.0}, {1.0, 1.0}
    };
    double minMargin = 1e9;
    for (const auto& uv : kPerimeter) {
        const WarpVertex v = projectPoint(uv[0], uv[1], request);
        const double px = v.u * W;
        const double py = v.v * H;
        const double m = std::min({px - 0.5, W - 0.5 - px, py - 0.5, H - 0.5 - py});
        if (m < minMargin) minMargin = m;
    }
    return minMargin;
}

std::vector<WarpVertex> Rs19OneWarp::buildMesh(const WarpRequest& request) const {
    const int rows = config_.rowBands + 1;
    const int cols = config_.columns + 1;
    std::vector<WarpVertex> mesh;
    mesh.reserve(rows * cols);

    for (int r = 0; r < rows; ++r) {
        const double v0 = static_cast<double>(r) / (rows - 1);
        for (int c = 0; c < cols; ++c) {
            const double u0 = static_cast<double>(c) / (cols - 1);
            mesh.push_back(projectPoint(u0, v0, request));
        }
    }
    return mesh;
}

}  // namespace gemini
