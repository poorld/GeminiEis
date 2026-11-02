#include "visual/async_visual_worker.h"

#include <cmath>
#include <limits>

namespace gemini {
namespace {

inline Vec2 unrotatePoint(const Vec2& p, const Vec3& r, const CameraModel& c) {
    const double x = (p.x - c.cx) / c.fx;
    const double y = (p.y - c.cy) / c.fy;
    const double den = 1.0 + r.y * x - r.x * y;
    const double invDen = std::abs(den) > 1e-4 ? (1.0 / den) : 1.0;
    const double xr = (x - r.y + r.z * y) * invDen;
    const double yr = (y + r.x - r.z * x) * invDen;
    return {xr * c.fx + c.cx, yr * c.fy + c.cy};
}

bool fitSimilarity2(const FeatureMatch& a, const FeatureMatch& b, Similarity2D* out) {
    const Vec2 p = b.prevPx - a.prevPx;
    const Vec2 q = b.currPx - a.currPx;
    const double pn = std::sqrt(p.x * p.x + p.y * p.y);
    const double qn = std::sqrt(q.x * q.x + q.y * q.y);
    if (pn < 1e-3 || qn < 1e-3) return false;

    const double scale = qn / pn;
    const double rot = std::atan2(q.y, q.x) - std::atan2(p.y, p.x);
    const double cs = std::cos(rot) * scale;
    const double sn = std::sin(rot) * scale;
    const double tx = a.currPx.x - (cs * a.prevPx.x - sn * a.prevPx.y);
    const double ty = a.currPx.y - (sn * a.prevPx.x + cs * a.prevPx.y);
    *out = {tx, ty, rot, scale, 0.0, 0};
    return true;
}

double reproj(const FeatureMatch& m, const Similarity2D& s) {
    const double cs = std::cos(s.rotationRad) * s.scale;
    const double sn = std::sin(s.rotationRad) * s.scale;
    const double x = cs * m.prevPx.x - sn * m.prevPx.y + s.txPx;
    const double y = sn * m.prevPx.x + cs * m.prevPx.y + s.tyPx;
    const double dx = x - m.currPx.x;
    const double dy = y - m.currPx.y;
    return std::sqrt(dx * dx + dy * dy);
}

}  // namespace

AsyncVisualWorkerMath::AsyncVisualWorkerMath(CameraModel camera, VisualConfig config)
    : camera_(camera), config_(config) {}

std::vector<FeatureMatch> AsyncVisualWorkerMath::gyroPrewarp(
        const std::vector<FeatureMatch>& matches,
        const FrameMeta& prevMeta,
        const FrameMeta& currMeta,
        const GyroRingBuffer& gyro) const {
    std::vector<FeatureMatch> out;
    out.reserve(matches.size());

    for (const auto& m : matches) {
        if (m.forwardBackwardErrorPx > config_.fbThresholdPx) continue;

        const double yp = clamp(m.prevPx.y / std::max(1, prevMeta.featureHeight), 0.0, 1.0);
        const double yc = clamp(m.currPx.y / std::max(1, currMeta.featureHeight), 0.0, 1.0);
        const int64_t tp = prevMeta.sofTimestampNs
            + static_cast<int64_t>((yp * prevMeta.readoutS + 0.5 * prevMeta.exposureS) * 1e9);
        const int64_t tc = currMeta.sofTimestampNs
            + static_cast<int64_t>((yc * currMeta.readoutS + 0.5 * currMeta.exposureS) * 1e9);

        const Vec3 dr = gyro.integrate(tp, tc);
        FeatureMatch corrected = m;
        corrected.currPx = unrotatePoint(m.currPx, dr, camera_);
        out.push_back(corrected);
    }
    return out;
}

Similarity2D AsyncVisualWorkerMath::estimateSimilarity(
        const std::vector<FeatureMatch>& matches,
        double exposureS,
        double angularSpeedRadS,
        double iso) const {
    Similarity2D best{};
    if (matches.size() < 2) return best;

    int bestInliers = -1;
    double bestErr = std::numeric_limits<double>::max();
    const int n = static_cast<int>(matches.size());
    const int maxPairs = n * (n - 1) / 2;
    const int iters = std::min(config_.ransacIterations, maxPairs);

    for (int k = 0; k < iters; ++k) {
        const int i = k % n;
        int j = (k * 7 + 3) % n;
        if (i == j) j = (j + 1) % n;

        Similarity2D s;
        if (!fitSimilarity2(matches[i], matches[j], &s)) continue;

        int inliers = 0;
        double err = 0.0;
        for (const auto& m : matches) {
            const double e = reproj(m, s);
            if (e <= config_.inlierThresholdPx) {
                ++inliers;
                err += e;
            }
        }
        if (inliers > bestInliers || (inliers == bestInliers && err < bestErr)) {
            best = s;
            bestInliers = inliers;
            bestErr = err;
        }
    }

    if (bestInliers <= 0) return {};

    double fbMean = 0.0;
    int fbCount = 0;
    for (const auto& m : matches) {
        if (reproj(m, best) <= config_.inlierThresholdPx) {
            fbMean += m.forwardBackwardErrorPx;
            ++fbCount;
        }
    }
    fbMean /= std::max(1, fbCount);

    const double inlierTrust = clamp(
        static_cast<double>(bestInliers) / std::max(1, config_.targetInliers), 0.0, 1.0);
    const double errorTrust = std::exp(-config_.trustFbAlpha * fbMean);

    double exposureTrust = 1.0;
    if (exposureS > config_.exposureGateS
            && angularSpeedRadS > config_.motionGateRadS) {
        const double expExcess = (exposureS - config_.exposureGateS)
                / std::max(1e-4, config_.exposureGateS);
        const double motionExcess = (angularSpeedRadS - config_.motionGateRadS)
                / std::max(1e-4, config_.motionGateRadS);
        exposureTrust = std::exp(-0.8 * expExcess * motionExcess);
    }

    double isoTrust = 1.0;
    if (iso > config_.isoGate) {
        const double isoExcess = (iso - config_.isoGate)
                / std::max(1.0, config_.maxIso - config_.isoGate);
        isoTrust = clamp(std::exp(-1.2 * isoExcess), 0.0, 1.0);
    }

    best.inliers = bestInliers;
    best.trust = clamp(inlierTrust * errorTrust * exposureTrust * isoTrust, 0.0, 1.0);
    if (best.trust < 0.05 || bestInliers < 4) {
        best.trust = 0.0;
    }
    return best;
}

}  // namespace gemini
