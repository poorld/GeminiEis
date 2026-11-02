#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <vector>

namespace gemini {

constexpr double kPi = 3.14159265358979323846;

template <typename T>
inline T clamp(T value, T lo, T hi) {
    return std::max(lo, std::min(value, hi));
}

struct Vec2 {
    double x = 0.0;
    double y = 0.0;
    Vec2 operator+(const Vec2& o) const { return {x + o.x, y + o.y}; }
    Vec2 operator-(const Vec2& o) const { return {x - o.x, y - o.y}; }
    Vec2 operator*(double s) const { return {x * s, y * s}; }
};

struct Vec3 {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
    Vec3 operator+(const Vec3& o) const { return {x + o.x, y + o.y, z + o.z}; }
    Vec3 operator-(const Vec3& o) const { return {x - o.x, y - o.y, z - o.z}; }
    Vec3 operator*(double s) const { return {x * s, y * s, z * s}; }
    double dot(const Vec3& o) const { return x * o.x + y * o.y + z * o.z; }
    double norm() const { return std::sqrt(dot(*this)); }
    Vec3 normalized() const {
        const double n = norm();
        return n > 1e-12 ? *this * (1.0 / n) : Vec3{};
    }
};

struct Mat3 {
    std::array<double, 9> v{1.0, 0.0, 0.0,
                            0.0, 1.0, 0.0,
                            0.0, 0.0, 1.0};
    static Mat3 identity() { return {}; }
    double& at(int r, int c) { return v[static_cast<size_t>(r * 3 + c)]; }
    double at(int r, int c) const { return v[static_cast<size_t>(r * 3 + c)]; }
    Vec3 operator*(const Vec3& p) const {
        return {
            at(0, 0) * p.x + at(0, 1) * p.y + at(0, 2) * p.z,
            at(1, 0) * p.x + at(1, 1) * p.y + at(1, 2) * p.z,
            at(2, 0) * p.x + at(2, 1) * p.y + at(2, 2) * p.z
        };
    }
};

struct Quat {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
    double w = 1.0;

    static Quat identity() { return {}; }

    Quat normalized() const {
        const double n = std::sqrt(x*x + y*y + z*z + w*w);
        return n > 1e-15 ? Quat{x/n, y/n, z/n, w/n} : identity();
    }
    Quat conjugate() const { return {-x, -y, -z, w}; }
    Quat inverse() const { return normalized().conjugate(); }

    Quat operator*(const Quat& q) const {
        return {
            w*q.x + x*q.w + y*q.z - z*q.y,
            w*q.y - x*q.z + y*q.w + z*q.x,
            w*q.z + x*q.y - y*q.x + z*q.w,
            w*q.w - x*q.x - y*q.y - z*q.z
        };
    }

    static Quat fromRotationVector(const Vec3& rv) {
        const double angle = rv.norm();
        if (angle < 1e-12) {
            return Quat{0.5*rv.x, 0.5*rv.y, 0.5*rv.z, 1.0}.normalized();
        }
        const double half = 0.5 * angle;
        const double s = std::sin(half) / angle;
        return {rv.x * s, rv.y * s, rv.z * s, std::cos(half)};
    }

    static Quat slerp(Quat a, Quat b, double t) {
        a = a.normalized(); b = b.normalized();
        double d = a.x*b.x + a.y*b.y + a.z*b.z + a.w*b.w;
        if (d < 0.0) {
            b = {-b.x, -b.y, -b.z, -b.w};
            d = -d;
        }
        d = std::clamp(d, 0.0, 1.0);
        if (d > 0.999999) {
            const double u = std::clamp(t, 0.0, 1.0);
            return Quat{
                (1-u)*a.x + u*b.x,
                (1-u)*a.y + u*b.y,
                (1-u)*a.z + u*b.z,
                (1-u)*a.w + u*b.w
            }.normalized();
        }
        const double theta = std::acos(d);
        const double den = std::max(1e-15, std::sin(theta));
        const double u = std::clamp(t, 0.0, 1.0);
        const double wa = std::sin((1-u)*theta) / den;
        const double wb = std::sin(u*theta) / den;
        return {
            wa*a.x + wb*b.x,
            wa*a.y + wb*b.y,
            wa*a.z + wb*b.z,
            wa*a.w + wb*b.w
        };
    }

    Mat3 matrix() const {
        const Quat q = normalized();
        const double xx = q.x*q.x, yy = q.y*q.y, zz = q.z*q.z;
        const double xy = q.x*q.y, xz = q.x*q.z, yz = q.y*q.z;
        const double wx = q.w*q.x, wy = q.w*q.y, wz = q.w*q.z;
        Mat3 r;
        r.v = {
            1 - 2*(yy+zz), 2*(xy-wz),   2*(xz+wy),
            2*(xy+wz),   1 - 2*(xx+zz), 2*(yz-wx),
            2*(xz-wy),   2*(yz+wx),   1 - 2*(xx+yy)
        };
        return r;
    }

    Vec3 apply(const Vec3& p) const { return matrix() * p; }

    double angle() const {
        return 2.0 * std::acos(std::clamp(std::abs(w), 0.0, 1.0));
    }
};

struct GyroSample {
    int64_t timestampNs = 0;
    Vec3 omegaRadS{};
};

struct AxisState {
    double angle = 0.0;
    double velocity = 0.0;
    double acceleration = 0.0;
};

struct PoseState {
    AxisState pitch{};
    AxisState yaw{};
    AxisState roll{};
};

struct Similarity2D {
    double txPx = 0.0;
    double tyPx = 0.0;
    double rotationRad = 0.0;
    double scale = 1.0;
    double trust = 0.0;
    int inliers = 0;
    bool hasHomography = false;
    std::array<double, 9> hInv{1.0, 0.0, 0.0,
                               0.0, 1.0, 0.0,
                               0.0, 0.0, 1.0};
};

struct FeatureMatch {
    Vec2 prevPx{};
    Vec2 currPx{};
    double forwardBackwardErrorPx = 0.0;
};

struct FrameMeta {
    int64_t sofTimestampNs = 0;
    double readoutS = 0.019;
    double exposureS = 0.010;
    double iso = 100.0;
    int featureWidth = 192;
    int featureHeight = 108;
};

struct VisualResidual {
    int64_t timestampNs = 0;
    Similarity2D similarity{};
};

struct CropBudget {
    double pitchRad = 0.10;
    double yawRad = 0.10;
    double rollRad = 0.08;
};

struct MotionState {
    int64_t timestampNs = 0;
    PoseState raw{};
    Vec3 angularAcceleration{};
    double panConfidence = 0.0;
    double oscillationScore = 0.0;
    Similarity2D delayedVisual{};
    CropBudget crop{};
};

struct WarpVertex {
    double u = 0.0;
    double v = 0.0;
};

}  // namespace gemini
