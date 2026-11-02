#pragma once

#include <array>
#include <vector>
#include "common/types.hpp"

namespace gemini {

struct Rs19Config {
    int columns = 32;
    int rowBands = 19;
    double fxPx = 1202.3;
    double fyPx = 1208.5;
    double cxPx = 1280.0;
    double cyPx = 720.0;
    int width = 2560;
    int height = 1440;
    double baseCrop = 0.80;
};

struct WarpRequest {
    Vec3 globalCorrectionRad{};
    Similarity2D visual{};
    std::array<Vec3, 19> rowCorrectionRad{};
    double zoom = 1.0;
    Quat qTarget{0, 0, 0, 1};
    Quat qCenter{0, 0, 0, 1};
    std::array<Quat, 20> rowQuats{};
    std::array<Quat, 20> rowTargets{};
    bool hasRowTargets = false;
    bool useQuats = false;
};

class Rs19OneWarp {
public:
    explicit Rs19OneWarp(Rs19Config config = {});
    WarpVertex projectPoint(double u0, double v0, const WarpRequest& request) const;
    double computeMarginPx(const WarpRequest& request) const;
    std::vector<WarpVertex> buildMesh(const WarpRequest& request) const;

private:
    Rs19Config config_;
};

}  // namespace gemini
