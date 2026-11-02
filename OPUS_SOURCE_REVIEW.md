# Opus Source Review: GeminiEis Golden 1492bd5

This is an exact, line-numbered extraction of critical implementation code from the exported golden baseline. It is intended for Claude Opus or any code reviewer whose tool cannot browse GitHub subdirectories. Every excerpt below is sourced from a real file in the same repository. Source files are unchanged.

**Important:** `README.md` in the original golden repository was historically stale. Actual `GeminiEisProcessLiveYPlane` uses 320x180 KLT and Camera2 JNI glue is present. Do not infer feature absence from old Pending sections.

## Critical facts to verify in code
- `src/core/gemini_eis_core.cpp`: golden uses `qTarget` from quaternion recursion; earlier MPC `targetAngleRad` is NOT authoritative. Second boundary gate can set per-row virtual `rowTargets`. Zoom generally 1.12 but can become 1.14 under margin pressure.
- `src/warp/rs19_one_warp.cpp`: output-to-source normalized UV mapping, Brown-Conrady k1=-0.015, iterative row-dependent physical source pose, 20x33 mesh.
- `src/api/gemini_eis_c_api.cpp`: residual gets converted to 660 UV deltas, per-vertex max magnitude/step, temporal filtering, breathScale and per-vertex clamp; non-translation homography deltas are set to zero and RBF disabled, but final mapping can still be non-similarity.
- `GeminiEisResult` public telemetry reports old MPC angles, not `qTarget`, rowTargets, per-vertex clamp hits; these must be newly logged for an exact Experiment 0.

## Evidence and limitations
Four videos `024605 / 024618 / 024630 / 024647` and separate `imu.csv/trace.jsonl/video_pts.csv` exist locally in `D:\Yuy\doc\EIS\Yaw`. They are **source recordings from YuyEIS2 with EIS disabled**, NOT captured golden GeminiEis per-frame final qTarget/mesh state. A deterministic golden replay and/or instrumentation is necessary. No claim of measured causal attribution is made here.

## Direct raw source links
- https://raw.githubusercontent.com/poorld/GeminiEis/main/src/core/gemini_eis_core.cpp
- https://raw.githubusercontent.com/poorld/GeminiEis/main/src/warp/rs19_one_warp.cpp
- https://raw.githubusercontent.com/poorld/GeminiEis/main/src/api/gemini_eis_c_api.cpp
- https://raw.githubusercontent.com/poorld/GeminiEis/main/include/core/gemini_eis_core.h
- https://raw.githubusercontent.com/poorld/GeminiEis/main/include/api/gemini_eis_c_api.h
- https://raw.githubusercontent.com/poorld/GeminiEis/main/android_app/app/src/main/java/com/yuy/y3realtime/GeminiCamera2EisController.java

## src/core/gemini_eis_core.cpp

### Lines 68-150
````text
  68 | 
  69 |     WarpRequest wr;
  70 |     wr.globalCorrectionRad = out.correctionRad;
  71 |     wr.visual = input.visual;
  72 |     wr.rowCorrectionRad = input.rs19;
  73 |     wr.useQuats = input.useQuats;
  74 | 
  75 |     if (!input.useQuats) {
  76 |         wr.zoom = (input.zoom > 0.0 && input.zoom <= 1.5) ? input.zoom : 1.0;
  77 |         out.mesh = warp_.buildMesh(wr);
  78 |         return out;
  79 |     }
  80 | 
  81 |     const Quat qCenter = input.qCenter.normalized();
  82 |     wr.qCenter = qCenter;
  83 |     wr.rowQuats = input.rowQuats;
  84 | 
  85 |     // Hybrid PixSmart + Virtual Tripod Deadband Architecture:
  86 |     //   1. Constant baseZoom (1.12x, zero heavy zoom pumping)
  87 |     //   2. Anchor Deadband Lock (tauAnchor = 60s inside 5.4 deg oscillation window so single/double-sided swings stay rock-solid)
  88 |     //   3. PixSmart Two-Rectangle (Inner Safe Rect + innerRatio=0.90 Outer Contour) 4-corner camera mapping
  89 |     //   4. 1/rho Reciprocal Boundary Release + 99.8% Tripod Hold inside safe contour
  90 |     //   5. Second Boundary Gate -> Per-Row RS Endpoint Target Trajectory (FilterSteadyGL)
  91 |     const double baseZoom = (input.zoom > 0.0 && input.zoom <= 1.24) ? input.zoom : 1.12;
  92 | 
  93 |     const double W = config_.warp.width > 0 ? static_cast<double>(config_.warp.width) : 2560.0;
  94 |     const double H = config_.warp.height > 0 ? static_cast<double>(config_.warp.height) : 1440.0;
  95 |     const double baseCrop = (config_.warp.baseCrop > 0.0 && config_.warp.baseCrop <= 1.0) ? config_.warp.baseCrop : 0.80;
  96 |     const double effScale = std::clamp(baseCrop / baseZoom, 0.50, 0.95);
  97 |     const double innerLeft   = 0.5 * (1.0 - effScale) * W;
  98 |     const double innerRight  = W - innerLeft;
  99 |     const double innerTop    = 0.5 * (1.0 - effScale) * H;
 100 |     const double innerBottom = H - innerTop;
 101 | 
 102 |     // PixSmart innerRatio = 0.90 (reserves ~36.6px outer buffer on 2560x1440 while maximizing +-9.2 deg swing budget)
 103 |     const double innerRatio  = 0.90;
 104 |     const double outerLeft   = (1.0 - innerRatio) * innerLeft;
 105 |     const double outerTop    = (1.0 - innerRatio) * innerTop;
 106 | 
 107 |     struct BoundaryEval {
 108 |         bool hit = false;
 109 |         double rho = 0.0;
 110 |         int side = 0; // 0 = A (top RS endpoint), 1 = B (bottom RS endpoint)
 111 |     };
 112 | 
 113 |     auto evalOneEndpoint = [&](const Quat& sensorQ, const Quat& trialQ) -> std::pair<bool, double> {
 114 |         WarpRequest wTest = wr;
 115 |         wTest.visual = Similarity2D{};
 116 |         wTest.zoom = baseZoom;
 117 |         wTest.qTarget = trialQ;
 118 |         wTest.hasRowTargets = false;
 119 |         wTest.rowQuats.fill(sensorQ);
 120 |         static constexpr double kCorners[4][2] = {
 121 |             {0.0, 0.0}, {1.0, 0.0}, {1.0, 1.0}, {0.0, 1.0}
 122 |         };
 123 |         bool anyHit = false;
 124 |         double maxRho = 0.0;
 125 |         const double mx = std::max(1.0, innerLeft - outerLeft);
 126 |         const double my = std::max(1.0, innerTop - outerTop);
 127 |         for (const auto& c : kCorners) {
 128 |             const WarpVertex v = warp_.projectPoint(c[0], c[1], wTest);
 129 |             const double px = v.u * W;
 130 |             const double py = v.v * H;
 131 |             double prho = 0.0;
 132 |             if (px < innerLeft - 1e-4)   prho = std::max(prho, (innerLeft - px) / mx);
 133 |             if (px > innerRight + 1e-4)  prho = std::max(prho, (px - innerRight) / mx);
 134 |             if (py < innerTop - 1e-4)    prho = std::max(prho, (innerTop - py) / my);
 135 |             if (py > innerBottom + 1e-4) prho = std::max(prho, (py - innerBottom) / my);
 136 |             if (prho > 0.0) {
 137 |                 anyHit = true;
 138 |                 maxRho = std::max(maxRho, prho);
 139 |             }
 140 |         }
 141 |         return {anyHit, maxRho};
 142 |     };
 143 | 
 144 |     auto evalBoundary = [&](const Quat& trialQ, const Quat& aQ, const Quat& bQ) -> BoundaryEval {
 145 |         const auto [aHit, aRho] = evalOneEndpoint(aQ, trialQ);
 146 |         const auto [bHit, bRho] = evalOneEndpoint(bQ, trialQ);
 147 |         if (aRho >= bRho) {
 148 |             return {aHit, aRho, 0};
 149 |         }
 150 |         return {bHit, bRho, 1};
````

### Lines 153-320
````text
 153 |     if (!quatInitialized_) {
 154 |         qTarget_ = qCenter;
 155 |         qAnchor_ = qCenter;
 156 |         currentZoom_ = baseZoom;
 157 |         quatInitialized_ = true;
 158 |         lastTimestampNs_ = input.timestampNs;
 159 |         frameCount_ = 1;
 160 |         wr.hasRowTargets = false;
 161 |     } else {
 162 |         double dtS = static_cast<double>(input.timestampNs - lastTimestampNs_) * 1e-9;
 163 |         if (!(dtS > 1e-4 && dtS < 0.25)) {
 164 |             dtS = 1.0 / 30.0;
 165 |         }
 166 |         lastTimestampNs_ = input.timestampNs;
 167 |         frameCount_++;
 168 |         currentZoom_ = baseZoom;
 169 | 
 170 |         const Quat A0 = input.rowQuats[0].normalized();
 171 |         const Quat B0 = input.rowQuats[19].normalized();
 172 | 
 173 |         // V5 PixSmart Pure Recursive FilterPose Control Law:
 174 |         // Directly chains from previous output pose P = qTarget_ without any discontinuous deadband state-machine jump!
 175 |         const double omegaMag = input.angularVelocityRadS.norm();
 176 |         if (frameCount_ <= 12) {
 177 |             const double alphaInit = 1.0 - std::exp(-dtS / 0.18);
 178 |             qTarget_ = Quat::slerp(qTarget_, qCenter, alphaInit);
 179 |             qAnchor_ = qTarget_;
 180 |         }
 181 |         const Quat P = qTarget_;
 182 | 
 183 |         // Build causal 4-sample lookahead window A[0..3], B[0..3]:
 184 |         // Only extrapolate velocity during confirmed intentional pan (never during bilateral/unilateral oscillation!)
 185 |         constexpr int kWin = 4;
 186 |         std::array<Quat, kWin> A{};
 187 |         std::array<Quat, kWin> B{};
 188 |         A[0] = A0;
 189 |         B[0] = B0;
 190 |         const bool allowLookahead = (out.panConfidence > 0.65 && out.oscillationScore < 0.25);
 191 |         for (int i = 1; i < kWin; ++i) {
 192 |             if (allowLookahead) {
 193 |                 const double dtLook = i * 0.015 * std::pow(0.65, i - 1);
 194 |                 const Quat dQ = Quat::fromRotationVector(input.angularVelocityRadS * dtLook);
 195 |                 A[static_cast<size_t>(i)] = (A0 * dQ).normalized();
 196 |                 B[static_cast<size_t>(i)] = (B0 * dQ).normalized();
 197 |             } else {
 198 |                 A[static_cast<size_t>(i)] = A0;
 199 |                 B[static_cast<size_t>(i)] = B0;
 200 |             }
 201 |         }
 202 | 
 203 |         // PixSmart FilterPose main loop: Farthest-future (N-1) down to current (0) with 1/rho reciprocal release
 204 |         Quat T = P;
 205 |         int k = kWin - 1;
 206 |         int releaseCount = 0;
 207 |         for (int i = kWin - 1; i >= 0; --i) {
 208 |             const double coeff = static_cast<double>(i + 1) / static_cast<double>(k + 1);
 209 |             const Quat V = Quat::slerp(P, T, coeff);
 210 |             const auto br = evalBoundary(V, A[static_cast<size_t>(i)], B[static_cast<size_t>(i)]);
 211 |             if (br.hit && br.rho > 1.0) {
 212 |                 const Quat& source = (br.side == 0) ? A[static_cast<size_t>(i)] : B[static_cast<size_t>(i)];
 213 |                 T = Quat::slerp(source, V, 1.0 / br.rho);
 214 |                 k = i;
 215 |                 releaseCount++;
 216 |             }
 217 |         }
 218 |         const Quat H = Quat::slerp(P, T, 1.0 / static_cast<double>(k + 1));
 219 | 
 220 |         // V6 Zero-Sway Tripod Anchor + PixSmart Smooth Reciprocal Release:
 221 |         //   - Pure recursive P = qTarget_ (zero discontinuous state-machine jumps)
 222 |         //   - Inside safe contour (releaseCount == 0): holdWeight = 0.9992 (tau = 41.7s, 99.6% swing suppression)
 223 |         //   - Calm/settled (omegaMag < 0.085 rad/s): holdWeight = 1.0000 (100% locked, zero post-settle drift)
 224 |         //   - Boundary release (releaseCount > 0): holdWeight = 0.9920 (smooth PixSmart 1/rho reciprocal release)
 225 |         const double distFromH = (A0.inverse() * H).angle();
 226 |         double holdWeight = 0.9992;
 227 |         if (releaseCount > 0) {
 228 |             holdWeight = 0.9920;
 229 |         } else if (omegaMag < 0.085 && distFromH < 0.060) {
 230 |             holdWeight = 1.0000;
 231 |         }
 232 |         Quat Q = Quat::slerp(A0, H, holdWeight);
 233 | 
 234 |         // PixSmart Second Boundary Gate -> Per-Row Target Trajectory (FilterSteadyGL)
 235 |         const auto second = evalBoundary(Q, A0, B0);
 236 |         if (second.hit && second.rho > 1.0) {
 237 |             const Quat& source = (second.side == 0) ? A0 : B0;
 238 |             const Quat E = Quat::slerp(source, Q, 1.0 / second.rho);
 239 |             wr.hasRowTargets = true;
 240 |             for (int r = 0; r < 20; ++r) {
 241 |                 const double tRow = static_cast<double>(r) / 19.0;
 242 |                 wr.rowTargets[static_cast<size_t>(r)] = (second.side == 0)
 243 |                     ? Quat::slerp(E, Q, tRow)
 244 |                     : Quat::slerp(Q, E, tRow);
 245 |             }
 246 |             // Smoothly pull center pose Q toward E only by the minimum required amount to stay >= 32px inside sensor
 247 |             WarpRequest wrCheck = wr;
 248 |             wrCheck.visual = Similarity2D{};
 249 |             wrCheck.zoom = baseZoom;
 250 |             wrCheck.qTarget = Q;
 251 |             wrCheck.hasRowTargets = false;
 252 |             if (warp_.computeMarginPx(wrCheck) < 32.0) {
 253 |                 double lo = 0.0, hi = 1.0;
 254 |                 for (int iter = 0; iter < 10; ++iter) {
 255 |                     const double mid = 0.5 * (lo + hi);
 256 |                     wrCheck.qTarget = Quat::slerp(Q, E, mid);
 257 |                     if (warp_.computeMarginPx(wrCheck) >= 32.0) hi = mid;
 258 |                     else lo = mid;
 259 |                 }
 260 |                 Q = Quat::slerp(Q, E, hi);
 261 |             }
 262 |         } else {
 263 |             wr.hasRowTargets = false;
 264 |         }
 265 |         qAnchor_ = Q;
 266 | 
 267 |         // Stage 2: Visual Homography/Similarity Margin Check at constant zoom
 268 |         Similarity2D visCand = input.visual;
 269 |         wr.qTarget = Q;
 270 |         wr.visual = visCand;
 271 |         wr.zoom = baseZoom;
 272 |         if (warp_.computeMarginPx(wr) < 2.0) {
 273 |             double lo = 0.0, hi = 1.0;
 274 |             for (int iter = 0; iter < 12; ++iter) {
 275 |                 const double mid = 0.5 * (lo + hi);
 276 |                 const double keep = 1.0 - mid;
 277 |                 wr.visual = visCand;
 278 |                 wr.visual.txPx = visCand.txPx * keep;
 279 |                 wr.visual.tyPx = visCand.tyPx * keep;
 280 |                 wr.visual.rotationRad = visCand.rotationRad * keep;
 281 |                 wr.visual.scale = 1.0 + (visCand.scale - 1.0) * keep;
 282 |                 if (visCand.hasHomography) {
 283 |                     for (int m = 0; m < 9; ++m) {
 284 |                         const double eyeVal = (m % 4 == 0) ? 1.0 : 0.0;
 285 |                         wr.visual.hInv[static_cast<size_t>(m)] =
 286 |                             eyeVal + (visCand.hInv[static_cast<size_t>(m)] - eyeVal) * keep;
 287 |                     }
 288 |                 }
 289 |                 if (warp_.computeMarginPx(wr) >= 2.0) hi = mid;
 290 |                 else lo = mid;
 291 |             }
 292 |             const double keep = 1.0 - hi;
 293 |             visCand.txPx *= keep;
 294 |             visCand.tyPx *= keep;
 295 |             visCand.rotationRad *= keep;
 296 |             visCand.scale = 1.0 + (visCand.scale - 1.0) * keep;
 297 |             if (visCand.hasHomography) {
 298 |                 for (int m = 0; m < 9; ++m) {
 299 |                     const double eyeVal = (m % 4 == 0) ? 1.0 : 0.0;
 300 |                     visCand.hInv[static_cast<size_t>(m)] =
 301 |                         eyeVal + (visCand.hInv[static_cast<size_t>(m)] - eyeVal) * keep;
 302 |                 }
 303 |             }
 304 |             wr.visual = visCand;
 305 |             if (warp_.computeMarginPx(wr) < 2.0) {
 306 |                 currentZoom_ = 1.14;
 307 |             }
 308 |         }
 309 | 
 310 |         qTarget_ = Q;
 311 |         out.visual = visCand;
 312 |     }
 313 | 
 314 |     wr.qTarget = qTarget_;
 315 |     wr.visual = out.visual;
 316 |     wr.zoom = currentZoom_;
 317 |     out.qTarget = qTarget_;
 318 |     out.zoom = currentZoom_;
 319 |     out.mesh = warp_.buildMesh(wr);
 320 |     return out;
````

## src/warp/rs19_one_warp.cpp

### Lines 10-0
````text
````

### Lines 161-0
````text
````

## src/api/gemini_eis_c_api.cpp

### Lines 555-628
````text
 555 | void applyDeltaMToMesh(
 556 |     const Context& ctx,
 557 |     const std::array<Vec2, kMeshVerts>& deltaM,
 558 |     GeminiEisResult* ioResult) {
 559 |     if (!ioResult) return;
 560 |     const double minU = 16.0 / std::max<uint32_t>(640, ctx.config.input_width);
 561 |     const double maxU = 1.0 - minU;
 562 |     const double minV = 16.0 / std::max<uint32_t>(360, ctx.config.input_height);
 563 |     const double maxV = 1.0 - minV;
 564 | 
 565 |     const size_t centerIdx = 10 * kMeshCols + 16;
 566 |     const double uCenter = gemini::clamp(ioResult->mesh[centerIdx].u, 0.25, 0.75);
 567 |     const double vCenter = gemini::clamp(ioResult->mesh[centerIdx].v, 0.25, 0.75);
 568 |     auto isSafeWithBreath = [&](double breathScale, double alpha) {
 569 |         for (size_t k = 0; k < kMeshVerts; ++k) {
 570 |             const double uRaw = ioResult->mesh[k].u + alpha * deltaM[k].x;
 571 |             const double vRaw = ioResult->mesh[k].v + alpha * deltaM[k].y;
 572 |             const double u = uCenter + breathScale * (uRaw - uCenter);
 573 |             const double v = vCenter + breathScale * (vRaw - vCenter);
 574 |             if (u < minU || u > maxU || v < minV || v > maxV) return false;
 575 |         }
 576 |         return true;
 577 |     };
 578 | 
 579 |     double breathScale = 1.0;
 580 |     double alphaSafe = 1.0;
 581 |     if (!isSafeWithBreath(1.0, 1.0)) {
 582 |         if (isSafeWithBreath(0.992, 1.0)) {
 583 |             double blo = 0.992, bhi = 1.0;
 584 |             for (int iter = 0; iter < 10; ++iter) {
 585 |                 const double bmid = 0.5 * (blo + bhi);
 586 |                 if (isSafeWithBreath(bmid, 1.0)) blo = bmid;
 587 |                 else bhi = bmid;
 588 |             }
 589 |             breathScale = blo;
 590 |         } else {
 591 |             breathScale = 0.992;
 592 |             double lo = 0.0, hi = 1.0;
 593 |             for (int iter = 0; iter < 14; ++iter) {
 594 |                 const double mid = 0.5 * (lo + hi);
 595 |                 if (isSafeWithBreath(breathScale, mid)) lo = mid;
 596 |                 else hi = mid;
 597 |             }
 598 |             alphaSafe = lo;
 599 |         }
 600 |     }
 601 | 
 602 |     for (size_t k = 0; k < kMeshVerts; ++k) {
 603 |         const double uRaw = ioResult->mesh[k].u + alphaSafe * deltaM[k].x;
 604 |         const double vRaw = ioResult->mesh[k].v + alphaSafe * deltaM[k].y;
 605 |         ioResult->mesh[k].u = gemini::clamp(uCenter + breathScale * (uRaw - uCenter), minU, maxU);
 606 |         ioResult->mesh[k].v = gemini::clamp(vCenter + breathScale * (vRaw - vCenter), minV, maxV);
 607 |     }
 608 | }
 609 | 
 610 | void applyHeldDeltaMToMesh(Context& ctx, GeminiEisResult* ioResult) {
 611 |     if (!ctx.hasDeltaM || !ioResult) return;
 612 | 
 613 |     const double decay = ctx.retireVisualCorrection ? 0.95 : 0.995;
 614 |     double maxMagnitude = 0.0;
 615 |     for (size_t k = 0; k < kMeshVerts; ++k) {
 616 |         ctx.lastDeltaM[k].x *= decay;
 617 |         ctx.lastDeltaM[k].y *= decay;
 618 |         maxMagnitude = std::max(maxMagnitude,
 619 |             std::hypot(ctx.lastDeltaM[k].x, ctx.lastDeltaM[k].y));
 620 |     }
 621 | 
 622 |     if (ctx.retireVisualCorrection && maxMagnitude < 1e-6) {
 623 |         ctx.lastDeltaM.fill({});
 624 |         ctx.hasDeltaM = false;
 625 |         ctx.retireVisualCorrection = false;
 626 |         return;
 627 |     }
 628 |     applyDeltaMToMesh(ctx, ctx.lastDeltaM, ioResult);
````

### Lines 631-675
````text
 631 | void applyCausal8DofAndRbfToMesh(
 632 |     Context& ctx,
 633 |     const std::vector<Vec2>& ptsObsIn,
 634 |     const std::vector<Vec2>& ptsTgt,
 635 |     double omegaNormRadS,
 636 |     GeminiEisResult* ioResult) {
 637 | 
 638 |     const size_t n = ptsObsIn.size();
 639 |     if (n < 12 || !ioResult) {
 640 |         applyHeldDeltaMToMesh(ctx, ioResult);
 641 |         return;
 642 |     }
 643 | 
 644 |     // Handheld-aware sub-pixel deadband to suppress 30Hz KLT quantization & sensor ISO noise
 645 |     std::vector<Vec2> ptsObs = ptsObsIn;
 646 |     if (omegaNormRadS < 0.085) {
 647 |         const double calmFactor = 1.0 - gemini::clamp(omegaNormRadS / 0.085, 0.0, 1.0);
 648 |         const double deadbandPx = 0.45 + 0.55 * calmFactor; // Up to 1.0px in 640x360 (0.25px in 160x90 KLT grid)
 649 |         for (size_t i = 0; i < n; ++i) {
 650 |             const double dx = ptsObs[i].x - ptsTgt[i].x;
 651 |             const double dy = ptsObs[i].y - ptsTgt[i].y;
 652 |             const double mag = std::hypot(dx, dy);
 653 |             if (mag <= deadbandPx) {
 654 |                 ptsObs[i] = ptsTgt[i];
 655 |             } else {
 656 |                 const double scale = (mag - deadbandPx) / mag;
 657 |                 ptsObs[i].x = ptsTgt[i].x + dx * scale;
 658 |                 ptsObs[i].y = ptsTgt[i].y + dy * scale;
 659 |             }
 660 |         }
 661 |     }
 662 | 
 663 |     // V6 RANSAC + Tukey Biweight Outlier Rejection on KLT feature displacements:
 664 |     // Prevents moving foreground objects (hands, pedestrians) or repetitive-texture false matches
 665 |     // from dragging the global visual lock!
 666 |     std::vector<double> dxList(n), dyList(n);
 667 |     for (size_t i = 0; i < n; ++i) {
 668 |         dxList[i] = ptsTgt[i].x - ptsObs[i].x;
 669 |         dyList[i] = ptsTgt[i].y - ptsObs[i].y;
 670 |     }
 671 |     std::vector<double> dxSorted = dxList, dySorted = dyList;
 672 |     std::nth_element(dxSorted.begin(), dxSorted.begin() + n / 2, dxSorted.end());
 673 |     std::nth_element(dySorted.begin(), dySorted.begin() + n / 2, dySorted.end());
 674 |     const double medDx = dxSorted[n / 2];
 675 |     const double medDy = dySorted[n / 2];
````

### Lines 770-905
````text
 770 |     // V5 Pure Rigid SE(2) Global Lock:
 771 |     // Lock non-translation affine/keystone terms to zero so all 660 mesh vertices move as a 100% solid rigid body,
 772 |     // completely eliminating wave-like left/right border wobble during yaw swings!
 773 |     delta[0] = 0.0;
 774 |     delta[1] = 0.0;
 775 |     delta[3] = 0.0;
 776 |     delta[4] = 0.0;
 777 |     delta[6] = 0.0;
 778 |     delta[7] = 0.0;
 779 |     ctx.smoothedDelta.fill(0.0);
 780 | 
 781 |     const double H_res_norm[9] = {
 782 |         1.0, 0.0, delta[2],
 783 |         0.0, 1.0, delta[5],
 784 |         0.0, 0.0, 1.0
 785 |     };
 786 |     const double T_norm[9] = {1.0 / cx, 0.0, -1.0, 0.0, 1.0 / cy, -1.0, 0.0, 0.0, 1.0};
 787 |     const double T_inv[9]  = {cx, 0.0, cx, 0.0, cy, cy, 0.0, 0.0, 1.0};
 788 |     double tmp[9]{}, H_res_pix[9]{}, H_p2[9]{}, H_tgt_to_obs[9]{};
 789 |     mul3x3(T_inv, H_res_norm, tmp);
 790 |     mul3x3(tmp, T_norm, H_res_pix);
 791 |     mul3x3(H_res_pix, H_rigid, H_p2);
 792 |     if (!invert3x3(H_p2, H_tgt_to_obs)) {
 793 |         if (!invert3x3(H_rigid, H_tgt_to_obs)) return;
 794 |     }
 795 | 
 796 |     std::vector<Vec2> rbfLoc;
 797 |     std::vector<Vec2> rbfRes;
 798 |     for (size_t i = 0; i < n; ++i) {
 799 |         const Vec2 mappedObs = applyHomography(H_tgt_to_obs, ptsTgt[i].x, ptsTgt[i].y);
 800 |         const double ru = (ptsObs[i].x - mappedObs.x) / 640.0;
 801 |         const double rv = (ptsObs[i].y - mappedObs.y) / 360.0;
 802 |         if (std::hypot(ru * 640.0, rv * 360.0) < 4.5) {
 803 |             rbfLoc.push_back({ptsTgt[i].x / 640.0, ptsTgt[i].y / 360.0});
 804 |             rbfRes.push_back({ru, rv});
 805 |         }
 806 |     }
 807 | 
 808 |     const double inv2Sig2 = 1.0 / (2.0 * 0.25 * 0.25);
 809 |     const double lamRbf = 80.0; // Strongly damp non-rigid local patch shimmering to preserve fine texture sharpness
 810 |     const double alphaRbf = 0.03;
 811 | 
 812 |     std::array<double, kMeshVerts> duTotal{};
 813 |     std::array<double, kMeshVerts> dvTotal{};
 814 | 
 815 |     for (int r = 0; r < kMeshRows; ++r) {
 816 |         const double vg = static_cast<double>(r) / (kMeshRows - 1);
 817 |         for (int c = 0; c < kMeshCols; ++c) {
 818 |             const double ug = static_cast<double>(c) / (kMeshCols - 1);
 819 |             const size_t idx = static_cast<size_t>(r * kMeshCols + c);
 820 |             const double gx = ug * 640.0 - 0.5;
 821 |             const double gy = vg * 360.0 - 0.5;
 822 |             const Vec2 gObs = applyHomography(H_tgt_to_obs, gx, gy);
 823 |             const double duHom = (gObs.x - gx) / 640.0;
 824 |             const double dvHom = (gObs.y - gy) / 360.0;
 825 | 
 826 |             (void)inv2Sig2; (void)lamRbf; (void)alphaRbf;
 827 |             ctx.duRbfSmooth[idx] = 0.0;
 828 |             ctx.dvRbfSmooth[idx] = 0.0;
 829 |             duTotal[idx] = duHom;
 830 |             dvTotal[idx] = dvHom;
 831 |         }
 832 |     }
 833 | 
 834 |     std::array<Vec2, kMeshVerts> deltaM{};
 835 |     constexpr double kMaxDeltaUv = 0.0042; // Bounded fine visual authority (+-10.7px on 2560x1440)
 836 |     for (int r = 0; r < kMeshRows; ++r) {
 837 |         const int rPrev = std::max(0, r - 1);
 838 |         const int rNext = std::min(kMeshRows - 1, r + 1);
 839 |         const double scaleV = (rNext > rPrev) ? (static_cast<double>(kMeshRows - 1) / (rNext - rPrev)) : 0.0;
 840 |         for (int c = 0; c < kMeshCols; ++c) {
 841 |             const int cPrev = std::max(0, c - 1);
 842 |             const int cNext = std::min(kMeshCols - 1, c + 1);
 843 |             const double scaleU = (cNext > cPrev) ? (static_cast<double>(kMeshCols - 1) / (cNext - cPrev)) : 0.0;
 844 |             const size_t idx = static_cast<size_t>(r * kMeshCols + c);
 845 | 
 846 |             const Vec2 dM_du{
 847 |                 (ioResult->mesh[r * kMeshCols + cNext].u - ioResult->mesh[r * kMeshCols + cPrev].u) * scaleU,
 848 |                 (ioResult->mesh[r * kMeshCols + cNext].v - ioResult->mesh[r * kMeshCols + cPrev].v) * scaleU
 849 |             };
 850 |             const Vec2 dM_dv{
 851 |                 (ioResult->mesh[rNext * kMeshCols + c].u - ioResult->mesh[rPrev * kMeshCols + c].u) * scaleV,
 852 |                 (ioResult->mesh[rNext * kMeshCols + c].v - ioResult->mesh[rPrev * kMeshCols + c].v) * scaleV
 853 |             };
 854 |             double dxUv = dM_du.x * duTotal[idx] + dM_dv.x * dvTotal[idx];
 855 |             double dyUv = dM_du.y * duTotal[idx] + dM_dv.y * dvTotal[idx];
 856 |             const double magUv = std::hypot(dxUv, dyUv);
 857 |             if (magUv > kMaxDeltaUv) {
 858 |                 const double s = kMaxDeltaUv / magUv;
 859 |                 dxUv *= s;
 860 |                 dyUv *= s;
 861 |             }
 862 |             deltaM[idx] = {dxUv, dyUv};
 863 |         }
 864 |     }
 865 | 
 866 |     ctx.retireVisualCorrection = false;
 867 |     // 1-Euro Speed-Adaptive Filter:
 868 |     //   - When innovation is tiny (< 0.00045 UV ~ 1.1px on 2560x1440), use ultra-low alpha=0.040
 869 |     //     to eliminate 30Hz KLT sub-pixel micro-jitter so fine details stay razor-sharp ("不发虚").
 870 |     //   - When innovation is large (> 0.0022 UV ~ 5.6px, real wrist translation during swing),
 871 |     //     ramp alpha smoothly up to 0.32 for instant parallax lock.
 872 |     const size_t centerIdx = 10 * kMeshCols + 16;
 873 |     const double innovUv = ctx.hasDeltaM
 874 |         ? std::hypot(deltaM[centerIdx].x - ctx.lastDeltaM[centerIdx].x,
 875 |                      deltaM[centerIdx].y - ctx.lastDeltaM[centerIdx].y)
 876 |         : 0.0015;
 877 |     const double speedGain = gemini::clamp((innovUv - 0.00038) / 0.00140, 0.0, 1.0);
 878 |     const double swingGain = gemini::clamp((omegaNormRadS - 0.060) / 0.250, 0.0, 1.0);
 879 |     // Fast parallax cancellation up to 0.55 rad/s, then gentle motion-blur gate above 0.55 rad/s
 880 |     const double blurGate = 1.0 - 0.55 * gemini::clamp((omegaNormRadS - 0.55) / 0.60, 0.0, 1.0);
 881 |     const double alphaMesh = (0.040 + 0.340 * std::max(speedGain, swingGain * speedGain)) * blurGate;
 882 |     constexpr double kMaxStepUv = 0.0028;
 883 |     if (!ctx.hasDeltaM) {
 884 |         for (size_t k = 0; k < kMeshVerts; ++k) {
 885 |             ctx.lastDeltaM[k].x = alphaMesh * deltaM[k].x;
 886 |             ctx.lastDeltaM[k].y = alphaMesh * deltaM[k].y;
 887 |         }
 888 |         ctx.hasDeltaM = true;
 889 |     } else {
 890 |         for (size_t k = 0; k < kMeshVerts; ++k) {
 891 |             double stepX = alphaMesh * (deltaM[k].x - ctx.lastDeltaM[k].x);
 892 |             double stepY = alphaMesh * (deltaM[k].y - ctx.lastDeltaM[k].y);
 893 |             const double stepMag = std::hypot(stepX, stepY);
 894 |             if (stepMag > kMaxStepUv) {
 895 |                 const double s = kMaxStepUv / stepMag;
 896 |                 stepX *= s;
 897 |                 stepY *= s;
 898 |             }
 899 |             ctx.lastDeltaM[k].x += stepX;
 900 |             ctx.lastDeltaM[k].y += stepY;
 901 |         }
 902 |     }
 903 |     applyDeltaMToMesh(ctx, ctx.lastDeltaM, ioResult);
 904 | }
 905 | 
````

### Lines 906-1050
````text
 906 | int processFrameInternal(Context& c, const GeminiEisFrameInfo* frame, GeminiEisResult* result, FramePoseSnapshot* outSnap) {
 907 |     std::memset(result, 0, sizeof(*result));
 908 | 
 909 |     const int64_t readoutNs = frame->rolling_shutter_skew_ns > 0
 910 |         ? frame->rolling_shutter_skew_ns : c.config.default_readout_ns;
 911 |     const int64_t centerCameraNs = frame->sensor_timestamp_ns
 912 |         + frame->exposure_time_ns / 2 + readoutNs / 2;
 913 |     const int64_t centerGyroNs = centerCameraNs + c.config.camera_to_gyro_offset_ns;
 914 | 
 915 |     const int64_t firstRowGyroNs = frame->sensor_timestamp_ns
 916 |         + frame->exposure_time_ns / 2
 917 |         + static_cast<int64_t>(readoutNs * (0.5 / GEMINI_EIS_RS_BANDS))
 918 |         + c.config.camera_to_gyro_offset_ns;
 919 |     const int64_t lastRowGyroNs = frame->sensor_timestamp_ns
 920 |         + frame->exposure_time_ns / 2
 921 |         + static_cast<int64_t>(readoutNs * ((GEMINI_EIS_RS_BANDS - 0.5)
 922 |             / GEMINI_EIS_RS_BANDS))
 923 |         + c.config.camera_to_gyro_offset_ns;
 924 | 
 925 |     if (!c.gyro.covers(firstRowGyroNs) || !c.gyro.covers(lastRowGyroNs)
 926 |             || !c.gyro.covers(centerGyroNs)) {
 927 |         return GEMINI_EIS_ECOVERAGE;
 928 |     }
 929 | 
 930 |     GyroSample center{};
 931 |     if (!c.gyro.sample(centerGyroNs, &center)) return GEMINI_EIS_ECOVERAGE;
 932 | 
 933 |     double frameDtS = 1.0 / 30.0;
 934 |     if (c.haveLastFrame) {
 935 |         frameDtS = (centerGyroNs - c.lastFrameCenterGyroNs) * 1e-9;
 936 |         if (!(frameDtS > 1e-4 && frameDtS < 0.25)) {
 937 |             return GEMINI_EIS_ESTATE;
 938 |         }
 939 |         c.rawAngle = c.rawAngle + signedIntegrate(
 940 |             c.gyro, c.lastFrameCenterGyroNs, centerGyroNs);
 941 |     }
 942 |     c.lastFrameCenterGyroNs = centerGyroNs;
 943 |     c.haveLastFrame = true;
 944 | 
 945 |     Vec3 accel{};
 946 |     if (c.haveLastOmega) {
 947 |         accel = (center.omegaRadS - c.lastOmega) * (1.0 / frameDtS);
 948 |     }
 949 |     c.lastOmega = center.omegaRadS;
 950 |     c.haveLastOmega = true;
 951 | 
 952 |     Quat qCenter = Quat::identity();
 953 |     c.gyro.orientationAt(centerGyroNs, &qCenter);
 954 | 
 955 |     FrameInput in;
 956 |     in.timestampNs = frame->sensor_timestamp_ns;
 957 |     in.rawAngleRad = c.rawAngle;
 958 |     in.angularVelocityRadS = center.omegaRadS;
 959 |     in.angularAccelerationRadS2 = accel;
 960 |     in.zoom = frame->zoom > 0.0 ? frame->zoom : c.config.default_zoom;
 961 |     in.qCenter = qCenter;
 962 |     in.useQuats = true;
 963 | 
 964 |     if (c.visualTimestampNs > 0 && c.visualTimestampNs <= frame->sensor_timestamp_ns) {
 965 |         const double ageS = (frame->sensor_timestamp_ns - c.visualTimestampNs) * 1e-9;
 966 |         if (ageS <= 0.20 && c.visual.trust > 0.01) {
 967 |             const double decay = std::exp(-ageS / 0.60);
 968 |             in.visual = c.visual;
 969 | 
 970 |             const int64_t visGyroNs = c.visualTimestampNs + c.config.camera_to_gyro_offset_ns;
 971 |             if (c.gyro.covers(visGyroNs) && c.gyro.covers(centerGyroNs)) {
 972 |                 const Vec3 dR = signedIntegrate(c.gyro, visGyroNs, centerGyroNs);
 973 |                 const double cosR = std::cos(dR.z);
 974 |                 const double sinR = std::sin(dR.z);
 975 |                 const double tx = c.visual.txPx;
 976 |                 const double ty = c.visual.tyPx;
 977 |                 in.visual.txPx = (cosR * tx - sinR * ty) * decay;
 978 |                 in.visual.tyPx = (sinR * tx + cosR * ty) * decay;
 979 |             } else {
 980 |                 in.visual.txPx *= decay;
 981 |                 in.visual.tyPx *= decay;
 982 |             }
 983 |             in.visual.rotationRad *= decay;
 984 |             in.visual.scale = 1.0 + (in.visual.scale - 1.0) * decay;
 985 |             in.visual.trust *= decay;
 986 |         }
 987 |     }
 988 | 
 989 |     for (uint32_t row = 0; row < 20; ++row) {
 990 |         const double y = static_cast<double>(row) / 19.0;
 991 |         const int64_t rowCameraNs = frame->sensor_timestamp_ns
 992 |             + frame->exposure_time_ns / 2
 993 |             + static_cast<int64_t>(readoutNs * y);
 994 |         const int64_t rowGyroNs = rowCameraNs + c.config.camera_to_gyro_offset_ns;
 995 |         Quat qRow = Quat::identity();
 996 |         c.gyro.orientationAt(rowGyroNs, &qRow);
 997 |         in.rowQuats[row] = qRow;
 998 |         if (row < GEMINI_EIS_RS_BANDS) {
 999 |             in.rs19[row] = signedIntegrate(c.gyro, centerGyroNs, rowGyroNs);
1000 |         }
1001 |     }
1002 | 
1003 |     const auto out = c.core->processFrame(in);
1004 |     result->valid = 1;
1005 |     result->applied = 1;
1006 |     result->frame_index = ++c.frameIndex;
1007 | 
1008 |     result->raw_angle_rad[0] = c.rawAngle.x;
1009 |     result->raw_angle_rad[1] = c.rawAngle.y;
1010 |     result->raw_angle_rad[2] = c.rawAngle.z;
1011 |     result->target_angle_rad[0] = out.targetAngleRad.x;
1012 |     result->target_angle_rad[1] = out.targetAngleRad.y;
1013 |     result->target_angle_rad[2] = out.targetAngleRad.z;
1014 |     result->correction_rad[0] = out.correctionRad.x;
1015 |     result->correction_rad[1] = out.correctionRad.y;
1016 |     result->correction_rad[2] = out.correctionRad.z;
1017 |     result->jerk_rad_s3[0] = out.jerkRadS3.x;
1018 |     result->jerk_rad_s3[1] = out.jerkRadS3.y;
1019 |     result->jerk_rad_s3[2] = out.jerkRadS3.z;
1020 |     result->pan_confidence = out.panConfidence;
1021 |     result->oscillation_score = out.oscillationScore;
1022 |     result->crop_budget_rad[0] = out.crop.pitchRad;
1023 |     result->crop_budget_rad[1] = out.crop.yawRad;
1024 |     result->crop_budget_rad[2] = out.crop.rollRad;
1025 |     result->visual_tx_px = out.visual.txPx;
1026 |     result->visual_ty_px = out.visual.tyPx;
1027 |     result->visual_rotation_rad = out.visual.rotationRad;
1028 |     result->visual_scale = out.visual.scale;
1029 |     result->visual_trust = out.visual.trust;
1030 | 
1031 |     result->mesh_vertex_count = static_cast<uint32_t>(
1032 |         std::min<std::size_t>(out.mesh.size(), GEMINI_EIS_MESH_VERTEX_COUNT));
1033 |     for (uint32_t i = 0; i < result->mesh_vertex_count; ++i) {
1034 |         result->mesh[i].u = out.mesh[i].u;
1035 |         result->mesh[i].v = out.mesh[i].v;
1036 |     }
1037 | 
1038 |     FramePoseSnapshot snap{};
1039 |     snap.sensorTsNs = frame->sensor_timestamp_ns;
1040 |     snap.exposureNs = frame->exposure_time_ns;
1041 |     snap.readoutNs = readoutNs;
1042 |     snap.zoom = out.zoom > 0.0 ? out.zoom : 1.12;
1043 |     snap.qCenter = qCenter;
1044 |     snap.qTarget = out.qTarget;
1045 |     snap.rowQuats = in.rowQuats;
1046 |     snap.omegaNormRadS = center.omegaRadS.norm();
1047 |     c.poseHistory.push_back(snap);
1048 |     while (c.poseHistory.size() > 180) c.poseHistory.pop_front();
1049 |     if (outSnap) *outSnap = snap;
1050 |     return GEMINI_EIS_OK;
````

### Lines 1168-1181
````text
1168 | int GeminiEisProcessFrame(GeminiEisHandle* handle,
1169 |                           const GeminiEisFrameInfo* frame,
1170 |                           GeminiEisResult* result) {
1171 |     if (!handle || !*handle || !frame || !result) return GEMINI_EIS_EINVAL;
1172 |     if (frame->sensor_timestamp_ns <= 0 || frame->exposure_time_ns <= 0) {
1173 |         return GEMINI_EIS_EINVAL;
1174 |     }
1175 |     auto& c = *static_cast<Context*>(*handle);
1176 |     int rc = processFrameInternal(c, frame, result, nullptr);
1177 |     if (rc != GEMINI_EIS_OK) return rc;
1178 |     applyHeldDeltaMToMesh(c, result);
1179 |     applyViewportToMesh(c, result);
1180 |     return GEMINI_EIS_OK;
1181 | }
````

### Lines 1183-1283
````text
1183 | int GeminiEisProcessLiveYPlane(
1184 |     GeminiEisHandle* handle,
1185 |     const GeminiEisFrameInfo* frame,
1186 |     const uint8_t* y_plane,
1187 |     int y_width,
1188 |     int y_height,
1189 |     int y_row_stride,
1190 |     GeminiEisResult* result) {
1191 |     if (!handle || !*handle || !frame || !result) return GEMINI_EIS_EINVAL;
1192 |     if (frame->sensor_timestamp_ns <= 0 || frame->exposure_time_ns <= 0) {
1193 |         return GEMINI_EIS_EINVAL;
1194 |     }
1195 |     auto& c = *static_cast<Context*>(*handle);
1196 |     FramePoseSnapshot currPose{};
1197 |     int rc = processFrameInternal(c, frame, result, &currPose);
1198 |     if (rc != GEMINI_EIS_OK) return rc;
1199 | 
1200 |     if (y_plane && y_width >= 64 && y_height >= 48 && y_row_stride >= y_width) {
1201 |         const int wWork = 320;
1202 |         const int hWork = 180;
1203 |         std::vector<uint8_t> curImg(static_cast<size_t>(wWork * hWork));
1204 |         // 2x2 box-filtered downsampling to suppress live sensor photon/ISO noise
1205 |         for (int y = 0; y < hWork; ++y) {
1206 |             const int sy0 = (y * y_height) / hWork;
1207 |             const int sy1 = std::min(y_height - 1, sy0 + 1);
1208 |             const uint8_t* row0 = y_plane + sy0 * y_row_stride;
1209 |             const uint8_t* row1 = y_plane + sy1 * y_row_stride;
1210 |             for (int x = 0; x < wWork; ++x) {
1211 |                 const int sx0 = (x * y_width) / wWork;
1212 |                 const int sx1 = std::min(y_width - 1, sx0 + 1);
1213 |                 const int sum = static_cast<int>(row0[sx0]) + static_cast<int>(row0[sx1])
1214 |                               + static_cast<int>(row1[sx0]) + static_cast<int>(row1[sx1]);
1215 |                 curImg[static_cast<size_t>(y * wWork + x)] = static_cast<uint8_t>((sum + 2) >> 2);
1216 |             }
1217 |         }
1218 | 
1219 |         std::vector<uint8_t> curImgPyr1;
1220 |         buildPyr1(curImg, wWork, hWork, curImgPyr1);
1221 | 
1222 |         // Only evict stale keyframes when virtual camera qTarget has panned > 6.0 deg (0.105 rad),
1223 |         // preventing mid-swing keyframe dropouts and 3-6px visual resets!
1224 |         while (!c.keyframes.empty()) {
1225 |             const double targetShiftRad = (c.keyframes.front().pose.qTarget.inverse() * currPose.qTarget).angle();
1226 |             if (targetShiftRad > 0.105) {
1227 |                 c.keyframes.pop_front();
1228 |                 c.retireVisualCorrection = c.keyframes.empty() && c.hasDeltaM;
1229 |             } else {
1230 |                 break;
1231 |             }
1232 |         }
1233 | 
1234 |         std::vector<Vec2> ptsObs;
1235 |         std::vector<Vec2> ptsTgt;
1236 |         for (const auto& kf : c.keyframes) {
1237 |             trackKeyframeKlt(c, kf, currPose, curImg, curImgPyr1, wWork, hWork, ptsObs, ptsTgt);
1238 |         }
1239 | 
1240 |         if (ptsObs.size() >= 12) {
1241 |             applyCausal8DofAndRbfToMesh(c, ptsObs, ptsTgt, currPose.omegaNormRadS, result);
1242 |         } else {
1243 |             applyHeldDeltaMToMesh(c, result);
1244 |         }
1245 | 
1246 |         const bool needFirstKf = c.keyframes.empty();
1247 |         const bool lowMotionBlur = (currPose.omegaNormRadS < 0.65);
1248 |         const int64_t minKfSpacingNs = (c.keyframes.size() < 2) ? 400000000LL : 1200000000LL;
1249 |         const bool spacedEnough = c.keyframes.empty() ||
1250 |             (currPose.sensorTsNs - c.keyframes.back().pose.sensorTsNs) > minKfSpacingNs;
1251 |         if (needFirstKf || (lowMotionBlur && spacedEnough)) {
1252 |             CausalKeyframe newKf;
1253 |             newKf.pose = currPose;
1254 |             newKf.width = wWork;
1255 |             newKf.height = hWork;
1256 |             newKf.yImg = std::move(curImg);
1257 |             newKf.yImgPyr1 = std::move(curImgPyr1);
1258 |             extractGridFeatures(newKf.yImg, wWork, hWork, newKf.sensorPts);
1259 |             newKf.targetVirt.reserve(newKf.sensorPts.size());
1260 |             for (const auto& sp : newKf.sensorPts) {
1261 |                 const Vec2 v = unwarpSensorPointToVirtual(
1262 |                     c.config, (sp.x + 0.5) / wWork, (sp.y + 0.5) / hWork, currPose);
1263 |                 Vec2 tgtV{v.x * 640.0, v.y * 360.0};
1264 |                 if (!needFirstKf) {
1265 |                     // Anchor subsequent keyframes in the exact same visually-locked virtual coordinate frame as kf[0]
1266 |                     // to prevent 1.2s keyframe-replacement target creep!
1267 |                     tgtV = applyHomography(c.lastRigidH.data(), tgtV.x, tgtV.y);
1268 |                 }
1269 |                 newKf.targetVirt.push_back(tgtV);
1270 |             }
1271 |             if (newKf.sensorPts.size() >= 16) {
1272 |                 c.keyframes.push_back(std::move(newKf));
1273 |                 while (c.keyframes.size() > 3) {
1274 |                     c.keyframes.erase(c.keyframes.begin() + 1);
1275 |                 }
1276 |             }
1277 |         }
1278 |     } else {
1279 |         applyHeldDeltaMToMesh(c, result);
1280 |     }
1281 | 
1282 |     applyViewportToMesh(c, result);
1283 |     return GEMINI_EIS_OK;
````

## android_app/app/src/main/java/com/yuy/y3realtime/GeminiCamera2EisController.java

### Lines 91-119
````text
  91 |     // Exact Y1 (AR_6769) verified gyroscope-to-camera extrinsic matrix from YuyEIS2-pc-reference-Y3
  92 |     public static final double[] Y1_SENSOR_TO_CAMERA = new double[]{
  93 |         0.0, 1.0,  0.0,
  94 |         1.0, 0.0,  0.0,
  95 |         0.0, 0.0, -1.0
  96 |     };
  97 |     public static final long Y1_CAMERA_TO_GYRO_OFFSET_NS = 4_000_000L; // Calibrated +4.0ms camera-to-gyro offset on Y1
  98 |     public static final double Y1_BASE_FX_2560 = 972.0;               // Calibrated raw-sensor optical fx on Y1 2560x1440
  99 |     public static final double Y1_BASE_FY_1440 = 976.0;               // Calibrated raw-sensor optical fy on Y1 2560x1440
 100 | 
 101 |     private static final int DEFAULT_SENSOR_W = 2560;
 102 |     private static final int DEFAULT_SENSOR_H = 1440;
 103 |     private static final int OUTPUT_W = 1920;
 104 |     private static final int OUTPUT_H = 1080;
 105 |     private static final int Y_TRACK_W = 640;
 106 |     private static final int Y_TRACK_H = 360;
 107 | 
 108 |     // Timestamp-indexed ring buffer for CaptureResult metadata
 109 |     private static final int META_RING_SIZE = 24;
 110 |     private static final class CaptureMetaSlot {
 111 |         long sensorTsNs = 0;
 112 |         long exposureNs = 10_000_000L;
 113 |         long readoutNs = 29_000_000L;
 114 |     }
 115 |     private final CaptureMetaSlot[] metaRing = new CaptureMetaSlot[META_RING_SIZE];
 116 |     private int metaWriteIdx = 0;
 117 |     private long latestCaptureTsNs = 0;
 118 |     private long lastKnownExposureNs = 28_000_000L; // Realistic indoor 30fps default exposure (~28ms) when CaptureResult lags
 119 |     private long lastKnownReadoutNs = 29_190_000L;  // Exact Y1 IMX476 rolling shutter skew (29.19ms)
````

### Lines 530-556
````text
 530 |         final float[] texTransform = "AR_6769".equals(Build.DEVICE) ? null : stMatrix;
 531 |         final int effRotDeg = computeEffectivePreviewRotationDeg();
 532 | 
 533 |         final boolean solveEis = eisEnabled.get();
 534 |         if (solveEis) {
 535 |             if (!eisWasEnabledOnGlThread) {
 536 |                 // OFF bypasses native frames, so the old timestamp would cause
 537 |                 // a persistent ESTATE after a long pause. Restart the causal
 538 |                 // solver before accepting the first new ON frame.
 539 |                 nativeSession.reset();
 540 |                 lastCenterU = -1f;
 541 |                 lastCenterV = -1f;
 542 |                 Log.i(TAG, "EIS OFF->ON: reset native causal frame state");
 543 |             }
 544 |             eisWasEnabledOnGlThread = true;
 545 |             rc = nativeSession.processFrameToGlesMesh(
 546 |                     matchedYBuf,
 547 |                     matchedYW,
 548 |                     matchedYH,
 549 |                     matchedYStride,
 550 |                     oesTsNs,
 551 |                     expNs,
 552 |                     skewNs,
 553 |                     texTransform,
 554 |                     vboBuf);
 555 |             if (rc != 0) {
 556 |                 coverageHoldCount++;
````
