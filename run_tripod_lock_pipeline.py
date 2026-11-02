# -*- coding: utf-8 -*-
import os, sys, time, json, subprocess, gc, shutil
from pathlib import Path
import cv2
import numpy as np
import imageio_ffmpeg
from scipy.signal import detrend
from scipy.ndimage import gaussian_filter1d

sys.path.append(r"D:/hongda/app/YuyEIS2-pc-reference-Y1/tools")
sys.path.append(r"D:/hongda/app/YuySuperEIS")
from YuyEis3.reference.pc_batch03.vision import track_pair, center_displacement, normalize_matrices

CASES = [
    ("024605", "20261017_024605_836", "Case 024605 (Low-speed sway 7.2 deg)"),
    ("024618", "20261017_024618_130", "Case 024618 (Med-speed sway 12.3 deg)"),
    ("024630", "20261017_024630_567", "Case 024630 (Fast sway 14.8 deg)"),
    ("024647", "20261017_024647_114", "Case 024647 (One-sided pan 12.4 deg)")
]

BUNDLE_ROOT = Path("D:/hongda/app/YuySuperEIS/artifacts/b38_restore_bundles_20261020_172849")
BATCH_DIR = Path("D:/Yuy/doc/EIS/eis10/03/EIS10_4DoF_BATCH_03")
OUT_DIR = Path("D:/hongda/app/GeminiEis/rendered_preview_fixed")
OUT_DIR.mkdir(parents=True, exist_ok=True)
PREVIEW_DIR = Path("D:/hongda/app/GeminiEis/rendered_preview")
PREVIEW_DIR.mkdir(parents=True, exist_ok=True)
EXE_PATH = Path("D:/hongda/app/GeminiEis/build/gemini_eis_replay.exe")

ROWS, COLS = 20, 33
out_w, out_h = 1280, 720

def build_sampler(output_w=1280, output_h=720):
    xs = np.arange(output_w, dtype=np.float32)
    ys = np.arange(output_h, dtype=np.float32)
    u = (xs + 0.5) / output_w
    v = (ys + 0.5) / output_h
    cf = np.minimum(u * 32.0, np.nextafter(np.float32(32.0), np.float32(0.0)))
    rf = np.minimum(v * 19.0, np.nextafter(np.float32(19.0), np.float32(0.0)))
    col = np.floor(cf).astype(np.int32)
    row = np.floor(rf).astype(np.int32)
    tx = (cf - col).astype(np.float32)
    ty = (rf - row).astype(np.float32)

    col2 = np.tile(col, output_h)
    row2 = np.repeat(row, output_w)
    tx2 = np.tile(tx, output_h)
    ty2 = np.repeat(ty, output_w)
    lower = (tx2 + ty2) <= 1.0

    tl = row2 * COLS + col2
    bl = tl + COLS
    tr = tl + 1
    br = bl + 1

    i1 = np.empty_like(tl); i2 = np.empty_like(tl); i3 = np.empty_like(tl)
    w1 = np.empty_like(tx2); w2 = np.empty_like(tx2); w3 = np.empty_like(tx2)

    i1[lower] = tl[lower]; i2[lower] = bl[lower]; i3[lower] = tr[lower]
    w1[lower] = (1.0 - tx2[lower]) - ty2[lower]
    w2[lower] = ty2[lower]
    w3[lower] = tx2[lower]

    upper = ~lower
    i1[upper] = tr[upper]; i2[upper] = bl[upper]; i3[upper] = br[upper]
    w1[upper] = 1.0 - ty2[upper]
    w2[upper] = 1.0 - tx2[upper]
    w3[upper] = (tx2[upper] + ty2[upper]) - 1.0
    return (i1, i2, i3, w1, w2, w3, output_w, output_h)

def get_remap_coords(vertices, weights, source_w=2560, source_h=1440):
    i1, i2, i3, w1, w2, w3, ow, oh = weights
    vx, vy = vertices[:, 0], vertices[:, 1]
    sx = w1 * vx[i1] + w2 * vx[i2] + w3 * vx[i3]
    sy = w1 * vy[i1] + w2 * vy[i2] + w3 * vy[i3]
    mx = (sx * source_w - 0.5).astype(np.float32).reshape(oh, ow)
    my = (sy * source_h - 0.5).astype(np.float32).reshape(oh, ow)
    return mx, my

def stream_active_frames(raw_video_path, indices):
    cap = cv2.VideoCapture(str(raw_video_path))
    if hasattr(cv2, "CAP_PROP_ORIENTATION_AUTO"):
        cap.set(cv2.CAP_PROP_ORIENTATION_AUTO, 0)
    wanted = 0
    try:
        for idx in range(indices[-1] + 1):
            ret, frame = cap.read()
            if not ret:
                break
            if idx < indices[wanted]:
                continue
            yield wanted, frame
            wanted += 1
            if wanted == len(indices):
                break
    finally:
        cap.release()

def extract_grid_features(gray, gx=6, gy=4, max_per_cell=25, border=10):
    h, w = gray.shape
    cw = (w - 2 * border) / gx
    ch = (h - 2 * border) / gy
    pts_list = []
    for iy in range(gy):
        for ix in range(gx):
            x0 = int(round(border + ix * cw))
            y0 = int(round(border + iy * ch))
            x1 = int(round(border + (ix + 1) * cw))
            y1 = int(round(border + (iy + 1) * ch))
            roi = gray[y0:y1, x0:x1]
            p = cv2.goodFeaturesToTrack(roi, maxCorners=max_per_cell, qualityLevel=0.008, minDistance=6, blockSize=7)
            if p is not None and len(p) > 0:
                p = p.reshape(-1, 2)
                p[:, 0] += x0
                p[:, 1] += y0
                pts_list.append(p)
    if not pts_list:
        return np.empty((0, 2), dtype=np.float32)
    return np.vstack(pts_list).astype(np.float32)

def match_points_lk(g0, g1, pts0, init_pts1=None, fb_thresh=1.0):
    if len(pts0) < 8:
        return np.empty((0, 2), dtype=np.float32), np.empty((0, 2), dtype=np.float32)
    p0 = np.ascontiguousarray(pts0, dtype=np.float32).reshape(-1, 1, 2)
    lk_params = dict(winSize=(25, 25), maxLevel=4,
                     criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 30, 0.01))
    if init_pts1 is not None:
        p1_init = np.ascontiguousarray(init_pts1, dtype=np.float32).reshape(-1, 1, 2).copy()
        p1, st1, _ = cv2.calcOpticalFlowPyrLK(g0, g1, p0, p1_init, flags=cv2.OPTFLOW_USE_INITIAL_FLOW, **lk_params)
    else:
        p1, st1, _ = cv2.calcOpticalFlowPyrLK(g0, g1, p0, None, **lk_params)
    p0r, st2, _ = cv2.calcOpticalFlowPyrLK(g1, g0, p1, None, **lk_params)
    fb = np.linalg.norm((p0 - p0r).reshape(-1, 2), axis=1)
    p1_flat = p1.reshape(-1, 2)
    h, w = g0.shape
    in_bounds = (p1_flat[:, 0] >= 4) & (p1_flat[:, 0] < w - 4) & (p1_flat[:, 1] >= 4) & (p1_flat[:, 1] < h - 4)
    ok = (st1.ravel() == 1) & (st2.ravel() == 1) & (fb < fb_thresh) & in_bounds
    return pts0[ok], p1_flat[ok]

def fit_rigid_se2(pts0, pts1, cx=320.0, cy=180.0):
    if len(pts0) < 8:
        return np.eye(3, dtype=np.float64), np.ones(len(pts0), dtype=bool)
    m, inliers = cv2.estimateAffinePartial2D(
        pts0.reshape(-1, 1, 2), pts1.reshape(-1, 1, 2),
        method=cv2.RANSAC, ransacReprojThreshold=1.8, maxIters=2000, confidence=0.995
    )
    if m is None:
        return np.eye(3, dtype=np.float64), np.ones(len(pts0), dtype=bool)
    mask = (inliers.ravel() == 1) if inliers is not None else np.ones(len(pts0), dtype=bool)
    if np.sum(mask) >= 6:
        p0_in = pts0[mask].astype(np.float64)
        p1_in = pts1[mask].astype(np.float64)
        c0 = np.mean(p0_in, axis=0)
        c1 = np.mean(p1_in, axis=0)
        q0 = p0_in - c0
        q1 = p1_in - c1
        num = np.sum(q0[:, 0] * q1[:, 1] - q0[:, 1] * q1[:, 0])
        den = np.sum(q0[:, 0] * q1[:, 0] + q0[:, 1] * q1[:, 1])
        th = np.arctan2(num, den)
        ca, sa = np.cos(th), np.sin(th)
        tx = c1[0] - (ca * c0[0] - sa * c0[1])
        ty = c1[1] - (sa * c0[0] + ca * c0[1])
    else:
        a, b = float(m[0, 0]), float(m[1, 0])
        s = max(np.hypot(a, b), 1e-8)
        ca, sa = a / s, b / s
        tx_raw, ty_raw = float(m[0, 2]), float(m[1, 2])
        fcx = a * cx - b * cy + tx_raw
        fcy = b * cx + a * cy + ty_raw
        tx = fcx - (ca * cx - sa * cy)
        ty = fcy - (sa * cx + ca * cy)
    se2 = np.array([[ca, -sa, tx], [sa, ca, ty], [0.0, 0.0, 1.0]], dtype=np.float64)
    return se2, mask

def apply_homography_pts(H, pts):
    if len(pts) == 0:
        return np.empty((0, 2), dtype=np.float64)
    p = np.column_stack([pts.astype(np.float64), np.ones(len(pts), dtype=np.float64)])
    q = (H @ p.T).T
    w = np.where(np.abs(q[:, 2:3]) > 1e-8, q[:, 2:3], 1e-8)
    return np.ascontiguousarray(q[:, :2] / w, dtype=np.float64)

def fit_regularized_8dof_delta(
    pts_src, pts_dst, H_base, w=640.0, h=360.0,
    lam_trans=0.02, lam_aff=0.15, lam_iso=0.50, lam_persp=0.30,
    max_aff=0.06, max_persp=0.035
):
    if len(pts_src) < 12:
        return np.zeros(8, dtype=np.float64)

    pts_base = apply_homography_pts(H_base, pts_src)
    cx, cy = w * 0.5, h * 0.5
    x = (pts_base[:, 0] - cx) / cx
    y = (pts_base[:, 1] - cy) / cy
    xp = (pts_dst[:, 0] - cx) / cx
    yp = (pts_dst[:, 1] - cy) / cy

    res_init = np.hypot(pts_dst[:, 0] - pts_base[:, 0], pts_dst[:, 1] - pts_base[:, 1])
    med_res = np.median(res_init)
    valid = res_init < max(10.0, med_res * 3.0)
    if np.sum(valid) < 12:
        return np.zeros(8, dtype=np.float64)

    x, y, xp, yp = x[valid], y[valid], xp[valid], yp[valid]
    n = len(x)

    A = np.zeros((2 * n, 8), dtype=np.float64)
    b = np.zeros(2 * n, dtype=np.float64)

    A[0::2, 0] = x
    A[0::2, 1] = y
    A[0::2, 2] = 1.0
    A[0::2, 6] = -x * xp
    A[0::2, 7] = -y * xp
    b[0::2] = xp - x

    A[1::2, 3] = x
    A[1::2, 4] = y
    A[1::2, 5] = 1.0
    A[1::2, 6] = -x * yp
    A[1::2, 7] = -y * yp
    b[1::2] = yp - y

    Reg = np.diag([lam_aff, lam_aff, lam_trans, lam_aff, lam_aff, lam_trans, lam_persp, lam_persp])
    iso_vec = np.array([1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0], dtype=np.float64)
    Reg += lam_iso * np.outer(iso_vec, iso_vec)

    weights = np.ones(2 * n, dtype=np.float64)
    delta = np.zeros(8, dtype=np.float64)
    huber_thresh = 1.2 / cx

    for _ in range(5):
        Aw = A * weights[:, None]
        lhs = A.T @ Aw + Reg
        rhs = A.T @ (weights * b)
        try:
            delta = np.linalg.solve(lhs, rhs)
        except np.linalg.LinAlgError:
            return np.zeros(8, dtype=np.float64)
        r = A @ delta - b
        r_pt = np.hypot(r[0::2], r[1::2])
        w_pt = np.where(r_pt <= huber_thresh, 1.0, huber_thresh / (r_pt + 1e-12))
        weights[0::2] = w_pt
        weights[1::2] = w_pt

    delta[0] = np.clip(delta[0], -max_aff, max_aff)
    delta[1] = np.clip(delta[1], -max_aff, max_aff)
    delta[3] = np.clip(delta[3], -max_aff, max_aff)
    delta[4] = np.clip(delta[4], -max_aff, max_aff)
    delta[6] = np.clip(delta[6], -max_persp, max_persp)
    delta[7] = np.clip(delta[7], -max_persp, max_persp)
    return delta

def delta_to_homography(delta, H_base, w=640.0, h=360.0):
    cx, cy = w * 0.5, h * 0.5
    H_res_norm = np.array([
        [1.0 + delta[0], delta[1], delta[2]],
        [delta[3], 1.0 + delta[4], delta[5]],
        [delta[6], delta[7], 1.0]
    ], dtype=np.float64)
    T_norm = np.array([[1.0 / cx, 0.0, -1.0], [0.0, 1.0 / cy, -1.0], [0.0, 0.0, 1.0]], dtype=np.float64)
    T_inv = np.array([[cx, 0.0, cx], [0.0, cy, cy], [0.0, 0.0, 1.0]], dtype=np.float64)
    return (T_inv @ H_res_norm @ T_norm) @ H_base

def build_virtual_tripod_anchor(se2_path, deadband_px=45.0, max_corr_px=26.0, cx=320.0, cy=180.0):
    n = len(se2_path)
    params = np.zeros((n, 3), dtype=np.float64)
    for i in range(n):
        m = se2_path[i]
        ca, sa = m[0, 0], m[1, 0]
        tx, ty = m[0, 2], m[1, 2]
        fcx = ca * cx - sa * cy + tx
        fcy = sa * cx + ca * cy + ty
        params[i, 0] = fcx - cx
        params[i, 1] = fcy - cy
        params[i, 2] = np.arctan2(sa, ca)

    anchor = np.zeros_like(params)
    anchor[0] = np.median(params[min(25, n - 1):min(140, n)], axis=0)
    dt = 1.0 / 30.0

    for i in range(1, n):
        diff = params[i] - anchor[i - 1]
        dist_px = np.hypot(diff[0], diff[1]) + abs(diff[2]) * 300.0
        if dist_px <= deadband_px:
            tau = 50.0
        else:
            excess = (dist_px - deadband_px) / 5.0
            tau = max(0.40, 50.0 / (1.0 + excess * excess * 15.0))
        alpha = dt / (tau + dt)
        anchor[i] = (1.0 - alpha) * anchor[i - 1] + alpha * params[i]

        corr_dx = params[i, 0] - anchor[i, 0]
        corr_dy = params[i, 1] - anchor[i, 1]
        corr_mag = np.hypot(corr_dx, corr_dy)
        if corr_mag > max_corr_px:
            scale_back = max_corr_px / corr_mag
            anchor[i, 0] = params[i, 0] - corr_dx * scale_back
            anchor[i, 1] = params[i, 1] - corr_dy * scale_back

    for k in range(3):
        anchor[:, k] = gaussian_filter1d(anchor[:, k], sigma=8.0, mode="nearest")

    m_anchor = np.zeros((n, 3, 3), dtype=np.float64)
    h_rigid = np.zeros((n, 3, 3), dtype=np.float64)
    for i in range(n):
        tx_a, ty_a, th_a = anchor[i]
        ca, sa = np.cos(th_a), np.sin(th_a)
        ma = np.array([
            [ca, -sa, cx + tx_a - (ca * cx - sa * cy)],
            [sa,  ca, cy + ty_a - (sa * cx + ca * cy)],
            [0.0, 0.0, 1.0]
        ], dtype=np.float64)
        m_anchor[i] = ma
        h_rigid[i] = ma @ np.linalg.inv(se2_path[i])
    return params, anchor, m_anchor, h_rigid

def match_multi_keyframes(gray_frames, kf_indices, kf_features, H_kf_locked, m_anchor, h_rigid, i, fb_thresh=0.95):
    pts_obs_list = []
    pts_tgt_list = []
    for k in kf_indices:
        pts_k = kf_features[k]
        # Predict position of keyframe k's features in frame i
        H_k_to_i = np.linalg.inv(h_rigid[i]) @ m_anchor[i] @ np.linalg.inv(m_anchor[k]) @ H_kf_locked[k]
        pred_i = apply_homography_pts(H_k_to_i, pts_k)
        mk, mi = match_points_lk(gray_frames[k], gray_frames[i], pts_k, init_pts1=pred_i, fb_thresh=fb_thresh)
        if len(mi) >= 10:
            H_k_target = m_anchor[i] @ np.linalg.inv(m_anchor[k]) @ H_kf_locked[k]
            pts_obs_list.append(mi)
            pts_tgt_list.append(apply_homography_pts(H_k_target, mk))
    return pts_obs_list, pts_tgt_list

def compute_stage2a_8dof_homographies(gray_pass1, max_corr_px=26.0):
    n = len(gray_pass1)
    se2_path = np.repeat(np.eye(3, dtype=np.float64)[None], n, axis=0)

    for i in range(1, n):
        p0 = extract_grid_features(gray_pass1[i - 1], gx=6, gy=4, max_per_cell=25)
        m0, m1 = match_points_lk(gray_pass1[i - 1], gray_pass1[i], p0)
        se2, _ = fit_rigid_se2(m0, m1)
        se2_path[i] = se2 @ se2_path[i - 1]

    _, anchor, m_anchor, h_rigid = build_virtual_tripod_anchor(
        se2_path, deadband_px=45.0, max_corr_px=max_corr_px
    )

    # Multi-keyframe ensemble locked to primary anchor keyframe k0=40
    vel = np.zeros(n, dtype=np.float64)
    vel[1:] = np.hypot(np.diff(anchor[:, 0]), np.diff(anchor[:, 1]))
    med_pose = np.median(anchor[min(32, n-1):], axis=0)
    dist_med = np.hypot(anchor[:, 0] - med_pose[0], anchor[:, 1] - med_pose[1])
    score = dist_med + 4.0 * vel
    kf_indices = []
    for w0, w1 in [(32, 90), (90, 155), (155, 220), (220, min(280, n))]:
        if w1 > w0:
            kf_indices.append(int(w0 + np.argmin(score[w0:w1])))
    if not kf_indices:
        kf_indices = [min(40, n - 1)]
    k0 = min(kf_indices, key=lambda k: score[k])
    kf_features = {k: extract_grid_features(gray_pass1[k], gx=6, gy=4, max_per_cell=28) for k in kf_indices}
    H_kf_locked = {k: h_rigid[k].copy() for k in kf_indices}

    # Align secondary keyframes in kf_indices directly to k0 so all keyframes share the exact same 8-DoF frame
    for k in kf_indices:
        if k == k0:
            continue
        pts_k0 = kf_features[k0]
        H_k0_to_k = np.linalg.inv(h_rigid[k]) @ m_anchor[k] @ np.linalg.inv(m_anchor[k0]) @ h_rigid[k0]
        pred_k = apply_homography_pts(H_k0_to_k, pts_k0)
        mk0, mk = match_points_lk(gray_pass1[k0], gray_pass1[k], pts_k0, init_pts1=pred_k, fb_thresh=0.9)
        if len(mk) >= 20:
            H_k0_target = m_anchor[k] @ np.linalg.inv(m_anchor[k0]) @ h_rigid[k0]
            tgt_k = apply_homography_pts(H_k0_target, mk0)
            dk = fit_regularized_8dof_delta(mk, tgt_k, h_rigid[k], lam_trans=0.01, lam_aff=0.1, lam_iso=0.3, lam_persp=0.2)
            H_kf_locked[k] = delta_to_homography(dk, h_rigid[k])

    deltas = np.zeros((n, 8), dtype=np.float64)
    for i in range(n):
        pts_obs_list, pts_tgt_list = match_multi_keyframes(
            gray_pass1, kf_indices, kf_features, H_kf_locked, m_anchor, h_rigid, i, fb_thresh=0.95
        )
        if pts_obs_list:
            pts_obs = np.vstack(pts_obs_list)
            pts_tgt = np.vstack(pts_tgt_list)
            deltas[i] = fit_regularized_8dof_delta(
                pts_obs, pts_tgt, h_rigid[i],
                lam_trans=0.02, lam_aff=0.15, lam_iso=0.50, lam_persp=0.30
            )

    # Temporally smooth ONLY the non-translation deformation terms (dh0, dh1, dh3, dh4, dh6, dh7)
    # while keeping translation (dh2, dh5) and rigid SE(2) instantaneous!
    for idx_col in [0, 1, 3, 4, 6, 7]:
        deltas[:, idx_col] = gaussian_filter1d(deltas[:, idx_col], sigma=2.0, mode="nearest")

    H_all = np.zeros((n, 3, 3), dtype=np.float64)
    for i in range(n):
        H_all[i] = delta_to_homography(deltas[i], h_rigid[i])

    return normalize_matrices(H_all, 640, 360)

def refine_stage2b_keyframe_mesh_lock(gray_pass2, mesh2, w_in=2560, h_in=1440):
    n = len(gray_pass2)
    se2_path = np.repeat(np.eye(3, dtype=np.float64)[None], n, axis=0)

    for i in range(1, n):
        p0 = extract_grid_features(gray_pass2[i - 1], gx=6, gy=4, max_per_cell=25)
        m0, m1 = match_points_lk(gray_pass2[i - 1], gray_pass2[i], p0, fb_thresh=0.85)
        se2, _ = fit_rigid_se2(m0, m1)
        se2_path[i] = se2 @ se2_path[i - 1]

    _, anchor, m_anchor, h_rigid = build_virtual_tripod_anchor(
        se2_path, deadband_px=45.0, max_corr_px=20.0
    )

    vel = np.zeros(n, dtype=np.float64)
    vel[1:] = np.hypot(np.diff(anchor[:, 0]), np.diff(anchor[:, 1]))
    med_pose = np.median(anchor[min(32, n-1):], axis=0)
    dist_med = np.hypot(anchor[:, 0] - med_pose[0], anchor[:, 1] - med_pose[1])
    score = dist_med + 4.0 * vel
    kf_indices = []
    for w0, w1 in [(32, 90), (90, 155), (155, 220), (220, min(280, n))]:
        if w1 > w0:
            kf_indices.append(int(w0 + np.argmin(score[w0:w1])))
    if not kf_indices:
        kf_indices = [min(40, n - 1)]
    k0 = min(kf_indices, key=lambda k: score[k])
    kf_features = {k: extract_grid_features(gray_pass2[k], gx=6, gy=4, max_per_cell=28) for k in kf_indices}
    H_kf_locked = {k: h_rigid[k].copy() for k in kf_indices}

    for k in kf_indices:
        if k == k0:
            continue
        pts_k0 = kf_features[k0]
        H_k0_to_k = np.linalg.inv(h_rigid[k]) @ m_anchor[k] @ np.linalg.inv(m_anchor[k0]) @ h_rigid[k0]
        pred_k = apply_homography_pts(H_k0_to_k, pts_k0)
        mk0, mk = match_points_lk(gray_pass2[k0], gray_pass2[k], pts_k0, init_pts1=pred_k, fb_thresh=0.85)
        if len(mk) >= 20:
            H_k0_target = m_anchor[k] @ np.linalg.inv(m_anchor[k0]) @ h_rigid[k0]
            tgt_k = apply_homography_pts(H_k0_target, mk0)
            dk = fit_regularized_8dof_delta(mk, tgt_k, h_rigid[k], lam_trans=0.01, lam_aff=0.1, lam_iso=0.3, lam_persp=0.2)
            H_kf_locked[k] = delta_to_homography(dk, h_rigid[k])

    ug = (np.arange(COLS, dtype=np.float64) / (COLS - 1.0))[None, :]
    vg = (np.arange(ROWS, dtype=np.float64) / (ROWS - 1.0))[:, None]
    ug_flat = np.broadcast_to(ug, (ROWS, COLS)).reshape(-1)
    vg_flat = np.broadcast_to(vg, (ROWS, COLS)).reshape(-1)
    pts_grid_640 = np.column_stack([ug_flat * 640.0 - 0.5, vg_flat * 360.0 - 0.5])

    deltas_p2 = np.zeros((n, 8), dtype=np.float64)
    du_rbf_all = np.zeros((n, ROWS * COLS), dtype=np.float64)
    dv_rbf_all = np.zeros((n, ROWS * COLS), dtype=np.float64)
    obs_cache = [None] * n

    for i in range(n):
        pts_obs_list, pts_tgt_list = match_multi_keyframes(
            gray_pass2, kf_indices, kf_features, H_kf_locked, m_anchor, h_rigid, i, fb_thresh=0.85
        )
        if pts_obs_list:
            pts_obs = np.vstack(pts_obs_list)
            pts_tgt = np.vstack(pts_tgt_list)
            obs_cache[i] = (pts_obs, pts_tgt)
            deltas_p2[i] = fit_regularized_8dof_delta(
                pts_obs, pts_tgt, h_rigid[i],
                lam_trans=0.02, lam_aff=0.12, lam_iso=0.40, lam_persp=0.25,
                max_aff=0.045, max_persp=0.025
            )

    # Smooth non-translation 8-DoF parameters across time
    for idx_col in [0, 1, 3, 4, 6, 7]:
        deltas_p2[:, idx_col] = gaussian_filter1d(deltas_p2[:, idx_col], sigma=2.0, mode="nearest")

    sigma_rbf = 0.20
    inv_2sig2 = 1.0 / (2.0 * sigma_rbf * sigma_rbf)
    lam_rbf = 2.0

    du_hom_all = np.zeros((n, ROWS * COLS), dtype=np.float64)
    dv_hom_all = np.zeros((n, ROWS * COLS), dtype=np.float64)

    for i in range(n):
        H_p2 = delta_to_homography(deltas_p2[i], h_rigid[i])
        try:
            H_tgt_to_obs = np.linalg.inv(H_p2)
        except np.linalg.LinAlgError:
            H_tgt_to_obs = np.linalg.inv(h_rigid[i])

        grid_obs = apply_homography_pts(H_tgt_to_obs, pts_grid_640)
        du_hom_all[i] = (grid_obs[:, 0] - pts_grid_640[:, 0]) / 640.0
        dv_hom_all[i] = (grid_obs[:, 1] - pts_grid_640[:, 1]) / 360.0

        if obs_cache[i] is not None:
            pts_obs, pts_tgt = obs_cache[i]
            tgt_mapped_obs = apply_homography_pts(H_tgt_to_obs, pts_tgt)
            res_u = (pts_obs[:, 0] - tgt_mapped_obs[:, 0]) / 640.0
            res_v = (pts_obs[:, 1] - tgt_mapped_obs[:, 1]) / 360.0
            res_mag_px = np.hypot(res_u * 640.0, res_v * 360.0)
            inl_rbf = res_mag_px < 4.0
            if np.sum(inl_rbf) >= 15:
                u_m = (pts_tgt[inl_rbf, 0] + 0.5) / 640.0
                v_m = (pts_tgt[inl_rbf, 1] + 0.5) / 360.0
                ru = res_u[inl_rbf]
                rv = res_v[inl_rbf]
                dist2 = (ug_flat[:, None] - u_m[None, :]) ** 2 + (vg_flat[:, None] - v_m[None, :]) ** 2
                W = np.exp(-dist2 * inv_2sig2)
                denom = lam_rbf + np.sum(W, axis=1)
                du_rbf_all[i] = (W @ ru) / denom
                dv_rbf_all[i] = (W @ rv) / denom

    # Temporally smooth the non-rigid RBF field so local edge parallax never shimmers
    du_rbf_all = gaussian_filter1d(du_rbf_all, sigma=2.2, axis=0, mode="nearest")
    dv_rbf_all = gaussian_filter1d(dv_rbf_all, sigma=2.2, axis=0, mode="nearest")

    du_all = du_hom_all + du_rbf_all
    dv_all = dv_hom_all + dv_rbf_all

    mesh_locked = mesh2.copy()
    min_u = 16.0 / w_in
    max_u = 1.0 - min_u
    min_v = 16.0 / h_in
    max_v = 1.0 - min_v

    for i in range(n):
        M = mesh2[i].reshape(ROWS, COLS, 2).astype(np.float64)
        dM_du = (COLS - 1.0) * np.gradient(M, axis=1)
        dM_dv = (ROWS - 1.0) * np.gradient(M, axis=0)
        du_grid = du_all[i].reshape(ROWS, COLS, 1)
        dv_grid = dv_all[i].reshape(ROWS, COLS, 1)
        delta_M = dM_du * du_grid + dM_dv * dv_grid

        alpha = 1.0
        for _ in range(10):
            cand = M + alpha * delta_M
            if (np.min(cand[:, :, 0]) >= min_u and np.max(cand[:, :, 0]) <= max_u and
                np.min(cand[:, :, 1]) >= min_v and np.max(cand[:, :, 1]) <= max_v):
                break
            alpha *= 0.82
        cand = M + alpha * delta_M
        cand[:, :, 0] = np.clip(cand[:, :, 0], min_u, max_u)
        cand[:, :, 1] = np.clip(cand[:, :, 1], min_v, max_v)
        mesh_locked[i] = cand.reshape(-1, 2).astype(np.float32)

    return mesh_locked

def evaluate_video_sway(video_path):
    cap = cv2.VideoCapture(str(video_path))
    grays = []
    while True:
        ret, f = cap.read()
        if not ret:
            break
        grays.append(cv2.cvtColor(cv2.resize(f, (640, 360), interpolation=cv2.INTER_AREA), cv2.COLOR_BGR2GRAY))
    cap.release()

    motions = []
    dxs, dys = [], []
    lx, ly, cx_r, cy_r, rx, ry = [0.0], [0.0], [0.0], [0.0], [0.0], [0.0]

    for i in range(len(grays) - 1):
        sim, _, _ = track_pair(grays[i], grays[i + 1])
        if sim is not None:
            d = center_displacement(sim[None], 640, 360)[0] * 2.0
            dxs.append(float(d[0]))
            dys.append(float(d[1]))
            if i >= 30:
                motions.append(float(np.linalg.norm(d)))
        else:
            dxs.append(0.0)
            dys.append(0.0)

        p0 = cv2.goodFeaturesToTrack(grays[i], maxCorners=400, qualityLevel=0.01, minDistance=8)
        if p0 is not None and len(p0) > 10:
            p1, st, _ = cv2.calcOpticalFlowPyrLK(grays[i], grays[i + 1], p0, None, winSize=(21, 21), maxLevel=3)
            p0r, st2, _ = cv2.calcOpticalFlowPyrLK(grays[i + 1], grays[i], p1, None, winSize=(21, 21), maxLevel=3)
            fb = np.linalg.norm(p0 - p0r, axis=2).ravel()
            ok = (st.ravel() == 1) & (st2.ravel() == 1) & (fb < 1.0)
            pts0 = p0[ok].reshape(-1, 2)
            flow = (p1[ok].reshape(-1, 2) - pts0) * 2.0
            l_m = pts0[:, 0] < 180
            c_m = (pts0[:, 0] >= 200) & (pts0[:, 0] <= 440)
            r_m = pts0[:, 0] > 460
            lx.append(lx[-1] + (float(np.median(flow[l_m, 0])) if np.sum(l_m) > 3 else 0.0))
            ly.append(ly[-1] + (float(np.median(flow[l_m, 1])) if np.sum(l_m) > 3 else 0.0))
            cx_r.append(cx_r[-1] + (float(np.median(flow[c_m, 0])) if np.sum(c_m) > 3 else 0.0))
            cy_r.append(cy_r[-1] + (float(np.median(flow[c_m, 1])) if np.sum(c_m) > 3 else 0.0))
            rx.append(rx[-1] + (float(np.median(flow[r_m, 0])) if np.sum(r_m) > 3 else 0.0))
            ry.append(ry[-1] + (float(np.median(flow[r_m, 1])) if np.sum(r_m) > 3 else 0.0))
        else:
            lx.append(lx[-1]); ly.append(ly[-1])
            cx_r.append(cx_r[-1]); cy_r.append(cy_r[-1])
            rx.append(rx[-1]); ry.append(ry[-1])

    motions_arr = np.array(motions)
    cx = detrend(np.cumsum(dxs)[30:])
    cy = detrend(np.cumsum(dys)[30:])
    lx_d = detrend(np.array(lx[30:]))
    ly_d = detrend(np.array(ly[30:]))
    cx_d = detrend(np.array(cx_r[30:]))
    cy_d = detrend(np.array(cy_r[30:]))
    rx_d = detrend(np.array(rx[30:]))
    ry_d = detrend(np.array(ry[30:]))
    lr_x_stretch = detrend(np.array(rx[30:]) - np.array(lx[30:]))
    lr_y_shear = detrend(np.array(ry[30:]) - np.array(ly[30:]))

    return {
        "p90_px": round(float(np.percentile(motions_arr, 90)), 3),
        "rms_px": round(float(np.sqrt(np.mean(motions_arr ** 2))), 3),
        "x_sway_ptp_px": round(float(np.ptp(cx)), 2),
        "y_sway_ptp_px": round(float(np.ptp(cy)), 2),
        "x_sway_rms_px": round(float(np.std(cx)), 2),
        "y_sway_rms_px": round(float(np.std(cy)), 2),
        "left_x_ptp_px": round(float(np.ptp(lx_d)), 2),
        "center_x_ptp_px": round(float(np.ptp(cx_d)), 2),
        "right_x_ptp_px": round(float(np.ptp(rx_d)), 2),
        "lr_x_stretch_ptp_px": round(float(np.ptp(lr_x_stretch)), 2),
        "left_y_ptp_px": round(float(np.ptp(ly_d)), 2),
        "right_y_ptp_px": round(float(np.ptp(ry_d)), 2),
        "lr_y_shear_ptp_px": round(float(np.ptp(lr_y_shear)), 2),
    }

if __name__ == '__main__':
    weights_640 = build_sampler(640, 360)
    weights_1280 = build_sampler(1280, 720)
    report = {}
    
    for case_id, capture_id, label in CASES:
        t0 = time.time()
        bundle_dir = BUNDLE_ROOT / case_id
        bundle_info = json.loads((bundle_dir / "bundle.json").read_text(encoding="utf-8"))
        indices = [int(f["source_index"]) for f in bundle_info["frames"]]
        orientation_deg = int(bundle_info.get("orientation_deg", 180))
        fps = float(bundle_info.get("fps", 29.605))
        w_in = int(bundle_info.get("source_width", 2560))
        h_in = int(bundle_info.get("source_height", 1440))
    
        out_video = OUT_DIR / f"{case_id}_GEMINI_STABLE.mp4"
        preview_video = PREVIEW_DIR / f"{case_id}_GEMINI.mp4"
        final_mesh_path = Path(f"gemini_mesh_{case_id}.f32")
        tripod_f64_path = Path(f"visual_tripod_{case_id}.f64")
    
        pass1_mesh_path = Path(f"mesh_pass1_{case_id}.f32")
        subprocess.run([str(EXE_PATH), case_id, str(pass1_mesh_path), "NONE"], check=True)
        mesh1 = np.fromfile(pass1_mesh_path, dtype=np.float32).reshape(-1, ROWS * COLS, 2)
    
        gray_pass1 = []
        for i, frame in stream_active_frames(bundle_dir / "source.mp4", indices):
            mx, my = get_remap_coords(mesh1[i], weights_640, w_in, h_in)
            g = cv2.remap(cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY), mx, my, cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT, borderValue=0)
            gray_pass1.append(g)
    
        norm_H8 = compute_stage2a_8dof_homographies(gray_pass1, max_corr_px=26.0)
        del gray_pass1
        gc.collect()
    
        norm_H8.astype(np.float64).tofile(tripod_f64_path)
        res2 = subprocess.run([str(EXE_PATH), case_id, str(final_mesh_path), str(tripod_f64_path)], capture_output=True, text=True, check=True)
        print(f"=== {case_id} C++ Replay Pass 2 (8-DoF Homography + RS19) ===")
        print(res2.stdout.strip(), flush=True)
    
        mesh2 = np.fromfile(final_mesh_path, dtype=np.float32).reshape(-1, ROWS * COLS, 2)
    
        gray_pass2 = []
        for i, frame in stream_active_frames(bundle_dir / "source.mp4", indices):
            mx, my = get_remap_coords(mesh2[i], weights_640, w_in, h_in)
            g = cv2.remap(cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY), mx, my, cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT, borderValue=0)
            gray_pass2.append(g)
    
        mesh_final = refine_stage2b_keyframe_mesh_lock(gray_pass2, mesh2, w_in, h_in)
        del gray_pass2
        gc.collect()
        mesh_final.astype(np.float32).tofile(final_mesh_path)
    
        if out_video.exists():
            out_video.unlink()
    
        writer = imageio_ffmpeg.write_frames(
            str(out_video),
            (out_w, out_h),
            fps=fps,
            codec="libx264",
            pix_fmt_in="bgr24",
            pix_fmt_out="yuv420p",
            output_params=["-crf", "18", "-preset", "veryfast", "-movflags", "+faststart"]
        )
        writer.send(None)
    
        for i, frame in stream_active_frames(bundle_dir / "source.mp4", indices):
            mx, my = get_remap_coords(mesh_final[i], weights_1280, w_in, h_in)
            stab = cv2.remap(frame, mx, my, cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT, borderValue=0)
            if orientation_deg == 180:
                stab = cv2.rotate(stab, cv2.ROTATE_180)
            writer.send(stab)
        writer.close()
        shutil.copy2(out_video, preview_video)
        gc.collect()
    
        src_eval = evaluate_video_sway(BATCH_DIR / capture_id / "SOURCE.mp4")
        gem_eval = evaluate_video_sway(out_video)
        report[case_id] = {
            "source": src_eval,
            "gemini": gem_eval,
            "render_sec": round(time.time() - t0, 1)
        }
        print(f"[{case_id}] SOURCE: p90={src_eval['p90_px']}px, Center_X={src_eval['x_sway_ptp_px']}px, LR_Stretch={src_eval['lr_x_stretch_ptp_px']}px, LR_YShear={src_eval['lr_y_shear_ptp_px']}px")
        print(f"[{case_id}] GEMINI: p90={gem_eval['p90_px']}px, Center_X={gem_eval['x_sway_ptp_px']}px, Left_X={gem_eval['left_x_ptp_px']}px, Right_X={gem_eval['right_x_ptp_px']}px, LR_Stretch={gem_eval['lr_x_stretch_ptp_px']}px, LR_YShear={gem_eval['lr_y_shear_ptp_px']}px", flush=True)
    
    print("TRIPOD_PIPELINE_DONE")
    print(json.dumps(report, indent=2))
