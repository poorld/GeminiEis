#!/usr/bin/env python3
"""Gemini Tripod V2, offline-pipeline input / causal per-frame Scale governor.

This module NEVER reads future values to choose frame i's Stage2 scale.
It consumes v1 visual_tripod and oracle matrices which were generated using
future frames, so the FULL PIPELINE IS STILL OFFLINE / NOT REALTIME ANDROID.

No modifications to src/, baseline meshes or original H264. Fixed-zoom mode
uses only the isolated replay_fixedzoom_114.exe from the V1 2x2 experiment.
"""
from __future__ import annotations
import argparse, json, math, re, subprocess, time
from pathlib import Path
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
BUNDLES = Path("D:/hongda/app/YuySuperEIS/artifacts/b38_restore_bundles_20261020_172849")
OUT = ROOT / "reports" / "gemini_tripod_v2"
# Matrix records use normalized coordinates. Perspective is not activated.
H_W_RATIO = 1440 / 2560
PIVOT = np.array([0.5, 0.5, 1.0], dtype=np.float64)
EXE_DYN = ROOT / "build" / "gemini_eis_replay.exe"
EXE_FIXED = ROOT / "reports" / "gemini_scale_ab" / "024630" / "replay_fixedzoom_114.exe"
EXE_FLOOR = {f"floor{v}": ROOT / "reports" / "gemini_tripod_v2" /
             "bin" / f"replay_zoom_floor_{v}.exe" for v in (108, 110, 112)}


def scale(mat: np.ndarray) -> np.ndarray:
    return np.hypot(mat[:, 0, 0], mat[:, 1, 0] * H_W_RATIO)


def causal_slew_logscale(raw_s: np.ndarray, limit_log_per_frame: float) -> np.ndarray:
    """Independent per-frame state, no all-sequence filters or lookahead."""
    if not (0.00001 < limit_log_per_frame < .08):
        raise ValueError("Unreasonable log-scale frame limit")
    states = np.empty(len(raw_s), dtype=np.float64)
    previous = float(np.log(raw_s[0]))
    for i, v in enumerate(raw_s):
        target = float(np.log(v))
        step = np.clip(target - previous,
                       -limit_log_per_frame, limit_log_per_frame)
        previous += float(step)
        states[i] = math.exp(previous)
    return states


def causal_outlier_logscale(raw_s: np.ndarray, rate: float, jump: float) -> np.ndarray:
    """Follow golden Stage2 exactly until a large one-frame scale jump;
    then rate limit ONLY its recovery. Strictly past+current evidence.
    """
    src=np.log(raw_s)
    result=np.empty(len(src),dtype=np.float64)
    prev=float(src[0]);prev_input=float(src[0]);in_safety=False
    for i,raw in enumerate(src):
        if abs(float(raw)-prev_input)>jump:
            in_safety=True
        if in_safety:
            prev+=float(np.clip(raw-prev,-rate,rate))
            if abs(raw-prev)<rate*1.02:
                in_safety=False
        else:
            prev=float(raw)
        result[i]=math.exp(prev)
        prev_input=float(raw)
    return result

def generate(case: str, rate: float, zoom_mode: str, adaptive_jump: float=0.0) -> dict:
    if case not in ("024605", "024618", "024630", "024647"):
        raise ValueError(case)
    if zoom_mode not in ("fixed114", "dynamic", *EXE_FLOOR):
        raise ValueError(zoom_mode)
    desc = (f"{zoom_mode}_outlierj{adaptive_jump:.3f}_slew{rate:.4f}" if adaptive_jump else
            f"{zoom_mode}_slew{rate:.4f}").replace(".", "p")
    directory = OUT / case / desc
    directory.mkdir(parents=True, exist_ok=True)
    oracle = np.fromfile(BUNDLES / case / "oracle_matrices.f64",
                         dtype=np.float64).reshape(-1, 3, 3)
    combined = np.fromfile(ROOT / f"visual_tripod_{case}.f64",
                           dtype=np.float64).reshape(-1, 3, 3)
    original_mesh = np.fromfile(ROOT / f"gemini_mesh_{case}.f32",
                                dtype=np.float32).reshape(-1, 20, 33, 2)
    if len(oracle) != len(combined) or len(combined) != len(original_mesh):
        raise RuntimeError("Frame mismatch, oracle/matrices/golden")
    extra = combined @ np.linalg.inv(oracle)
    if np.max(np.abs(extra[:, 2] - [0, 0, 1])) > 2e-4:
        raise RuntimeError("Stage2 extra not an affine similarity")
    raw_scales = scale(extra)
    governed = (causal_outlier_logscale(raw_scales,rate,adaptive_jump)
                if adaptive_jump else causal_slew_logscale(raw_scales,rate))
    E = extra.copy()
    # Keep the Stage2 residual rotation, translation & transformed optical
    # center exactly equal to V1 for each frame, but only bound log-scale.
    E[:, :2, :2] *= (governed / raw_scales)[:, None, None]
    anchor = np.einsum("nij,j->ni", oracle, PIVOT)
    anchor_xy = anchor[:, :2] / anchor[:, 2, None]
    E[:, :2, 2] += np.einsum("nij,nj->ni",
                            extra[:, :2, :2] - E[:, :2, :2], anchor_xy)
    new_visual = E @ oracle
    pivot_original = np.einsum("nij,j->ni", combined, PIVOT)
    pivot_new = np.einsum("nij,j->ni", new_visual, PIVOT)
    center_error = np.max(np.abs(pivot_original[:, :2] / pivot_original[:, 2, None]
                                 - pivot_new[:, :2] / pivot_new[:, 2, None]))
    if center_error > 1e-8:
        raise RuntimeError(f"Changed virtual image center by {center_error}")
    matfile = directory / "visual_governed.f64"
    new_visual.tofile(matfile)
    meshfile = directory / "mesh_governed.f32"
    replay = (EXE_FIXED if zoom_mode == "fixed114" else
              EXE_DYN if zoom_mode == "dynamic" else EXE_FLOOR[zoom_mode])
    if not replay.exists():
        raise RuntimeError(f"Missing isolated replay {replay}")
    t0 = time.perf_counter()
    run = subprocess.run([str(replay), case, str(meshfile), str(matfile)],
                         cwd=ROOT, capture_output=True, text=True, timeout=95)
    if run.returncode or "GEMINI_EIS_REPLAY_GATE_PASS" not in run.stdout:
        raise RuntimeError(f"Native RS19 FAIL: {run.stdout} {run.stderr}")
    newmesh = np.fromfile(meshfile, dtype=np.float32).reshape(-1, 20, 33, 2)
    assert len(newmesh) == len(original_mesh)
    span = ((newmesh[:, 9, 32, 0] + newmesh[:, 10, 32, 0]
             - newmesh[:, 9, 0, 0] - newmesh[:, 10, 0, 0]) / 2 * 2560)
    original_span = ((original_mesh[:, 9, 32, 0] + original_mesh[:, 10, 32, 0]
                      - original_mesh[:, 9, 0, 0] - original_mesh[:, 10, 0, 0]) / 2 * 2560)
    invalid_frames = int(np.any((newmesh < 0) | (newmesh > 1), axis=(1, 2, 3)).sum())
    if invalid_frames:
        raise RuntimeError("New candidate went outside sensor UV")
    result = {
        "case": case, "variant": desc, "rate_log_scale_per_frame": rate,
        "causal_outlier_trigger_logscale_step": adaptive_jump,
        "zoom_mode": zoom_mode, "visual_Stage2_logscale_is_causal": True,
        "oracle_and_input_camera_quaternion_causal": False,
        "source_oracle_provenance": "ORIGINAL_V1_OFFLINE_GAUSSIAN_LOOKAHEAD",
        "stage2_scale_range_raw": [float(raw_scales.min()), float(raw_scales.max())],
        "stage2_scale_range_new": [float(governed.min()), float(governed.max())],
        "max_abs_scale_step_pct": float(np.max(abs(np.diff(governed))) * 100),
        "original_max_abs_scale_step_pct": float(np.max(abs(np.diff(raw_scales))) * 100),
        "optical_center_mapping_equal_max": float(center_error),
        "rs19_zero_uv_bad_frames": invalid_frames,
        "mesh_midline_span_p90_step_rawpx": float(np.percentile(abs(np.diff(span)), 90)),
        "mesh_midline_span_max_step_rawpx": float(np.max(abs(np.diff(span)))),
        "mesh_event67_span_step_rawpx": float(span[67]-span[66]),
        "baseline_mesh_event67_span_step_rawpx": float(original_span[67]-original_span[66]),
        "output_video_pixels_gate": "NOT_TESTED_MESH_ONLY",
        "CXX_RS19_replay_output": run.stdout.strip(),
        "replay_elapsed_sec": round(time.perf_counter() - t0, 2),
        "visual_matrices": str(matfile), "mesh": str(meshfile),
    }
    (directory / "manifest.json").write_text(
        json.dumps(result, ensure_ascii=False, indent=2), encoding="utf-8")
    print("V2_MESH_CANDIDATE", json.dumps({k: v for k, v in result.items()
        if k in ("case","variant","max_abs_scale_step_pct",
                 "mesh_midline_span_p90_step_rawpx",
                 "mesh_midline_span_max_step_rawpx",
                 "mesh_event67_span_step_rawpx",
                 "rs19_zero_uv_bad_frames")}, ensure_ascii=False), flush=True)
    return result


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--cases", nargs="+", default=["024630"])
    ap.add_argument("--rates", nargs="+", type=float, default=[.0015,.003,.006,.012])
    ap.add_argument("--zooms", nargs="+", choices=["fixed114","dynamic",*EXE_FLOOR],
                    default=["fixed114", "dynamic"])
    ap.add_argument("--outlier-jump",type=float,default=0.0)
    o = ap.parse_args()
    manifests = [generate(k, r, z, o.outlier_jump) for k in o.cases
                 for r in o.rates for z in o.zooms]
    (OUT / "candidate_manifest.json").write_text(
        json.dumps({"candidates": manifests,
                    "actual_pixel_result": "PENDING",
                    "offlineV1_future_dependency": True},
                   ensure_ascii=False, indent=2), encoding="utf-8")

if __name__ == "__main__":
    main()
