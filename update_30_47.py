# -*- coding: utf-8 -*-
import os, sys, time, json, subprocess, gc, shutil
from pathlib import Path
import cv2
import numpy as np
import imageio_ffmpeg

# 1. Update gemini_eis_core.cpp so initial settle covers frames 0..32 with tau=0.35s,
# centering qAnchor_ in the active oscillation range of 024630 and 024647
cpp_path = Path("D:/hongda/app/GeminiEis/src/core/gemini_eis_core.cpp")
cpp = cpp_path.read_text(encoding="utf-8")
cpp = cpp.replace("if (frameCount_ <= 25) {\n            const double alphaInit = 1.0 - std::exp(-dtS / 0.65);",
                  "if (frameCount_ <= 32) {\n            const double alphaInit = 1.0 - std::exp(-dtS / 0.32);")
cpp = cpp.replace("const double minGyroMarginPx = 32.0;", "const double minGyroMarginPx = 40.0;")
cpp_path.write_text(cpp, encoding="utf-8")

subprocess.run(["cmake", "--build", "build", "--config", "Release"], check=True)

# 2. Update run_tripod_lock_pipeline.py with adaptive sharp keyframe selection and 16px vignette guard
py_path = Path("D:/hongda/app/GeminiEis/run_tripod_lock_pipeline.py")
py = py_path.read_text(encoding="utf-8")

old_kf_block = """    k0 = min(40, n - 1)
    kf_indices = [idx for idx in [40, 110, 185] if idx < n]
    if not kf_indices:
        kf_indices = [0]"""

new_kf_block = """    vel = np.zeros(n, dtype=np.float64)
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
    k0 = min(kf_indices, key=lambda k: score[k])"""

py = py.replace(old_kf_block, new_kf_block)
py = py.replace("min_u = 2.0 / w_in", "min_u = 16.0 / w_in")
py = py.replace("min_v = 2.0 / h_in", "min_v = 16.0 / h_in")
py_path.write_text(py, encoding="utf-8")

# 3. Run pipeline for 024630 and 024647
from run_tripod_lock_pipeline import (
    BUNDLE_ROOT, BATCH_DIR, OUT_DIR, PREVIEW_DIR, EXE_PATH, ROWS, COLS, out_w, out_h,
    build_sampler, get_remap_coords, stream_active_frames,
    compute_stage2a_8dof_homographies, refine_stage2b_keyframe_mesh_lock, evaluate_video_sway
)

weights_640 = build_sampler(640, 360)
weights_1280 = build_sampler(1280, 720)

TARGET_CASES = [
    ("024630", "20261017_024630_567", "Case 024630 (Fast sway 14.8 deg)"),
    ("024647", "20261017_024647_114", "Case 024647 (One-sided pan 12.4 deg)")
]

for case_id, capture_id, label in TARGET_CASES:
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
    print(f"=== {case_id} C++ Replay Pass 2 ===", flush=True)
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

print("FINISHED_30_47", flush=True)
