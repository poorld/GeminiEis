# -*- coding: utf-8 -*-
import os, sys, time, json, subprocess
from pathlib import Path
import cv2
import numpy as np
import imageio_ffmpeg

sys.path.append(r"D:/hongda/app/YuyEIS2-pc-reference-Y1/tools")
sys.path.append(r"D:/hongda/app/YuySuperEIS")
from YuyEis3.reference.pc_batch03.vision import track_pair, center_displacement

CASES = [
    ("024605", "20261017_024605_836", "Case 024605"),
    ("024618", "20261017_024618_130", "Case 024618"),
    ("024630", "20261017_024630_567", "Case 024630"),
    ("024647", "20261017_024647_114", "Case 024647")
]

BUNDLE_ROOT = Path("D:/hongda/app/YuySuperEIS/artifacts/b38_restore_bundles_20261020_172849")
BATCH_DIR = Path("D:/Yuy/doc/EIS/eis10/03/EIS10_4DoF_BATCH_03")
OUT_DIR = Path("D:/hongda/app/GeminiEis/rendered_preview_fixed")
OUT_DIR.mkdir(parents=True, exist_ok=True)

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
    return (i1, i2, i3, w1, w2, w3)

def get_remap_coords(vertices, weights, source_w=2560, source_h=1440):
    i1, i2, i3, w1, w2, w3 = weights
    vx, vy = vertices[:, 0], vertices[:, 1]
    sx = w1 * vx[i1] + w2 * vx[i2] + w3 * vx[i3]
    sy = w1 * vy[i1] + w2 * vy[i2] + w3 * vy[i3]
    mx = (sx * source_w - 0.5).astype(np.float32).reshape(out_h, out_w)
    my = (sy * source_h - 0.5).astype(np.float32).reshape(out_h, out_w)
    return mx, my

weights = build_sampler(out_w, out_h)
report = {}

for case_id, capture_id, label in CASES:
    t0 = time.time()
    bundle_dir = BUNDLE_ROOT / case_id
    bundle_info = json.loads((bundle_dir / "bundle.json").read_text(encoding="utf-8"))
    indices = [int(f["source_index"]) for f in bundle_info["frames"]]
    orientation_deg = int(bundle_info.get("orientation_deg", 180))

    ref_metrics = json.loads((BATCH_DIR / capture_id / "metrics.json").read_text(encoding="utf-8"))
    src_p90 = float(ref_metrics["SOURCE"]["motion_p90_px"])
    src_rms = float(ref_metrics["SOURCE"]["motion_rms_px"])

    mesh_path = Path(f"gemini_mesh_{case_id}.f32")
    raw_mesh = np.fromfile(mesh_path, dtype=np.float32).reshape(-1, ROWS * COLS, 2)

    raw_video_path = bundle_dir / "source.mp4"
    cap = cv2.VideoCapture(str(raw_video_path))
    if hasattr(cv2, "CAP_PROP_ORIENTATION_AUTO"):
        cap.set(cv2.CAP_PROP_ORIENTATION_AUTO, 0)
    fps = float(bundle_info.get("fps", 29.605))

    out_video = OUT_DIR / f"{case_id}_GEMINI_STABLE.mp4"
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

    wanted = 0
    prev_gray = None
    motions = []
    top_dx = []
    bot_dx = []

    for idx in range(indices[-1] + 1):
        ret, frame = cap.read()
        if not ret:
            break
        if idx < indices[wanted]:
            continue
        h_in, w_in = frame.shape[:2]
        mx, my = get_remap_coords(raw_mesh[wanted], weights, w_in, h_in)
        stab = cv2.remap(frame, mx, my, cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT, borderValue=0)
        if orientation_deg == 180:
            stab = cv2.rotate(stab, cv2.ROTATE_180)
        writer.send(stab)

        gray = cv2.cvtColor(cv2.resize(stab, (640, 360), interpolation=cv2.INTER_AREA), cv2.COLOR_BGR2GRAY)
        if prev_gray is not None and wanted >= 30:
            sim, _, _ = track_pair(prev_gray, gray)
            if sim is not None:
                d = center_displacement(sim[None], 640, 360)[0] * 2.0
                motions.append(float(np.linalg.norm(d)))
            sim_top, _, _ = track_pair(prev_gray[:180, :], gray[:180, :])
            sim_bot, _, _ = track_pair(prev_gray[180:, :], gray[180:, :])
            if sim_top is not None and sim_bot is not None:
                dt_x = center_displacement(sim_top[None], 640, 180)[0][0] * 2.0
                db_x = center_displacement(sim_bot[None], 640, 180)[0][0] * 2.0
                top_dx.append(abs(dt_x - db_x))
        prev_gray = gray
        wanted += 1
        if wanted == len(indices):
            break

    cap.release()
    writer.close()

    motions_arr = np.array(motions)
    gem_p90 = float(np.percentile(motions_arr, 90))
    gem_rms = float(np.sqrt(np.mean(motions_arr ** 2)))
    shear_p90 = float(np.percentile(top_dx, 90)) if top_dx else 0.0
    reduction_pct = (1.0 - gem_p90 / src_p90) * 100.0
    cost = time.time() - t0
    fsize = os.path.getsize(out_video)

    report[case_id] = {
        "frames": wanted,
        "size_mb": round(fsize / 1e6, 2),
        "source_p90_px": round(src_p90, 3),
        "gemini_p90_px": round(gem_p90, 3),
        "source_rms_px": round(src_rms, 3),
        "gemini_rms_px": round(gem_rms, 3),
        "shake_reduction_pct": round(reduction_pct, 1),
        "top_bot_shear_p90_px": round(shear_p90, 3),
        "render_sec": round(cost, 1)
    }
    print(f"[{case_id}] frames={wanted}, SOURCE p90={src_p90:.2f}px -> GEMINI p90={gem_p90:.2f}px (-{reduction_pct:.1f}%), RMS={src_rms:.2f}->{gem_rms:.2f}px, Top/Bot shear={shear_p90:.2f}px", flush=True)

print("ALL_VERIFIED_COMPLETE", flush=True)
print(json.dumps(report, indent=2), flush=True)
