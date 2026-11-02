# -*- coding: utf-8 -*-
import os, time, json
from pathlib import Path
import cv2
import numpy as np
import imageio_ffmpeg

CASES = [
    ("024605", "20261017_024605_836", "Case 024605"),
    ("024618", "20261017_024618_130", "Case 024618"),
    ("024630", "20261017_024630_567", "Case 024630"),
    ("024647", "20261017_024647_114", "Case 024647")
]

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
    mesh_path = Path(f"gemini_mesh_{case_id}.f32")
    raw_mesh = np.fromfile(mesh_path, dtype=np.float32).reshape(-1, ROWS * COLS, 2)
    src_video = BATCH_DIR / capture_id / "SOURCE.mp4"
    cap = cv2.VideoCapture(str(src_video))
    fps = cap.get(cv2.CAP_PROP_FPS) or 29.605
    total_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    frames_to_render = min(total_frames, len(raw_mesh))

    out_video = OUT_DIR / f"{case_id}_GEMINI_STABLE.mp4"
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

    for idx in range(frames_to_render):
        ret, frame = cap.read()
        if not ret: break
        h_in, w_in = frame.shape[:2]
        mx, my = get_remap_coords(raw_mesh[idx], weights, w_in, h_in)
        stab = cv2.remap(frame, mx, my, cv2.INTER_LINEAR)
        writer.send(stab)

    cap.release()
    writer.close()
    cost = time.time() - t0
    fsize = os.path.getsize(out_video)
    report[case_id] = {
        "frames": frames_to_render,
        "size_bytes": fsize,
        "render_sec": round(cost, 1)
    }
    print(f"[{case_id}] {label} -> {frames_to_render} frames, {cost:.1f}s, {fsize/1e6:.2f}MB, H264 Playable", flush=True)

print("ALL_H264_RENDER_COMPLETE", flush=True)
print(json.dumps(report, indent=2), flush=True)
