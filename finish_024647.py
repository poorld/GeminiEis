import os, json
from pathlib import Path
import cv2
import numpy as np

BATCH_DIR = Path(r"D:\Yuy\doc\EIS\eis10\03\EIS10_4DoF_BATCH_03")
ROWS, COLS = 20, 33
POINTS = ROWS * COLS

def build_sampler(out_w=1280, out_h=720):
    xs = np.arange(out_w, dtype=np.float32)
    ys = np.arange(out_h, dtype=np.float32)
    u = (xs + 0.5) / out_w
    v = (ys + 0.5) / out_h
    cf = np.minimum(u * 32.0, np.nextafter(np.float32(32.0), np.float32(0.0)))
    rf = np.minimum(v * 19.0, np.nextafter(np.float32(19.0), np.float32(0.0)))
    col = np.floor(cf).astype(np.int32)
    row = np.floor(rf).astype(np.int32)
    tx = (cf - col).astype(np.float32)
    ty = (rf - row).astype(np.float32)

    col2 = np.tile(col, out_h)
    row2 = np.repeat(row, out_w)
    tx2 = np.tile(tx, out_h)
    ty2 = np.repeat(ty, out_w)
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

def maps(vertices, weights, src_w, src_h, out_w, out_h):
    i1, i2, i3, w1, w2, w3 = weights
    vx, vy = vertices[:, 0], vertices[:, 1]
    sx = w1 * vx[i1] + w2 * vx[i2] + w3 * vx[i3]
    sy = w1 * vy[i1] + w2 * vy[i2] + w3 * vy[i3]
    mx = (sx * src_w - 0.5).astype(np.float32).reshape(out_h, out_w)
    my = (sy * src_h - 0.5).astype(np.float32).reshape(out_h, out_w)
    return mx, my

out_dir = Path("rendered_preview_fixed")
weights = build_sampler(1280, 720)

case_id = "024647"
capture_id = "20261017_024647_114"
mesh_path = Path(f"gemini_mesh_{case_id}.f32")
raw_mesh = np.fromfile(mesh_path, dtype=np.float32).reshape(-1, POINTS, 2)

manifest_file = BATCH_DIR / capture_id / "manifest.json"
manifest = json.loads(manifest_file.read_text(encoding="utf-8"))
raw_src_path = Path(manifest["source"])

cap = cv2.VideoCapture(str(raw_src_path))
src_indices = set(manifest.get("source_indices", list(range(291))))
fps = cap.get(cv2.CAP_PROP_FPS) or 29.61

out_video = out_dir / f"{case_id}_GEMINI_STABLE.mp4"
writer = cv2.VideoWriter(str(out_video), cv2.VideoWriter_fourcc(*"avc1"), fps, (1280, 720))
if not writer.isOpened():
    writer = cv2.VideoWriter(str(out_video), cv2.VideoWriter_fourcc(*"mp4v"), fps, (1280, 720))

frame_idx = 0
cur_mesh_idx = 0
while cur_mesh_idx < len(raw_mesh):
    ret, frame = cap.read()
    if not ret:
        break
    if frame_idx in src_indices:
        mx, my = maps(raw_mesh[cur_mesh_idx], weights, frame.shape[1], frame.shape[0], 1280, 720)
        stab = cv2.remap(frame, mx, my, cv2.INTER_LINEAR, borderMode=cv2.BORDER_REFLECT)
        writer.write(stab)
        cur_mesh_idx += 1
    frame_idx += 1

cap.release()
writer.release()
print(f"Done 024647, size: {out_video.stat().st_size} bytes")