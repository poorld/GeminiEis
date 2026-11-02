import os, time
from pathlib import Path
import cv2
import numpy as np

BATCH_DIR = Path(r"D:\Yuy\doc\EIS\eis10\03\EIS10_4DoF_BATCH_03")
CASES = [
    ("024605", "20261017_024605_836"),
    ("024618", "20261017_024618_130"),
    ("024630", "20261017_024630_567"),
    ("024647", "20261017_024647_114")
]

ROWS, COLS = 20, 33
out_w, out_h = 1280, 720

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

out_dir = Path("rendered_preview_fixed")
out_dir.mkdir(exist_ok=True)
fourcc = cv2.VideoWriter_fourcc(*"mp4v")

for case_id, capture_id in CASES:
    t0 = time.time()
    mesh_path = Path(f"gemini_mesh_{case_id}.f32")
    raw_mesh = np.fromfile(mesh_path, dtype=np.float32).reshape(-1, 660, 2)
    src_video = BATCH_DIR / capture_id / "SOURCE.mp4"
    cap = cv2.VideoCapture(str(src_video))
    fps = cap.get(cv2.CAP_PROP_FPS) or 29.6
    total_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))

    out_video = out_dir / f"{case_id}_GEMINI_STABLE.mp4"
    writer = cv2.VideoWriter(str(out_video), fourcc, fps, (out_w, out_h))

    frames_to_render = min(total_frames, len(raw_mesh))
    for idx in range(frames_to_render):
        ret, frame = cap.read()
        if not ret: break
        h_in, w_in = frame.shape[:2]
        vx = raw_mesh[idx, :, 0]
        vy = raw_mesh[idx, :, 1]
        sx = w1 * vx[i1] + w2 * vx[i2] + w3 * vx[i3]
        sy = w1 * vy[i1] + w2 * vy[i2] + w3 * vy[i3]
        mx = (sx * w_in - 0.5).astype(np.float32).reshape(out_h, out_w)
        my = (sy * h_in - 0.5).astype(np.float32).reshape(out_h, out_w)
        stab = cv2.remap(frame, mx, my, cv2.INTER_LINEAR)
        writer.write(stab)

    cap.release()
    writer.release()
    print(f"[{case_id}] ONE-WARP rendered {frames_to_render} frames in {time.time()-t0:.2f}s | Size: {os.path.getsize(out_video)}")

print("ONE_WARP_RENDER_SUCCESS")
