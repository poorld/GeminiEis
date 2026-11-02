import os, time
from pathlib import Path
import cv2
import numpy as np

BATCH_DIR = Path(r"D:\Yuy\doc\EIS\eis10\03\EIS10_4DoF_BATCH_03")
CASES = [
    ("024605", "20261017_024605_836", "低速来回摆动 (7.2°)"),
    ("024618", "20261017_024618_130", "中速来回摆动 (12.3°)"),
    ("024630", "20261017_024630_567", "快速来回摆动 (14.8°)"),
    ("024647", "20261017_024647_114", "慢速单边摆动 (12.4°)")
]

out_dir = Path("rendered_preview_fixed")
out_dir.mkdir(exist_ok=True)
fourcc = cv2.VideoWriter_fourcc(*"mp4v")

for case_id, capture_id, desc in CASES:
    t0 = time.time()
    mesh_path = Path(f"gemini_mesh_{case_id}.f32")
    raw_mesh = np.fromfile(mesh_path, dtype=np.float32).reshape(-1, 20, 33, 2)
    src_video = BATCH_DIR / capture_id / "SOURCE.mp4"
    cap = cv2.VideoCapture(str(src_video))
    fps = cap.get(cv2.CAP_PROP_FPS) or 29.6
    total_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))

    out_video = out_dir / f"{case_id}_GEMINI_STABLE.mp4"
    writer = cv2.VideoWriter(str(out_video), fourcc, fps, (1280, 720))

    frames_to_render = min(total_frames, len(raw_mesh))
    for idx in range(frames_to_render):
        ret, frame = cap.read()
        if not ret: break
        h_in, w_in = frame.shape[:2]
        m = raw_mesh[idx]
        mx = cv2.resize((m[..., 0] * w_in - 0.5).astype(np.float32), (1280, 720), interpolation=cv2.INTER_LINEAR)
        my = cv2.resize((m[..., 1] * h_in - 0.5).astype(np.float32), (1280, 720), interpolation=cv2.INTER_LINEAR)
        stab = cv2.remap(frame, mx, my, cv2.INTER_LINEAR)
        writer.write(stab)

    cap.release()
    writer.release()
    print(f"[{case_id}] Rendered {frames_to_render} frames in {time.time()-t0:.2f}s | Size: {os.path.getsize(out_video)}")

print("ALL_FIXED_RENDER_DONE")
