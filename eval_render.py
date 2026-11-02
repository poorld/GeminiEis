import os, subprocess
from pathlib import Path
import cv2
import numpy as np

CASES = [
    ("024605", "20261017_024605_836"),
    ("024618", "20261017_024618_130"),
    ("024630", "20261017_024630_567"),
    ("024647", "20261017_024647_114")
]

BATCH_DIR = Path(r"D:\Yuy\doc\EIS\eis10\03\EIS10_4DoF_BATCH_03")
ROWS, COLS = 20, 33  # rowBands=19 means 20 rows!
POINTS = ROWS * COLS  # 660

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

def get_remap_coords(vertices, weights, source_w=2560, source_h=1440, output_w=1280, output_h=720):
    i1, i2, i3, w1, w2, w3 = weights
    vx, vy = vertices[:, 0], vertices[:, 1]
    sx = w1 * vx[i1] + w2 * vx[i2] + w3 * vx[i3]
    sy = w1 * vy[i1] + w2 * vy[i2] + w3 * vy[i3]
    mx = (sx * source_w - 0.5).astype(np.float32).reshape(output_h, output_w)
    my = (sy * source_h - 0.5).astype(np.float32).reshape(output_h, output_w)
    return mx, my

weights = build_sampler(1280, 720)
out_dir = Path("rendered_preview_fixed")
out_dir.mkdir(exist_ok=True)

metrics_report = {}

for case_id, capture_id in CASES:
    print(f"\nProcessing {case_id}...")
    case_data_dir = BATCH_DIR / capture_id
    mesh_path = Path(f"gemini_mesh_{case_id}.f32")
    
    subprocess.run(["build/gemini_eis_replay.exe", case_id, str(mesh_path)], check=True)

    src_video = case_data_dir / "SOURCE.mp4"
    raw_mesh = np.fromfile(mesh_path, dtype=np.float32).reshape(-1, POINTS, 2)
    cap = cv2.VideoCapture(str(src_video))
    fps = cap.get(cv2.CAP_PROP_FPS) or 29.7
    total_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))

    out_video = out_dir / f"{case_id}_GEMINI_STABLE.mp4"
    fourcc = cv2.VideoWriter_fourcc(*"mp4v")
    writer = cv2.VideoWriter(str(out_video), fourcc, fps, (1280, 720))

    frames_to_render = min(total_frames, len(raw_mesh))
    motions = []
    rotations = []
    prev_gray = None

    for idx in range(frames_to_render):
        ret, frame = cap.read()
        if not ret:
            break
        
        mx, my = get_remap_coords(raw_mesh[idx], weights, 2560, 1440, 1280, 720)
        stab = cv2.remap(frame, mx, my, cv2.INTER_LINEAR)
        writer.write(stab)

        gray = cv2.cvtColor(stab, cv2.COLOR_BGR2GRAY)
        if prev_gray is not None:
            flow = cv2.calcOpticalFlowFarneback(prev_gray, gray, None, 0.5, 3, 15, 3, 5, 1.2, 0)
            mag = np.sqrt(flow[..., 0]**2 + flow[..., 1]**2)
            motions.append(np.percentile(mag, 90))
        prev_gray = gray

    cap.release()
    writer.release()
    
    m_p90 = np.percentile(motions, 90) if motions else 0.0
    metrics_report[case_id] = {"motion_p90_px": m_p90}
    print(f"  -> [{case_id}] Motion p90: {m_p90:.3f} px (Official V3 reference target: ~0.52 - 1.35 px)")

print("\nAll videos successfully re-rendered with correct 20x33 mesh sampling and evaluated!")
print(json.dumps(metrics_report, indent=2))