import os
import subprocess
import webbrowser
from pathlib import Path
import cv2
import numpy as np

CASES = [
    ("024605", "低速来回摆动 (7.2°)"),
    ("024618", "中速来回摆动 (12.3°)"),
    ("024630", "快速来回摆动 (14.8°)"),
    ("024647", "慢速单边摆动 (12.4°)"),
]

BUNDLE_ROOT = Path(r"D:\hongda\app\YuySuperEIS\artifacts\b38_restore_bundles_20261020_172849")
OUT_DIR = Path(r"D:\hongda\app\GeminiEis\rendered_preview")
OUT_DIR.mkdir(parents=True, exist_ok=True)

ROWS, COLS = 20, 33

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

    i1 = np.empty_like(tl)
    i2 = np.empty_like(tl)
    i3 = np.empty_like(tl)
    w1 = np.empty_like(tx2)
    w2 = np.empty_like(tx2)
    w3 = np.empty_like(tx2)

    i1[lower] = tl[lower]; i2[lower] = bl[lower]; i3[lower] = tr[lower]
    w1[lower] = (1.0 - tx2[lower]) - ty2[lower]
    w2[lower] = ty2[lower]
    w3[lower] = tx2[lower]

    upper = ~lower
    i1[upper] = tr[upper]; i2[upper] = bl[upper]; i3[upper] = br[upper]
    w1[upper] = 1.0 - ty2[upper]
    w2[upper] = 1.0 - tx2[upper]
    w3[upper] = (tx2[upper] + ty2[upper]) - 1.0
    return i1, i2, i3, w1, w2, w3

def get_remap_coords(vertices, weights, source_w=2560, source_h=1440, output_w=1280, output_h=720):
    i1, i2, i3, w1, w2, w3 = weights
    vx, vy = vertices[:, 0], vertices[:, 1]
    sx = w1 * vx[i1] + w2 * vx[i2] + w3 * vx[i3]
    sy = w1 * vy[i1] + w2 * vy[i2] + w3 * vy[i3]
    mx = (sx * source_w - 0.5).astype(np.float32).reshape(output_h, output_w)
    my = (sy * source_h - 0.5).astype(np.float32).reshape(output_h, output_w)
    return mx, my

def main():
    weights = build_sampler(1280, 720)
    
    for case_id, label in CASES:
        case_dir = BUNDLE_ROOT / case_id
        src_video = case_dir / "source.mp4"
        mesh_file = case_dir / "gemini_mesh.f32"
        
        if not src_video.exists():
            print(f"Source video not found: {src_video}")
            continue
        if not mesh_file.exists():
            print(f"Mesh file not found: {mesh_file}")
            continue
            
        print(f"Rendering {case_id} ({label})...")
        raw_mesh = np.fromfile(mesh_file, dtype=np.float32).reshape(-1, ROWS * COLS, 2)
        
        cap = cv2.VideoCapture(str(src_video))
        fps = cap.get(cv2.CAP_PROP_FPS) or 29.7
        total_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
        
        out_gemini = OUT_DIR / f"{case_id}_GEMINI.mp4"
        out_compare = OUT_DIR / f"{case_id}_COMPARE_SOURCE_GEMINI.mp4"
        
        fourcc = cv2.VideoWriter_fourcc(*"avc1")
        writer_gemini = cv2.VideoWriter(str(out_gemini), fourcc, fps, (1280, 720))
        if not writer_gemini.isOpened():
            fourcc = cv2.VideoWriter_fourcc(*"mp4v")
            writer_gemini = cv2.VideoWriter(str(out_gemini), fourcc, fps, (1280, 720))
        
        writer_compare = cv2.VideoWriter(str(out_compare), fourcc, fps, (2560, 720))
        
        frames_to_render = min(total_frames, len(raw_mesh))
        for idx in range(frames_to_render):
            ret, frame = cap.read()
            if not ret:
                break
                
            mx, my = get_remap_coords(raw_mesh[idx], weights)
            stab = cv2.remap(frame, mx, my, cv2.INTER_LINEAR)
            
            # Simple center-crop for source video to match 1280x720 aspect
            src_resized = cv2.resize(frame, (1280, 720))
            
            writer_gemini.write(stab)
            
            # Left: SOURCE, Right: GEMINI
            comp = np.hstack([src_resized, stab])
            writer_compare.write(comp)
            
        cap.release()
        writer_gemini.release()
        writer_compare.release()
        print(f"  -> Generated {out_gemini.name} & {out_compare.name}")

    # Generate html viewer
    html_path = OUT_DIR / "gemini_playback_comparison.html"
    html_content = """<!doctype html>
<html lang="zh-CN">
<head>
  <meta charset="utf-8">
  <title>GeminiEis 实时混合防抖渲染效果比对</title>
  <style>
    body { background: #0b1120; color: #f1f5f9; font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, sans-serif; margin: 0; padding: 24px; }
    h1 { font-size: 20px; font-weight: 600; color: #38bdf8; margin: 0 0 16px 0; }
    .toolbar { display: flex; gap: 12px; align-items: center; margin-bottom: 20px; background: #1e293b; padding: 12px 16px; border-radius: 8px; border: 1px solid #334155; }
    select, button { background: #0f172a; color: #f8fafc; border: 1px solid #475569; padding: 8px 16px; border-radius: 6px; font-size: 14px; cursor: pointer; }
    button:hover { background: #1e293b; border-color: #38bdf8; }
    .video-container { display: grid; grid-template-columns: 1fr 1fr; gap: 20px; }
    .card { background: #1e293b; padding: 16px; border-radius: 10px; border: 1px solid #334155; box-shadow: 0 4px 6px -1px rgba(0, 0, 0, 0.3); }
    .card-header { display: flex; justify-content: space-between; align-items: center; margin-bottom: 12px; }
    .card-title { font-size: 15px; font-weight: 500; color: #94a3b8; margin: 0; }
    .card-badge { font-size: 12px; padding: 2px 8px; border-radius: 4px; background: #334155; color: #38bdf8; }
    video { width: 100%; height: auto; border-radius: 6px; display: block; background: #000; }
    .timeline-info { font-size: 13px; color: #94a3b8; font-variant-numeric: tabular-nums; }
  </style>
</head>
<body>
  <h1>GeminiEis 实时因果混合防抖 — 实测录像渲染效果评测</h1>
  <div class="toolbar">
    <label for="caseSelect">切换测试序列：</label>
    <select id="caseSelect" onchange="changeClip()">
      <option value="024605">024605 — 低速来回摆动 (7.2°)</option>
      <option value="024618">024618 — 中速来回摆动 (12.3°)</option>
      <option value="024630">024630 — 快速来回摆动 (14.8°)</option>
      <option value="024647">024647 — 慢速单边摆动 (12.4°)</option>
    </select>
    <button onclick="togglePlay()" id="btnPlay">同步播放</button>
    <button onclick="changeSpeed(0.5)">0.5x 慢速</button>
    <button onclick="changeSpeed(1.0)">1.0x 正常</button>
    <span class="timeline-info" id="timeLabel">0.00s / 9.80s</span>
  </div>

  <div class="video-container">
    <div class="card">
      <div class="card-header">
        <h3 class="card-title">原始物理相机画面 (SOURCE)</h3>
        <span class="card-badge">无防抖输入</span>
      </div>
      <video id="vSrc" loop muted playsinline></video>
    </div>
    <div class="card">
      <div class="card-header">
        <h3 class="card-title">GeminiEis 实时稳定输出 (GEMINI)</h3>
        <span class="card-badge" style="background:#0369a1;color:#e0f2fe;">动态包络呼吸跟随</span>
      </div>
      <video id="vGem" loop muted playsinline></video>
    </div>
  </div>

  <script>
    const clips = {
      "024605": { src: "024605_COMPARE_SOURCE_GEMINI.mp4", gem: "024605_GEMINI.mp4" },
      "024618": { src: "024618_COMPARE_SOURCE_GEMINI.mp4", gem: "024618_GEMINI.mp4" },
      "024630": { src: "024630_COMPARE_SOURCE_GEMINI.mp4", gem: "024630_GEMINI.mp4" },
      "024647": { src: "024647_COMPARE_SOURCE_GEMINI.mp4", gem: "024647_GEMINI.mp4" },
    };

    const vSrc = document.getElementById("vSrc");
    const vGem = document.getElementById("vGem");
    const btnPlay = document.getElementById("btnPlay");
    const timeLabel = document.getElementById("timeLabel");

    function changeClip() {
      const id = document.getElementById("caseSelect").value;
      const base = "D:/hongda/app/YuySuperEIS/artifacts/b38_restore_bundles_20261020_172849/" + id + "/source.mp4";
      vSrc.src = base;
      vGem.src = clips[id].gem;
      vSrc.load();
      vGem.load();
      btnPlay.textContent = "同步播放";
    }

    function togglePlay() {
      if (vSrc.paused) {
        vSrc.currentTime = vGem.currentTime = Math.min(vSrc.currentTime, vGem.currentTime);
        vSrc.play();
        vGem.play();
        btnPlay.textContent = "暂停";
      } else {
        vSrc.pause();
        vGem.pause();
        btnPlay.textContent = "同步播放";
      }
    }

    function changeSpeed(rate) {
      vSrc.playbackRate = rate;
      vGem.playbackRate = rate;
    }

    vSrc.ontimeupdate = () => {
      if (Math.abs(vSrc.currentTime - vGem.currentTime) > 0.03) {
        vGem.currentTime = vSrc.currentTime;
      }
      timeLabel.textContent = vSrc.currentTime.toFixed(2) + "s / 9.80s";
    };

    changeClip();
  </script>
</body>
</html>
"""
    html_path.write_text(html_content, encoding="utf-8")
    print(f"HTML Viewer written to: {html_path}")

if __name__ == "__main__":
    main()