import os, subprocess, webbrowser
from pathlib import Path
import cv2
import numpy as np

CASES = [
    ("024605", "20261017_024605_836", "低速来回摆动 (7.2°)"),
    ("024618", "20261017_024618_130", "中速来回摆动 (12.3°)"),
    ("024630", "20261017_024630_567", "快速来回摆动 (14.8°)"),
    ("024647", "20261017_024647_114", "慢速单边摆动 (12.4°)")
]

BATCH_DIR = Path(r"D:\Yuy\doc\EIS\eis10\03\EIS10_4DoF_BATCH_03")
ROWS, COLS = 19, 33
POINTS = 660

def build_sampler(output_w=1280, output_h=720):
    xs = np.arange(output_w, dtype=np.float32)
    ys = np.arange(output_h, dtype=np.float32)
    u = (xs + 0.5) / output_w
    v = (ys + 0.5) / output_h
    cf = np.minimum(u * 32.0, np.nextafter(np.float32(32.0), np.float32(0.0)))
    rf = np.minimum(v * 18.0, np.nextafter(np.float32(18.0), np.float32(0.0)))
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
out_dir = Path("rendered_preview")
out_dir.mkdir(exist_ok=True)

for case_id, capture_id, label in CASES:
    print(f"Replaying {case_id} to get correct mesh...")
    mesh_path = Path(f"gemini_mesh_{case_id}.f32")
    subprocess.run(["build/gemini_eis_replay.exe", case_id, str(mesh_path)], check=True)

    src_video = BATCH_DIR / capture_id / "SOURCE.mp4"
    raw_mesh = np.fromfile(mesh_path, dtype=np.float32).reshape(-1, POINTS, 2)
    cap = cv2.VideoCapture(str(src_video))
    fps = cap.get(cv2.CAP_PROP_FPS) or 29.7
    total_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))

    out_video = out_dir / f"{case_id}_GEMINI_NEW.mp4"
    fourcc = cv2.VideoWriter_fourcc(*"mp4v")
    writer = cv2.VideoWriter(str(out_video), fourcc, fps, (1280, 720))

    frames_to_render = min(total_frames, len(raw_mesh))
    for idx in range(frames_to_render):
        ret, frame = cap.read()
        if not ret:
            break
        mx, my = get_remap_coords(raw_mesh[idx], weights)
        stab = cv2.remap(frame, mx, my, cv2.INTER_LINEAR)
        writer.write(stab)

    cap.release()
    writer.release()
    print(f"  Fixed & Generated: {out_video.resolve()}")

options_html = "".join([f'<option value="{c[0]}">{c[0]} - {c[2]}</option>' for c in CASES])
data_js = "{" + ", ".join([f'"{c[0]}": {{"src": "'.replace("'", '"') + str(BATCH_DIR / c[1] / "SOURCE.mp4").replace("\\", "/") + f'", "gem": "{c[0]}_GEMINI_NEW.mp4"}}' for c in CASES]) + "}"

html_path = out_dir / "gemini_compare_viewer.html"
html_content = f"""<!doctype html>
<html>
<head>
  <meta charset="utf-8">
  <title>GeminiEis 动态包络防抖效果对比</title>
  <style>
    body {{ background: #0b1120; color: #f1f5f9; font-family: system-ui, sans-serif; margin: 24px; }}
    h1 {{ font-size: 20px; margin-bottom: 8px; color: #38bdf8; }}
    .controls {{ margin: 16px 0; display: flex; gap: 12px; align-items: center; }}
    select, button {{ background: #1e293b; color: white; border: 1px solid #334155; padding: 8px 16px; border-radius: 6px; font-size: 14px; cursor: pointer; }}
    .grid {{ display: grid; grid-template-columns: 1fr 1fr; gap: 16px; }}
    .card {{ background: #1e293b; padding: 12px; border-radius: 8px; border: 1px solid #334155; }}
    .card h3 {{ margin: 0 0 8px 0; font-size: 14px; color: #94a3b8; }}
    video {{ width: 100%; height: auto; border-radius: 4px; display: block; }}
  </style>
</head>
<body>
  <h1>GeminiEis 最新动态包络防抖对比评估</h1>
  <div class="controls">
    <label>测试场景：</label>
    <select id="caseSel" onchange="changeCase()">
      {options_html}
    </select>
    <button onclick="togglePlay()">播放 / 暂停</button>
    <span id="framePos" style="font-size: 13px; color: #94a3b8;"></span>
  </div>
  <div class="grid">
    <div class="card">
      <h3>原始输入 (SOURCE.mp4)</h3>
      <video id="vSrc" loop muted controls></video>
    </div>
    <div class="card">
      <h3>GeminiEis 最新防抖渲染 (GEMINI_NEW.mp4)</h3>
      <video id="vGem" loop muted controls></video>
    </div>
  </div>
  <script>
    const data = {data_js};
    const vSrc = document.getElementById('vSrc');
    const vGem = document.getElementById('vGem');
    function changeCase() {{
      const k = document.getElementById('caseSel').value;
      vSrc.src = data[k].src;
      vGem.src = data[k].gem;
      vSrc.load(); vGem.load();
    }}
    function togglePlay() {{
      if (vSrc.paused) {{ vSrc.play(); vGem.play(); }} else {{ vSrc.pause(); vGem.pause(); }}
    }}
    vSrc.ontimeupdate = () => {{
      if (Math.abs(vSrc.currentTime - vGem.currentTime) > 0.04) vGem.currentTime = vSrc.currentTime;
      document.getElementById('framePos').textContent = "时间: " + vSrc.currentTime.toFixed(2) + "s / 9.80s";
    }};
    changeCase();
  </script>
</body>
</html>"""
html_path.write_text(html_content, encoding="utf-8")
print(f"All rendered successfully! Opening browser: {html_path.resolve()}")
webbrowser.open(f"file:///{html_path.resolve()}")