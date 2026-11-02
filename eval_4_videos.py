# -*- coding: utf-8 -*-
import sys, json, shutil, subprocess, cv2, numpy as np, imageio_ffmpeg
from pathlib import Path
from scipy.signal import detrend

sys.path.append(r"D:/hongda/app/YuyEIS2-pc-reference-Y1/tools")
sys.path.append(r"D:/hongda/app/YuySuperEIS")
from YuyEis3.reference.pc_batch03.vision import track_pair, center_displacement

# Fix run_tripod_lock_pipeline.py to have if __name__ == '__main__':
p_script = Path("D:/hongda/app/GeminiEis/run_tripod_lock_pipeline.py")
s = p_script.read_text(encoding="utf-8")
if "if __name__" not in s:
    idx = s.index("weights_640 = build_sampler(640, 360)")
    s = s[:idx] + "if __name__ == '__main__':\n" + "\n".join("    " + line for line in s[idx:].splitlines()) + "\n"
    p_script.write_text(s, encoding="utf-8")

from run_tripod_lock_pipeline import evaluate_video_sway, CASES, BATCH_DIR, OUT_DIR, PREVIEW_DIR

ffmpeg_exe = imageio_ffmpeg.get_ffmpeg_exe()
res = {}
for cid, cap_id, label in CASES:
    vp = OUT_DIR / (cid + "_GEMINI_STABLE.mp4")
    pp = PREVIEW_DIR / (cid + "_GEMINI.mp4")
    # Ensure vp is intact; if interrupted during 024618, copy from pp
    if pp.exists():
        chk_vp = subprocess.run([ffmpeg_exe, "-v", "error", "-i", str(vp), "-f", "null", "-"], capture_output=True, text=True)
        cap_vp = cv2.VideoCapture(str(vp))
        n_vp = int(cap_vp.get(cv2.CAP_PROP_FRAME_COUNT))
        cap_vp.release()
        if len(chk_vp.stderr.strip()) > 0 or n_vp < 280:
            shutil.copy2(pp, vp)

    gem = evaluate_video_sway(vp)
    src = evaluate_video_sway(BATCH_DIR / cap_id / "SOURCE.mp4")
    cap = cv2.VideoCapture(str(vp))
    min_luma = 255.0
    frames_cnt = 0
    while True:
        ret, f = cap.read()
        if not ret:
            break
        frames_cnt += 1
        g = cv2.cvtColor(f, cv2.COLOR_BGR2GRAY)
        c_luma = min(float(g[:16, :16].mean()), float(g[:16, -16:].mean()), float(g[-16:, :16].mean()), float(g[-16:, -16:].mean()))
        min_luma = min(min_luma, c_luma)
    cap.release()
    chk = subprocess.run([ffmpeg_exe, "-v", "error", "-i", str(vp), "-f", "null", "-"], capture_output=True, text=True)
    gem["frames"] = frames_cnt
    gem["min_corner_luma"] = round(min_luma, 1)
    gem["ffmpeg_errors"] = len(chk.stderr.strip())
    res[cid] = {"source": src, "gemini": gem}
    print(f"[{cid}] frames={frames_cnt}, min_luma={min_luma:.1f}, err={gem['ffmpeg_errors']} | p90={gem['p90_px']}px, CenterX={gem['x_sway_ptp_px']}px (rms={gem['x_sway_rms_px']}px), LeftX={gem['left_x_ptp_px']}px, RightX={gem['right_x_ptp_px']}px, LR_Stretch={gem['lr_x_stretch_ptp_px']}px, LR_YShear={gem['lr_y_shear_ptp_px']}px", flush=True)

Path("D:/hongda/app/GeminiEis/tripod_8dof_report.json").write_text(json.dumps(res, indent=2), encoding="utf-8")
print("EVAL_DONE")
