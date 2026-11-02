#!/usr/bin/env python3
"""C2 causal SHADOW candidate real raw-source H264 render + output-pixel gate.

This renderer only CONSUMES already generated causal C2 mesh files. It never
updates qVirtual/Visual/Zoom and cannot expose V1 future to C2. Identical
source.mp4 SourceID window 4..294 and same output 1280x720/30fps/orientation
as prior V2 benchmarks. Compare fixed original Gemini V1 GOLDEN separately.
"""
from __future__ import annotations
import argparse,json,subprocess,time
from pathlib import Path
import imageio_ffmpeg,cv2,numpy as np
from c1_stream_host import ROOT,CASES,BUNDLE_ROOT,frame_from_meta
from render_and_evaluate import sample_weights,official_and_regions

OUT=ROOT/"reports"/"gemini_tripod_v2"/"c2_shadow"
VARIANTS=("c2_geometry_only","c2_joint_visual")
W,H=1280,720
cv2.setNumThreads(3)

def render(case,variants=VARIANTS):
    folder=BUNDLE_ROOT/case
    info=json.loads((folder/"bundle.json").read_text(encoding="utf-8"))
    frames=[frame_from_meta(f) for f in info["frames"]]
    idx=[f.source_id for f in frames]
    assert len(frames)==291 and idx[0]==4 and idx[-1]==294
    dest=OUT/case
    dest.mkdir(parents=True,exist_ok=True)
    output={}
    for label in variants:
        data=np.fromfile(dest/(label+"_mesh.f32"),dtype=np.float32)
        if data.size!=291*660*2:raise RuntimeError("C2 mesh does not match exact same Camera frame SourceIDs")
        path=dest/(label+"_output.mp4")
        writer=imageio_ffmpeg.write_frames(
            str(path),(W,H),fps=float(info.get("fps",29.605)),
            codec="libx264",pix_fmt_in="bgr24",pix_fmt_out="yuv420p",
            output_params=["-crf","19","-preset","veryfast","-movflags","+faststart"])
        writer.send(None)
        output[label]={"mesh":data.reshape(291,660,2),
                       "file":path,"writer":writer,"frames":0}
    (corners,weights)=sample_weights()
    inds=[i.astype(np.int64) for i in corners]
    wts=[v.astype(np.float32) for v in weights]
    cap=cv2.VideoCapture(str(folder/"source.mp4"))
    cap.set(cv2.CAP_PROP_ORIENTATION_AUTO,0)
    assert cap.isOpened()
    count=0
    start=time.perf_counter()
    try:
        for ordinal in range(idx[-1]+1):
            ok,src=cap.read()
            if not ok:raise RuntimeError(f"Original RAW movie missing source ordinal {ordinal}")
            if ordinal<idx[0]:continue
            if ordinal!=idx[count] or src.shape[:2]!=(1440,2560):
                raise RuntimeError("Camera Video frame/size/source ID mismatch")
            pts_ms=float(cap.get(cv2.CAP_PROP_POS_MSEC))
            if abs(pts_ms-frames[count].pts_us*.001)>1.1:
                raise RuntimeError(f"Raw to C2 warp PTS mismatch SourceID{ordinal} {pts_ms}")
            for label,data in output.items():
                mesh=data["mesh"][count]
                mx=(wts[0]*mesh[inds[0],0]+wts[1]*mesh[inds[1],0]+
                    wts[2]*mesh[inds[2],0])*2560-.5
                my=(wts[0]*mesh[inds[0],1]+wts[1]*mesh[inds[1],1]+
                    wts[2]*mesh[inds[2],1])*1440-.5
                dst=cv2.remap(src,mx.reshape(H,W).astype(np.float32),
                    my.reshape(H,W).astype(np.float32),cv2.INTER_LINEAR,
                    borderMode=cv2.BORDER_CONSTANT,borderValue=0)
                if info.get("orientation_deg",180)==180:
                    dst=cv2.rotate(dst,cv2.ROTATE_180)
                data["writer"].send(dst)
                data["frames"]+=1
            count+=1
            if count%70==0:
                print("C2_TRUE_RAW_SINGLE_WARP_RENDER",case,count,
                    "seconds",round(time.perf_counter()-start,1),flush=True)
    finally:
        cap.release()
        for data in output.values():
            data["writer"].close()
    if count!=291 or any(z["frames"]!=291 for z in output.values()):
        raise RuntimeError("Wrong C2 frame count")
    ff=imageio_ffmpeg.get_ffmpeg_exe()
    for label,z in output.items():
        cmd=[ff,"-v","error","-i",str(z["file"]),"-f","null","-"]
        run=subprocess.run(cmd,capture_output=True,text=True,timeout=90)
        if run.returncode or run.stderr.strip():
            raise RuntimeError(f"Encoder H264 failed {label}: {run.stderr[-500:]}")
        info={
            "case":case,"variant":label,
            "source_frame_ID_window":[4,294],
            "original_camera_source_frames":count,
            "output_frames_H264_verified":291,"output_pixel_domain":[W,H],
            "fps":float(info.get("fps",29.605)),
            "physical_RS19_20x33_valid":True,
            "orientation_180_normalized":True,
            "One_Warp":True,
            "C2_control_has_no_offline_Oracle":True,
            "C2_Golden_never_used_as_control_input":True,
            "elapsed_render_seconds":round(time.perf_counter()-start,1),
            "video_path":str(z["file"])
        }
        (OUT/case/(label+"_render_gate.json")).write_text(
            json.dumps(info,ensure_ascii=False,indent=2),encoding="utf-8")
        print("C2_H264_DECODABLE_SINGLE_SOURCE_PASS",json.dumps(info,ensure_ascii=False),flush=True)

def evaluate(case,variants=VARIANTS):
    golden=ROOT/"rendered_preview_fixed"/f"{case}_GEMINI_STABLE.mp4"
    assert golden.is_file()
    reference=official_and_regions(golden)
    evals={"case":case,"comparison":"V1 frozen video vs C2 same RAW SourceID input",
           "V1_GOLDEN":reference,"candidates":{}}
    for name in variants:
        movie=OUT/case/(name+"_output.mp4")
        if not movie.is_file():raise RuntimeError("Candidate needs real H264 before pixel Gate")
        cur=official_and_regions(movie)
        metrics=("official_motion_p90_px","center_dx_abs_p90",
                  "edge_antisymmetric_abs_p90","edge_antisymmetric_max_abs")
        cur["delta_vs_V1"]={k:cur[k]-reference[k] for k in metrics}
        # Treat modest center/overall losses as failures: preserve V1's
        # already-good gross yaw stabilization, plus reduce edge extrema.
        cur["V1_anti_sway_and_edge_pass"] = (
            cur["official_motion_p90_px"]<=reference["official_motion_p90_px"]*1.05 and
            cur["center_dx_abs_p90"]<=reference["center_dx_abs_p90"]*1.05 and
            cur["edge_antisymmetric_abs_p90"]<=reference["edge_antisymmetric_abs_p90"]*1.05 and
            cur["edge_antisymmetric_max_abs"]<=reference["edge_antisymmetric_max_abs"]*.75)
        evals["candidates"][name]=cur
        print("C2_TRUE_H264_PIXEL_GOLDEN_GATE",case,name,json.dumps(
            {k:cur[k] for k in metrics},
            ensure_ascii=False), "accepted=",cur["V1_anti_sway_and_edge_pass"],
            flush=True)
    dest=OUT/case/"pixel_effect_vs_V1.json"
    dest.write_text(json.dumps(evals,ensure_ascii=False,indent=2),encoding="utf-8")
    return evals

def main():
    p=argparse.ArgumentParser()
    p.add_argument("--cases",nargs="+",choices=CASES,default=list(CASES))
    p.add_argument("--variants",nargs="+",choices=VARIANTS,default=list(VARIANTS))
    p.add_argument("--render-only",action="store_true")
    p.add_argument("--eval-only",action="store_true")
    p.add_argument("--output-root",type=Path,default=None)
    a=p.parse_args()
    global OUT
    if a.output_root is not None:OUT=a.output_root
    summary=[]
    for case in a.cases:
        if not a.eval_only:render(case,tuple(a.variants))
        if not a.render_only:summary.append(evaluate(case,tuple(a.variants)))
    if summary:
        (OUT/"c2_pixels_all.json").write_text(
          json.dumps({"cases":summary,
                      "strict_effect_gate":"Must pass ALL FOUR against frozen V1",
                      "controller_input_offline_oracle":False},
                      ensure_ascii=False,indent=2),encoding="utf-8")
if __name__=="__main__":main()
