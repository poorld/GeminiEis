#!/usr/bin/env python3
"""Gemini Tripod V2: identical raw-source H264 renders and final pixel gates.

Output is HOST_OFFLINE because input oracle and V1 Stage2 visual maps are
non-causal. Candidate GOVERNOR is forward-only; the pipeline is not.
"""
from __future__ import annotations
import argparse, json, math, sys, subprocess, time
from pathlib import Path
import cv2
import numpy as np
import imageio_ffmpeg

ROOT = Path(__file__).resolve().parents[2]
BUNDLES = Path("D:/hongda/app/YuySuperEIS/artifacts/b38_restore_bundles_20261020_172849")
OUT = ROOT / "reports" / "gemini_tripod_v2"
W,H = 1280,720
cv2.setNumThreads(4)

def sample_weights():
    xx=(np.arange(W,dtype=np.float32)+.5)/W*32
    yy=(np.arange(H,dtype=np.float32)+.5)/H*19
    cx=np.minimum(np.floor(xx).astype(np.int32),31)
    cy=np.minimum(np.floor(yy).astype(np.int32),18)
    tx=np.tile(xx-cx,H);ty=np.repeat(yy-cy,W)
    col=np.tile(cx,H);row=np.repeat(cy,W)
    first=tx+ty<=1.
    tl=row*33+col
    corners=(np.where(first,tl,tl+1),tl+33,np.where(first,tl+1,tl+34))
    weights=(np.where(first,1.-tx-ty,1.-ty),np.where(first,ty,1.-tx),
             np.where(first,tx,tx+ty-1.))
    return corners,weights

def render(case,variants):
    bundle=BUNDLES/case
    info=json.loads((bundle/"bundle.json").read_text(encoding="utf-8"))
    indices=[int(z["source_index"]) for z in info["frames"]]
    fps=float(info.get("fps",29.605))
    cases={}
    for variant in variants:
        base=OUT/case/variant
        manifest=json.loads((base/"manifest.json").read_text(encoding="utf-8"))
        mesh=np.fromfile(base/"mesh_governed.f32",dtype=np.float32).reshape(-1,20*33,2)
        if len(mesh)!=len(indices):raise RuntimeError("Mesh vs original SourceID count")
        out=base/"gemini_v2_output.mp4"
        writer=imageio_ffmpeg.write_frames(
            str(out),(W,H),fps=fps,codec="libx264",
            pix_fmt_in="bgr24",pix_fmt_out="yuv420p",
            output_params=["-crf","18","-preset","veryfast","-movflags","+faststart"])
        writer.send(None)
        cases[variant]={"mesh":mesh,"writer":writer,"output":out,"base":base}
    (i1,i2,i3),(a,b,c)=sample_weights()
    camera=cv2.VideoCapture(str(bundle/"source.mp4"))
    camera.set(cv2.CAP_PROP_ORIENTATION_AUTO,0)
    if not camera.isOpened():raise RuntimeError("Raw source cannot decode")
    count=0;t0=time.perf_counter()
    try:
        for file_ordinal in range(indices[-1]+1):
            ok,image=camera.read()
            if not ok:raise RuntimeError(f"Missing source ordinal {file_ordinal}")
            if file_ordinal<indices[count]:continue
            if file_ordinal!=indices[count] or image.shape[:2]!=(1440,2560):
                raise RuntimeError("Wrong camera source frame, orientation or size")
            for label,case_data in cases.items():
                vertex=case_data["mesh"][count]
                mx=((a*vertex[i1,0]+b*vertex[i2,0]+c*vertex[i3,0])*2560-.5).astype(np.float32).reshape(H,W)
                my=((a*vertex[i1,1]+b*vertex[i2,1]+c*vertex[i3,1])*1440-.5).astype(np.float32).reshape(H,W)
                result=cv2.remap(image,mx,my,cv2.INTER_LINEAR,
                                  borderMode=cv2.BORDER_CONSTANT,borderValue=0)
                if info.get("orientation_deg",180)==180:
                    result=cv2.rotate(result,cv2.ROTATE_180)
                case_data["writer"].send(result)
            count+=1
            if count%80==0:
                print("V2_H264_RENDER",case,count,len(indices),
                      round(time.perf_counter()-t0,1),flush=True)
            if count==len(indices):break
    finally:
        camera.release()
        for data in cases.values():data["writer"].close()
    if count!=len(indices):raise RuntimeError("Not all true raw source frames rendered")
    ff=imageio_ffmpeg.get_ffmpeg_exe()
    for name,data in cases.items():
        outcome=subprocess.run([ff,"-v","error","-i",str(data["output"]),"-f","null","-"],
                               capture_output=True,text=True,timeout=90)
        if outcome.returncode or outcome.stderr.strip():
            raise RuntimeError("Bad encoded H264 "+name+outcome.stderr[-500:])
        info={
            "case":case,"variant":name,"raw_source":"b38_restore_bundles/source.mp4",
            "original_SourceID_window":[indices[0],indices[-1]],
            "source_frames":count,"output_width":W,"output_height":H,
            "one_warp":True,"native_orientation_180_after_one_warp":True,
            "ffmpeg_decoder_errors":0,
            "output":str(data["output"]),
            "size_MiB":round(data["output"].stat().st_size/1048576,2),
            "elapsed_group_render_sec":round(time.perf_counter()-t0,2)}
        (data["base"]/"render_gate.json").write_text(
            json.dumps(info,ensure_ascii=False,indent=2),encoding="utf-8")
        print("V2_H264_VALID",json.dumps(info),flush=True)

def official_and_regions(video):
    sys.path.extend(["D:/hongda/app/YuyEIS2-pc-reference-Y1/tools",
                    "D:/hongda/app/YuySuperEIS"])
    from YuyEis3.reference.pc_batch03.vision import track_pair,center_displacement
    cap=cv2.VideoCapture(str(video))
    cap.set(cv2.CAP_PROP_ORIENTATION_AUTO,0)
    old=None;reported=[];dxs=[];idx=0
    while True:
        ok,frame=cap.read()
        if not ok:break
        gray=cv2.cvtColor(cv2.resize(frame,(640,360),interpolation=cv2.INTER_AREA),
                            cv2.COLOR_BGR2GRAY)
        if old is not None:
            if idx>=31:
                sim,_,_=track_pair(old,gray)
                if sim is not None:
                    diff=center_displacement(sim[None],640,360)[0]*2.0
                    reported.append(float(np.linalg.norm(diff)))
            mask=np.zeros((360,640),np.uint8)
            mask[45:315,26:615]=255
            corners=cv2.goodFeaturesToTrack(old,maxCorners=620,
                        qualityLevel=.012,minDistance=5,mask=mask)
            if corners is not None:
                curr,valid,_=cv2.calcOpticalFlowPyrLK(old,gray,corners,None,
                    winSize=(17,17),maxLevel=2)
                back,valid2,_=cv2.calcOpticalFlowPyrLK(gray,old,curr,None,
                    winSize=(17,17),maxLevel=2)
                pos=corners.reshape(-1,2)
                b=back.reshape(-1,2)
                good=(valid.ravel()>0)&(valid2.ravel()>0)&(np.linalg.norm(pos-b,axis=1)<.7)
                dif=(curr.reshape(-1,2)[:,0]-pos[:,0])*2
                x=pos[:,0]
                mask_z=[good&(x>=640*a)&(x<=640*b)
                       for a,b in ((.04,.29),(.38,.62),(.71,.96))]
                if min(z.sum() for z in mask_z)>=10:
                    areas=np.asarray([float(np.median(dif[z])) for z in mask_z])
                    dxs.append([idx,*areas, (areas[2]-areas[0])/2])
        old=gray;idx+=1
    cap.release()
    if not reported or not dxs:raise RuntimeError("Missing video optical-flow evaluation")
    a=np.asarray(dxs)
    l,c,r,ext=[a[:,i] for i in (1,2,3,4)]
    events={}
    for frameid in (57,66,67,68,69,91,92,93,114,115):
        ix=np.where(a[:,0].astype(int)==frameid)[0]
        if len(ix):
            j=ix[0]
            events[frameid]={"left":float(l[j]),"center":float(c[j]),
                             "right":float(r[j]),"edge_breath":float(ext[j])}
    return {
        "frames":idx,"real_LK_valid_3zones":len(dxs),
        "official_motion_p90_px":float(np.percentile(reported,90)),
        "official_motion_rms_px":float(np.sqrt(np.mean(np.square(reported)))),
        "center_dx_abs_p90":float(np.percentile(abs(c),90)),
        "left_relative_center_abs_p90":float(np.percentile(abs(l-c),90)),
        "right_relative_center_abs_p90":float(np.percentile(abs(r-c),90)),
        "edge_antisymmetric_abs_p90":float(np.percentile(abs(ext),90)),
        "edge_antisymmetric_max_abs":float(np.max(abs(ext))),
        "events":events,
        "roi_series": [{"frame":int(z[0]),"left_dx":float(z[1]),
                        "center_dx":float(z[2]),"right_dx":float(z[3]),
                        "edge_breath":float(z[4])} for z in dxs]
    }


def verify(case,variants):
    baseline=official_and_regions(ROOT/"rendered_preview_fixed"/f"{case}_GEMINI_STABLE.mp4")
    outcomes={"case":case,"GOLDEN":baseline,"variants":{}}
    for var in variants:
        dir=OUT/case/var
        cand=official_and_regions(dir/"gemini_v2_output.mp4")
        cand["delta_vs_golden"]={
            key: float(cand[key]-baseline[key])
            for key in ("official_motion_p90_px","center_dx_abs_p90",
                        "edge_antisymmetric_abs_p90","edge_antisymmetric_max_abs")}
        cand["accept_as_golden"]=(
            cand["edge_antisymmetric_max_abs"]<baseline["edge_antisymmetric_max_abs"]*.5
            and cand["official_motion_p90_px"] <= baseline["official_motion_p90_px"]*1.05
            and cand["center_dx_abs_p90"] <= baseline["center_dx_abs_p90"]*1.05
            and cand["edge_antisymmetric_abs_p90"] <= baseline["edge_antisymmetric_abs_p90"]*1.05)
        outcomes["variants"][var]=cand
        print("V2_TRUE_H264_PIXEL_GATE",case,var,
              json.dumps({k:v for k,v in cand.items() if k not in ("events","roi_series")},ensure_ascii=False),
              flush=True)
    dest=OUT/case/"real_pixel_comparison.json"
    dest.write_text(json.dumps(outcomes,ensure_ascii=False,indent=2),encoding="utf-8")
    return outcomes


if __name__=="__main__":
    parser=argparse.ArgumentParser()
    parser.add_argument("--case",required=True)
    parser.add_argument("--variants",nargs="+",required=True)
    parser.add_argument("--no-render",action="store_true")
    args=parser.parse_args()
    if not args.no_render:render(args.case,args.variants)
    res=verify(args.case,args.variants)
    print("V2_CASE_COMPLETE",json.dumps({"case":args.case,
        "candidate_pass_golden":[k for k,v in res["variants"].items()
                                 if v["accept_as_golden"]]},ensure_ascii=False))
