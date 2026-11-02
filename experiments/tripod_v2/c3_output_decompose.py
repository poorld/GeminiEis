#!/usr/bin/env python3
"""C2-C3 independent, output-side 4DoF & tiled residual decomposition.

Offline EVALUATOR ONLY. Never feeds V1 Golden/future pixels to C3 realtime
controller. Uses original 1280x720 final H264 videos at 640x360 tracking
(×2 to output pixels). Separates center translation, vertical, roll, scale,
peripheral affine-consensus residual and true left/right antisymmetric drift.
"""
from __future__ import annotations
import argparse,json,math,time
from pathlib import Path
import cv2,numpy as np
from c1_stream_host import ROOT,CASES

OUT=ROOT/"reports"/"gemini_tripod_v2"/"c3_global_path"
OUTPUT=ROOT/"reports"/"gemini_tripod_v2"
VIDEOS={
 "V1":ROOT/"rendered_preview_fixed",
 "C2_CENTER":OUTPUT/"c2_shadow_cal",
 "C2_EDGE":OUTPUT/"c2_shadow_edge",
 "C3_GLOBAL":OUTPUT/"c3_path_4dof",
 "C3_FORECAST":OUTPUT/"c3_path_forecast",
 "C3_YAW_FOLLOW":OUTPUT/"c3_path_yawfollow"
}
W,H=640,360
cv2.setNumThreads(3)
def p90(xs):
    a=np.asarray([x for x in xs if x is not None and np.isfinite(x)],float)
    return float(np.percentile(np.abs(a),90)) if len(a) else None
def process(case:str,variant:str):
    if variant=="V1":
        video=VIDEOS["V1"]/(case+"_GEMINI_STABLE.mp4")
    else:video=VIDEOS[variant]/case/"c2_joint_visual_output.mp4"
    assert video.exists(),str(video)
    cap=cv2.VideoCapture(str(video))
    cap.set(cv2.CAP_PROP_ORIENTATION_AUTO,0)
    assert cap.isOpened()
    N=int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    assert N==291,N
    mask=np.zeros((H,W),np.uint8)
    mask[22:H-22,22:W-22]=255
    last=None;metrics=[];index=0
    t0=time.perf_counter()
    try:
      while True:
        ok,im=cap.read()
        if not ok:break
        assert im.shape[:2]==(720,1280)
        now=cv2.cvtColor(cv2.resize(im,(W,H),interpolation=cv2.INTER_AREA),
                             cv2.COLOR_BGR2GRAY)
        if last is not None:
          rec={"frame":index,"source_id":index+4,"verified_video_frame":True,
               "status":"BAD","corners_checked":0,"FB_tracks":0,
               "global_inliers":0,"global_spatial_support":0}
          features=cv2.goodFeaturesToTrack(last,maxCorners=650,qualityLevel=.013,
                                            minDistance=5,mask=mask)
          if features is not None:
            xy=features.reshape(-1,2)
            rec["corners_checked"]=len(xy)
            curr,va,_=cv2.calcOpticalFlowPyrLK(
                last,now,features,None,winSize=(17,17),maxLevel=2,
                criteria=(cv2.TERM_CRITERIA_EPS|cv2.TERM_CRITERIA_COUNT,18,.015))
            if curr is not None:
              back,vb,_=cv2.calcOpticalFlowPyrLK(
                  now,last,curr,None,winSize=(17,17),maxLevel=2,
                  criteria=(cv2.TERM_CRITERIA_EPS|cv2.TERM_CRITERIA_COUNT,18,.015))
              if back is not None:
                curr=curr.reshape(-1,2)
                valid=((va.reshape(-1)>0)&(vb.reshape(-1)>0)&
                    (np.linalg.norm(xy-back.reshape(-1,2),axis=1)<.85)&
                    (np.linalg.norm(curr-xy,axis=1)<30))
                p=xy[valid];q=curr[valid]
                rec["FB_tracks"]=int(len(p))
                if len(p)>=30:
                  mat,ins=cv2.estimateAffinePartial2D(p,q,method=cv2.RANSAC,
                   ransacReprojThreshold=1.3,maxIters=650,confidence=.995,
                   refineIters=8)
                  if mat is not None and ins is not None:
                    ins=ins.ravel()>0
                    rec["global_inliers"]=int(ins.sum())
                    sp=np.unique(np.clip(np.floor(p[ins,0]/(W/3)),0,2).astype(int))
                    rec["global_spatial_support"]=int(len(sp))
                    if ins.sum()>=24 and len(sp)==3:
                      a=float(mat[0,0]);b=float(mat[1,0])
                      s=math.hypot(a,b)
                      center=np.array([W*.5,H*.5])
                      shift=((mat[:,:2]@center+mat[:,2])-center)*2
                      xp=(p@mat[:,:2].T+mat[:,2])
                      err=(q-xp)*2
                      zones={}
                      for key,lo,hi in (("left",.04,.29),("center",.36,.64),
                                           ("right",.71,.96)):
                        sub=(p[:,0]>=W*lo)&(p[:,0]<W*hi)&ins
                        if sub.sum()>=7:
                           zones[key]={
                             "tracks":int(sub.sum()),
                             "raw_dx_px":float(2*np.median((q-p)[sub,0])),
                             "raw_dy_px":float(2*np.median((q-p)[sub,1])),
                             "unmodeled_dx_px":float(np.median(err[sub,0])),
                             "unmodeled_dy_px":float(np.median(err[sub,1])),
                             "unmodeled_abs_p90_px":float(np.percentile(
                                 np.linalg.norm(err[sub],axis=1),90)),
                           }
                      rec.update({
                        "status":"ACCEPT","center_global_dx_px":float(shift[0]),
                        "center_global_dy_px":float(shift[1]),
                        "rotation_rad":math.atan2(b,a),
                        "logscale":math.log(s),
                        "raw_translation_tx_px":float(mat[0,2]*2),
                        "raw_translation_ty_px":float(mat[1,2]*2),
                        "global_inlier_model_residual_p90_px":float(np.percentile(
                           np.linalg.norm(err[ins],axis=1),90)),
                        "zones":zones
                      })
                      if all(v in zones for v in ("left","center","right")):
                        rec["left_minus_center_dx_px"]=(zones["left"]["raw_dx_px"]-
                                                     zones["center"]["raw_dx_px"])
                        rec["right_minus_center_dx_px"]=(zones["right"]["raw_dx_px"]-
                                                     zones["center"]["raw_dx_px"])
                        rec["edge_antisymmetric_raw_dx_px"]=(
                         zones["right"]["raw_dx_px"]-zones["left"]["raw_dx_px"])*.5
                        rec["edge_antisymmetric_unmodeled_dx_px"]=(
                         zones["right"]["unmodeled_dx_px"]-
                         zones["left"]["unmodeled_dx_px"])*.5
          metrics.append(rec)
        last=now
        index+=1
    finally:cap.release()
    assert index==291 and len(metrics)==290,(index,len(metrics))
    dest=OUT/case;dest.mkdir(parents=True,exist_ok=True)
    dest.joinpath(f"{variant}_output_4dof_frames.jsonl").write_text(
       "".join(json.dumps(m,ensure_ascii=False,separators=(",",":"))+"\n"
               for m in metrics),encoding="utf-8")
    valid=[r for r in metrics if r["status"]=="ACCEPT"]
    assert len(valid)>=230,("Too few trusted global output frames",case,variant,len(valid))
    def arr(key):return [r.get(key) for r in valid]
    summary={
      "case":case,"variant":variant,"source_id_verified":True,
      "video_source":str(video),
      "frames":index,"pairs":len(metrics),"accepted_4dof":len(valid),
      "global_center_dx_p90_px":p90(arr("center_global_dx_px")),
      "global_center_dy_p90_px":p90(arr("center_global_dy_px")),
      "global_rotation_p90_deg":p90([math.degrees(v) for v in arr("rotation_rad")]),
      "global_logscale_p90_milli":p90([v*1000 for v in arr("logscale")]),
      "global_model_inlier_residual_p90_px":p90(
          arr("global_inlier_model_residual_p90_px")),
      "left_relative_center_dx_p90":p90(arr("left_minus_center_dx_px")),
      "right_relative_center_dx_p90":p90(arr("right_minus_center_dx_px")),
      "edge_antisymmetric_raw_dx_p90_px":p90(arr("edge_antisymmetric_raw_dx_px")),
      "edge_antisymmetric_unmodeled_dx_p90_px":p90(arr("edge_antisymmetric_unmodeled_dx_px")),
      "elapsed_eval_sec":round(time.perf_counter()-t0,1),
      "offline_only_never_controller_input":True,
      "Android_device":"NOT_TESTED"
    }
    (dest/f"{variant}_output_4dof_summary.json").write_text(
         json.dumps(summary,ensure_ascii=False,indent=2),encoding="utf-8")
    print("C3_FINAL_H264_4DOF_DECOMPOSITION",json.dumps(summary,ensure_ascii=False),
          flush=True)
    return summary

if __name__=="__main__":
  arg=argparse.ArgumentParser()
  arg.add_argument("--cases",nargs="+",choices=CASES,default=list(CASES))
  arg.add_argument("--variants",nargs="+",choices=list(VIDEOS),default=list(VIDEOS))
  a=arg.parse_args()
  summary=[process(c,v) for c in a.cases for v in a.variants]
  OUT.mkdir(parents=True,exist_ok=True)
  (OUT/"c3_global_motion_decomp_all.json").write_text(
     json.dumps(summary,ensure_ascii=False,indent=2),encoding="utf-8")
