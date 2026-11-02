#!/usr/bin/env python3
"""C1 objective calibration: *actual input raw* pair motion before/after
buffer-native Gyro+RS19 pre-stabilization at identical SourceIDs.

Compares like-with-like LK on the 320x180 native RAW and gyro-prewarped
feature pair. Never uses Gemini V1 visual oracle or its output video.
This is a diagnostic, not a C1 controller gate based on future evidence.
"""
from __future__ import annotations
import argparse,json,time
from pathlib import Path
import numpy as np
import cv2
from c1_stream_host import BUNDLE_ROOT,RESULT_ROOT,CausalCSVGyroFeeder,frame_from_meta,C1StreamingObserver
from c1_gyro_rs import pose_rows,resize_native_gray
from c1_tile_observer import estimate

def compare(case,step=5):
    bundle=BUNDLE_ROOT/case
    info=json.loads((bundle/"bundle.json").read_text(encoding="utf-8"))
    frames=[frame_from_meta(d) for d in info["frames"]]
    feeder=CausalCSVGyroFeeder(bundle/"gyro_camera.csv")
    ob=C1StreamingObserver(info)
    reader=cv2.VideoCapture(str(bundle/"source.mp4"))
    reader.set(cv2.CAP_PROP_ORIENTATION_AUTO,0)
    previous=None
    sampled=[]
    mark=set(range(1,len(frames),step))
    for ordinal in range(frames[-1].source_id+1):
        ok,bgr=reader.read()
        if not ok:raise RuntimeError("Raw Video incomplete")
        if ordinal<frames[0].source_id:continue
        k=ordinal-frames[0].source_id
        f=frames[k]
        frame=resize_native_gray(bgr)
        ob.push_gyro_until_frame_deadline(f,feeder)
        rows=pose_rows(f,ob.gyro)
        center=ob.gyro.pose_at(f.center_gyro_ts_ns)
        if previous and k in mark:
            pf,prev_gray,prev_rows=previous
            curwarp,curmask=ob.warper.remap(frame,rows,center)
            prvwarp,prvmask=ob.warper.remap(prev_gray,prev_rows,center)
            ones=np.ones_like(curmask)
            raw=estimate(prev_gray,frame,ones,ones)
            stabilized=estimate(prvwarp,curwarp,prvmask,curmask)
            if raw.accepted and stabilized.accepted:
                rm=float(np.hypot(raw.center_dx,raw.center_dy))
                sm=float(np.hypot(stabilized.center_dx,stabilized.center_dy))
                sampled.append({
                    "source_id":f.source_id,"sensor_ts_ns":f.sensor_ts_ns,
                    "raw_motion_320px":rm,"gyro_rs19_motion_320px":sm,
                    "raw_dx":raw.center_dx,"raw_dy":raw.center_dy,
                    "derotated_dx":stabilized.center_dx,
                    "derotated_dy":stabilized.center_dy,
                    "raw_trust":raw.trust,"derotated_trust":stabilized.trust})
        previous=(f,frame,rows)
    reader.release()
    feeder.close()
    if len(sampled)<15:raise RuntimeError(f"Too few identical accepted raw vs gyro pairs: {len(sampled)}")
    ra=np.array([r["raw_motion_320px"] for r in sampled])
    st=np.array([r["gyro_rs19_motion_320px"] for r in sampled])
    report={
       "case":case,"sampled_pairs":len(sampled),
       "same_raw_SensorTS_pairs":True,
       "both_LK_estimates_accepted":True,
       "gyro_provided_by_real_400Hz_native_camera_stream":True,
       "no_offline_oracle":True,
       "raw_center_motion_p90_px_at320":float(np.percentile(ra,90)),
       "prewarped_center_motion_p90_px_at320":float(np.percentile(st,90)),
       "p90_pair_suppression_percent":float((1-np.percentile(st,90)/np.percentile(ra,90))*100),
       "median_pair_suppression_percent":float((1-np.median(st)/np.median(ra))*100),
       "fraction_pairs_prewarp_less_motion":float(np.mean(st<ra)),
       "pairs":sampled,
       "quality_gate":"PASS" if np.percentile(st,90)<np.percentile(ra,90)*.70 else "FAIL_GYRO_CONVENTION_CHECK"
    }
    path=RESULT_ROOT/case/"gyro_prewarp_validation.json"
    path.parent.mkdir(parents=True,exist_ok=True)
    path.write_text(json.dumps(report,indent=2,ensure_ascii=False),encoding="utf-8")
    print("C1_GYRO_PREWARP_ACTUAL_INPUT_AB",json.dumps({k:v for k,v in report.items() if k!="pairs"},
      ensure_ascii=False),flush=True)
    return report

if __name__=="__main__":
    p=argparse.ArgumentParser()
    p.add_argument("--cases",nargs="+",default=["024630"])
    p.add_argument("--step",type=int,default=5)
    a=p.parse_args()
    result=[compare(case,a.step) for case in a.cases]
    if any(x["quality_gate"]!="PASS" for x in result):
        raise SystemExit("C1 gyro orientation/offset projection Gate failed")
