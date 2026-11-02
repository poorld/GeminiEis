#!/usr/bin/env python3
"""Four frozen real-RAW async C++ one-inflight deadline gate, SHADOW ONLY.

Host-model deadline starts BEFORE original source.mp4 software decode:
  release wall-clock + 1/29.605sec (33.778ms).
This is deliberately strict; future Android Camera YUV direct path is NOT
being simulated as if it were already proven. Any expired/late current result
is rejected, never reused for following sourceID. No C2, no target/zoom.
"""
from __future__ import annotations
import argparse,json,time
from pathlib import Path
import cv2
import numpy as np
from c1_stream_host import (
    ROOT,BUNDLE_ROOT,CASES,CausalCSVGyroFeeder,frame_from_meta,gyro_center_omega)
from c1_gyro_rs import (
    NativeRs19Prewarper,CausalGyro,pose_rows,resize_native_gray)
from c1b_native_async import NativeAsync
OUT=ROOT/"reports/gemini_tripod_v2/c1b_native"
FPS=29.605
def pctl(rows,key,p):
    x=[r[key] for r in rows if r.get(key) is not None]
    return float(np.percentile(x,p)) if x else None
def replay(case,limit=None,deadline_mode="full"):
    if deadline_mode not in ("full","feature"):
        raise ValueError(deadline_mode)
    directory=BUNDLE_ROOT/case
    manifest=json.loads((directory/"bundle.json").read_text(encoding="utf-8"))
    frames=[frame_from_meta(r) for r in manifest["frames"]]
    assert len(frames)==291 and frames[0].source_id==4 and frames[-1].source_id==294
    if limit:frames=frames[:limit]
    feeder=CausalCSVGyroFeeder(directory/"gyro_camera.csv")
    gyro=CausalGyro()
    warper=NativeRs19Prewarper(
        manifest["fx"],manifest["fy"],manifest["cx"],manifest["cy"],
        manifest["source_width"],manifest["source_height"])
    cap=cv2.VideoCapture(str(directory/"source.mp4"))
    cap.set(cv2.CAP_PROP_ORIENTATION_AUTO,0)
    n=0;prev=None
    rows=[]
    worker=NativeAsync(0)
    try:
        for ordinal in range(frames[-1].source_id+1):
            now=worker.now_ns()
            ok,bgr=cap.read()
            if not ok:raise RuntimeError(f"Cannot decode original camera ordinal {ordinal}")
            pts_ms=float(cap.get(cv2.CAP_PROP_POS_MSEC))
            if ordinal<4:continue
            f=frames[n]
            if f.source_id!=ordinal or abs(pts_ms-f.pts_us/1000)>1.1:
                raise RuntimeError(f"SourceID/PTS mismatch at {ordinal}")
            decoded=resize_native_gray(bgr)
            feeder.feed_until(f.availability_deadline_ns,gyro)
            if not all(gyro.coverage(f.gyro_time_at(i/19)) for i in range(20)):
                raise RuntimeError("C1-B async source missing actual Gyro row coverage")
            pose=pose_rows(f,gyro)
            center=gyro.pose_at(f.center_gyro_ts_ns)
            decoder_completed_ns=worker.now_ns()
            if prev:
                prv_f,prv_gray,prv_pose=prev
                if f.source_id!=prv_f.source_id+1:
                    raise RuntimeError("Non-consecutive frame")
                # Both input feature images use the SAME current frame pose
                # and only the historical frame plus present gyro coverage.
                left,lmask=warper.remap(prv_gray,prv_pose,center)
                right,rmask=warper.remap(decoded,pose,center)
                prepared_ns=worker.now_ns()
                # Full: original MP4 decoder + prewarp + Native C++ must
                # finish within 33.78ms of source decode start.
                # Feature: ISOLATED lower-level Native worker test, after the
                # images/gyro/RS are already prepared, with 20ms budget.
                # Feature mode cannot prove Camera-to-encoder 30fps.
                deadline_ns=(now+int(1e9/FPS) if deadline_mode=="full"
                             else prepared_ns+20_000_000)
                # Same-frame current image -> real C++ worker. If preparing
                # features already exhausted the frame, fail closed.
                code=worker.submit(f.source_id,f.sensor_ts_ns,deadline_ns,
                                   left,right,lmask,rmask)
                last_native_result=None
                if code==0:
                    status,res=worker.wait(f.source_id,deadline_ns)
                    if status==0:
                        last_native_result={
                            "status":int(res.status),"dx":float(res.center_dx),
                            "dy":float(res.center_dy),"trust":float(res.trust),
                            "inliers":int(res.inliers),
                            "native_lk_worker_ms":float(res.elapsed_ms)}
                else:status={"1":"BUSY","3":"ORDER_FAIL","4":"PREP_ALREADY_LATE"}.get(str(code),str(code))
                done=worker.now_ns()
                recorded={
                    "source_id":f.source_id,"sensor_ts_ns":f.sensor_ts_ns,
                    "pts_us":f.pts_us,
                    "camera_gyro_last_submitted_ns":gyro.maximum_submitted_ns,
                    "gyro_query_max_ns":gyro.max_query_ns,
                    "same_frame_id_ready":code==0 and status==0,
                    "scheduler_submit_code":code,
                    "scheduler_poll_code":status,
                    "late_current_frame_rejected":status!=0,
                    "native_result":last_native_result,
                    "decode_and_preprocess_ms":(decoder_completed_ns-now)*1e-6,
                    "pair_prewarp_and_prep_ms":(prepared_ns-decoder_completed_ns)*1e-6,
                    "submit_plus_worker_wait_ms":(done-prepared_ns)*1e-6,
                    "host_full_source_to_visual_delivery_ms":(done-now)*1e-6,
                    "29p605fps_frame_budget_ms":1000/FPS,
                    "deadline_mode":deadline_mode,
                    "deadline_ns_absolute_host_mono":deadline_ns,
                    "result_reused_across_SourceID":False,
                    "ORACLE_USED":False,
                    "C2_TARGET_CHANGED":False
                }
                assert gyro.max_query_ns<=f.availability_deadline_ns
                rows.append(recorded)
            prev=(f,decoded,pose)
            n+=1
            if n%70==0:
                print("C1B_ASYNC_RAW",case,"processed",n,
                      "same_frame_ready",sum(r["same_frame_id_ready"] for r in rows),
                      "expired",sum(r["late_current_frame_rejected"] for r in rows),
                      flush=True)
        assert n==len(frames)
        stats=worker.stats()
    finally:
        cap.release();feeder.close();worker.close()
    target=(OUT/case if deadline_mode=="full"
            else OUT/"feature_ready"/case)
    target.mkdir(parents=True,exist_ok=True)
    (target/"async_frame_deadline.jsonl").write_text(
        "".join(json.dumps(r,ensure_ascii=False,separators=(",",":"))+"\n" for r in rows),
        encoding="utf-8")
    ready=[r for r in rows if r["same_frame_id_ready"]]
    rejected=[r for r in rows if not r["same_frame_id_ready"]]
    result={
        "case":case,"source_frames":n,
        "same_frame_pairs":len(rows),
        "async_same_frame_ready":len(ready),
        "async_frame_rejected":len(rejected),
        "same_frame_ready_percent":len(ready)/max(1,len(rows))*100,
        "policy":("HOST_FULL_ORIGINAL_MP4_START_TO_29p605FPS_DEADLINE"
                  if deadline_mode=="full" else
                  "HOST_FEATURE_ALREADY_READY_20MS_CPP_WORKER_DEADLINE"),
        "host_source_start_to_GATE_DECISION_p90_ms":pctl(rows,"host_full_source_to_visual_delivery_ms",90),
        "decode_to_feature_p90_ms":pctl(rows,"decode_and_preprocess_ms",90),
        "RS19_prepare_p90_ms":pctl(rows,"pair_prewarp_and_prep_ms",90),
        "native_thread_wait_p90_ms":pctl(rows,"submit_plus_worker_wait_ms",90),
        "native_worker_lk_p90_ms":float(np.percentile([
             r["native_result"]["native_lk_worker_ms"] for r in ready],90))
             if ready else None,
        "strict_source_order_verified":True,
        "C2_would_receive_only_exact_SourceID_on_time":True,
        "C2_not_connected":True,
        "device_30fps":"NOT_TESTED",
        "offline_oracle_loaded":False,
        "native_async_stats":stats,
        "result":"HOST_ASYNC_GATE_PASS" if not rejected else
                 "HOST_ASYNC_LATENCY_GATE_FAIL_CLOSED",
    }
    (target/"async_deadline_summary.json").write_text(
        json.dumps(result,ensure_ascii=False,indent=2),encoding="utf-8")
    print("C1B_ASYNC_PHYSICAL_RAW_HOST_DEADLINE",
          json.dumps(result,ensure_ascii=False),flush=True)
    return result

def main():
    a=argparse.ArgumentParser()
    a.add_argument("--cases",nargs="+",choices=CASES,default=CASES)
    a.add_argument("--max-frames",type=int)
    a.add_argument("--mode",choices=("full","feature"),default="full")
    arg=a.parse_args()
    out=[replay(c,arg.max_frames,arg.mode) for c in arg.cases]
    dest=(OUT/"async_deadline_all.json" if arg.mode=="full" else
          OUT/"feature_ready"/"async_deadline_all.json")
    dest.parent.mkdir(parents=True,exist_ok=True)
    dest.write_text(
        json.dumps({"cases":out,"timestamp":"2026-10-24",
                    "C2_actuation":False,
                    "same_frame_drop_on_late":True},ensure_ascii=False,indent=2),
        encoding="utf-8")
if __name__=="__main__":main()
