#!/usr/bin/env python3
"""V2C-C1 real-RAW streaming, strictly causal Gyro+RS19 + 3x3 tile LK.

This script is C1 SHADOW ONLY. It neither reads Gemini offline oracle nor
modifies target/zoom/GPU output nor claims Android 30fps. One raw camera frame
is decoded sequentially and sensor samples are pushed only up to its physical
readout deadline. Source frame ordinal+MP4 PTS+SensorTS are certified.
"""
from __future__ import annotations
import argparse,csv,json,math,time,sys
from pathlib import Path
from bisect import bisect_left
import cv2
import numpy as np
from c1_gyro_rs import CameraFrame,CausalGyro,NativeRs19Prewarper,pose_rows,resize_native_gray
from c1_tile_observer import estimate,CausalTurnState,PairResult

ROOT=Path(__file__).resolve().parents[2]
BUNDLE_ROOT=Path("D:/hongda/app/YuySuperEIS/artifacts/b38_restore_bundles_20261020_172849")
RESULT_ROOT=ROOT/"reports"/"gemini_tripod_v2"/"c1_causal"
CASES=("024605","024618","024630","024647")
cv2.setNumThreads(2)


class CausalCSVGyroFeeder:
    """Physical streaming Host substitute for live IMU callback.

    CSV is consumed sequentially; only gyro records whose sensor timestamps
    are <= current released Camera frame deadline are submitted. At most one
    extra file record can be pending for boundary detection, and it is NEVER
    applied, queried, integrated or passed into the visual observer state.
    """
    def __init__(self,path:Path):
        self.file=path.open("r",encoding="utf-8",newline="")
        self.reader=csv.DictReader(self.file)
        self.pending=None
        self.last_timestamp=0
        self.submitted=0
        self.last_deadline=0
    def feed_until(self,deadline_ns:int,gyro:CausalGyro)->int:
        if deadline_ns<self.last_deadline:
            raise ValueError("Camera frame time moved backward")
        self.last_deadline=deadline_ns
        added=0
        while True:
            if self.pending is None:
                self.pending=next(self.reader,None)
            if self.pending is None:break
            t=int(self.pending["timestamp_ns"])
            if t>deadline_ns:break
            if t<=self.last_timestamp:raise ValueError("Gyro source stream out of order")
            g=np.array([float(self.pending[k]) for k in ("x","y","z")],
                       dtype=np.float64)
            gyro.push(t,g)
            self.last_timestamp=t
            self.submitted+=1
            added+=1
            self.pending=None
        assert gyro.maximum_submitted_ns<=deadline_ns
        return added
    def close(self):
        self.file.close()

def read_gyro(p:Path)->tuple[np.ndarray,np.ndarray]:
    tm=[];g=[]
    with p.open("r",encoding="utf-8",newline="") as f:
        rd=csv.DictReader(f)
        for row in rd:
            tm.append(int(row["timestamp_ns"]))
            g.append((float(row["x"]),float(row["y"]),float(row["z"])))
    times=np.asarray(tm,np.int64)
    omega=np.asarray(g,np.float64)
    if not len(times) or np.any(np.diff(times)<=0):
        raise RuntimeError("Gyro time not strictly increasing")
    return times,omega

def frame_from_meta(d:dict)->CameraFrame:
    return CameraFrame(
        int(d["source_index"]),int(d["pts_us"]),int(d["sensor_ts_ns"]),
        int(d["exposure_ns"]),int(d["skew_ns"]))

def gyro_center_omega(t:int,stream:CausalGyro)->np.ndarray:
    if not stream.coverage(t):
        raise ValueError("No Gyro center coverage")
    k=bisect_left(stream.ts,int(t))
    if stream.ts[k]==t:return stream.g[k]
    a=k-1
    f=(t-stream.ts[a])/float(stream.ts[k]-stream.ts[a])
    return stream.g[a]*(1-f)+stream.g[k]*f

def result_to_record(pair:PairResult)->dict:
    return {
      "status":pair.status,
      "accepted":pair.accepted,
      "trust":pair.trust,
      "tracked":pair.tracked,"checked":pair.checked,
      "inliers":pair.inliers,"support_tiles":pair.support_tiles,
      "support_cols":pair.support_cols,
      "dx_feature_px":pair.center_dx,"dy_feature_px":pair.center_dy,
      "rotation_rad":pair.rotation_rad,
      "logscale":pair.logscale,
      "fb_p90_px":pair.global_fb_p90,
      "residual_fit_p90_px":pair.residual_p90,
      "tiles":pair.tiles,
      "model_matrix":pair.mat
    }

class C1StreamingObserver:
    def __init__(self,cfg:dict):
        w,h=int(cfg["source_width"]),int(cfg["source_height"])
        self.warper=NativeRs19Prewarper(
            float(cfg["fx"]),float(cfg["fy"]),
            float(cfg["cx"]),float(cfg["cy"]),
            w,h)
        self.gyro=CausalGyro()
        self.turn=CausalTurnState()
        self.prev:tuple[CameraFrame,np.ndarray,np.ndarray]|None=None
        self.key:tuple[CameraFrame,np.ndarray,np.ndarray]|None=None
        self.accum=np.zeros(2,dtype=np.float64)
        self.accum_key=np.zeros(2,dtype=np.float64)
        self.key_age=0
        self.key_resets=0
        self.key_lost=0
        self.last_vision_ns=0
        self.stream_cursor=0
        self.gyro_max_ready=0

    def push_gyro_until_frame_deadline(self,f:CameraFrame,
                                       feeder:CausalCSVGyroFeeder)->int:
        deadline=f.availability_deadline_ns
        if deadline<self.gyro_max_ready:
            raise ValueError("CameraTS out of order")
        added=feeder.feed_until(deadline,self.gyro)
        self.stream_cursor=feeder.submitted
        self.gyro_max_ready=deadline
        # Absolutely forbid submitting Gyro ahead of the actual frame deadline.
        assert self.gyro.maximum_submitted_ns<=deadline
        return added

    def process(self,f:CameraFrame,gray:np.ndarray,source_pts_ms:float)->dict:
        start=time.perf_counter()
        if abs(source_pts_ms - f.pts_us*.001)>1.1:
            raise ValueError(f"SourceID/encoded PTS mismatch: {f.source_id} "
                             f"{source_pts_ms} vs {f.pts_us/1000}")
        if self.prev and f.source_id!=self.prev[0].source_id+1:
            raise ValueError("Source ordinal missing: C1 cannot fabricate time history")
        if self.prev and f.sensor_ts_ns<=self.prev[0].sensor_ts_ns:
            raise ValueError("Non-monotonic SensorTS")
        qts=[f.gyro_time_at(i/19) for i in range(20)]
        if not all(self.gyro.coverage(t) for t in qts):
            self.prev=None;self.key=None
            return {"source_id":f.source_id,"sensor_ts_ns":f.sensor_ts_ns,
                    "pts_us":f.pts_us,"status":"NO_GYRO_ROW_COVERAGE",
                    "accepted":False,"trust":0.,"available_deadline_ns":f.availability_deadline_ns,
                    "gyro_last_submitted_ns":self.gyro.maximum_submitted_ns,
                    "c1_total_ms":(time.perf_counter()-start)*1e3}
        center_q=self.gyro.pose_at(f.center_gyro_ts_ns)
        current_rows=pose_rows(f,self.gyro)
        imu_omega=gyro_center_omega(f.center_gyro_ts_ns,self.gyro)
        motion=self.turn.update(f.sensor_ts_ns,float(imu_omega[1]))
        t0=time.perf_counter()
        current_warp,current_valid=self.warper.remap(gray,current_rows,center_q)
        minwarp_ms=(time.perf_counter()-t0)*1e3
        if self.prev is None:
            self.prev=(f,gray,current_rows)
            self.key=self.prev
            return {"source_id":f.source_id,"sensor_ts_ns":f.sensor_ts_ns,
                    "pts_us":f.pts_us,"status":"INIT","accepted":False,
                    "trust":0.,"last_gyro_ts":self.gyro.maximum_submitted_ns,
                    "available_deadline_ns":f.availability_deadline_ns,
                    "gyro_query_max_ns":self.gyro.max_query_ns,
                    "lowres_prewarp_ms":minwarp_ms,
                    "c1_total_ms":(time.perf_counter()-start)*1e3,
                    **motion}
        t0=time.perf_counter()
        prev_f,prev_gray,prev_rows=self.prev
        prev_warp,prev_valid=self.warper.remap(prev_gray,prev_rows,center_q)
        pairwarps_ms=(time.perf_counter()-t0)*1e3
        t0=time.perf_counter()
        pair=estimate(prev_warp,current_warp,prev_valid,current_valid)
        pairms=(time.perf_counter()-t0)*1e3
        # Current frame vs historical keyframe is optional verification
        # of cumulative drift. Its role is observational, not actuation.
        key_result=None
        keyms=0.
        if self.key is not None and self.key_age==9:
            keyf,keygray,keyrows=self.key
            tk=time.perf_counter()
            keywarp,keymask=self.warper.remap(keygray,keyrows,center_q)
            key_result=estimate(keywarp,current_warp,keymask,current_valid)
            keyms=(time.perf_counter()-tk)*1e3
        if pair.accepted:
            self.accum+=np.array([pair.center_dx,pair.center_dy])
            self.last_vision_ns=f.sensor_ts_ns
        if key_result is not None and key_result.accepted:
            key_observed=np.array([key_result.center_dx,key_result.center_dy])
            key_disagree=float(np.linalg.norm(
                key_observed-(self.accum-self.accum_key)))
        else:key_disagree=None

        # Keyframe buffer can ONLY be replaced with the CURRENT frame. This
        # never sees future photos or inherits unvalidated oracle transforms.
        self.key_age+=1
        reset=self.key is None or self.key_age>=10 or (
            key_result is not None and not key_result.accepted
            and self.key_age>=5)
        if reset:
            self.key=(f,gray,current_rows)
            self.accum_key=self.accum.copy()
            self.key_age=0
            self.key_resets+=1
        self.prev=(f,gray,current_rows)
        elapsed=(time.perf_counter()-start)*1e3
        output={
            "source_id":f.source_id,"sensor_ts_ns":f.sensor_ts_ns,
            "pts_us":f.pts_us,"source_pts_ms":source_pts_ms,
            "gyro_center_timestamp_ns":f.center_gyro_ts_ns,
            "available_deadline_ns":f.availability_deadline_ns,
            "gyro_last_submitted_ns":self.gyro.maximum_submitted_ns,
            "gyro_max_query_ns":self.gyro.max_query_ns,
            "raw_yaw_rad_s":float(imu_omega[1]),
            "prewarp_coverage_ratio":float(np.count_nonzero(current_valid))/current_valid.size,
            "prewarp_current_ms":minwarp_ms,
            "prewarp_prev_ms":pairwarps_ms,
            "LK_pair_ms":pairms,"keyframe_check_ms":keyms,
            "c1_total_ms":elapsed,
            "feature_coordinates":"BUFFER_NATIVE_320x180_PRESTABILIZED",
            "not_applied_to_fullres_warp":True,
            "keyframe_reset_this_frame":reset,
            "keyframe_age":self.key_age,
            "keyframe_trust":key_result.trust if key_result else None,
            "keyframe_status":key_result.status if key_result else None,
            "keyframe_path_disagreement_px":key_disagree,
            "cumulative_short_path_px":self.accum.tolist(),
            **motion,**result_to_record(pair)}
        return output

def parse_case(case:str,max_frames:int|None=None,
               output_root:Path=RESULT_ROOT,write_preview:bool=True):
    if case not in CASES:raise ValueError(case)
    bundle_dir=BUNDLE_ROOT/case
    info=json.loads((bundle_dir/"bundle.json").read_text(encoding="utf-8"))
    frames=[frame_from_meta(x) for x in info["frames"]]
    if len(frames)!=291 or frames[0].source_id!=4 or frames[-1].source_id!=294:
        raise RuntimeError("Source identity dataset assumption failed")
    if max_frames is not None:frames=frames[:int(max_frames)]
    feeder=CausalCSVGyroFeeder(bundle_dir/"gyro_camera.csv")
    source=bundle_dir/"source.mp4"
    cap=cv2.VideoCapture(str(source))
    cap.set(cv2.CAP_PROP_ORIENTATION_AUTO,0)
    if not cap.isOpened():raise RuntimeError("Cannot open original source.mp4")
    observer=C1StreamingObserver(info)
    target=output_root/case
    target.mkdir(parents=True,exist_ok=True)
    trace=target/"frames.jsonl"
    statuses={}
    observations=[]
    t_start=time.perf_counter()
    wanted=0
    skipped=0
    preview_ordinal={57,67,92,114,210,260}
    try:
        with trace.open("w",encoding="utf-8") as f:
            for ordinal in range(frames[-1].source_id+1):
                t0=time.perf_counter()
                ok,raw=cap.read()
                if not ok:raise RuntimeError("Missing raw source frame ordinal")
                pts_ms=float(cap.get(cv2.CAP_PROP_POS_MSEC))
                if ordinal<frames[wanted].source_id:
                    skipped+=1;continue
                frame=frames[wanted]
                if ordinal!=frame.source_id:
                    raise RuntimeError("SourceID physically noncontiguous")
                t1=time.perf_counter()
                gray=resize_native_gray(raw)
                decoder_ms=(t1-t0)*1e3
                new_samples=observer.push_gyro_until_frame_deadline(
                    frame,feeder)
                row=observer.process(frame,gray,pts_ms)
                row["decode_and_resize_ms"]=decoder_ms
                row["gyro_samples_submitted_this_frame"]=new_samples
                row["c1_total_with_decode_ms"]=(
                    row["c1_total_ms"]+decoder_ms)
                row["feature_stream_status"]="HOST_SHADOW_ONLY"
                row["future_video_frame_access"]=False
                row["offline_oracle_used"]=False
                row["noncausal_smoothing_used"]=False
                if row.get("gyro_max_query_ns",0)>frame.availability_deadline_ns:
                    raise RuntimeError("C1 prohibited future Gyro read")
                observations.append(row)
                statuses[row["status"]]=statuses.get(row["status"],0)+1
                f.write(json.dumps(row,ensure_ascii=False,separators=(",",":"))+"\n")
                if write_preview and wanted in preview_ordinal:
                    im,_=observer.warper.remap(
                        gray,pose_rows(frame,observer.gyro),
                        observer.gyro.pose_at(frame.center_gyro_ts_ns))
                    cv2.imwrite(str(target/f"prewarp_source_{frame.source_id}.png"),im)
                wanted+=1
                if wanted%55==0:
                    print("C1_CAUSAL_HOST",case,"frames",wanted,
                          "accepted",statuses.get("ACCEPT",0),
                          "elapsed_sec",round(time.perf_counter()-t_start,1),
                          flush=True)
                if wanted==len(frames):break
    finally:
        cap.release()
        feeder.close()
    if wanted!=len(frames):
        raise RuntimeError(f"Source stream incomplete {wanted} vs {len(frames)}")
    observe=[x for x in observations if "status" in x]
    pairs=[x for x in observe if x["status"] not in ("INIT","NO_GYRO_ROW_COVERAGE")]
    acc=[x for x in observe if x["status"]=="ACCEPT"]
    validts=max((r.get("gyro_max_query_ns",0)<=r["available_deadline_ns"])
                for r in observe)
    if not validts:raise RuntimeError("C1 future IMU access")
    def percentile(k,which=observe,p=90):
        nums=[float(r[k]) for r in which if k in r and r[k] is not None]
        return float(np.percentile(nums,p)) if nums else None
    summary={
      "case":case,"total_source_frames":len(frames),
      "source_indices":[frames[0].source_id,frames[-1].source_id],
      "original_video_codec":"source.mp4 decoded sequentially",
      "source_resolution":[2560,1440],
      "feature_resolution":[320,180],
      "source_pts_verified_each_frame":True,
      "source_sensor_ts_monotonic":True,
      "camera_gyro_offset_ns":8_000_000,
      "gyro_never_pushed_beyond_camera_frame_deadline":True,
      "gyro_max_query_never_beyond_camera_frame_deadline":True,
      "zero_future_camera_frames":True,
      "oracle_matrices_loaded":False,
      "noncausal_gaussian_loaded":False,
      "original_gemini_v1_untouched":True,
      "visual_actuation_enabled":False,
      "status_histogram":statuses,
      "noninit_pairs":len(pairs),
      "accepted_pairs":len(acc),
      "accepted_ratio_of_pairs":len(acc)/len(pairs) if pairs else 0,
      "accepted_dx_abs_p90_320px":percentile("dx_feature_px",acc),
      "accepted_logscale_abs_p90":(
          float(np.percentile([abs(r["logscale"]) for r in acc],90))
          if acc else None),
      "accepted_trust_p50":percentile("trust",acc,50),
      "prewarp_coverage_p10":percentile("prewarp_coverage_ratio",observe,10),
      "LK_pair_host_p90_ms":percentile("LK_pair_ms",observe),
      "prewarp_current_host_p90_ms":percentile("prewarp_current_ms",observe),
      "prewarp_prev_host_p90_ms":percentile("prewarp_prev_ms",observe),
      "C1_host_without_decode_p90_ms":percentile("c1_total_ms",observe),
      "C1_host_with_decode_p90_ms":percentile("c1_total_with_decode_ms",observe),
      "reversal_count":sum(bool(r.get("reversal",False)) for r in observe),
      "keyframe_reset_count":sum(bool(r.get("keyframe_reset_this_frame",False)) for r in observe),
      "processing_elapsed_sec":round(time.perf_counter()-t_start,1),
      "result_path":str(trace),
      "gate_status":"HOST_C1_OBSERVATION_ONLY_NO_EFFECT_CLAIM",
      "android_29p6fps_gate":"NOT_TESTED",
      "c2_target_integration":"OFF"
    }
    (target/"summary.json").write_text(
         json.dumps(summary,ensure_ascii=False,indent=2),encoding="utf-8")
    print("C1_CAUSAL_CASE_RESULT",
         json.dumps({k:summary[k] for k in (
            "case","total_source_frames","accepted_ratio_of_pairs",
            "status_histogram","C1_host_without_decode_p90_ms",
            "C1_host_with_decode_p90_ms","reversal_count",
            "processing_elapsed_sec","gate_status")},ensure_ascii=False),
         flush=True)
    return summary

if __name__=="__main__":
    p=argparse.ArgumentParser()
    p.add_argument("--cases",nargs="+",choices=CASES,default=["024630"])
    p.add_argument("--max-frames",type=int)
    p.add_argument("--out",type=Path,default=RESULT_ROOT)
    p.add_argument("--no-preview",action="store_true")
    a=p.parse_args()
    result=[parse_case(c,a.max_frames,a.out,not a.no_preview) for c in a.cases]
    (a.out/"run_summary.json").write_text(
         json.dumps({"cases":result,"count":len(result)},
                    ensure_ascii=False,indent=2),encoding="utf-8")
