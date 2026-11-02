#!/usr/bin/env python3
"""V2C C1-B2: true source YUV420p luma -> Native Y downsample -> C++ RS19
prewarp -> bounded C++ async 3x3 FB LK. All 291 SourceIDs are preserved.

CRITICAL TIMING DISTINCTION:
  HOST_SOFTWARE_MP4_DECODE is measured OUTSIDE the modeled Camera-ready clock.
  CAMERA_READY is the instant cv2 returns an actual unconverted decoder Y
  plane. Treating this arrival as a Camera YUV_420_888 callback is a *model*,
  NOT an Android device result. Current-frame Gyro queries use only causal
  CameraTS/row deadlines. 33.78ms after ready is a HOST upper-bound budget,
  not a measured physical GPU Warp-submission deadline.

No offline Oracle, no future image, no C2 actuation, no product V1 edits.
"""
from __future__ import annotations
import argparse,json,math,time
from pathlib import Path
import numpy as np,cv2
from c1_stream_host import (
    ROOT,BUNDLE_ROOT,CASES,frame_from_meta,CausalCSVGyroFeeder)
from c1_gyro_rs import CausalGyro,pose_rows
from c1b_native_async import NativeAsync
from c1b2_yuv_native import C1B2Yuv

OUT=ROOT/"reports"/"gemini_tripod_v2"/"c1b2_yuv"
BASE=ROOT/"reports"/"gemini_tripod_v2"/"c1b_native"
REF=ROOT/"reports"/"gemini_tripod_v2"/"c1_causal"
PERIOD_NS=round(1e9/29.605)

def pctl(rows,key,p=90):
    vals=[float(r[key]) for r in rows if r.get(key) is not None]
    return float(np.percentile(vals,p)) if vals else None

def run(case:str,max_frames:int|None=None,budget_ms:float=33.78):
    root=BUNDLE_ROOT/case
    meta=json.loads((root/"bundle.json").read_text(encoding="utf-8"))
    frames=[frame_from_meta(f) for f in meta["frames"]]
    if len(frames)!=291 or frames[0].source_id!=4 or frames[-1].source_id!=294:
        raise RuntimeError("Unknown source frame mapping")
    if max_frames:frames=frames[:max_frames]
    cap=cv2.VideoCapture(str(root/"source.mp4"))
    if not cap.isOpened():raise RuntimeError("Missing original video")
    cap.set(cv2.CAP_PROP_ORIENTATION_AUTO,0)
    cap.set(cv2.CAP_PROP_CONVERT_RGB,0)
    if cap.get(cv2.CAP_PROP_CONVERT_RGB)!=0.:
        raise RuntimeError("Decoder cannot provide raw native Y plane")
    gyro=CausalGyro()
    feeder=CausalCSVGyroFeeder(root/"gyro_camera.csv")
    native=C1B2Yuv()
    worker=NativeAsync()
    fx=float(meta["fx"])*320/2560
    fy=float(meta["fy"])*180/1440
    cx=float(meta["cx"])*320/2560
    cy=float(meta["cy"])*180/1440
    old_ref=[json.loads(x) for x in
             (BASE/case/"frames.jsonl").read_text(encoding="utf-8").splitlines()]
    assert len(old_ref)==291
    records=[]
    prev=None
    frame_index=0
    try:
        for ordinal in range(frames[-1].source_id+1):
            started_ns=worker.now_ns()
            ok,plane=cap.read()
            if not ok:raise RuntimeError(f"SourceID {ordinal}: software decode failure")
            decoded_ns=worker.now_ns()
            pts_ms=float(cap.get(cv2.CAP_PROP_POS_MSEC))
            if ordinal<4:continue
            current=frames[frame_index]
            if ordinal!=current.source_id or plane.shape!=(1440,2560) or plane.dtype!=np.uint8:
                raise RuntimeError("Camera native Y plane format or frame ordinal mismatch")
            if abs(pts_ms-current.pts_us*.001)>1.1:
                raise RuntimeError("SourceID -> encoded PTS mismatch")
            if prev and current.sensor_ts_ns<=prev[0].sensor_ts_ns:
                raise RuntimeError("Noncausal/duplicate SensorTS")
            camera_ready_ns=decoded_ns
            small=native.downsample_y(plane)
            t_down=worker.now_ns()
            fed=feeder.feed_until(current.availability_deadline_ns,gyro)
            if not all(gyro.coverage(current.gyro_time_at(k/19)) for k in range(20)):
                raise RuntimeError("Gyro cannot cover true physical RS row")
            rows=pose_rows(current,gyro)
            center=gyro.pose_at(current.center_gyro_ts_ns)
            t_gyro=worker.now_ns()
            if prev:
                earlier,prev_small,prev_rows=prev
                if current.source_id!=earlier.source_id+1:
                    raise RuntimeError("Historical frame order broken")
                source_prev,valid_prev=native.prewarp(
                    prev_small,prev_rows,center,fx,fy,cx,cy)
                t_prev=worker.now_ns()
                source_cur,valid_cur=native.prewarp(
                    small,rows,center,fx,fy,cx,cy)
                t_cur=worker.now_ns()
                deadline=camera_ready_ns+int(budget_ms*1e6)
                submit=worker.submit(current.source_id,current.sensor_ts_ns,
                                     deadline,source_prev,source_cur,
                                     valid_prev,valid_cur)
                t_submitted=worker.now_ns()
                status,res=(worker.wait(current.source_id,deadline)
                            if submit==0 else (3,None))
                decided_ns=worker.now_ns()
                on_time=submit==0 and status==0
                observation=on_time and int(res.status)==0
                result={
                    "source_id":current.source_id,
                    "sensor_ts_ns":current.sensor_ts_ns,
                    "pts_us":current.pts_us,
                    "Y_plane_source":"OpenCV_FFmpeg_decoder_yuv420p_raw_8UC1",
                    "camera_ready_proxy":"end_of_software_decode_not_real_camera_callback",
                    "Y_plane_row_stride":int(plane.strides[0]),
                    "Y_plane_pixel_stride":int(plane.strides[1]),
                    "SourceID_and_PTS_verified":True,
                    "gyro_samples_submitted":fed,
                    "gyro_largest_submitted_ts_ns":gyro.maximum_submitted_ns,
                    "gyro_max_queried_ts_ns":gyro.max_query_ns,
                    "physical_available_deadline_ns":current.availability_deadline_ns,
                    "frame_deadline_proxy_ns":deadline,
                    "budget_ms":budget_ms,
                    "strict_worker_submit":submit,"strict_worker_poll":status,
                    "same_frame_result_on_time":bool(on_time),
                    "accepted_native_global_model":bool(observation),
                    "late_visual_discarded":bool(not on_time),
                    "old_frame_visual_reused":False,
                    "visual_output":{
                        "dx_feature_px":float(res.center_dx),
                        "dy_feature_px":float(res.center_dy),
                        "rotation_rad":float(res.rotation_rad),
                        "logscale":float(res.logscale),
                        "trust":float(res.trust),
                        "inliers":int(res.inliers),
                        "native_LK_core_ms":float(res.elapsed_ms)}
                    if on_time else None,
                    "host_MP4_software_decode_ms":(decoded_ns-started_ns)*1e-6,
                    "native_y_area8_ms":(t_down-camera_ready_ns)*1e-6,
                    "host_gyro_pose_ms":(t_gyro-t_down)*1e-6,
                    "native_prewarp_previous_ms":(t_prev-t_gyro)*1e-6,
                    "native_prewarp_current_ms":(t_cur-t_prev)*1e-6,
                    "native_async_submit_ms":(t_submitted-t_cur)*1e-6,
                    "native_async_wait_ms":(decided_ns-t_submitted)*1e-6,
                    "camera_ready_to_gate_decision_ms":(decided_ns-camera_ready_ns)*1e-6,
                    "host_decode_start_to_gate_decision_ms":(decided_ns-started_ns)*1e-6,
                    "under_20ms_from_camera_ready":bool(on_time and
                        decided_ns-camera_ready_ns<=20_000_000),
                    "under_33p78ms_from_camera_ready":bool(on_time and
                        decided_ns-camera_ready_ns<=33_780_000),
                    "C2_target_actuation":False,
                    "final_fullres_GPU_warp_included":False,
                    "Android_real_camera_used":False,
                    "offline_oracle_used":False
                }
                assert gyro.max_query_ns<=current.availability_deadline_ns
                records.append(result)
            prev=(current,small,rows)
            frame_index+=1
            if frame_index%60==0:
                print("C1B2_CAMERA_READY_HOST",case,"frames",frame_index,
                    "accepted_current",sum(int(r["same_frame_result_on_time"]) for r in records),
                    "observations",sum(int(r["accepted_native_global_model"]) for r in records),
                    flush=True)
        if frame_index!=len(frames):raise RuntimeError("Physical source truncated")
        stats=worker.stats()
    finally:
        feeder.close()
        worker.close()
        cap.release()
    folder=OUT/case
    folder.mkdir(parents=True,exist_ok=True)
    trace=folder/"camera_ready_frames.jsonl"
    trace.write_text("".join(json.dumps(r,ensure_ascii=False,separators=(",",":"))+"\n"
                             for r in records),encoding="utf-8")
    if len(records)!=len(frames)-1:raise RuntimeError("Not exactly one pair per SourceID")
    errors=[]
    for rec in records:
        if rec["accepted_native_global_model"]:
            base=old_ref[rec["source_id"]-4]
            if base["source_id"]!=rec["source_id"]:raise RuntimeError("Ref identity mismatch")
            mine=rec["visual_output"]
            errors.append({
                "source_id":rec["source_id"],
                "dx_abs_error":abs(mine["dx_feature_px"]-base["dx_feature_px"]),
                "dy_abs_error":abs(mine["dy_feature_px"]-base["dy_feature_px"]),
                "rotation_abs_error":abs(mine["rotation_rad"]-base["rotation_rad"]),
                "logscale_abs_error":abs(mine["logscale"]-base["logscale"])})
    accepted=sum(r["same_frame_result_on_time"] for r in records)
    valid=sum(r["accepted_native_global_model"] for r in records)
    report={
      "case":case,"source_frames":len(frames),
      "original_Y_plane_input":"FFmpeg_yuv420p_raw_luma_after_software_decode",
      "original_camera_raw_size":[2560,1440],
      "feature_size":[320,180],
      "real_Android_callback_used":False,
      "Y_plane_based_native_area8_and_full_rs19_feature_prewarp":True,
      "same_SourceID_SensorTS_PTS_all_pairs":True,
      "strict_Host_frame_ready_budget_ms":budget_ms,
      "same_frame_current_worker_ready":accepted,
      "same_frame_current_native_model_accepted":valid,
      "accepted_fraction":accepted/len(records),
      "native_global_valid_fraction":valid/len(records),
      "late_visual_refused":len(records)-accepted,
      "missing_Camera_Gyro_coverage":0,
      "source_gyro_consistency":all(
          r["gyro_max_queried_ts_ns"]<=r["physical_available_deadline_ns"]
          for r in records),
      "C2_output_disabled":True,
      "camera_ready_to_gate_p50_ms":pctl(records,"camera_ready_to_gate_decision_ms",50),
      "camera_ready_to_gate_p90_ms":pctl(records,"camera_ready_to_gate_decision_ms",90),
      "camera_ready_to_gate_p99_ms":pctl(records,"camera_ready_to_gate_decision_ms",99),
      "MP4_software_decode_p90_ms":pctl(records,"host_MP4_software_decode_ms",90),
      "native_Y_area8_p90_ms":pctl(records,"native_y_area8_ms",90),
      "gyro_python_p90_ms":pctl(records,"host_gyro_pose_ms",90),
      "two_native_RS19_p90_ms":float(np.percentile([
          r["native_prewarp_previous_ms"]+r["native_prewarp_current_ms"]
          for r in records],90)),
      "C1B2_async_wait_p90_ms":pctl(records,"native_async_wait_ms",90),
      "native_LK_core_p90_ms":pctl([
         {"core":r["visual_output"]["native_LK_core_ms"]}
         for r in records if r["visual_output"] is not None],"core",90),
      "HOST_FULL_MP4_decode_to_result_p90_ms":pctl(records,"host_decode_start_to_gate_decision_ms",90),
      "Y_plane_vs_old_OpenCV_native_dx_abs_error_p90":pctl(errors,"dx_abs_error",90),
      "Y_plane_vs_old_OpenCV_native_logscale_abs_error_p90":pctl(errors,"logscale_abs_error",90),
      "camera_ready_worker_20ms_passes":sum(r["under_20ms_from_camera_ready"] for r in records),
      "camera_ready_worker_33p78ms_passes":sum(r["under_33p78ms_from_camera_ready"] for r in records),
      "worker_stats":stats,
      "HOST_camera_ready_33p78ms_gate":"PASS" if accepted==len(records) and valid==len(records) else "FAIL_CLOSED",
      "HOST_full_software_decode_33p78ms_gate":"NOT_EQUIVALENT_TO_CAMERA",
      "Y1_DEVICE_30fps":"NOT_TESTED",
      "Gemini_V1_golden_unchanged":True,
      "FULL_QVIRTUAL_VISUAL_ZOOM_C2":"NOT_INTEGRATED"
    }
    (folder/"summary.json").write_text(
        json.dumps(report,ensure_ascii=False,indent=2),encoding="utf-8")
    print("C1B2_NATIVE_REAL_Y_PLANE_CAMERA_READY",
          json.dumps({k:v for k,v in report.items() if k not in (
            "worker_stats",)},ensure_ascii=False),flush=True)
    return report

def main():
    p=argparse.ArgumentParser()
    p.add_argument("--cases",nargs="+",choices=CASES,default=list(CASES))
    p.add_argument("--max-frames",type=int)
    p.add_argument("--budget-ms",type=float,default=33.78)
    p.add_argument("--output-root",type=Path,default=None,
                   help="Use a separate directory for shortened prefix replays")
    a=p.parse_args()
    global OUT
    if a.output_root is not None:OUT=a.output_root
    summaries=[run(case,a.max_frames,a.budget_ms) for case in a.cases]
    OUT.mkdir(parents=True,exist_ok=True)
    (OUT/"all_cases.json").write_text(
        json.dumps({"cases":summaries,"mode":"CAMERA_READY_Y_PLANE_HOST_ONLY",
                    "full_real_camera_zero_lookahead_device":"NOT_TESTED"},
                   ensure_ascii=False,indent=2),encoding="utf-8")
if __name__=="__main__":main()
