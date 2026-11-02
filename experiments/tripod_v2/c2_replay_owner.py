#!/usr/bin/env python3
"""C2 PC-only causal qVirtual+Visual+Zoom unique-owner SHADOW sweep.

True RAW Camera SourceID4..294 / SensorTS / rolling shutter 20 gyros from
causal C1. C1-B2 Native LK accepted residuals from SAME source/timestamp.
No Oracle, no offline Gemini V1 mesh, no bidirectional smoothing in solver.
V1 is frozen; OFFLINE reference only used by separate real pixel evaluator.
"""
from __future__ import annotations
import argparse,json,time
from pathlib import Path
import numpy as np
from c1_stream_host import ROOT,CASES,BUNDLE_ROOT,frame_from_meta,CausalCSVGyroFeeder,gyro_center_omega
from c1_gyro_rs import CausalGyro,pose_rows
from c2_joint_native import C2Shadow

OUT=ROOT/"reports"/"gemini_tripod_v2"/"c2_shadow"
INOBS=ROOT/"reports"/"gemini_tripod_v2"/"c1b2_yuv"
NAME={False:"c2_geometry_only",True:"c2_joint_visual"}
COMPILED_DLL=None
def run(case:str,max_frames:int|None=None):
    folder=BUNDLE_ROOT/case
    info=json.loads((folder/"bundle.json").read_text(encoding="utf-8"))
    frames=[frame_from_meta(i) for i in info["frames"]]
    assert len(frames)==291 and frames[0].source_id==4 and frames[-1].source_id==294
    if max_frames:frames=frames[:max_frames]
    observations={}
    for line in (INOBS/case/"camera_ready_frames.jsonl").read_text(encoding="utf-8").splitlines():
        row=json.loads(line)
        sid=int(row["source_id"])
        observations[sid]=row
    assert len(observations)==290
    with C2Shadow(False,dll=COMPILED_DLL) as geom,C2Shadow(True,dll=COMPILED_DLL) as visual:
        controllers={False:geom,True:visual}
        gyro=CausalGyro()
        feeder=CausalCSVGyroFeeder(folder/"gyro_camera.csv")
        all_rows={name:[] for name in NAME.values()}
        all_mesh={name:[] for name in NAME.values()}
        try:
            for count,f in enumerate(frames):
                feeder.feed_until(f.availability_deadline_ns,gyro)
                if not all(gyro.coverage(f.gyro_time_at(i/19)) for i in range(20)):
                    raise RuntimeError(f"Missing causal RS19 row on {f.source_id}")
                q=gyro.pose_at(f.center_gyro_ts_ns)
                row_quats=pose_rows(f,gyro)
                # Gyro center rate is sampled only from the already submitted
                # physical-time gyro series (NO future video/gyro).
                rate=gyro_center_omega(f.center_gyro_ts_ns,gyro)
                record=observations.get(f.source_id)
                if count>0:
                    if not record or record["source_id"]!=f.source_id or record["sensor_ts_ns"]!=f.sensor_ts_ns or record["pts_us"]!=f.pts_us:
                        raise RuntimeError("C1-B2 accepted result wrong SourceID / SensorTS / PTS")
                    if not record["same_frame_result_on_time"]:
                        raise RuntimeError("C1B2 visual late: reject, not silently accept")
                    vision=record["visual_output"]
                    visual_input={**vision,"accepted":record["accepted_native_global_model"]}
                else:visual_input=None
                for mode,c in controllers.items():
                    out,mesh,target=c.step(f.source_id,f.sensor_ts_ns,
                                           q,row_quats,float(rate[1]),visual_input)
                    label=NAME[mode]
                    all_mesh[label].append(mesh.astype(np.float32))
                    all_rows[label].append({
                      "source_id":f.source_id,"pts_us":f.pts_us,
                      "sensor_ts_ns":f.sensor_ts_ns,
                      "gyro_max_query_ns":gyro.max_query_ns,
                      "physical_deadline_ns":f.availability_deadline_ns,
                      "q_camera_xyzw":q.tolist(),
                      "q_virtual_xyzw":target.tolist(),
                      "yaw_rad_s":float(rate[1]),
                      "qVirtual_Visual_Zoom_single_owner":True,
                      "valid_C1B2_source":f.source_id==4 or vision is not None,
                      "decision_reason":out.reason,
                      "used_visual":bool(out.visual_used),
                      "tested_full_RS19_alternatives":out.alternatives_tested,
                      "geometry_veto_candidates":out.rejected_safety,
                      "zoom":out.zoom,
                      "visual_tx_source_px":out.visual_tx_source_px,
                      "visual_ty_source_px":out.visual_ty_source_px,
                      "visual_rotation_rad":out.visual_rotation_rad,
                      "visual_scale":out.visual_scale,
                      "margin_source_px":out.min_margin_source_px,
                      "mesh_center_x_source_px":out.center_x_source_px,
                      "mesh_center_y_source_px":out.center_y_source_px,
                      "mesh_span_source_px":out.span_source_px,
                      "qVirtual_step_rad":out.target_step_rad,
                      "mesh_span_step_px":out.previous_mesh_span_step_px,
                      "predicted_visual_residual_feature_px":out.predicted_remaining_visual_px320,
                      "C2_native_solve_ms":out.solve_ms,
                      "future_frame_used":False,
                      "oracle_input_used":False,
                      "golden_input_used":False,
                      "rs19_full_mesh_vertices":660,
                      "final_render_warps":1
                    })
                if (count+1)%58==0:
                    print("C2_JOINT_OWNER_TRUE_RAW",case,count+1,
                          "geom_margin",round(all_rows[NAME[False]][-1]["margin_source_px"],1),
                          "visual_margin",round(all_rows[NAME[True]][-1]["margin_source_px"],1),
                          flush=True)
        finally:feeder.close()
    dest=OUT/case
    dest.mkdir(parents=True,exist_ok=True)
    s={}
    for name in NAME.values():
        rows=all_rows[name]
        meshes=np.asarray(all_mesh[name],dtype=np.float32)
        assert len(rows)==len(frames) and meshes.shape==(len(frames),660,2)
        assert np.isfinite(meshes).all()
        if not all(r["margin_source_px"]>=2.0 for r in rows):
            raise RuntimeError("C2 violates RS19 physical margin")
        meshpath=dest/(name+"_mesh.f32")
        meshes.tofile(meshpath)
        trace=dest/(name+"_frames.jsonl")
        trace.write_text("".join(json.dumps(r,ensure_ascii=False,separators=(",",":"))+"\n" for r in rows),encoding="utf-8")
        def p90(key,absval=False):
            values=np.asarray([r[key] for r in rows],float)
            if absval:values=np.abs(values)
            return float(np.percentile(values,90))
        def maxabs(key):
            return float(np.max(np.abs([r[key] for r in rows])))
        v={
            "case":case,"variant":name,"source_frames":len(rows),
            "SourceID_range":[rows[0]["source_id"],rows[-1]["source_id"]],
            "full_real_20x33_RS19_gate":"HOST_PASS",
            "min_margin_source_px":min(r["margin_source_px"] for r in rows),
            "max_zoom":max(r["zoom"] for r in rows),
            "min_zoom":min(r["zoom"] for r in rows),
            "max_zoom_step_percent":100*max(abs(np.diff(np.log([r["zoom"] for r in rows])))) if len(rows)>1 else 0,
            "visual_frames_used":sum(r["used_visual"] for r in rows),
            "qVirtual_step_rad_p90":p90("qVirtual_step_rad",True),
            "qVirtual_step_rad_max":maxabs("qVirtual_step_rad"),
            "edge_span_step_source_px_p90":p90("mesh_span_step_px",True),
            "edge_span_step_source_px_max":maxabs("mesh_span_step_px"),
            "C2_native_solve_p90_ms":p90("C2_native_solve_ms"),
            "C2_native_solve_max_ms":maxabs("C2_native_solve_ms"),
            "all_frames_exact_SourceID_TS_and_RS":True,
            "input_C1B2_same_frame_causal":True,
            "input_offline_oracle_or_golden_as_control":False,
            "qVirtual_Visual_Zoom_at_once":True,
            "Gemini_V1_Golden_untouched":True,
            "Android_Device":"NOT_TESTED",
            "real_1080p_visual_pixel_effect":"NOT_TESTED",
            "mesh_path":str(meshpath),"frames_path":str(trace)
        }
        (dest/(name+"_summary.json")).write_text(
            json.dumps(v,ensure_ascii=False,indent=2),encoding="utf-8")
        s[name]=v
        print("C2_JOINT_CAUSAL_HOST_GEOMETRY",json.dumps({k:val for k,val in v.items() if k not in ("frames_path","mesh_path")},ensure_ascii=False),flush=True)
    return s

def main():
    a=argparse.ArgumentParser()
    a.add_argument("--cases",nargs="+",choices=CASES,default=list(CASES))
    a.add_argument("--max-frames",type=int)
    a.add_argument("--compiled-dll",type=Path,default=None)
    a.add_argument("--output-root",type=Path,default=None)
    arg=a.parse_args()
    global OUT,COMPILED_DLL
    if arg.output_root is not None:OUT=arg.output_root
    COMPILED_DLL=arg.compiled_dll
    results={case:run(case,arg.max_frames) for case in arg.cases}
    (OUT/"c2_geometry_all.json").write_text(
      json.dumps({"cases":results,"phase":"CAUSAL_HOST_SHADOW_FULL_RS19",
        "actual_pixel_gate":"NOT_YET_EVALUATED",
        "C2_does_not_use_frozen_Golden_for_control":True},
                 ensure_ascii=False,indent=2),encoding="utf-8")
if __name__=="__main__":main()
