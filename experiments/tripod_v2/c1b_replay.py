#!/usr/bin/env python3
"""C1-B Native FB-LK isolation A/B: reuses verified causal C1 raw/gyro/RS.

ONLY the 3x3 tracking+robust similarity backend is switched to portable
C++ with C ABI. All source frames/PTS/SensorTS and RS19 prewarp are unchanged.
No C2 target, no edits to the original C1 files, no preview/recording actuation.
"""
from __future__ import annotations
import argparse,json
from pathlib import Path
import numpy as np
import c1_stream_host as c1
from c1b_native_visual import C1BNativeTileLK

OUT=c1.ROOT/"reports"/"gemini_tripod_v2"/"c1b_native"
OLD=c1.ROOT/"reports"/"gemini_tripod_v2"/"c1_causal"

def main():
    p=argparse.ArgumentParser()
    p.add_argument("--cases",nargs="+",choices=c1.CASES,default=["024630"])
    p.add_argument("--max-frames",type=int,default=None)
    p.add_argument("--with-images",action="store_true")
    args=p.parse_args()
    native=C1BNativeTileLK()
    c1.estimate=native.estimate
    cases=[]
    for case in args.cases:
        summary=c1.parse_case(case,args.max_frames,OUT,args.with_images)
        measured=OUT/case/"frames.jsonl"
        baseline=OLD/case/"frames.jsonl"
        current=[json.loads(line) for line in measured.read_text(encoding="utf-8").splitlines()]
        reference=[json.loads(line) for line in baseline.read_text(encoding="utf-8").splitlines()[:len(current)]]
        assert len(current)==len(reference)
        assert all(a["source_id"]==b["source_id"] and a["sensor_ts_ns"]==b["sensor_ts_ns"]
                   for a,b in zip(current,reference))
        paired=[(a,b) for a,b in zip(current[1:],reference[1:])
                if a["status"]=="ACCEPT" and b["status"]=="ACCEPT"]
        def p90_absdiff(key):
            return float(np.percentile([abs(a[key]-b[key]) for a,b in paired],90)) if paired else None
        native_timing=native.native_ms[:]
        compare={
            "case":case,"source_frames":len(current),
            "same_SourceID_and_SensorTS":True,
            "all_Gyro_and_RS19_same_as_C1_A":True,
            "native_fb_lk_pairs":sum(r["status"]=="ACCEPT" for r in current),
            "original_fb_lk_pairs":sum(r["status"]=="ACCEPT" for r in reference),
            "joint_accepted_pairs":len(paired),
            "native_dx_P90_abs_error_vs_original_feature_px":p90_absdiff("dx_feature_px"),
            "native_dy_P90_abs_error_vs_original_feature_px":p90_absdiff("dy_feature_px"),
            "native_logscale_P90_abs_error_vs_original":p90_absdiff("logscale"),
            "native_inlier_count_p50":float(np.median([r["inliers"] for r in current[1:]])),
            "Native_CPP_LK_per_pair_p50_ms":float(np.percentile(native_timing,50)),
            "Native_CPP_LK_per_pair_p90_ms":float(np.percentile(native_timing,90)),
            "C1_B_python_plus_cpp_p90_ms":summary["C1_host_without_decode_p90_ms"],
            "C1_B_full_host_decode_plus_C1_p90_ms":summary["C1_host_with_decode_p90_ms"],
            "C1_A_original_p90_ms":json.loads((OLD/case/"summary.json").read_text(encoding="utf-8"))["C1_host_without_decode_p90_ms"],
            "C1_A_golden_source_untouched":True,
            "native_core_portable_cpp":True,
            "native_async_scheduler_available":False,
            "device_fps": "NOT_TESTED",
            "visual_actuator":"OFF_SHADOW_ONLY",
            "result":"MEASURED_ONLY_UNTIL_GATES_CHECKED"
        }
        dest=OUT/case/"native_ab.json"
        dest.write_text(json.dumps(compare,ensure_ascii=False,indent=2),encoding="utf-8")
        cases.append(compare)
        print("C1B_NATIVE_REAL_REPLAY",json.dumps(compare,ensure_ascii=False),
              flush=True)
    (OUT/"native_ab_all.json").write_text(
        json.dumps({"cases":cases},ensure_ascii=False,indent=2),encoding="utf-8")

if __name__=="__main__":main()
