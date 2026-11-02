#!/usr/bin/env python3
"""Full-image evidence for the V2 experimental SINGLE C++ target owner.
Uses already verified raw+SourceID RS19 H264 renderer and final pixel LK.
Never overwrites the earlier seven-candidate V2 benchmark.
"""
from __future__ import annotations
import argparse,csv,json,re,shutil
from pathlib import Path
from render_and_evaluate import ROOT, OUT, render, official_and_regions

VAR="unified_joint"
NAMES=["event","sensor_ts_ns","reason","attempted","accepted",
       "margin_px","margin_new_px","span_source_px","span_new_source_px",
       "zoom","zoom_new","scale","scale_new","gyro_yaw_rad_s",
       "center_shift_source_px"]

def prepare(case: str) -> dict:
    directory=OUT/case/VAR
    mesh=directory/"mesh_joint.f32"
    if not mesh.exists():raise FileNotFoundError(mesh)
    shutil.copyfile(mesh,directory/"mesh_governed.f32")
    diag=directory/"budget_trace.csv"
    with diag.open(encoding="utf-8",newline="") as f:
        rows=list(csv.DictReader(f,fieldnames=NAMES))
    rows=[r for r in rows if r["event"]=="V2_BUDGET"]
    if len(rows)!=291:raise RuntimeError(f"Expected exactly 291 diagnostic frames {len(rows)}")
    manifest={"case":case,"variant":VAR,"architecture":"UNIFIED_SINGLE_CPP_TARGET_OWNER",
      "algorithm":("single C++ core, candidate edits output only, V1 Zoom state bitexact"
                   if VAR=="unified_output_only" else
                   "single C++ core, RS19 exact precommit, temporal zoom+scale geometry feedback"),
      "input_original_source_index":[4,294],
      "upstream_offline_oracle_noncausal":True,
      "V2_component_current_and_past_only":True,
      "probe_accepted_frames":sum(r["accepted"]=="1" for r in rows),
      "probe_attempted_frames":sum(int(r["attempted"])>0 for r in rows),
      "all_rs_mesh_20x33":True,
      "zero_additional_future_frames":True}
    (directory/"manifest.json").write_text(json.dumps(manifest,indent=2),encoding="utf-8")
    return manifest

def main():
    global VAR
    p=argparse.ArgumentParser()
    p.add_argument("--variant",default="unified_joint")
    p.add_argument("--cases",nargs="+",default=["024630"])
    p.add_argument("--no-render",action="store_true")
    args=p.parse_args()
    VAR=args.variant
    if VAR not in ("unified_joint","unified_output_only"):raise ValueError(VAR)
    for case in args.cases:
        manifest=prepare(case)
        if not args.no_render:render(case,[VAR])
        root=ROOT/"rendered_preview_fixed"/(case+"_GEMINI_STABLE.mp4")
        v2=OUT/case/VAR/"gemini_v2_output.mp4"
        gold=official_and_regions(root)
        experiment=official_and_regions(v2)
        limits=["official_motion_p90_px","center_dx_abs_p90",
                "edge_antisymmetric_abs_p90","edge_antisymmetric_max_abs"]
        changes={k:experiment[k]-gold[k] for k in limits}
        # For final promotion all four videos, each must pass independently.
        test=(experiment["edge_antisymmetric_max_abs"]<
               gold["edge_antisymmetric_max_abs"]*.5 and
              all(experiment[k]<=gold[k]*1.05 for k in limits[:3]))
        outcome={"case":case,"GOLDEN":gold,"UNIFIED_V2":experiment,
                 "delta":changes,"candidate_single_case_accept":bool(test),
                 "runtime_manifest":manifest,"FINAL_ANDROID_REALTIME":False}
        (OUT/case/(VAR+"_real_pixel_comparison.json")).write_text(
            json.dumps(outcome,ensure_ascii=False,indent=2),encoding="utf-8")
        print("V2_UNIFIED_TRUE_PIXEL",json.dumps({"case":case,"variant":VAR,
           "accepted":manifest["probe_accepted_frames"],
           "delta":changes,"GOLDEN_GATE":test,
           "v2_overall_motion_p90":experiment["official_motion_p90_px"],
           "v2_center_p90":experiment["center_dx_abs_p90"],
           "v2_edge_p90":experiment["edge_antisymmetric_abs_p90"],
           "v2_edge_peak":experiment["edge_antisymmetric_max_abs"]},
             ensure_ascii=False),flush=True)

if __name__=="__main__":main()
