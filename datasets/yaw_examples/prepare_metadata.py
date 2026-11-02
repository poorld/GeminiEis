#!/usr/bin/env python3
"""Build the public, minimal, non-secret yaw research manifest from source logs.

This DOES NOT transcode/replace mp4. User selected two case IDs:
024618 medium bilateral yaw, 024647 one-sided yaw.
Original source MP4s are byte-exact copies of the local EIS-OFF recordings.
"""
from __future__ import annotations
import argparse
import csv
import hashlib
import json
from pathlib import Path
import cv2
import numpy as np

CASES={
    "024618":{"slug":"medium_yaw","label":"Medium-amplitude repeated bilateral yaw"},
    "024647":{"slug":"one_sided_yaw","label":"Final one-sided yaw/return motion"}
}
RAW=Path(r"D:\Yuy\doc\EIS\Yaw")
DATA=Path(__file__).resolve().parent

def sha(path):
    h=hashlib.sha256()
    with path.open("rb") as f:
        for block in iter(lambda:f.read(1<<20),b""):
            h.update(block)
    return h.hexdigest()

def video_meta(path:Path):
    cap=cv2.VideoCapture(str(path))
    if not cap.isOpened():
        raise RuntimeError(f"Cannot open: {path}")
    info={"name":path.name,
          "file_size_bytes":path.stat().st_size,
          "sha256":sha(path),
          "width":int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)),
          "height":int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT)),
          "frame_count":int(cap.get(cv2.CAP_PROP_FRAME_COUNT)),
          "fps_container":round(float(cap.get(cv2.CAP_PROP_FPS)),6)}
    ok,img=cap.read()
    if not ok or img.ndim!=3:raise RuntimeError(f"Undecodable video: {path}")
    info["decoded_first_frame"]=True
    cap.release()
    return info

def contact_sheet(raw,golden,out):
    # Viewer only, not calibrated alignment: the baseline render contains fewer frames
    # than original camera source, and its time/frame relationship needs replay metadata.
    images=[]
    for tag,path in (("EIS OFF SOURCE",raw),("V7 GOLDEN RENDER",golden)):
        cap=cv2.VideoCapture(str(path))
        total=int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
        imgs=[]
        for frac in (.20,.50,.80):
            idx=round(frac*(total-1))
            cap.set(cv2.CAP_PROP_POS_FRAMES,idx)
            ok,im=cap.read()
            if not ok:raise RuntimeError(f"Failed preview {path.name} {idx}")
            im=cv2.resize(im,(640,360),interpolation=cv2.INTER_AREA)
            cv2.rectangle(im,(0,0),(640,41),(12,17,25),-1)
            cv2.putText(im,f"{tag} | ~{frac:.0%} | frame {idx}",(8,28),
                        cv2.FONT_HERSHEY_SIMPLEX,.58,(240,238,233),1,cv2.LINE_AA)
            imgs.append(im)
        cap.release()
        images.append(cv2.hconcat(imgs))
    sheet=cv2.vconcat(images)
    if not cv2.imwrite(str(out),sheet,[cv2.IMWRITE_JPEG_QUALITY,89]):
        raise RuntimeError(f"Writing contact sheet failed {out}")
    return {"name":out.name,"approximate_time_alignment":True,
            "warning":"Frame fractions are illustrative, NOT exact timestamp-matched correspondences"}

def make(case):
    folder=DATA/(case+"_"+CASES[case]["slug"])
    source=folder/(case+"_source_no_eis_2560x1440.mp4")
    golden=folder/(case+"_golden_v7_1920x1080_h264.mp4")
    native=list((RAW/"Movies").glob("*"+case+"*.mp4"))
    d=list((RAW/"rec_diagnostics").glob("20261017_"+case+"_*"))
    if len(native)!=1 or len(d)!=1:raise RuntimeError(f"Ambiguous raw archive source {case}")
    for path in (source,golden):
        if not path.exists():raise RuntimeError(f"Video absent {path}")
    orig_sha=sha(native[0])
    if orig_sha!=sha(source):raise RuntimeError(f"Source MP4 not byte-identical {case}")
    source_meta=video_meta(source);golden_meta=video_meta(golden)
    if (source_meta["width"],source_meta["height"])!=(2560,1440):
        raise AssertionError("Raw source dimension changed")
    if (golden_meta["width"],golden_meta["height"])!=(1920,1080):
        raise AssertionError("Golden output dimension changed")
    if source_meta["frame_count"]<golden_meta["frame_count"]:
        raise AssertionError("Unexpected golden output longer than source")

    meta={}
    frames=[]
    with (d[0]/"trace.jsonl").open("r",encoding="utf-8") as f:
        for line in f:
            rec=json.loads(line)
            if rec.get("type")=="camera_metadata":
                meta[int(rec["sensor_timestamp_ns"])]=rec
            if rec.get("type")=="recorded_frame":
                frames.append(rec)
    frames.sort(key=lambda j:int(j["index"]))
    out_cam=folder/(case+"_camera_metadata.csv")
    columns=["recorded_frame_index","sensor_timestamp_ns","exposure_ns","rolling_shutter_skew_ns","iso","focal_length_mm","active_array_crop_region"]
    matches=0
    with out_cam.open("w",encoding="utf-8",newline="") as fp:
        writer=csv.DictWriter(fp,fieldnames=columns)
        writer.writeheader()
        for rec in frames:
            ts=int(rec["surface_timestamp_ns"])
            dat=meta.get(ts)
            if dat is None:continue
            matches+=1
            writer.writerow({"recorded_frame_index":int(rec["index"]),
                "sensor_timestamp_ns":ts,
                "exposure_ns":int(dat["exposure_ns"]),
                "rolling_shutter_skew_ns":int(dat["rolling_shutter_skew_ns"]),
                "iso":dat.get("iso",""),
                "focal_length_mm":dat.get("focal_length_mm",""),
                "active_array_crop_region":json.dumps(dat.get("crop_region",[]),separators=(",",":"))})
    if matches < 280:raise RuntimeError(f"Too little metadata: {case} matches={matches}")
    sheet=contact_sheet(source,golden,folder/(case+"_preview_contact_sheet.jpg"))
    with (d[0]/"imu.csv").open("r",encoding="utf-8-sig") as f:
        imucount=sum(1 for _ in csv.DictReader(f))
    with (d[0]/"video_pts.csv").open("r",encoding="utf-8-sig") as f:
        ptscount=sum(1 for _ in csv.DictReader(f))
    manifest={"id":case,"label":CASES[case]["label"],
        "original_capture":"YuyEIS2 EIS OFF, 2560x1440, same-source used in V7 golden rendering",
        "source_preserved_byte_exact":True,
        "videos":{"source_eis_off":source_meta,"v7_golden_render":golden_meta},
        "source_gyro_csv":case+"_imu.csv","source_video_pts_csv":case+"_video_pts.csv",
        "source_camera_metadata_csv":out_cam.name,
        "recorded_frame_metadata_matches":matches,"imu_count":imucount,"video_pts_count":ptscount,
        "contact_sheet":sheet,
        "caveats":["Source MP4 and golden render do not contain identical numbers of frames. Do not compare by naive frame index.",
                   "Golden is a reference offline render, not a fresh Y1 live recording or proof of bit-exact device parity.",
                   "Geometry optimization on compressed/re-encoded golden video is discouraged; use the original source MP4 + gyro metadata.",
                   "The contact sheet is for visual inspection; samples were selected at similar fractions of each recording, not strict timestamp alignment."]}
    out_manifest=folder/(case+"_manifest.json")
    out_manifest.write_text(json.dumps(manifest,indent=2,ensure_ascii=False),encoding="utf-8")
    print(case,json.dumps({"source_MB":round(source.stat().st_size/1048576,2),
                           "golden_MB":round(golden.stat().st_size/1048576,2),
                           "source_frames":source_meta["frame_count"],
                           "golden_frames":golden_meta["frame_count"],
                           "camera_metadata_matches":matches,"gyro_rows":imucount,
                           "pts_rows":ptscount,"source_same_sha":True,
                           "contact_sheet_kb":round((folder/(case+"_preview_contact_sheet.jpg")).stat().st_size/1024)},ensure_ascii=False))

if __name__=="__main__":
    parser=argparse.ArgumentParser()
    parser.add_argument("--case",choices=list(CASES)+["all"],default="all")
    args=parser.parse_args()
    for selected in list(CASES) if args.case=="all" else [args.case]:
        make(selected)
