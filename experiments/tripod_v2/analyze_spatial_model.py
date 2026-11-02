#!/usr/bin/env python3
"""Held-out pixel test: can 4DoF/8DoF explain leftover edge motion?

No model is applied to video here. Same raw scene final H264 only.
Train on spatially interleaved corners, report error on independent
spatial held-out points to avoid fit-overreach masquerading as improvement.
"""
from __future__ import annotations
import argparse,json,math
from pathlib import Path
import cv2
import numpy as np
ROOT=Path(__file__).resolve().parents[2]
OUT=ROOT/"reports"/"gemini_tripod_v2"
cv2.setNumThreads(3)

def stats(xs):
    a=np.asarray(xs,float)
    return {"n":len(a),"median_px":float(np.median(a)) if len(a) else None,
            "p90_px":float(np.percentile(a,90)) if len(a) else None}
def interpret_video(path,samples):
    video=cv2.VideoCapture(str(path))
    frames={}
    need={t for i in samples for t in (i-1,i)}
    for i in range(max(need)+1):
        ok,frame=video.read()
        if not ok:raise RuntimeError("video ended early "+str(path))
        if i in need:
            frames[i]=cv2.cvtColor(cv2.resize(frame,(640,360),interpolation=cv2.INTER_AREA),
                                     cv2.COLOR_BGR2GRAY)
    video.release()
    records=[]
    for now in samples:
        old=frames[now-1];new=frames[now]
        mask=np.zeros((360,640),np.uint8)
        mask[35:325,20:620]=255
        points=cv2.goodFeaturesToTrack(old,maxCorners=1050,qualityLevel=.008,
                                         minDistance=5,mask=mask,blockSize=5)
        if points is None:continue
        q,good,_=cv2.calcOpticalFlowPyrLK(old,new,points,None,
                              winSize=(21,21),maxLevel=3)
        back,good2,_=cv2.calcOpticalFlowPyrLK(new,old,q,None,
                              winSize=(21,21),maxLevel=3)
        pos=points.reshape(-1,2);nxt=q.reshape(-1,2);b=back.reshape(-1,2)
        ok=(good.ravel()>0)&(good2.ravel()>0)&(np.linalg.norm(pos-b,axis=1)<.8)
        pos=pos[ok];nxt=nxt[ok]
        if len(pos)<75:continue
        # Spatial interleave, not random identical-location split.
        bucket=(np.floor(pos[:,0]/65).astype(int)+np.floor(pos[:,1]/55).astype(int))%2
        train=bucket==0;test=~train
        if train.sum()<35 or test.sum()<35:continue
        if len({int(x//200) for x in pos[train,0]})<3:continue
        aff,_=cv2.estimateAffinePartial2D(pos[train],nxt[train],method=cv2.RANSAC,
                                   ransacReprojThreshold=1.25,maxIters=1000)
        H,_=cv2.findHomography(pos[train],nxt[train],cv2.RANSAC,1.25,
                               maxIters=1000,confidence=.98)
        if aff is None or H is None or not np.isfinite(H).all():continue
        pts=np.c_[pos[test],np.ones(int(test.sum()))]
        a=pts@aff.T
        h=pts@H.T
        valid=np.abs(h[:,2])>1e-5
        if valid.sum()<30:continue
        hxy=h[valid,:2]/h[valid,2,None]
        target=nxt[test][valid]
        xy=pos[test][valid]
        e0=np.linalg.norm(a[valid]-target,axis=1)*2
        e1=np.linalg.norm(hxy-target,axis=1)*2
        zones={"all":np.ones(len(xy),bool),
               "left":xy[:,0]<640*.30,
               "center":(xy[:,0]>640*.35)&(xy[:,0]<640*.65),
               "right":xy[:,0]>640*.70}
        row={"output_frame":now,"all_matched_lk":len(pos),
             "train_points":int(train.sum()),"held_out_points":len(xy),
             "per_zone":{}}
        for name,zone in zones.items():
            if zone.sum()<8:continue
            row["per_zone"][name]={"4dof":stats(e0[zone]),
                                   "8dof":stats(e1[zone])}
        records.append(row)
    return records

def aggregate(records):
    acc={}
    for row in records:
        for zone,d in row["per_zone"].items():
            for typ,v in d.items():
                if v["n"]<8:continue
                acc.setdefault(zone,{}).setdefault(typ,[]).append(v["p90_px"])
    return {zone:{typ:stats(values) for typ,values in bytyp.items()}
            for zone,bytyp in acc.items()}

if __name__=="__main__":
    parser=argparse.ArgumentParser()
    parser.add_argument("--case",default="024630")
    parser.add_argument("--variants",nargs="+",default=["GOLDEN","floor108_slew0p0015",
                               "floor110_slew0p0015"])
    args=parser.parse_args()
    samples=sorted(set(list(range(50,77))+list(range(89,120))+
                       list(range(127,150))+list(range(199,214))))
    versions={}
    for var in args.variants:
        path=(ROOT/"rendered_preview_fixed"/f"{args.case}_GEMINI_STABLE.mp4"
              if var=="GOLDEN" else OUT/args.case/var/"gemini_v2_output.mp4")
        if not path.exists():raise FileNotFoundError(path)
        rows=interpret_video(path,samples)
        versions[var]={"analyzed_frame_pairs":len(rows),
                       "train_test_spatially_disjoint":True,
                       "agg":aggregate(rows),
                       "extrema":{str(i):next((r for r in rows if r["output_frame"]==i),None)
                                  for i in (56,57,66,67,68,69,92,93,114)},
                       "rows":rows}
        print("V2_SPATIAL_MODEL_DIAGNOSTIC",var,
              json.dumps({"n":len(rows),"by_zone":versions[var]["agg"],
                          "frame67":versions[var]["extrema"]["67"]},
                         ensure_ascii=False),flush=True)
    dest=OUT/args.case/"spatial_4dof_vs_8dof_model_heldout.json"
    dest.write_text(json.dumps({"method":"final encoded MP4 held-out LK points",
        "nonrigid_claim":"NOT_GRANTED_BY_HOMOGRAPHY_IMPROVEMENT_ALONE",
        "live_realtime_parity":"NOT_TESTED",
        "versions":versions},ensure_ascii=False,indent=2),encoding="utf-8")
    print("V2_SPATIAL_MODEL_EVIDENCE",dest)
