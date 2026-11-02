#!/usr/bin/env python3
"""V2C C1: center-weighted, scene-agnostic 3x3 tiled LK global residual.

Only current and past images. This module DOES NOT move the virtual camera
or pass raw gyro-correlated visual deltas into the original V1 controller.
All displacements are in 320x180 BUFFER-NATIVE feature-space pixels, after
physically modeled Gyro+RS19 current-frame stabilization.

No target screen ROI, no 8DoF, no future frames, no full-sequence smoothing.
"""
from __future__ import annotations
from dataclasses import dataclass,field
import math
import numpy as np
import cv2

W,H=320,180
COLS=ROWS=3

@dataclass
class PairResult:
    status:str="INIT"
    trust:float=0.
    checked:int=0
    tracked:int=0
    inliers:int=0
    support_tiles:int=0
    support_cols:int=0
    center_dx:float=0.
    center_dy:float=0.
    rotation_rad:float=0.
    logscale:float=0.
    global_fb_p90:float=0.
    residual_p90:float=0.
    tiles:list[dict]=field(default_factory=list)
    mat:list[list[float]]=field(default_factory=lambda:[
        [1.,0.,0.],[0.,1.,0.]])
    @property
    def accepted(self)->bool:
        return self.status=="ACCEPT"

def _weight(zone:int)->float:
    # Priority in the central area, but NEVER put zero weights at edges.
    return 2.0 if zone==4 else 1.30 if zone in (1,3,5,7) else .95

def _robust_global_similarity(p:np.ndarray,q:np.ndarray,
                              tile_ids:np.ndarray,fb:np.ndarray)->PairResult:
    r=PairResult(tracked=len(p),support_tiles=int(np.unique(tile_ids).size))
    if len(p)<25:
        r.status="TOO_FEW_FB_GOOD";return r
    model,m=cv2.estimateAffinePartial2D(
        p.astype(np.float32),q.astype(np.float32),
        method=cv2.RANSAC,ransacReprojThreshold=1.5,maxIters=480,
        confidence=.995,refineIters=8)
    if model is None or m is None:
        r.status="RANSAC_FAIL";return r
    m=m.ravel().astype(bool)
    if m.sum()<18:
        r.status="LOW_RANSAC_INLIERS";return r
    zones=np.unique(tile_ids[m])
    r.support_cols=len(set(int(z)%3 for z in zones))
    r.support_tiles=int(len(zones))
    if r.support_cols<3 or len(zones)<4:
        r.status="SPATIAL_CONSENSUS_FAIL";return r
    # Weighted full-frame 4DoF LS: RANSAC determines only trusted
    # background tracks, then every valid spatial tile has nonzero vote.
    p2=p[m];q2=q[m]
    wt=np.sqrt(np.asarray([_weight(int(t)) for t in tile_ids[m]],float))
    A=np.zeros((len(p2)*2,4),np.float64)
    B=np.empty(len(p2)*2,np.float64)
    A[0::2]=np.stack((p2[:,0],-p2[:,1],
                       np.ones(len(p2)),np.zeros(len(p2))),axis=1)
    A[1::2]=np.stack((p2[:,1],p2[:,0],
                       np.zeros(len(p2)),np.ones(len(p2))),axis=1)
    B[0::2]=q2[:,0];B[1::2]=q2[:,1]
    coef=np.linalg.lstsq(A*np.repeat(wt,2)[:,None],
                        B*np.repeat(wt,2),rcond=None)[0]
    a,b,tx,ty=[float(v) for v in coef]
    scale=math.hypot(a,b)
    angle=math.atan2(b,a)
    mat=np.array([[a,-b,tx],[b,a,ty]])
    pred=(np.c_[p,np.ones(len(p))]@mat.T)
    errors=np.linalg.norm(pred-q,axis=1)
    # Second-layer spatial tile residual agreement; local motion is not a
    # reason to replace global geometry with a free local 8DoF warp.
    tile_records=[]
    spread=[]
    for z in range(9):
        idx=np.where(tile_ids==z)[0]
        accepted=np.logical_and(m,tile_ids==z)
        ngood=int(accepted.sum())
        rr={
          "tile":z,"col":z%3,"row":z//3,"priority":_weight(z),
          "fb_good":len(idx),"global_inliers":ngood,
          "median_raw_dx":float(np.median(q[idx,0]-p[idx,0])) if len(idx) else None,
          "median_raw_dy":float(np.median(q[idx,1]-p[idx,1])) if len(idx) else None,
          "median_unmodeled_err":float(np.median(errors[idx])) if len(idx) else None
        }
        if ngood>=3:
            spread.append(float(np.median(errors[accepted])))
        tile_records.append(rr)
    r.tiles=tile_records
    r.inliers=int(m.sum())
    r.global_fb_p90=float(np.percentile(fb[m],90))
    r.residual_p90=float(np.percentile(errors[m],90))
    r.center_dx=(a-1.)*W*.5-b*H*.5+tx
    r.center_dy=b*W*.5+(a-1.)*H*.5+ty
    r.rotation_rad=angle
    r.logscale=math.log(max(scale,1e-8))
    r.mat=mat.tolist()
    fraction=r.inliers/max(1,len(p))
    spatial=min(1.,r.support_tiles/6.)
    confidence=min(1.,r.inliers/65.)*fraction*spatial*math.exp(-.7*r.residual_p90)
    r.trust=float(np.clip(confidence,0,1))
    if r.global_fb_p90>1.1:
        r.status="FB_UNRELIABLE"
    elif r.residual_p90>2.0:
        r.status="NONRIGID_OR_BLUR"
    elif r.trust<.14:
        r.status="LOW_TRUST"
    elif abs(r.logscale)>.045 or abs(r.rotation_rad)>.09:
        r.status="MODEL_SCALE_ROTATION_OUTLIER"
    elif math.hypot(r.center_dx,r.center_dy)>22:
        r.status="RESIDUAL_TOO_LARGE"
    else:
        r.status="ACCEPT"
    return r

def estimate(prev:np.ndarray,cur:np.ndarray,
             mask_prev:np.ndarray,mask_curr:np.ndarray)->PairResult:
    if prev.shape!=(H,W) or cur.shape!=(H,W):
        raise ValueError("C1 must use true 320x180 buffer-native gray frame")
    if np.count_nonzero(mask_curr)<W*H*.55 or np.count_nonzero(mask_prev)<W*H*.55:
        return PairResult(status="PREWARP_COVERAGE_LOW")
    pts=[];zones=[]
    # Feature points are independently detected in all nine physical zones.
    # A centered computer monitor isn't a special case or hard-coded ROI.
    for row in range(ROWS):
        for col in range(COLS):
            zone=row*COLS+col
            x0=int(col*W/COLS)+6;x1=int((col+1)*W/COLS)-6
            y0=int(row*H/ROWS)+5;y1=int((row+1)*H/ROWS)-5
            m=np.zeros((H,W),np.uint8)
            m[y0:y1,x0:x1]=mask_prev[y0:y1,x0:x1]
            corner=cv2.goodFeaturesToTrack(prev,maxCorners=18,
                qualityLevel=.011,minDistance=5,mask=m,blockSize=5)
            if corner is None:continue
            p=corner.reshape(-1,2)
            pts.append(p);zones.extend([zone]*len(p))
    if not pts:
        return PairResult(status="NO_FEATURES")
    p=np.concatenate(pts,axis=0).astype(np.float32)
    zon=np.asarray(zones,np.int32)
    q,valid,_=cv2.calcOpticalFlowPyrLK(
        prev,cur,p.reshape(-1,1,2),None,
        winSize=(17,17),maxLevel=2,
        criteria=(cv2.TERM_CRITERIA_EPS|cv2.TERM_CRITERIA_COUNT,22,.01))
    if q is None:return PairResult(status="LK_FAILURE",checked=len(p))
    back,backok,_=cv2.calcOpticalFlowPyrLK(
        cur,prev,q,None,
        winSize=(17,17),maxLevel=2,
        criteria=(cv2.TERM_CRITERIA_EPS|cv2.TERM_CRITERIA_COUNT,22,.01))
    if back is None:return PairResult(status="LK_BACKWARD_FAILURE",checked=len(p))
    q=q.reshape(-1,2);bak=back.reshape(-1,2)
    fb=np.linalg.norm(p-bak,axis=1)
    x=np.clip(np.rint(q[:,0]).astype(np.int32),0,W-1)
    y=np.clip(np.rint(q[:,1]).astype(np.int32),0,H-1)
    keep=(valid.ravel()>0)&(backok.ravel()>0)&(fb<.85)
    keep &= (mask_curr[y,x]>0)
    keep &= (np.linalg.norm(p-q,axis=1)<30)
    r=_robust_global_similarity(p[keep],q[keep],zon[keep],fb[keep])
    r.checked=len(p)
    return r

class CausalTurnState:
    def __init__(self):
        self.last_ts_ns=0
        self.ema_yaw=0.
        self.stable_sign=0
        self.frames_in_sign=0
        self.reversal_events=0
        self.mode="INIT"
    def update(self,ts:int,yaw_rate:float)->dict:
        if self.last_ts_ns and ts<=self.last_ts_ns:
            raise ValueError("C1 causal turn order violation")
        dt=min(.12,max(.002,(ts-self.last_ts_ns)*1e-9)) if self.last_ts_ns else 1/30
        alpha=1-math.exp(-dt/.055)
        self.ema_yaw+=alpha*(yaw_rate-self.ema_yaw)
        sign=1 if self.ema_yaw>.045 else -1 if self.ema_yaw<-.045 else 0
        reversal=False
        if sign:
            if sign!=self.stable_sign:
                if self.stable_sign and self.frames_in_sign>=3:
                    reversal=True
                    self.reversal_events+=1
                self.stable_sign=sign
                self.frames_in_sign=1
            else:self.frames_in_sign+=1
        self.mode=("REVERSAL" if reversal else
                   "PAN_CANDIDATE" if self.frames_in_sign>17 and sign else
                   "SWAY_HALF_CYCLE" if sign else "STOP_OR_SLOW")
        self.last_ts_ns=ts
        return {"yaw_ema_rad_s":float(self.ema_yaw),
                "reversal":reversal,"reversal_count":self.reversal_events,
                "turn_mode":self.mode,"stable_halfcycle_count":self.frames_in_sign}
