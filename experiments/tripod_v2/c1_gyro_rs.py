#!/usr/bin/env python3
"""V2C C1: physically causal IMU orientation and 19-interval RS pre-warp.

Unlike Gemini V1, this module NEVER reads oracle_matrices or processed videos.
Only Gyro samples explicitly pushed before the *current frame readout deadline*
can affect orientation or image projection. The model matches the native Gemini
q_row.inverse()*q_reference 3D camera convention, including optical intrinsic K.

All feature frames are in buffer-native coordinates 320x180. This is a
low-resolution analysis prewarp, NOT the eventual full-resolution output Warp.
"""
from __future__ import annotations
from bisect import bisect_left
from dataclasses import dataclass
import math
import numpy as np
import cv2

LOW_W,LOW_H=320,180
RS_ROWS=20
CAMERA_TO_GYRO_OFFSET_NS=8_000_000

def qmul(a:np.ndarray,b:np.ndarray)->np.ndarray:
    x,y,z,w=[float(v) for v in a]
    X,Y,Z,W=[float(v) for v in b]
    return np.array((w*X+x*W+y*Z-z*Y,
                     w*Y-x*Z+y*W+z*X,
                     w*Z+x*Y-y*X+z*W,
                     w*W-x*X-y*Y-z*Z),dtype=np.float64)

def qrot(v:np.ndarray,quat:np.ndarray)->np.ndarray:
    """Vectorized quaternion rotation in XYZW; q broadcast to same raster."""
    qv=quat[...,:3]
    q0=quat[...,3:4]
    cross1=np.cross(qv,v)
    return v+2*(q0*cross1+np.cross(qv,cross1))

def norm(q:np.ndarray)->np.ndarray:
    return q/np.maximum(np.linalg.norm(q,axis=-1,keepdims=True),1e-15)

def quat_from_rotvec(v:np.ndarray)->np.ndarray:
    angle=float(np.linalg.norm(v))
    if angle<1e-10:
        return norm(np.array([*(v*.5),1.],dtype=np.float64))
    return np.array([*(v*(math.sin(angle*.5)/angle)),math.cos(angle*.5)],dtype=np.float64)

class CausalGyro:
    """Same trapezoid gyro integration / q-multiplication as C++ ring buffer.

    Does not retain accessible future gyro poses. Only append() advances time.
    Client may parse a CSV into a file-read cache, but must never pass unseen
    samples to this state before the current camera release/deadline.
    """
    def __init__(self):
        self.ts:list[int]=[]
        self.g:list[np.ndarray]=[]
        self.q:list[np.ndarray]=[]
        self.maximum_submitted_ns=0
        self.samples_submitted=0
        self.max_query_ns=0
    def push(self,t:int,omega:np.ndarray):
        t=int(t)
        if self.ts and t<=self.ts[-1]:
            raise ValueError("Nonincreasing Gyro stream")
        omega=np.asarray(omega,np.float64)
        if not np.all(np.isfinite(omega)):raise ValueError("Nonfinite Gyro")
        if self.ts:
            dt=(t-self.ts[-1])*1e-9
            prev=self.q[-1]
            if 0<dt<.05:
                dq=quat_from_rotvec((omega+self.g[-1])*.5*dt)
                now=norm(qmul(prev,dq))
            else: now=prev
        else:now=np.array([0.,0.,0.,1.],np.float64)
        self.ts.append(t)
        self.g.append(omega)
        self.q.append(now)
        self.maximum_submitted_ns=t
        self.samples_submitted+=1
    def coverage(self,t:int)->bool:
        return len(self.ts)>=2 and self.ts[0]<=t<=self.ts[-1]
    def pose_at(self,t:int)->np.ndarray:
        t=int(t)
        if not self.coverage(t):
            raise ValueError(f"Gyro sensor coverage missing at {t}; "
                             f"has {self.ts[0] if self.ts else None} .. "
                             f"{self.ts[-1] if self.ts else None}")
        self.max_query_ns=max(self.max_query_ns,t)
        k=bisect_left(self.ts,t)
        if self.ts[k]==t: return self.q[k].copy()
        a=k-1
        f=(t-self.ts[a])/float(self.ts[k]-self.ts[a])
        qa=self.q[a];qb=self.q[k]
        if float(qa@qb)<0:qb=-qb
        return norm((1-f)*qa+f*qb)

@dataclass(frozen=True)
class CameraFrame:
    source_id:int
    pts_us:int
    sensor_ts_ns:int
    exposure_ns:int
    skew_ns:int
    def gyro_time_at(self,frac:float)->int:
        return int(self.sensor_ts_ns+self.exposure_ns//2+
                   self.skew_ns*frac+CAMERA_TO_GYRO_OFFSET_NS)
    @property
    def availability_deadline_ns(self)->int:
        # Conservative end-of-exposure + last RS row and calibrated offset:
        # queries stop at (SOF+half-exposure+lastRow+offset), but gyro may be
        # delivered up to this physical frame deadline for bracketing.
        return int(self.sensor_ts_ns+self.exposure_ns+self.skew_ns+
                   CAMERA_TO_GYRO_OFFSET_NS)
    @property
    def center_gyro_ts_ns(self)->int:
        return self.gyro_time_at(.5)

def pose_rows(f:CameraFrame,gyro:CausalGyro)->np.ndarray:
    return norm(np.stack([gyro.pose_at(f.gyro_time_at(i/19))
                          for i in range(RS_ROWS)]).astype(np.float64))

class Rs19Prewarper:
    def __init__(self,fx:float,fy:float,cx:float,cy:float,
                 source_w:int=2560,source_h:int=1440,
                 width:int=LOW_W,height:int=LOW_H):
        self.w=width;self.h=height
        self.fx=fx*width/source_w; self.fy=fy*height/source_h
        self.cx=cx*width/source_w; self.cy=cy*height/source_h
        xx,yy=np.meshgrid(np.arange(width,dtype=np.float64)+.5,
                          np.arange(height,dtype=np.float64)+.5)
        self.ray=np.stack(((xx-self.cx)/self.fx,
                           (yy-self.cy)/self.fy,
                           np.ones_like(xx)),axis=-1)
        self.initial_row=np.broadcast_to(
            np.linspace(.5/height,1.-.5/height,height)[:,None],
            (height,width)).copy()
    def remap(self,gray:np.ndarray,row_quats:np.ndarray,
              reference_q:np.ndarray)->tuple[np.ndarray,np.ndarray]:
        if gray.shape!=(self.h,self.w) or row_quats.shape!=(20,4):
            raise ValueError("Bad analysis resolution/19RS row poses")
        frac=self.initial_row.copy()
        qref=np.asarray(reference_q,np.float64)
        xx=yy=den=None
        for _ in range(3):
            r=np.clip(frac*19,0,19-1e-8)
            i=np.floor(r).astype(np.int32)
            f=(r-i)[...,None]
            q0=row_quats[i];q1=row_quats[i+1]
            flip=np.where(np.sum(q0*q1,axis=-1,keepdims=True)<0,-1.,1.)
            qi=norm(q0*(1-f)+q1*f*flip)
            inv=qi.copy(); inv[...,:3]*=-1.
            # Gemini native renderer computes R(qRow^-1 * qVirtual).
            qrel=np.empty_like(inv)
            x,y,z,w=np.moveaxis(inv,-1,0)
            X,Y,Z,W=[qref[k] for k in range(4)]
            qrel[...,0]=w*X+x*W+y*Z-z*Y
            qrel[...,1]=w*Y-x*Z+y*W+z*X
            qrel[...,2]=w*Z+x*Y-y*X+z*W
            qrel[...,3]=w*W-x*X-y*Y-z*Z
            projected=qrot(self.ray,qrel)
            den=projected[...,2]
            safe=np.maximum(den,1e-6)
            xx=self.fx*projected[...,0]/safe+self.cx-.5
            yy=self.fy*projected[...,1]/safe+self.cy-.5
            frac=np.clip((yy+.5)/self.h,0,1)
        mx=xx.astype(np.float32);my=yy.astype(np.float32)
        valid=(den>1e-4)&(mx>=.5)&(my>=.5)&(mx<self.w-1.5)&(my<self.h-1.5)
        img=cv2.remap(gray,mx,my,interpolation=cv2.INTER_LINEAR,
                      borderMode=cv2.BORDER_CONSTANT,borderValue=0)
        return img,(valid.astype(np.uint8)*255)

def resize_native_gray(raw_bgr:np.ndarray,size=(LOW_W,LOW_H))->np.ndarray:
    if raw_bgr.shape[:2]!=(1440,2560):
        raise ValueError(f"Expected true unrotated camera raw 2560x1440, got {raw_bgr.shape}")
    return cv2.resize(cv2.cvtColor(raw_bgr,cv2.COLOR_BGR2GRAY),
                      size,interpolation=cv2.INTER_AREA)

# Native C++ executes same 20-row physical RS projection on all pixels.
# Python still owns ONLY diagnostic feature-frame and causal IMU streaming.
class NativeRs19Prewarper(Rs19Prewarper):
    def __init__(self,*a,**kw):
        import ctypes
        from pathlib import Path
        super().__init__(*a,**kw)
        base=Path(__file__).resolve().parents[2]
        directory=base/"reports"/"gemini_tripod_v2"/"c1_causal"/"bin"
        import os
        variant=os.getenv("GEMINI_C1_GEOMETRY_BACKEND","fine")
        if variant not in ("fine","mesh","dense"):
            raise ValueError("Bad C1 geometry backend")
        suffix={"fine":"_fine","mesh":"_fast","dense":""}[variant]
        if sys_platform_windows():
            path=directory/f"gemini_v2c_rs_prewarp{suffix}.dll"
        else:
            path=directory/f"libgemini_v2c_rs_prewarp{suffix}.so"
        if not path.exists():
            raise FileNotFoundError(f"Portable C1 RS19 native prewarp not built: {path}")
        self.dll=ctypes.CDLL(str(path))
        f=self.dll.gemini_v2c_rs19_prewarp_map
        doublep=ctypes.POINTER(ctypes.c_double)
        floatp=ctypes.POINTER(ctypes.c_float)
        bytep=ctypes.POINTER(ctypes.c_uint8)
        f.argtypes=[ctypes.c_int,ctypes.c_int,
                    ctypes.c_double,ctypes.c_double,ctypes.c_double,ctypes.c_double,
                    doublep,doublep,floatp,floatp,bytep]
        f.restype=ctypes.c_int
        self._ffi=f
    def remap(self,gray:np.ndarray,row_quats:np.ndarray,
              reference_q:np.ndarray)->tuple[np.ndarray,np.ndarray]:
        if gray.shape!=(self.h,self.w):
            raise ValueError("Wrong current feature frame size")
        if np.asarray(row_quats).shape!=(20,4) or np.asarray(reference_q).shape!=(4,):
            raise ValueError("C1 RS19 native ABI requires exactly 20 XYZW row quaternions")
        qs=np.ascontiguousarray(row_quats,np.float64)
        ref=np.ascontiguousarray(reference_q,np.float64)
        mx=np.empty((self.h,self.w),dtype=np.float32)
        my=np.empty_like(mx)
        valid=np.empty((self.h,self.w),dtype=np.uint8)
        import ctypes
        rc=self._ffi(self.w,self.h,self.fx,self.fy,self.cx,self.cy,
            qs.ctypes.data_as(ctypes.POINTER(ctypes.c_double)),
            ref.ctypes.data_as(ctypes.POINTER(ctypes.c_double)),
            mx.ctypes.data_as(ctypes.POINTER(ctypes.c_float)),
            my.ctypes.data_as(ctypes.POINTER(ctypes.c_float)),
            valid.ctypes.data_as(ctypes.POINTER(ctypes.c_uint8)))
        if rc:raise RuntimeError(f"C1 native RS19 prewarp failed code={rc}")
        warp=cv2.remap(gray,mx,my,cv2.INTER_LINEAR,
                       borderMode=cv2.BORDER_CONSTANT,borderValue=0)
        return warp,valid

def sys_platform_windows():
    import sys
    return sys.platform.startswith("win")

