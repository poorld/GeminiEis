#!/usr/bin/env python3
"""Native C1-B portable FB-LK ABI; zero future frames, scene-agnostic 3x3.

Each call consumes a pair of currently-available (previous, current) native
320x180 Gyro+RS19 prewarps and valid masks. ctypes CDLL releases Python GIL
during the actual C++ work. Does NOT use outputEIS or old Oracle matrices.
"""
from __future__ import annotations
import ctypes
import math
from pathlib import Path
from typing import Optional
import numpy as np
from c1_tile_observer import PairResult

class NativeResult(ctypes.Structure):
    _fields_=[
        ("status",ctypes.c_int32),("checked",ctypes.c_int32),
        ("tracked",ctypes.c_int32),("inliers",ctypes.c_int32),
        ("support_tiles",ctypes.c_int32),("support_cols",ctypes.c_int32),
        ("trust",ctypes.c_double),("center_dx",ctypes.c_double),
        ("center_dy",ctypes.c_double),("rotation_rad",ctypes.c_double),
        ("logscale",ctypes.c_double),("fb_p90",ctypes.c_double),
        ("fit_p90",ctypes.c_double),
        ("matrix",ctypes.c_double*6),
        ("tile_checked",ctypes.c_int32*9),
        ("tile_inliers",ctypes.c_int32*9),
        ("tile_dx",ctypes.c_double*9),
        ("tile_dy",ctypes.c_double*9),
        ("tile_fit",ctypes.c_double*9),
        ("elapsed_ms",ctypes.c_double)
    ]
STATUS={
    0:"ACCEPT",1:"PREWARP_COVERAGE_LOW",2:"NO_FEATURES",
    3:"TOO_FEW_FB_GOOD",4:"RANSAC_FAIL",5:"SPATIAL_CONSENSUS_FAIL",
    6:"FB_UNRELIABLE",7:"NONRIGID_OR_BLUR",8:"LOW_TRUST",
    9:"MODEL_SCALE_ROTATION_OUTLIER",-1:"INVALID_ARGUMENT"
}
class C1BNativeTileLK:
    def __init__(self,dll:Optional[Path]=None):
        if dll is None:
            root=Path(__file__).resolve().parents[2]
            dll=root/"reports/gemini_tripod_v2/c1b_native/bin/c1b_native_tile_lk.dll"
        if not dll.is_file():raise FileNotFoundError(dll)
        self.lib=ctypes.CDLL(str(dll))
        p=ctypes.POINTER(ctypes.c_uint8)
        f=self.lib.c1b_visual_pair
        f.argtypes=[p,p,p,p,ctypes.c_int,ctypes.c_int,ctypes.c_int,
                    ctypes.POINTER(NativeResult)]
        f.restype=ctypes.c_int
        self.call=f
        self.call_count=0
        self.native_ms=[]
    def estimate(self,prev:np.ndarray,cur:np.ndarray,
                 prev_mask:np.ndarray,cur_mask:np.ndarray)->PairResult:
        images=[]
        for frame in (prev,cur,prev_mask,cur_mask):
            a=np.asarray(frame)
            if a.shape!=(180,320) or a.dtype!=np.uint8:
                raise ValueError("Native C1-B must receive 320x180 uint8 feature arrays")
            images.append(np.ascontiguousarray(a))
        pointers=[a.ctypes.data_as(ctypes.POINTER(ctypes.c_uint8))
                  for a in images]
        result=NativeResult()
        rc=self.call(*pointers,320,180,320,ctypes.byref(result))
        if rc:raise RuntimeError(f"C1-B native visual error {rc}")
        self.call_count+=1
        self.native_ms.append(result.elapsed_ms)
        tiles=[]
        for z in range(9):
            dx=float(result.tile_dx[z])
            dy=float(result.tile_dy[z])
            err=float(result.tile_fit[z])
            tiles.append({
                "tile":z,"col":z%3,"row":z//3,
                "priority":2. if z==4 else 1.3 if z in (1,3,5,7) else .95,
                "fb_good":int(result.tile_checked[z]),
                "global_inliers":int(result.tile_inliers[z]),
                "median_raw_dx":dx if math.isfinite(dx) else None,
                "median_raw_dy":dy if math.isfinite(dy) else None,
                "median_unmodeled_err":err if math.isfinite(err) else None
            })
        mat=[[float(result.matrix[i*3+j]) for j in range(3)] for i in range(2)]
        return PairResult(
            status=STATUS[int(result.status)],trust=float(result.trust),
            checked=result.checked,tracked=result.tracked,inliers=result.inliers,
            support_tiles=result.support_tiles,
            support_cols=result.support_cols,center_dx=result.center_dx,
            center_dy=result.center_dy,rotation_rad=result.rotation_rad,
            logscale=result.logscale,global_fb_p90=result.fb_p90,
            residual_p90=result.fit_p90,tiles=tiles,mat=mat)

_NATIVE=None
def estimate(prev,cur,mask_prev,mask_curr):
    global _NATIVE
    if _NATIVE is None:_NATIVE=C1BNativeTileLK()
    return _NATIVE.estimate(prev,cur,mask_prev,mask_curr)
