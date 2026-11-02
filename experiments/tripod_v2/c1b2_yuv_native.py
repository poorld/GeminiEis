#!/usr/bin/env python3
"""V2C C1-B2: native Y-plane camera buffer -> 320x180 feature + RS19.

The source must already be an 8-bit Camera Y plane (NV12/NV21/YUV_420_888
luminance). Supports rowStride and pixelStride without color conversion. No
Python OpenCV routines in the frame-to-feature path. Previous/current images
are both projected at the CURRENT frame's qCenter; no future samples.
"""
from __future__ import annotations
import ctypes
from pathlib import Path
import numpy as np

class C1B2Yuv:
    def __init__(self,dll=None):
        if dll is None:
            dll=Path(__file__).resolve().parents[2]/"reports/gemini_tripod_v2/c1b2_yuv/bin/c1b2_native_yuv_rs.dll"
        if not Path(dll).is_file():raise FileNotFoundError(dll)
        lib=ctypes.CDLL(str(dll))
        p=ctypes.POINTER(ctypes.c_uint8)
        dbl=ctypes.POINTER(ctypes.c_double)
        f=lib.gemini_c1b2_luma_area8
        f.argtypes=[p,ctypes.c_int,ctypes.c_int,ctypes.c_int,ctypes.c_int,
                    p,ctypes.c_int]
        f.restype=ctypes.c_int
        w=lib.gemini_c1b2_prewarp_gray
        w.argtypes=[p,ctypes.c_int,
                    ctypes.c_double,ctypes.c_double,ctypes.c_double,ctypes.c_double,
                    dbl,dbl,p,p]
        w.restype=ctypes.c_int
        self._down=f
        self._warp=w

    def downsample_y(self,y_plane:np.ndarray,out=None)->np.ndarray:
        src=np.asarray(y_plane)
        if src.shape!=(1440,2560) or src.dtype!=np.uint8:
            raise ValueError("Requires actual 2560x1440 uint8 native Y plane")
        if src.strides[0]<=0 or src.strides[1]<=0:
            raise ValueError("Y-plane rows/pixels must have positive strides")
        if out is None:out=np.empty((180,320),np.uint8)
        if out.shape!=(180,320) or out.dtype!=np.uint8 or not out.flags.c_contiguous:
            raise ValueError("Native output must be owned continuous 320x180 gray")
        p=ctypes.POINTER(ctypes.c_uint8)
        rc=self._down(src.ctypes.data_as(p),2560,1440,
                      int(src.strides[0]),int(src.strides[1]),
                      out.ctypes.data_as(p),320)
        if rc:raise RuntimeError(f"C1-B2 native area8 failed {rc}")
        return out

    def prewarp(self,small:np.ndarray,row_quats:np.ndarray,reference:np.ndarray,
                fx:float,fy:float,cx:float,cy:float):
        g=np.asarray(small)
        if g.shape!=(180,320) or g.dtype!=np.uint8 or not g.flags.c_contiguous:
            raise ValueError("C1-B2 prewarp requires contiguous 320x180 Y")
        rows=np.ascontiguousarray(row_quats,np.float64)
        q=np.ascontiguousarray(reference,np.float64)
        if rows.shape!=(20,4) or q.shape!=(4,):
            raise ValueError("Physical RS19 requires 20 XYZW gyro row quaternions")
        out=np.empty((180,320),np.uint8)
        valid=np.empty((180,320),np.uint8)
        dp=ctypes.POINTER(ctypes.c_double)
        bp=ctypes.POINTER(ctypes.c_uint8)
        rc=self._warp(g.ctypes.data_as(bp),320,fx,fy,cx,cy,
                      rows.ctypes.data_as(dp),q.ctypes.data_as(dp),
                      out.ctypes.data_as(bp),valid.ctypes.data_as(bp))
        if rc:raise RuntimeError(f"C1-B2 native RS19 prewarp error {rc}")
        return out,valid
