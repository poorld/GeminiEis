#!/usr/bin/env python3
"""Strictly causal PC C++ C2 Shadow ONE state owner C ABI.

Input is physical Camera0 SourceID/PTS/SensorTS, CURRENT + historical gyro,
20 real RS19 row quaternion samples and C1-B2 Native scene-agnostic LK. NO
offline Gemini V1 mesh, anchor, oracle, or future Gaussian in the controller.
Frozen GOLDEN exists only for downstream independent pixel quality comparison.
"""
from __future__ import annotations
import ctypes
from pathlib import Path
import numpy as np

class Frame(ctypes.Structure):
    _fields_=[
      ("source_id",ctypes.c_int64),("sensor_ts_ns",ctypes.c_int64),
      ("q_center",ctypes.c_double*4),("q_rows20",ctypes.c_double*80),
      ("yaw_rad_s",ctypes.c_double),
      ("visual_dx_320px",ctypes.c_double),
      ("visual_dy_320px",ctypes.c_double),
      ("visual_rotation_rad",ctypes.c_double),
      ("visual_logscale",ctypes.c_double),
      ("visual_trust",ctypes.c_double),
      ("visual_inliers",ctypes.c_int),
      ("visual_accepted",ctypes.c_int)
    ]
class Result(ctypes.Structure):
    _fields_=[
      ("code",ctypes.c_int),("reason",ctypes.c_int),
      ("visual_used",ctypes.c_int),("alternatives_tested",ctypes.c_int),
      ("rejected_safety",ctypes.c_int),
      ("source_id",ctypes.c_int64),("sensor_ts_ns",ctypes.c_int64),
      ("target_xyzw",ctypes.c_double*4),
      ("zoom",ctypes.c_double),("visual_tx_source_px",ctypes.c_double),
      ("visual_ty_source_px",ctypes.c_double),
      ("visual_rotation_rad",ctypes.c_double),
      ("visual_scale",ctypes.c_double),
      ("mesh_uv",ctypes.c_double*1320),
      ("min_margin_source_px",ctypes.c_double),
      ("center_x_source_px",ctypes.c_double),
      ("center_y_source_px",ctypes.c_double),
      ("span_source_px",ctypes.c_double),
      ("target_step_rad",ctypes.c_double),
      ("previous_mesh_span_step_px",ctypes.c_double),
      ("predicted_remaining_visual_px320",ctypes.c_double),
      ("solve_ms",ctypes.c_double)
    ]
class C2Shadow:
    def __init__(self,visual_enabled=False,dll=None):
        if dll is None:
            dll=Path(__file__).resolve().parents[2]/"reports/gemini_tripod_v2/c2_shadow/bin/c2_joint_shadow.dll"
        if not Path(dll).is_file():raise FileNotFoundError(dll)
        self.lib=ctypes.CDLL(str(dll))
        self.lib.c2_shadow_create.argtypes=[ctypes.c_int]
        self.lib.c2_shadow_create.restype=ctypes.c_void_p
        self.lib.c2_shadow_step.argtypes=[
            ctypes.c_void_p,ctypes.POINTER(Frame),ctypes.POINTER(Result)]
        self.lib.c2_shadow_step.restype=ctypes.c_int
        self.lib.c2_shadow_destroy.argtypes=[ctypes.POINTER(ctypes.c_void_p)]
        self.handle=ctypes.c_void_p(self.lib.c2_shadow_create(int(visual_enabled)))
        if not self.handle.value:raise RuntimeError("C2 owner allocation failed")
    def step(self,source_id,sensor_ts_ns,q_center,q_rows,yaw_rad_s,
             visual=None):
        q=np.asarray(q_center,dtype=np.float64)
        rows=np.asarray(q_rows,dtype=np.float64)
        if q.shape!=(4,) or rows.shape!=(20,4):
            raise ValueError("C2 requires exactly current gyro quaternion +20 physical RS19 rows")
        inp=Frame()
        inp.source_id=int(source_id)
        inp.sensor_ts_ns=int(sensor_ts_ns)
        inp.q_center[:]=q.tolist()
        inp.q_rows20[:]=rows.ravel().tolist()
        inp.yaw_rad_s=float(yaw_rad_s)
        if visual:
            inp.visual_dx_320px=float(visual.get("dx_feature_px",0))
            inp.visual_dy_320px=float(visual.get("dy_feature_px",0))
            inp.visual_rotation_rad=float(visual.get("rotation_rad",0))
            inp.visual_logscale=float(visual.get("logscale",0))
            inp.visual_trust=float(visual.get("trust",0))
            inp.visual_inliers=int(visual.get("inliers",0))
            inp.visual_accepted=int(visual.get("accepted",False))
        out=Result()
        code=self.lib.c2_shadow_step(self.handle,ctypes.byref(inp),
                                      ctypes.byref(out))
        if code or out.code:
            raise RuntimeError(f"C2 qVirtual/Visual/Zoom full RS19 gate code {code} "
                               f"SourceID {source_id}")
        xy=np.ctypeslib.as_array(out.mesh_uv).reshape((660,2)).copy()
        qTarget=np.ctypeslib.as_array(out.target_xyzw).copy()
        if not np.all(np.isfinite(xy)) or out.min_margin_source_px<2:
            raise RuntimeError(f"C2 mesh safety gate BROKEN SourceID {source_id}")
        return out,xy,qTarget
    def close(self):
        if self.handle.value:
            self.lib.c2_shadow_destroy(ctypes.byref(self.handle))
    def __enter__(self):return self
    def __exit__(self,*unused):self.close()
