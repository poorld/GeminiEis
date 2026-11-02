#!/usr/bin/env python3
"""Portable C1-B bounded native async scheduler C ABI host verification.

Proof of frame ownership and deadline here is a C++ real worker with one
outstanding job and monotonic wallclock deadline, NOT a claim that an Android
Camera callback is already delivering images before its genuine RS deadline.
"""
from __future__ import annotations
import ctypes,time
from pathlib import Path
import numpy as np
from c1b_native_visual import NativeResult

class Stats(ctypes.Structure):
    _fields_=[(k,ctypes.c_uint64) for k in (
        "submitted","busy","rejected_order","rejected_deadline","completed",
        "ready_on_time","late","cancelled","destroyed_pending")]+[
        ("last_seen_source",ctypes.c_int64),
        ("last_seen_sensor_ts",ctypes.c_int64)]
class NativeAsync:
    def __init__(self,diagnostic_delay_ms:int=0):
        root=Path(__file__).resolve().parents[2]
        path=root/"reports/gemini_tripod_v2/c1b_native/bin/c1b_native_async_lk.dll"
        self.dll=ctypes.CDLL(str(path))
        d=self.dll
        pointer=ctypes.POINTER(ctypes.c_uint8)
        d.c1b_async_create.argtypes=[ctypes.c_int]
        d.c1b_async_create.restype=ctypes.c_void_p
        d.c1b_async_monotonic_ns.argtypes=[]
        d.c1b_async_monotonic_ns.restype=ctypes.c_uint64
        d.c1b_async_submit.argtypes=[
            ctypes.c_void_p,ctypes.c_int64,ctypes.c_int64,ctypes.c_uint64,
            pointer,pointer,pointer,pointer]
        d.c1b_async_submit.restype=ctypes.c_int
        d.c1b_async_poll.argtypes=[
            ctypes.c_void_p,ctypes.c_int64,ctypes.POINTER(NativeResult)]
        d.c1b_async_poll.restype=ctypes.c_int
        d.c1b_async_cancel.argtypes=[ctypes.c_void_p,ctypes.c_int64]
        d.c1b_async_cancel.restype=ctypes.c_int
        d.c1b_async_stats.argtypes=[ctypes.c_void_p,ctypes.POINTER(Stats)]
        d.c1b_async_stats.restype=ctypes.c_int
        d.c1b_async_destroy.argtypes=[ctypes.POINTER(ctypes.c_void_p)]
        self.handle=ctypes.c_void_p(d.c1b_async_create(diagnostic_delay_ms))
        if not self.handle.value:raise RuntimeError("Cannot allocate native C1-B worker")
    def now_ns(self):
        return self.dll.c1b_async_monotonic_ns()
    def submit(self,source_id:int,sensor_ts_ns:int,deadline_ns:int,
               prev,cur,prev_mask,cur_mask):
        images=[]
        for frame in (prev,cur,prev_mask,cur_mask):
            a=np.asarray(frame)
            if a.shape!=(180,320) or a.dtype!=np.uint8:
                raise ValueError("C1B async requires 320x180 single-channel image/mask")
            images.append(np.ascontiguousarray(a))
        # C++ copies the full frame quadruplet synchronously under its mutex.
        ptr=ctypes.POINTER(ctypes.c_uint8)
        return self.dll.c1b_async_submit(self.handle,source_id,sensor_ts_ns,deadline_ns,
            *(a.ctypes.data_as(ptr) for a in images))
    def poll(self,source_id:int):
        out=NativeResult()
        status=self.dll.c1b_async_poll(self.handle,source_id,ctypes.byref(out))
        return status,out
    def cancel(self,source_id:int):
        return self.dll.c1b_async_cancel(self.handle,source_id)
    def stats(self):
        r=Stats()
        if self.dll.c1b_async_stats(self.handle,ctypes.byref(r)):
            raise RuntimeError("Native async stats failed")
        return {k:getattr(r,k) for k,_ in r._fields_}
    def wait(self,source_id:int,deadline_ns:int):
        while True:
            code,out=self.poll(source_id)
            if code!=1:return code,out
            if self.now_ns()>=deadline_ns:return self.poll(source_id)
            time.sleep(.0002)
    def close(self):
        if self.handle.value:
            self.dll.c1b_async_destroy(ctypes.byref(self.handle))
    def __enter__(self):return self
    def __exit__(self,*args):self.close()
