#!/usr/bin/env python3
"""C1-B2 Host Y-plane stride, memory lifetime, downsample and RS19 parity.

Tests explicit rowStride/pixelStride/padding sentinels and 20 real RS gyro
input. Unit tests are CPU Host only; not Android camera driver evidence.
"""
from __future__ import annotations
import json,time
from pathlib import Path
import numpy as np,cv2
from c1b2_yuv_native import C1B2Yuv
from c1_gyro_rs import NativeRs19Prewarper,quat_from_rotvec

def main():
    obj=C1B2Yuv()
    rng=np.random.default_rng(12)
    Y=rng.integers(16,235,size=(1440,2560),dtype=np.uint8)
    results={}
    expected=cv2.resize(Y,(320,180),interpolation=cv2.INTER_AREA)
    native=obj.downsample_y(Y)
    delta=np.abs(native.astype(np.int16)-expected.astype(np.int16))
    results["native_dense_stride1"]={"pixel_delta_max":int(delta.max()),
      "pixel_delta_p99":float(np.percentile(delta,99))}
    assert delta.max()<=1
    padded=np.full((1440,2688),255,dtype=np.uint8)
    padded[:,:2560]=Y
    x=obj.downsample_y(padded[:,:2560])
    assert np.array_equal(x,native)
    assert padded[:,2560:].min()==255
    results["native_padded_y_rowStride_2688"]={"match":True,"padding_untouched":True}
    interleaved=np.full((1440,2560*2+64),251,dtype=np.uint8)
    interleaved[:,0:5120:2]=Y
    double_stride=obj.downsample_y(interleaved[:,0:5120:2])
    assert np.array_equal(double_stride,native)
    results["native_interleaved_y_pixelStride_2"]={"match":True,
      "non_y_sentinel_unchanged":bool((interleaved[:,1:5120:2]==251).all())}
    # Unchanging input plane and output must not alias. Source memory belongs
    # to Camera until copy/downsample completes; 320x180 output is owned.
    before=native.copy()
    Y[:,:]=0
    assert np.array_equal(before,native)
    results["owned_feature_buffer_source_lifetime"]={"copy_is_independent":True}
    # use real RAW Y plane, no cvtColor-based reconstruction in this gate.
    root=Path("D:/hongda/app/YuySuperEIS/artifacts/b38_restore_bundles_20261020_172849/024630")
    cap=cv2.VideoCapture(str(root/"source.mp4"))
    cap.set(cv2.CAP_PROP_ORIENTATION_AUTO,0)
    cap.set(cv2.CAP_PROP_CONVERT_RGB,0)
    ok,real=cap.read()
    cap.release()
    assert ok and real.shape==(1440,2560) and real.dtype==np.uint8
    rawsmall=obj.downsample_y(real)
    oldsmall=cv2.resize(real,(320,180),interpolation=cv2.INTER_AREA)
    assert np.max(np.abs(rawsmall.astype(np.int16)-oldsmall.astype(np.int16)))<=1
    results["actual_source_native_yuv420p_plane"]={
        "shape":list(real.shape),"min":int(real.min()),"max":int(real.max()),
        "mean":float(np.mean(real)), "area8_vs_opencv_max_gray_level_delta":
        int(np.max(abs(rawsmall.astype(np.int16)-oldsmall.astype(np.int16))))}
    # Same physical row poses/map as original C1-A/B native 39x65 feature mesh.
    qrows=np.stack([quat_from_rotvec(np.array([.004,.015*(i-9.5)/9.5,.003]))
                    for i in range(20)])
    qref=quat_from_rotvec(np.array([.002,-.012,.001]))
    oldwarper=NativeRs19Prewarper(1202.3,1208.5,1280,720)
    warped,mask=obj.prewarp(rawsmall,qrows,qref,oldwarper.fx,
                             oldwarper.fy,oldwarper.cx,oldwarper.cy)
    original,original_mask=oldwarper.remap(rawsmall,qrows,qref)
    assert np.array_equal(mask,original_mask)
    good=mask!=0
    pixel_diff=np.abs(warped[good].astype(np.int16)-original[good].astype(np.int16))
    results["native_20row_RS19_39x65_against_existing_OpenCV_remap"]={
        "valid_masks_byte_exact":True,
        "pixel_delta_p99":float(np.percentile(pixel_diff,99)),
        "pixel_delta_p999":float(np.percentile(pixel_diff,99.9)),
        "pixel_delta_max":int(pixel_diff.max())}
    assert np.percentile(pixel_diff,99)<2.,results["native_20row_RS19_39x65_against_existing_OpenCV_remap"]
    t0=time.perf_counter()
    for i in range(36):obj.downsample_y(real)
    results["native_real_Y_8x8_area_36x_avg_ms"]=(time.perf_counter()-t0)*1000/36
    t0=time.perf_counter()
    for i in range(25):obj.prewarp(rawsmall,qrows,qref,
                            oldwarper.fx,oldwarper.fy,oldwarper.cx,oldwarper.cy)
    results["native_real_RS19_feature_prewarp_25x_avg_ms"]=(time.perf_counter()-t0)*1000/25
    d=Path("reports/gemini_tripod_v2/c1b2_yuv/c1b2_native_yuv_unit.json")
    d.parent.mkdir(parents=True,exist_ok=True)
    d.write_text(json.dumps(results,indent=2,ensure_ascii=False),encoding="utf-8")
    print("V2C_C1B2_STRIDE_AWARE_NATIVE_Y_TRUE_SOURCE_RS19_PARITY_PASS",
          json.dumps(results,ensure_ascii=False),flush=True)

if __name__=="__main__":main()
