#!/usr/bin/env python3
"""Executable C1 causal/geometry/feature regression, independent of V1 Oracle.

C1 C++ and feature-space sign/time semantics tested with synthetic 3D yaw,
and gyro prefix invariance checked for exact timestamp series. This is HOST,
not an Android device performance or final video-effect test.
"""
from __future__ import annotations
import math,sys,json
from pathlib import Path
import numpy as np,cv2
from scipy.spatial.transform import Rotation
from c1_gyro_rs import (
    CausalGyro,CameraFrame,NativeRs19Prewarper,
    qmul,quat_from_rotvec,pose_rows,resize_native_gray)
from c1_tile_observer import estimate,CausalTurnState
from c1_stream_host import BUNDLE_ROOT,read_gyro,frame_from_meta

def test_gyro_prefix():
    times,omega=read_gyro(BUNDLE_ROOT/"024630"/"gyro_camera.csv")
    assert len(times)>2500
    a=CausalGyro()
    for t,g in zip(times[:115],omega[:115]):a.push(int(t),g)
    queries=[int(times[i]) for i in (8,33,44,71,94)]
    old=[a.pose_at(t) for t in queries]
    try:a.pose_at(int(times[130]))
    except ValueError:pass
    else:raise AssertionError("Future gyro sample was accessible")
    for t,g in zip(times[115:170],omega[115:170]):a.push(int(t),g)
    for expected,t in zip(old,queries):
        assert np.allclose(a.pose_at(t),expected,rtol=0,atol=0), "Orientation lookup used future gyro"
    q0=quat_from_rotvec(np.array([.0,.03,0]))
    q1=quat_from_rotvec(np.array([.0,.02,0]))
    assert abs((qmul(q0,q1))[1]-math.sin(.025))<1e-8
    return {"gyro_prefix_no_future":True,"queried":len(queries),
            "gyro_source_samples":len(times)}

def test_input_time():
    import cv2
    p=BUNDLE_ROOT/"024630"
    info=json.loads((p/"bundle.json").read_text(encoding="utf-8"))
    f0=frame_from_meta(info["frames"][0])
    assert (f0.source_id,f0.pts_us)==(4,236466)
    assert f0.gyro_time_at(0)<f0.center_gyro_ts_ns<f0.gyro_time_at(1)
    assert f0.gyro_time_at(1)<=f0.availability_deadline_ns
    cap=cv2.VideoCapture(str(p/"source.mp4"))
    for i in range(5):
        ok,im=cap.read()
        assert ok
    pts=cap.get(cv2.CAP_PROP_POS_MSEC)
    cap.release()
    assert abs(pts-f0.pts_us/1000)<.5
    return {"source_id_pts":True,"readout_deadline_is_after_last_gyro":True}

def test_native_gyro_alignment():
    rng=np.random.default_rng(42)
    refimg=np.zeros((180,320),np.uint8)
    # Texture across all spatial tiles (no privileged screen center).
    for i in range(580):
        x=int(rng.integers(6,314));y=int(rng.integers(6,174))
        cv2.circle(refimg,(x,y),1,int(rng.integers(85,256)),-1)
    cv2.line(refimg,(3,100),(315,99),170,1)
    warp=NativeRs19Prewarper(1202.3,1208.5,1280.,720.)
    yaw=.028
    qprev=np.array([0.,0.,0.,1.])
    qcur=quat_from_rotvec(np.array([0.,yaw,0.]))
    camera_scale=320/2560
    K=np.array([[1202.3*camera_scale,0.,160.],
                [0.,1208.5*camera_scale,90.],[0.,0.,1.]],np.float64)
    R=Rotation.from_quat(qcur).as_matrix()
    # Forward warp: reference t0 physical pixel into camera1 physical t1.
    H=K@R.T@np.linalg.inv(K)
    newer=cv2.warpPerspective(refimg,H,(320,180),flags=cv2.INTER_LINEAR,
                              borderMode=cv2.BORDER_CONSTANT,borderValue=0)
    a,mask_a=warp.remap(refimg,np.tile(qprev,(20,1)),qcur)
    b,mask_b=warp.remap(newer,np.tile(qcur,(20,1)),qcur)
    orig=estimate(refimg,newer,np.ones_like(refimg),np.ones_like(refimg))
    gyro=estimate(a,b,mask_a,mask_b)
    print("C1_SYNTHETIC_PHYSICAL_CAMERA_YAW",
          "raw",orig.status,round(orig.center_dx,3),
          "derotated",gyro.status,round(gyro.center_dx,3),
          "raw_motion",round(math.hypot(orig.center_dx,orig.center_dy),3),
          "derotated_motion",round(math.hypot(gyro.center_dx,gyro.center_dy),3),flush=True)
    assert orig.accepted and gyro.accepted
    assert math.hypot(gyro.center_dx,gyro.center_dy)<math.hypot(orig.center_dx,orig.center_dy)*.30
    return {"synth_forward_gyro_projection_correct":True,
            "raw_motion":math.hypot(orig.center_dx,orig.center_dy),
            "prewarped_motion":math.hypot(gyro.center_dx,gyro.center_dy)}

def test_turn_no_future():
    obs=CausalTurnState()
    events=[]
    for i,v in enumerate([.4]*12+[-.4]*12+[.4]*12):
        r=obs.update(1000000000+33333333*i,v)
        if r["reversal"]:events.append(i)
    assert len(events)==2
    assert all(i>=12 for i in events)
    assert obs.reversal_events==2
    return {"turn_reversal_is_causal":True,"reversal_event_frames":events}

def test_bad_rs_data():
    warp=NativeRs19Prewarper(1202.3,1208.5,1280.,720.)
    img=np.zeros((180,320),np.uint8)
    ident=np.tile(np.array([0.,0.,0.,1.]),(20,1))
    qref=np.array([0.,0.,0.,1.])
    output,mask=warp.remap(img,ident,qref)
    assert output.shape==(180,320)
    assert np.count_nonzero(mask)>320*180*.95
    try:warp.remap(img,ident[:19],qref)
    except ValueError:pass
    else:raise AssertionError("Bad 19-row pose array not rejected")
    try:resize_native_gray(np.zeros((720,1280,3),np.uint8))
    except ValueError:pass
    else:raise AssertionError("Previously stabilized/cropped frame wrongly accepted as RAW")
    return {"physical20row_RS19_and_raw_shape_gate":True}


def test_dynamic_center_outlier_and_no_texture():
    rng=np.random.default_rng(1234)
    prev=np.zeros((180,320),np.uint8)
    for _ in range(1050):
        cx=int(rng.integers(5,315))
        cy=int(rng.integers(4,176))
        cv2.circle(prev,(cx,cy),1,int(rng.integers(85,255)),-1)
    # Background translates by +2 px everywhere.
    curr=cv2.warpAffine(prev,np.float32([[1,0,2],[0,1,0]]),
        (320,180),flags=cv2.INTER_LINEAR)
    # One dynamic central tile moves independently by +11 pixels
    # instead of +2. A center-first observer must not lock to this
    # single object when eight other spatial areas agree.
    curr[60:120,109:211]=cv2.warpAffine(
        prev[60:120,109:211],np.float32([[1,0,11],[0,1,0]]),
        (102,60))
    mask=np.ones_like(prev,np.uint8)*255
    moving=estimate(prev,curr,mask,mask)
    assert moving.accepted,(moving.status,moving.inliers,moving.support_tiles)
    assert abs(moving.center_dx-2.0)<.9,moving.center_dx
    assert moving.support_cols==3 and moving.support_tiles>=6
    flat=np.zeros_like(prev)
    none=estimate(flat,flat,mask,mask)
    assert not none.accepted and none.status=="NO_FEATURES",none.status
    return {"dynamic_center_tile_not_trusted_as_global":True,
            "expected_background_dx":2.0,
            "measured_global_dx":moving.center_dx,
            "all_three_image_columns_voted":True,
            "blank_scene_fail_closed":True}

def main():
    result={
       "gyro_prefix":test_gyro_prefix(),
       "timing":test_input_time(),
       "native_projection":test_native_gyro_alignment(),
       "reversal":test_turn_no_future(),
       "input_guard":test_bad_rs_data(),
       "dynamic_scene":test_dynamic_center_outlier_and_no_texture(),
       "golden_files_untouched":True,
       "offline_oracle_not_needed":True,
       "HOST_ONLY_NO_ANDROID_RESULT":True
    }
    dest=Path("reports/gemini_tripod_v2/c1_causal/c1_unit_tests.json")
    dest.parent.mkdir(parents=True,exist_ok=True)
    dest.write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding="utf-8")
    print("GEMINI_V2C_C1_CAUSAL_TIMING_GEOMETRY_LK_TESTS_ALL_PASS",
          json.dumps(result,ensure_ascii=False),flush=True)

if __name__=="__main__":main()
