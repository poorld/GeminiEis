#!/usr/bin/env python3
"""C2 true Native unique owner safety, stale-visual, timestamp and causality.

No V1 future anchor or processed V1 mesh, no lookahead real images. Physical
RS 20 row quaternions. Test ensures no state poisoning on duplicate TS/IDs.
"""
from __future__ import annotations
import json,math
from pathlib import Path
import numpy as np
from c1_gyro_rs import quat_from_rotvec
from c2_joint_native import C2Shadow

def one(mode,n,sequence):
    history=[]
    with C2Shadow(mode) as c:
        for i,angle in enumerate(sequence):
            q=quat_from_rotvec(np.array([.002,angle,.001]))
            # Real physical 20RS row poses around center in a synthetic turn.
            rows=np.stack([quat_from_rotvec(np.array([
                 .002,angle+(j-9.5)*.00035,.001]))
                 for j in range(20)])
            vis={
                "dx_feature_px":math.sin(i*.4)*.8,
                "dy_feature_px":.13*math.cos(i*.5),
                "rotation_rad":math.sin(i*.2)*.001,
                "logscale":math.sin(i*.23)*.0005,
                "trust":.80,"inliers":85,"accepted":True}
            out,mesh,virtual=c.step(i+4,1_000_000_000+i*33_777_777,
                                     q,rows,.05,vis if mode else None)
            assert out.mesh_uv is not None
            assert mesh.shape==(660,2) and np.isfinite(mesh).all()
            assert out.min_margin_source_px>=2 and out.zoom<=1.14
            assert out.source_id==i+4
            history.append({
              "qVirtual":virtual,"zoom":out.zoom,
              "visual":[out.visual_tx_source_px,out.visual_ty_source_px,
                        out.visual_rotation_rad,out.visual_scale],
              "min_margin":out.min_margin_source_px,
              "mesh":mesh})
    return history
def main():
    earlier=[.16*math.sin(i*.17) for i in range(50)]
    later1=earlier+[.24*math.sin(i*.20) for i in range(50,80)]
    later2=earlier+[-.28*math.sin(i*.44) for i in range(50,80)]
    result={}
    for mode in (0,1):
        a=one(mode,0,later1)
        b=one(mode,0,later2)
        assert len(a)==len(b)==80
        for i in range(50):
            for key in ("qVirtual","zoom","visual","min_margin","mesh"):
                assert np.allclose(a[i][key],b[i][key],rtol=0,atol=1e-13),(
                     "FUTURE_INFLUENCED_PREFIX",mode,i,key)
        result[f"mode_{mode}_causal_prefix_independent_50_frames"]=True
    with C2Shadow(True) as c:
        q=np.array([0.,0.,0.,1.])
        rows=np.tile(q,(20,1))
        out,_,_=c.step(4,1_000_000_000,q,rows,0)
        failed=0
        for sid,ts in ((4,1_030_000_000),(5,1_000_000_000),
                       (6,1_040_000_000)):
            try:c.step(sid,ts,q,rows,0)
            except RuntimeError:failed+=1
            else:raise AssertionError("C2 accepted broken SourceID/TS")
        assert failed==3
        out,_,_=c.step(5,1_033_777_777,q,rows,0)
        assert out.source_id==5 and out.min_margin_source_px>=2
        result["broken_SourceID_TS_rejected_without_corrupting_owner"]=True
    dest=Path("reports/gemini_tripod_v2/c2_shadow/c2_unit_tests.json")
    dest.parent.mkdir(parents=True,exist_ok=True)
    dest.write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding="utf-8")
    print("C2_ONE_OWNER_CAUSAL_PREFIX_ALTERNATE_FUTURE_NATIVE_20ROW_RS19_STOP_BAD_TS_PASS",
          json.dumps(result,ensure_ascii=False),flush=True)

if __name__=="__main__":main()
