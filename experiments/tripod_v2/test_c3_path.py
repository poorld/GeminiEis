#!/usr/bin/env python3
"""V2C C3 native C++ Global Path + causal yaw halfcycle regression tests.
Ablations: full global 4DoF, forecast full budget, forecast follow-only.
The V1 noncausal Gold is never opened by this test.
"""
from __future__ import annotations
import json,math
from pathlib import Path
import numpy as np
from c1_gyro_rs import quat_from_rotvec
from c2_joint_native import C2Shadow

ROOT=Path(__file__).resolve().parents[2]
DLLS={
  "global4dof":ROOT/"reports/gemini_tripod_v2/c2_shadow/bin/c3_global4dof.dll",
  "forecast_full":ROOT/"reports/gemini_tripod_v2/c2_shadow/bin/c3_global4dof_forecast.dll",
  "yaw_follow_only":ROOT/"reports/gemini_tripod_v2/c2_shadow/bin/c3_yaw_follow_only.dll"
}
def simulate(dll,seed_future,n=100,need_output=True):
    output=[]
    with C2Shadow(visual_enabled=True,dll=dll) as engine:
        for t in range(n):
            if t<60:
                ang=.13*math.sin(t*.13)
            elif seed_future==1:
                ang=.12*math.sin(t*.14)
            else:
                ang=.12*math.sin((t-60)*.3)+.16
            q=quat_from_rotvec(np.asarray([.004,ang,.002]))
            rows=np.stack([quat_from_rotvec(
                np.asarray([.004,ang+(j-9.5)*.00032,.002]))
                for j in range(20)])
            yaw=.7*math.cos(t*.17)*(1 if t<60 else seed_future)
            visual={
              "dx_feature_px":.4*math.sin(t*.31),
              "dy_feature_px":.13*math.cos(t*.18),
              "rotation_rad":.0013*math.sin(t*.24),
              "logscale":.0007*math.cos(t*.22),
              "trust":.82,"inliers":90,"accepted":True
            }
            ret,mesh,target=engine.step(4+t,1_000_000_000+33_777_777*t,
                                       q,rows,yaw,visual)
            assert ret.code==0 and ret.min_margin_source_px>=2.0
            assert 1.0<=ret.zoom<=1.14
            assert mesh.shape==(660,2) and np.isfinite(mesh).all()
            assert all(0<=x<=1 for x in mesh.ravel())
            output.append((
                target.copy(),mesh.copy(),ret.zoom,
                ret.visual_tx_source_px,ret.visual_ty_source_px,
                ret.visual_rotation_rad,ret.visual_scale))
    return output

def main():
    gates={}
    for label,lib in DLLS.items():
        assert lib.is_file(),lib
        a,b=simulate(lib,1),simulate(lib,-1)
        assert len(a)==len(b)==100
        for f in range(60):
            for x,y in zip(a[f],b[f]):
                assert np.array_equal(x,y),f"noncausal C3 {label} frame {f}"
        # Changing future after frame 60 must affect later frames,
        # without changing any of the shared 0..59 decisions.
        differs=sum(1 for k in range(60,100) if not np.array_equal(
                    a[k][1],b[k][1]))
        assert differs>0,(label,"inert C3")
        with C2Shadow(True,dll=lib) as s:
            q=np.array([0.,0.,0.,1.],float)
            row=np.tile(q,(20,1))
            r,_,_=s.step(4,1_000_000_000,q,row,.1)
            assert r.source_id==4
            for idx,t in ((4,1_100_000_000),(5,1_000_000_000),(6,1_300_000_000)):
                try:s.step(idx,t,q,row,.1)
                except RuntimeError:pass
                else:raise AssertionError((label,"accepted stale SourceID"))
            r,_,_=s.step(5,1_033_777_777,q,row,.1)
            assert r.source_id==5
        gates[label]={
           "same_first_60_with_different_future":"PASS",
           "later_future_actually_responds":differs,
           "no_unsafe_mesh":"PASS",
           "single_Zoom_Visual_qVirtual_atomic":"PASS",
           "bad_TS_source_ID_refused":"PASS"
        }
    out=ROOT/"reports/gemini_tripod_v2/c3_global_path/c3_causal_unit_gates.json"
    out.parent.mkdir(parents=True,exist_ok=True)
    out.write_text(json.dumps(gates,ensure_ascii=False,indent=2),encoding="utf-8")
    print("C3_GLOBAL_PATH_3_NATIVE_MODES_CAUSAL_60F_PREFIX_AND_BROKEN_TS_RS19_PASS",
          json.dumps(gates,ensure_ascii=False),flush=True)
if __name__=="__main__":main()
