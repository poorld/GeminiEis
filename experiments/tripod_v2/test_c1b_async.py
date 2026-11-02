#!/usr/bin/env python3
"""C1-B native async Host failure-injection and realtime-deadline contract.

Tests frame identity, source timestamp, no completed previous LK reuse,
bounded one-in-flight, overload, deadline expiration, cancellation, teardown.
"""
from __future__ import annotations
import json,time,ctypes
from pathlib import Path
import numpy as np,cv2
from c1b_native_async import NativeAsync

def images():
    rng=np.random.default_rng(452)
    im=np.zeros((180,320),np.uint8)
    for _ in range(1000):
        x=int(rng.integers(7,313));y=int(rng.integers(7,173))
        cv2.circle(im,(x,y),1,int(rng.integers(70,240)),-1)
    mask=np.full_like(im,255)
    nextim=cv2.warpAffine(im,np.float32([[1.,0,2.],[0,1.,0]]),
                          (320,180),flags=cv2.INTER_LINEAR)
    return im,nextim,mask

def main():
    a,b,mask=images()
    tests={}
    with NativeAsync() as worker:
        deadline=worker.now_ns()+150_000_000
        before=time.perf_counter()
        assert worker.submit(200,2_000_000_000,deadline,a,b,mask,mask)==0
        submission_ms=(time.perf_counter()-before)*1000
        # Even valid future SourceID cannot start until previous job
        # is committed or dropped; no hidden parallel frame backlog.
        assert worker.submit(201,2_033_333_333,worker.now_ns()+150_000_000,
                             a,b,mask,mask)==1
        assert worker.poll(201)[0]==3
        rc,result=worker.wait(200,deadline)
        assert rc==0,(rc,worker.stats())
        assert result.status==0 and abs(result.center_dx-2.)<.5
        assert worker.submit(199,2_066_666_666,worker.now_ns()+100_000_000,
                             a,b,mask,mask)==3
        assert worker.submit(202,2_066_666_666,worker.now_ns()+200_000_000,
                             a,b,mask,mask)==0
        rc,result=worker.wait(202,worker.now_ns()+200_000_000)
        assert rc==0
        s=worker.stats()
        assert s["busy"]==1 and s["ready_on_time"]==2
        assert s["rejected_order"]==1
        tests["same_frame_ready_and_backpressure"]={
           "status":"PASS","async_submit_ms":submission_ms,
           "accepted_frames":s["ready_on_time"],"busy_rejected":s["busy"],
           "old_source_id_never_promoted":True}
    with NativeAsync(85) as worker:
        deadline=worker.now_ns()+25_000_000
        assert worker.submit(100,1_000_000_000,deadline,a,b,mask,mask)==0
        t0=time.perf_counter()
        code,_=worker.wait(100,deadline)
        assert code==2,(code,worker.stats())
        # Worker still busy so current frame must not consume stale result.
        assert worker.poll(101)[0]==3
        time.sleep(.090)
        code2,_=worker.poll(100)
        assert code2==4,(code2,worker.stats())
        assert worker.submit(101,1_033_333_333,worker.now_ns()+35_000_000,
                             a,b,mask,mask)==0
        assert worker.cancel(101)==0
        assert worker.poll(101)[0]==4
        s=worker.stats()
        assert s["late"]==1 and s["cancelled"]==1,(s)
        tests["deadline_and_expired_cancel"]={
          "status":"PASS","forced_worker_delay_ms":85,
          "late":s["late"],"cancelled":s["cancelled"],
          "no_prev_visual_leaked":True,
          "wait_for_late_ms":(time.perf_counter()-t0)*1000}
    with NativeAsync(100) as worker:
        assert worker.submit(700,7_000_000_000,worker.now_ns()+150_000_000,
                             a,b,mask,mask)==0
        # Pending/active job must be joined/released safely on close.
    tests["teardown_inflight"]={"status":"PASS"}
    dest=Path("reports/gemini_tripod_v2/c1b_native/async_failure_injection.json")
    dest.parent.mkdir(parents=True,exist_ok=True)
    dest.write_text(json.dumps(tests,ensure_ascii=False,indent=2),encoding="utf-8")
    print("C1B_NATIVE_ASYNC_STRICT_SOURCEID_DEADLINE_BUSY_CANCEL_STOP_ALL_PASS",
          json.dumps(tests,ensure_ascii=False),flush=True)

if __name__=="__main__":main()
