#!/usr/bin/env python3
"""V2 pure governor causality, no-lookahead prefix, optical-center identity."""
import math
from pathlib import Path
import numpy as np
from zoom_scale_causal_governor import (
    ROOT, BUNDLES, causal_slew_logscale, causal_outlier_logscale, scale,
)
PIVOT=np.array([.5,.5,1.])
count=0
for case in ('024605','024618','024630','024647'):
    oracle=np.fromfile(BUNDLES/case/'oracle_matrices.f64',dtype=np.float64).reshape(-1,3,3)
    visual=np.fromfile(ROOT/f'visual_tripod_{case}.f64',dtype=np.float64).reshape(-1,3,3)
    extra=visual@np.linalg.inv(oracle)
    obs=scale(extra)
    assert len(obs)==291 and np.all(np.isfinite(obs))
    for rate in (.0015,.003):
        for jump in (0.0,.012):
            fn=(lambda z:causal_outlier_logscale(z,rate,jump) if jump else
                causal_slew_logscale(z,rate))
            whole=fn(obs)
            for stop in (1,2,10,40,67,68,80,145,291):
                # Prefix invariance proves THIS STATE MACHINE not leaking
                # future samples; it does not cleanse upstream Oracle.
                assert np.allclose(whole[:stop],fn(obs[:stop]),rtol=0,atol=1e-13)
            E=extra.copy()
            E[:,:2,:2]*=(whole/obs)[:,None,None]
            pos=np.einsum('nij,j->ni',oracle,PIVOT)
            pos=pos[:,:2]/pos[:,2,None]
            E[:,:2,2]+=np.einsum('nij,nj->ni',extra[:,:2,:2]-E[:,:2,:2],pos)
            final=E@oracle
            mapped=np.einsum('nij,j->ni',final,PIVOT)
            origin=np.einsum('nij,j->ni',visual,PIVOT)
            err=np.max(abs(mapped[:,:2]/mapped[:,2,None]-
                             origin[:,:2]/origin[:,2,None]))
            assert err<1e-10,(case,rate,jump,err)
            if not jump:assert max(abs(np.diff(np.log(whole))))<=rate+1e-9
            count+=1
print('TRIPOD_V2_PYTHON_CAUSAL_PREFIX_AND_OPTICAL_CENTER_PASS',
      'cases',4,'configurations',count,'upstream_noncausal_ORACLE_STILL_TRUE')
