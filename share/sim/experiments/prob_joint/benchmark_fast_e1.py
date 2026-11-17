#!/usr/bin/env python3
"""Jetson microbenchmark on actual fixed-cohort frames: original E1 vs exact fast E1.

Samples the original 894 NPZs; does NOT replace the full checkpointed evaluation.
Measures candidate generation+selection combined update() walltime, and asserts
q_ref and q_des equality for every sampled frame. No physical robot commands.

Usage (from share/sim):
 OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1 \
 python experiments/prob_joint/benchmark_fast_e1.py \
  --cohort outputs/ex04_prob_894/cohort_data --samples 60 --grid 15 \
  --out outputs/ex04_prob_894/fast_e1_pilot.json
"""
from __future__ import annotations
import argparse
import json
import sys
import time
from pathlib import Path
import numpy as np

SIM=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(SIM))
import baselines as bl
import kinematics as k
import run_baselines as rb
from prob_methods import ProbJointBranch


def main():
    ap=argparse.ArgumentParser()
    ap.add_argument("--cohort",type=Path,required=True)
    ap.add_argument("--samples",type=int,default=60)
    ap.add_argument("--grid",type=float,default=15.)
    ap.add_argument("--seed",type=int,default=6157)
    ap.add_argument("--out",type=Path)
    a=ap.parse_args()
    rng=np.random.default_rng(a.seed)
    manifest=json.loads((a.cohort/"manifest.json").read_text())
    files=sorted(item["file"] for item in manifest["files"])
    if not files:raise SystemExit("Empty cohort")
    m=k.Nero.from_urdf()
    measurements=[]
    for i in range(a.samples):
        path=a.cohort/files[int(rng.integers(len(files)))]
        with np.load(path) as d:
            frames=rb.frames(m,d,bl.PsiMap("mirror"))
            f=frames[int(rng.integers(len(frames)))]
        q0=rng.uniform(m.lower+.1,m.upper-.1)
        legacy=ProbJointBranch("E1",a.grid,fast_e1=False)
        fast=ProbJointBranch("E1",a.grid,fast_e1=True)
        legacy.reset(m,q0)
        fast.reset(m,q0)
        # Alternate evaluation order to reduce drift in apparent timing.
        methods=(("legacy",legacy),("fast",fast))
        if i%2:methods=methods[::-1]
        vals={}
        for label,meth in methods:
            t=time.perf_counter()
            meth.update(f,q0.copy())
            vals[label]=(meth.q_ref,meth.q_des,(time.perf_counter()-t)*1000)
        oldref,olddes,oldt=vals["legacy"]
        newref,newdes,newt=vals["fast"]
        if (oldref is None)!=(newref is None):
            raise AssertionError(f"reference fallback changed: {path}/{i}")
        if oldref is not None:
            if np.max(abs(k.wrap(oldref-newref)))>1e-8:
                raise AssertionError(f"canonical reference changed: {path}/{i}")
        if np.max(abs(k.wrap(olddes-newdes)))>1e-8:
            raise AssertionError(f"requested target changed: {path}/{i}")
        measurements.append(dict(file=str(path),legacy_ms=oldt,fast_ms=newt,
                                 candidate_count=fast.last_candidate_count,
                                 psi_evaluated=(fast.psi_evaluated_hist[-1] if fast.psi_evaluated_hist else None),
                                 psi_total=(fast.psi_total_hist[-1] if fast.psi_total_hist else None)))
        if (i+1)%10==0:print(f"PASS {i+1}/{a.samples} input frames matched; timing collected",flush=True)
    old=np.asarray([r["legacy_ms"] for r in measurements])
    fast=np.asarray([r["fast_ms"] for r in measurements])
    report={"type":"JETSON_FIXED_COHORT_MICROBENCHMARK",
            "grid_deg":a.grid,"samples":a.samples,"seed":a.seed,
            "all_references_and_targets_equivalent":True,
            "mean_ms":{"legacy":float(old.mean()),"fast":float(fast.mean())},
            "p95_ms":{"legacy":float(np.percentile(old,95)),
                       "fast":float(np.percentile(fast,95))},
            "speedup_mean_ratio":float(old.mean()/fast.mean()),
            "camera15hz_deadline_miss_pct":{"legacy":float(100*np.mean(old>1000/15)),
                                             "fast":float(100*np.mean(fast>1000/15))},
            "mean_psi_evaluated_ratio":float(np.mean([r["psi_evaluated"]/r["psi_total"] for r in measurements if r["psi_total"]])),
            "limitations":["Performance reflects this CPU and existing load; report hardware, governor and BLAS thread count",
                           "Single update only; excludes perception, ROS, actuator and continuous backlog",
                           "Each frame is an independently sampled snapshot; full stateful 894 evaluation is separate"],
            "frames":measurements}
    print(json.dumps({k:v for k,v in report.items() if k!="frames"},indent=2))
    if a.out:
        a.out.parent.mkdir(parents=True,exist_ok=True)
        a.out.write_text(json.dumps(report,indent=2)+"\n")
if __name__=="__main__":main()
