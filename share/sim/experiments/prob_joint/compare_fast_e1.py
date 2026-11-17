#!/usr/bin/env python3
"""Compare original E1 vs exact-accelerated E1 on IDENTICAL trajectory keys.

Strictly audit ALL task-quality metrics and show Jetson camera-update
deadline/IK timing speedups. Runtime, deadline, entropy and candidate-count
diagnostics are excluded from numerical equivalence because they depend on
measurement load, or fast E1 prunes the full posterior by construction.

Example:
 python compare_fast_e1.py \
  --legacy ../../outputs/ex04_prob_894/run_all/per_traj.csv \
  --fast ../../outputs/ex04_prob_894/run_e1_fast15/per_traj.csv \
  --grid 15 --expected 894 \
  --out ../../../11.17_exp/fast_e1_comparison.json
"""
from __future__ import annotations
import argparse
import csv
import json
from pathlib import Path
import numpy as np

IGNORED_PREFIXES=("runtime_","deadline_","cand_gen_","selection_","branch_entropy_",
                  "branch_prob_change_","candidate_count_")
IGNORED_FIELDS={"method","group","file","grid","fallback_pct",
                "branch_switch_count_exact","branch_switch_proxy"}
# Meta and wallclock-related data must NOT be used for correctness comparison.
KEY_FIELDS=("wrist_mm_p50","wrist_mm_p95","wrist_deg_p50","wrist_deg_p95",
            "psi_p50","psi_p90","psi_p95","jerk_rms","jerk_p95",
            "recovery_jump_deg","recovery_path_deg","recovery_time_s",
            "recovery_capped_pct","joint_limit_margin_min_deg","joint_dev_deg_p50",
            "joint_dev_deg_p90","held_pct","solver_fail_pct",
            "qd_rms_deg_s","qd_p95_deg_s","qdd_rms_deg_s2","qdd_p95_deg_s2")

def read(path,grid):
    with path.open(newline="") as f:
        rows=list(csv.DictReader(f))
    result={}
    for row in rows:
        if row.get("method")!="E1" or str(row.get("grid"))!=str(grid):
            continue
        key=(row["group"],row["file"])
        if key in result:raise ValueError(f"Duplicate E1 trajectory {key} in {path}")
        result[key]=row
    return result

def num(value):
    try:return float(value)
    except (TypeError,ValueError):return float("nan")

def collect(rows,field):
    return np.asarray([num(v.get(field)) for v in rows],float)

def main():
    p=argparse.ArgumentParser()
    p.add_argument("--legacy",required=True,type=Path)
    p.add_argument("--fast",required=True,type=Path)
    p.add_argument("--grid",default="15")
    p.add_argument("--expected",type=int,default=894)
    p.add_argument("--tol",type=float,default=1e-6)
    p.add_argument("--out",type=Path)
    a=p.parse_args()
    old=read(a.legacy,a.grid)
    new=read(a.fast,a.grid)
    if len(old)!=a.expected or len(new)!=a.expected or old.keys()!=new.keys():
        raise SystemExit(f"ABORT cohort mismatch: legacy={len(old)}, fast={len(new)}, intersection={len(old.keys()&new.keys())}")
    maxdiff={}
    for field in KEY_FIELDS:
        differences=[]
        for key in sorted(old):
            x=num(old[key].get(field));y=num(new[key].get(field))
            if np.isnan(x) and np.isnan(y):continue
            if not (np.isfinite(x) and np.isfinite(y)):
                raise SystemExit(f"Mismatch missing/finite: {key} {field} old={x} fast={y}")
            differences.append(abs(x-y))
        maxdiff[field]=float(max(differences)) if differences else None
        if differences and max(differences)>a.tol:
            raise SystemExit(f"QUALITY REGRESSION: {field} max diff={max(differences)} > {a.tol}")
    rows_old=[old[k] for k in sorted(old)]
    rows_new=[new[k] for k in sorted(new)]
    def stat(rowset,field):
        arr=collect(rowset,field);arr=arr[np.isfinite(arr)]
        if not len(arr):return None
        return {"n":len(arr),"p50":float(np.median(arr)), "p95":float(np.percentile(arr,95)),
                "mean":float(arr.mean())}
    payload={
      "comparison":"original E1 vs exact certified fast E1",
      "n_trajectories":len(old),
      "grid":str(a.grid),
      "quality_equivalent":True,
      "quality_max_abs_diffs":maxdiff,
      "note":"Both per-trajectory P95 and median-of-trajectory P95 are reported; neither is the pooled per-frame P95",
      "jetson_runtime":{},
    }
    for field in ("runtime_p95_ms","deadline_miss_pct","cand_gen_p95_ms","selection_p95_ms"):
        payload["jetson_runtime"][field]={"legacy":stat(rows_old,field),"fast":stat(rows_new,field)}
    if a.out:
        a.out.parent.mkdir(parents=True,exist_ok=True)
        a.out.write_text(json.dumps(payload,ensure_ascii=False,indent=2)+"\n")
    print(json.dumps(payload,ensure_ascii=False,indent=2))
    print("PASS: all available task-quality metrics equivalent within tolerance.")
if __name__=="__main__":main()
