#!/usr/bin/env python3
"""Build an EXACT 894-trajectory cohort, audit method availability and run readiness.

This does not execute a simulation. The original EX04 full2 per_traj.csv is the
authority for the fixed cohort; the synthetic manifest may have 899 files.
Run in a checkout containing the original Jetson-local EX04 outputs.
"""
from __future__ import annotations
import argparse
import csv
import hashlib
import json
import os
import subprocess
import sys
from pathlib import Path

def sha256(path):
    h = hashlib.sha256()
    with open(path, "rb") as f:
        while True:
            b=f.read(1024*1024)
            if not b: break
            h.update(b)
    return h.hexdigest()

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument("--data", type=Path, default=Path("share/sim/outputs/synthetic"))
    p.add_argument("--reference-csv", type=Path, default=Path("share/sim/outputs/baselines_full2/per_traj.csv"))
    p.add_argument("--out", type=Path, default=Path("share/sim/outputs/ex04_prob_894"))
    p.add_argument("--expected", type=int, default=894)
    p.add_argument("--require-methods", nargs="+", default=["B7","B8","P","B7c","Pc","E0","E1","E2","E3"])
    args=p.parse_args()
    data=args.data.resolve()
    ref=args.reference_csv.resolve()
    if not (data/"manifest.json").is_file(): sys.exit(f"DATA MISSING: {data/'manifest.json'}")
    if not ref.is_file(): sys.exit(f"REFERENCE MISSING: {ref}. Need the Jetson-local 894-run CSV.")
    manifest=json.loads((data/"manifest.json").read_text())
    filemap={item["file"]:item for item in manifest["files"]}
    with open(ref,newline="") as f:
        rr=list(csv.DictReader(f))
    if not rr or "group" not in rr[0] or "file" not in rr[0] or "method" not in rr[0]:
        sys.exit("Invalid reference CSV: expected group,file,method columns")
    cohort=sorted({f'{r["group"]}/{r["file"]}' for r in rr})
    if len(cohort)!=args.expected:
        sys.exit(f"COHORT ERROR: reference CSV has {len(cohort)} unique trajectories, expected {args.expected}")
    per_method={}
    for r in rr: per_method.setdefault(r["method"],set()).add(f'{r["group"]}/{r["file"]}')
    for mid,s in sorted(per_method.items()):
        if s != set(cohort):sys.exit(f"INCOMPLETE REFERENCE: {mid}: {len(s)}/{len(cohort)}; aborting")
    missing=[f for f in cohort if f not in filemap or not (data/f).is_file()]
    if missing: sys.exit(f"DATA MISSING: {len(missing)} missing trajectories, first={missing[:5]}")
    out=args.out.resolve()
    out.mkdir(parents=True,exist_ok=True)
    selected=out/"cohort_data"
    selected.mkdir(exist_ok=True)
    for sub in sorted({Path(f).parent for f in cohort}): (selected/sub).mkdir(parents=True,exist_ok=True)
    for f in cohort:
        link=selected/f
        src=data/f
        if link.is_symlink() and link.resolve()==src:continue
        if link.exists() or link.is_symlink():sys.exit(f"REFUSING TO OVERWRITE: {link}")
        link.symlink_to(src)
    new=dict(manifest)
    new["files"]=[filemap[f] for f in cohort]
    new["cohort_origin_csv"]=str(ref)
    new["cohort_origin_sha256"]=sha256(ref)
    new["cohort_size"]=len(cohort)
    new["original_manifest_sha256"]=sha256(data/"manifest.json")
    tmp=selected/"manifest.json.tmp"
    tmp.write_text(json.dumps(new,indent=2,ensure_ascii=False)+"\n")
    os.replace(tmp,selected/"manifest.json")
    (out/"cohort_894.json").write_text(json.dumps({
      "n":len(cohort),"methods_in_reference":{k:len(v) for k,v in per_method.items()},
      "reference_sha256":sha256(ref),"original_manifest_sha256":sha256(data/"manifest.json"),
      "paths":cohort},indent=2)+"\n")
    print(f"PASS: fixed {len(cohort)} exact historical trajectories -> {selected}",flush=True)
    print(f"Reference methods: {sorted(per_method)}",flush=True)
    sim=Path(__file__).resolve().parents[2]
    sys.path.insert(0,str(sim))
    runner=sim/"run_baselines.py"
    source=runner.read_text()
    absent=[mid for mid in args.require_methods if mid not in ("E0","E1","E2","E3") and f'"{mid}"' not in source]
    # Method availability check is intentionally conservative; registration/behavior must still be reviewed.
    print(f"Runner: {runner}")
    if absent:
        print(f"BLOCKED: methods not registered in runner: {absent}")
        print("Merge the Jetson-local P/Pc first. E0-E3 are registered by run_894_methods.py.")
        print("Do not treat this as a completed 894-run. No simulation was executed.")
        return 2
    print("PRECHECK PASSED: verify the registered P/Pc methods and shared shield / metrics.")
    print("Next command (same fixed cohort):")
    print(f"  cd {sim} && OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1 python -u experiments/prob_joint/run_894_methods.py {selected} --psi-map mirror --methods {' '.join(args.require_methods)} --out {out/'solver_results'}")
    return 0

if __name__=="__main__":raise SystemExit(main())
