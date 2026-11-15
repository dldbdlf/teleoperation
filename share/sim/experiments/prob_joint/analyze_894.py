#!/usr/bin/env python3
"""Audit all methods against the EXACT same 894 trajectory keys and report paired CI.

Use alongside prepare_894.py and run_894_methods.py. Does not claim G1
PASS automatically: pre-registered primary measure and safety must be checked.
"""
from __future__ import annotations
import argparse
import csv
import json
from collections import defaultdict
from pathlib import Path
import numpy as np

METRICS=("wrist_mm_p50","wrist_mm_p95","wrist_deg_p50","wrist_deg_p95",
         "psi_p90","jerk_rms","recovery_jump_deg","recovery_path_deg",
         "held_pct","runtime_p95_ms",
         "joint_limit_margin_min_deg","deadline_miss_pct",
         "branch_switch_count_exact","recovery_time_s")

def read(path):
    with open(path,newline="") as f: return list(csv.DictReader(f))

def float_or_nan(v):
    try: return float(v)
    except (ValueError,TypeError):return float("nan")

def mean_ci(x,n=2000,seed=1903):
    x=np.asarray(x,float);x=x[np.isfinite(x)]
    if not len(x):return None
    rng=np.random.default_rng(seed)
    b=np.empty(n)
    for j in range(n):b[j]=rng.choice(x,size=len(x),replace=True).mean()
    return dict(n=len(x),mean=float(x.mean()),lo=float(np.quantile(b,.025)),hi=float(np.quantile(b,.975)))

def main():
    p=argparse.ArgumentParser()
    p.add_argument("new_csv",type=Path)
    p.add_argument("--old-csv",type=Path)
    p.add_argument("--cohort",type=Path,help="prepare_894.py cohort_894.json")
    p.add_argument("--out",type=Path,default=Path("share/sim/outputs/ex04_prob_894/analysis.md"))
    p.add_argument("--expected",type=int,default=894)
    args=p.parse_args()
    data=read(args.new_csv)
    if not data or any(k not in data[0] for k in ("group","file","method")):
        raise SystemExit("Invalid new CSV")
    ix={}
    methods=set()
    for row in data:
        key=(row["group"],row["file"],row["method"])
        if key in ix: raise SystemExit(f"Duplicate result row: {key}")
        ix[key]=row; methods.add(row["method"])
    cohort=sorted({(g,f) for g,f,_ in ix})
    if len(cohort)!=args.expected:
        raise SystemExit(f"Expected {args.expected} trajectories, got {len(cohort)}")
    if args.cohort:
        declared=json.loads(args.cohort.read_text())
        expected_keys={(str(Path(k).parent),Path(k).name) for k in declared["paths"]}
        if set(cohort)!=expected_keys: raise SystemExit("Cohort mismatch vs fixed historical 894 keys")
    for method in methods:
        got={(g,f) for g,f,m in ix if m==method}
        if got!=set(cohort):raise SystemExit(f"Incomplete method {method}: {len(got)}/{len(cohort)}")
    methods=sorted(methods)
    old=read(args.old_csv) if args.old_csv else []
    oldix={(r["group"],r["file"],r["method"]):r for r in old}
    lines=[
        "# EX04 894-trajectory paired revalidation",
        "",
        f"- New CSV: \`{args.new_csv}\`",
        f"- Exact trajectory count: **{len(cohort)}**",
        f"- Complete methods: **{', '.join(methods)}**",
        "- Aggregation: mean of each trajectory's metric; not a pooled per-frame percentile",
        "- No physical robot test; numeric results are from the local runner only",
        "",
        "## Summary (lower is better for errors, jerk and latency)",
        "",
        "| Method | "+ " | ".join(METRICS)+" |",
        "|"+"---|"*(len(METRICS)+1),
    ]
    for method in methods:
        vs=[ix[g,f,method] for g,f in cohort]
        vals=[]
        for met in METRICS:
            if met not in vs[0]:vals.append("NOT MEASURED");continue
            a=np.array([float_or_nan(v.get(met)) for v in vs])
            vals.append(f"{np.nanmean(a):.3f}" if np.isfinite(a).any() else "NOT MEASURED")
        lines.append("| "+method+" | "+" | ".join(vals)+" |")
    lines+=["","## Paired bootstrap (2000 trajectory-level resamples)","",
            "Differences are first method minus second. A negative difference is favorable for error/jump/jerk.",
            ""]
    pairs=[("P","B7"),("Pc","P"),("E1","E0"),("E2","E1"),("E3","E1"),
           ("E1","P"),("E2","P"),("E3","P"),("E1","Pc"),("E2","Pc"),("E3","Pc")]
    for a,b in pairs:
        if a not in methods or b not in methods:continue
        lines.append(f"### {a} - {b}")
        lines.append("| Metric | n paired | Delta mean | 95% CI |")
        lines.append("|---|---:|---:|---|")
        for metric in METRICS:
            diffs=[]
            for g,f in cohort:
                v=float_or_nan(ix[g,f,a].get(metric))
                w=float_or_nan(ix[g,f,b].get(metric))
                if np.isfinite(v) and np.isfinite(w):diffs.append(v-w)
            d=mean_ci(diffs)
            if d:lines.append(f"| {metric} | {d['n']} | {d['mean']:+.3f} | [{d['lo']:+.3f}, {d['hi']:+.3f}] |")
            else:lines.append(f"| {metric} | 0 | NOT MEASURED | — |")
        lines.append("")
    lines+=["## Occlusion stratification", "", "| Group | Count | Method | Wrist P50 mm | psi P90 deg | Recovery jump deg |", "|---|---:|---|---:|---:|---:|"]
    groups=sorted({g for g,f in cohort})
    for group in groups:
        keys=[(g,f) for g,f in cohort if g==group]
        for method in methods:
            def avg(m):
                x=[float_or_nan(ix[g,f,method].get(m)) for g,f in keys]
                finite=[v for v in x if np.isfinite(v)]
                return f"{np.mean(finite):.2f}" if finite else "NOT MEASURED"
            lines.append(f"| {group} | {len(keys)} | {method} | {avg('wrist_mm_p50')} | {avg('psi_p90')} | {avg('recovery_jump_deg')} |")
    if old:
        lines+=["","## Baseline rerun consistency (existing original 894 results)","",
                "Per-trajectory numeric max absolute difference; runtime excluded because it is load-dependent.",
                ""]        
        for method in sorted(set(methods) & {r["method"] for r in old}):
            for metric in ("wrist_mm_p50","wrist_mm_p95","psi_p90","jerk_rms","recovery_jump_deg"):
                diffs=[]
                for g,f in cohort:
                    key=(g,f,method)
                    if key not in oldix:raise SystemExit(f"Historical baseline missing {key}")
                    v=float_or_nan(ix[key].get(metric));w=float_or_nan(oldix[key].get(metric))
                    if np.isfinite(v) and np.isfinite(w):diffs.append(abs(v-w))
                if diffs:lines.append(f"- {method} {metric}: max absolute diff {max(diffs):.12g}")
    lines+=["","## Interpretation requirements","",
       "- Verify original P/Pc are imported from the unpushed local branch, not aliases of E methods.",
       "- Solver timing on Jetson must include candidate enumeration and state tracking.",
       "- Solver P95 per-trajectory is not an exact camera-frame deadline miss measurement.",
       "- Missing collision/acceleration safety or raw framewise metrics must be labelled NOT MEASURED.",
       "- No G1 PASS without the exact pre-registered baseline, subgroup and primary endpoint.",
       "- The 894 episodes are *synthetic Oracle belief*, not D415 perception success.",
       ""]
    args.out.parent.mkdir(parents=True,exist_ok=True)
    args.out.write_text("\n".join(lines)+"\n")
    print(f"Validated {len(cohort)} exact episodes, {len(methods)} methods. Report: {args.out}")

if __name__=="__main__":main()
