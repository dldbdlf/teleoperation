#!/usr/bin/env python3
"""Full statistical analysis for the EX04 894-trajectory revalidation (2026-11-15/16 prompt,
sections 6/9/10). Supersedes analyze_894.py for this run: that script assumes one row per
(group, file, method) and raises "Duplicate result row" the moment a grid-dependent method (E0-E3,
B7c, Pc) appears more than once for the same file at different psi-grid resolutions -- which is
exactly what the 3-grid ablation (10/15/20deg) produces. This script keys on
(group, file, method, grid) instead and adds the grid-ablation / E1-vs-E2-vs-E3 / paired-bootstrap
tables the prompt requires.

Usage:
  python3 analyze_ex04.py --csv outputs/ex04_prob_894/run_all/per_traj.csv \
                          [--csv outputs/ex04_prob_894/run_probdiag/per_traj.csv] \
                          --cohort outputs/ex04_prob_894/cohort_894.json \
                          --out-dir ../../../11.15_exp

When more than one --csv is given, later files' rows for an overlapping
(group,file,method,grid) key replace earlier ones' extra columns (used to merge the probdiag
supplement's branch-entropy/candidate-timing columns onto the main run's rows without discarding
either -- the two runs' shared, non-diagnostic metrics are bit-for-bit reproductions of each other,
same code path, same inputs, verified by the kill-test resume comparison).
"""
from __future__ import annotations

import argparse
import csv
import json
from collections import defaultdict
from pathlib import Path

import numpy as np

NON_GRID = ("B7", "B8", "P")
GRID15_ONLY = ("B7c", "Pc")
ABLATION = ("E0", "E1", "E2", "E3")
STAGE15_METHODS = ("B7", "B8", "P", "B7c", "Pc", "E0", "E1", "E2", "E3")

CORE_METRICS = [
    "wrist_mm_p50", "wrist_mm_p95", "wrist_deg_p50", "wrist_deg_p95",
    "psi_feasible_pct", "psi_p50", "psi_p90", "psi_p95", "regret_p50",
    "psi_jumps", "jerk_rms", "jerk_p95", "joint_jump_max_deg",
    "qd_rms_deg_s", "qd_p95_deg_s", "qdd_rms_deg_s2", "qdd_p95_deg_s2",
    "joint_limit_margin_min_deg", "branch_switch_count_exact", "branch_switch_proxy",
    "joint_dev_deg_p50", "joint_dev_deg_p90",
    "recovery_jump_deg", "recovery_path_deg", "recovery_capped_pct", "recovery_time_s",
    "held_pct", "solver_fail_pct", "deadline_miss_pct",
    "runtime_p50_ms", "runtime_p95_ms", "runtime_p99_ms",
]
PROB_METRICS = [
    "cand_gen_p50_ms", "cand_gen_p95_ms", "cand_gen_p99_ms",
    "selection_p50_ms", "selection_p95_ms", "selection_p99_ms",
    "branch_entropy_p50", "branch_entropy_p90",
    "branch_prob_change_p50", "branch_prob_change_p90",
    "candidate_count_mean", "fallback_pct",
]
ALL_METRICS = CORE_METRICS + PROB_METRICS


def f_or_nan(v):
    try:
        x = float(v)
        return x
    except (TypeError, ValueError):
        return float("nan")


def load_rows(csv_paths):
    merged = {}
    for p in csv_paths:
        with open(p, newline="") as fh:
            for row in csv.DictReader(fh):
                key = (row["group"], row["file"], row["method"], row.get("grid", "na"))
                if key in merged:
                    merged[key].update(row)
                else:
                    merged[key] = dict(row)
    return list(merged.values())


def verify_cohort(rows, cohort_json, expected=894):
    files = sorted({(r["group"], r["file"]) for r in rows})
    if len(files) != expected:
        raise SystemExit(f"[FAIL] expected {expected} distinct trajectory files, found {len(files)}")
    if cohort_json:
        declared = json.loads(Path(cohort_json).read_text())
        want = {(str(Path(p).parent), Path(p).name) for p in declared["paths"]}
        got = set(files)
        if want != got:
            missing = want - got
            extra = got - want
            raise SystemExit(f"[FAIL] cohort mismatch vs cohort_894.json: "
                              f"{len(missing)} missing, {len(extra)} unexpected (e.g. {list(missing)[:3] or list(extra)[:3]})")
    return files


def group_by(rows, keyfn):
    out = defaultdict(list)
    for r in rows:
        out[keyfn(r)].append(r)
    return out


def col_values(rows, metric):
    return np.array([f_or_nan(r.get(metric)) for r in rows], float)


def summary_row(rows, metric):
    v = col_values(rows, metric)
    v = v[np.isfinite(v)]
    if not len(v):
        return dict(n=0, mean=float("nan"), p50=float("nan"), p90=float("nan"), p95=float("nan"))
    return dict(n=len(v), mean=float(v.mean()), p50=float(np.percentile(v, 50)),
                p90=float(np.percentile(v, 90)), p95=float(np.percentile(v, 95)))


def paired_bootstrap_diff(rows_a, rows_b, metric, n_boot=2000, seed=1915):
    """Paired (per-trajectory) bootstrap 95% CI for mean(A) - mean(B) on `metric`, resampling the
    common set of (group,file) trajectories jointly so A and B always see the same resample --
    this is paired, not two independent single-sample bootstraps, so it is valid even though A and
    B are not independent (same trajectories, same occlusion draws)."""
    by_a = {(r["group"], r["file"]): f_or_nan(r.get(metric)) for r in rows_a}
    by_b = {(r["group"], r["file"]): f_or_nan(r.get(metric)) for r in rows_b}
    common = [k for k in by_a if k in by_b and np.isfinite(by_a[k]) and np.isfinite(by_b[k])]
    if len(common) < 10:
        return dict(n=len(common), diff=float("nan"), lo=float("nan"), hi=float("nan"))
    a = np.array([by_a[k] for k in common])
    b = np.array([by_b[k] for k in common])
    obs = float(a.mean() - b.mean())
    rng = np.random.default_rng(seed)
    n = len(common)
    boot = np.empty(n_boot)
    for i in range(n_boot):
        idx = rng.integers(0, n, n)
        boot[i] = a[idx].mean() - b[idx].mean()
    return dict(n=n, diff=obs, lo=float(np.quantile(boot, 0.025)), hi=float(np.quantile(boot, 0.975)))


def scenario_key(group):
    """S1..S3 are plain scenario ids; S4/S5/S6 carry an occlusion-duration suffix
    (S4_drop200ms, S4_drop500ms, S4_drop1000ms, S4_drop2000ms, ...). Returns (base, duration)."""
    if "_drop" in group:
        base, dur = group.split("_drop", 1)
        return base, dur.rstrip("s")
    return group, None


def fmt(x, nd=2):
    if x is None or (isinstance(x, float) and not np.isfinite(x)):
        return "n/a"
    return f"{x:.{nd}f}"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--csv", action="append", required=True, dest="csvs")
    ap.add_argument("--cohort", type=Path, default=None)
    ap.add_argument("--expected", type=int, default=894)
    ap.add_argument("--out-dir", type=Path, required=True)
    args = ap.parse_args()

    rows = load_rows(args.csvs)
    files = verify_cohort(rows, args.cohort, args.expected)
    methods_present = sorted({r["method"] for r in rows})
    grids_present = sorted({r.get("grid", "na") for r in rows})
    args.out_dir.mkdir(parents=True, exist_ok=True)

    lines = []
    lines.append("# EX04 894-trajectory statistical analysis\n")
    lines.append(f"- Input CSVs: {', '.join(str(c) for c in args.csvs)}")
    lines.append(f"- Distinct trajectory files: {len(files)} (expected {args.expected})")
    lines.append(f"- Methods present: {methods_present}")
    lines.append(f"- Grid values present: {grids_present}")
    lines.append(f"- Total rows: {len(rows)}\n")

    # ---------------- 1) 9-method comparison at 15deg (apples-to-apples condition) ----------------
    by_mg = group_by(rows, lambda r: (r["method"], r.get("grid", "na")))
    lines.append("## 1. 9-method comparison @ psi grid 15deg (same cohort/conditions)\n")
    header = "| method | n | " + " | ".join(CORE_METRICS[:12]) + " |"
    lines.append(header)
    lines.append("|---" * (2 + 12) + "|")
    stage15_table = []
    for mid in STAGE15_METHODS:
        gkey = "na" if mid in NON_GRID else "15"
        rs = by_mg.get((mid, gkey), [])
        if not rs:
            lines.append(f"| {mid} | 0 | " + " | ".join(["MISSING"] * 12) + " |")
            continue
        cells = [fmt(summary_row(rs, c)["p50"]) for c in CORE_METRICS[:12]]
        lines.append(f"| {mid} | {len(rs)} | " + " | ".join(cells) + " |")
        stage15_table.append((mid, len(rs)))
    lines.append("")
    lines.append("(table above shows per-metric P50 across the 894-trajectory distribution; "
                  "full P50/P90/P95/mean per metric for every method x grid is in "
                  "ex04_full_summary.csv next to this report)\n")

    # ---------------- 2) grid ablation (E0-E3 @ 10/15/20) ----------------
    lines.append("## 2. psi-grid ablation: E0-E3 @ 10 / 15 / 20 deg\n")
    for mid in ABLATION:
        lines.append(f"### {mid}")
        lines.append("| grid | n | " + " | ".join(["wrist_mm_p50", "psi_p50", "psi_feasible_pct",
                     "held_pct", "runtime_p95_ms", "cand_gen_p95_ms", "branch_entropy_p50"]) + " |")
        lines.append("|---" * 8 + "|")
        for g in ("10", "15", "20"):
            rs = by_mg.get((mid, g), [])
            if not rs:
                lines.append(f"| {g} | 0 | " + " | ".join(["MISSING"] * 7) + " |")
                continue
            cells = [fmt(summary_row(rs, c)["p50"]) for c in
                     ("wrist_mm_p50", "psi_p50", "psi_feasible_pct", "held_pct",
                      "runtime_p95_ms", "cand_gen_p95_ms", "branch_entropy_p50")]
            lines.append(f"| {g} | {len(rs)} | " + " | ".join(cells) + " |")
        lines.append("")

    # ---------------- 3) E1 vs E2 vs E3, and vs E0 (paired bootstrap, 15deg) ----------------
    lines.append("## 3. E0 vs E1 vs E2 vs E3 @ 15deg (paired bootstrap, 95% CI, n=2000)\n")
    pairs = [("E1", "E0"), ("E2", "E0"), ("E3", "E0"), ("E2", "E1"), ("E3", "E2"), ("E3", "E1")]
    lines.append("| A | B | metric | n_pairs | mean(A)-mean(B) | 95% CI |")
    lines.append("|---|---|---|---|---|---|")
    for a, b in pairs:
        ra, rb = by_mg.get((a, "15"), []), by_mg.get((b, "15"), [])
        for metric in ("wrist_mm_p50", "psi_p50", "held_pct", "joint_dev_deg_p50"):
            res = paired_bootstrap_diff(ra, rb, metric)
            lines.append(f"| {a} | {b} | {metric} | {res['n']} | {fmt(res['diff'])} | "
                          f"[{fmt(res['lo'])}, {fmt(res['hi'])}] |")
    lines.append("")

    # ---------------- 4) improvement over P / Pc (paired bootstrap, 15deg) ----------------
    lines.append("## 4. Probabilistic methods (E0-E3) vs P / Pc @ 15deg (paired bootstrap)\n")
    lines.append("| method | baseline | metric | n_pairs | mean(method)-mean(baseline) | 95% CI |")
    lines.append("|---|---|---|---|---|---|")
    for mid in ABLATION:
        rs = by_mg.get((mid, "15"), [])
        for base in ("P", "Pc"):
            bkey = "na" if base == "P" else "15"
            rb = by_mg.get((base, bkey), [])
            for metric in ("wrist_mm_p50", "psi_p50", "held_pct"):
                res = paired_bootstrap_diff(rs, rb, metric)
                lines.append(f"| {mid} | {base} | {metric} | {res['n']} | {fmt(res['diff'])} | "
                              f"[{fmt(res['lo'])}, {fmt(res['hi'])}] |")
    lines.append("")

    # ---------------- 5) B7 vs B7c, P vs Pc (does the canonical grid-aware fix matter) ----------
    lines.append("## 5. canonical-grid variants: B7 vs B7c, P vs Pc @ 15deg\n")
    for a, b in [("B7c", "B7"), ("Pc", "P")]:
        ra, rb = by_mg.get((a, "15" if a in GRID15_ONLY else "na"), []), by_mg.get((b, "na"), [])
        for metric in ("wrist_mm_p50", "psi_p50", "held_pct"):
            res = paired_bootstrap_diff(ra, rb, metric)
            lines.append(f"- {a} - {b}, {metric}: n={res['n']} diff={fmt(res['diff'])} "
                          f"CI=[{fmt(res['lo'])}, {fmt(res['hi'])}]")
    lines.append("")

    # ---------------- 6) S1-S6 / occlusion-duration breakdown ----------------
    lines.append("## 6. Scenario (S1-S6) and occlusion-duration breakdown @ 15deg\n")
    scen = group_by(rows, lambda r: scenario_key(r["group"]))
    lines.append("| scenario | duration | method | n | wrist_mm_p50 | held_pct | deadline_miss_pct |")
    lines.append("|---|---|---|---|---|---|---|")
    for (base, dur) in sorted(scen, key=lambda t: (t[0], t[1] or "")):
        rs_all = scen[(base, dur)]
        for mid in STAGE15_METHODS:
            gkey = "na" if mid in NON_GRID else "15"
            rs = [r for r in rs_all if r["method"] == mid and r.get("grid", "na") == gkey]
            if not rs:
                continue
            lines.append(f"| {base} | {dur or '-'} | {mid} | {len(rs)} | "
                          f"{fmt(summary_row(rs,'wrist_mm_p50')['p50'])} | "
                          f"{fmt(summary_row(rs,'held_pct')['p50'])} | "
                          f"{fmt(summary_row(rs,'deadline_miss_pct')['p50'])} |")
    lines.append("")

    # ---------------- 7) failure-case analysis (never excluded) ----------------
    lines.append("## 7. Failure-case analysis (worst trajectories, not excluded from any table above)\n")
    for mid in methods_present:
        rs = [r for r in rows if r["method"] == mid]
        bad = sorted(rs, key=lambda r: -f_or_nan(r.get("solver_fail_pct", 0)))[:5]
        bad = [r for r in bad if f_or_nan(r.get("solver_fail_pct", 0)) > 0]
        if bad:
            lines.append(f"### {mid}: top solver_fail_pct trajectories")
            for r in bad:
                lines.append(f"- {r['group']}/{r['file']} grid={r.get('grid')}: "
                              f"solver_fail_pct={r.get('solver_fail_pct')} held_pct={r.get('held_pct')} "
                              f"deadline_miss_pct={r.get('deadline_miss_pct')}")
    lines.append("")

    report_path = args.out_dir / "ex04_statistical_analysis.md"
    report_path.write_text("\n".join(lines))

    # ---------------- full summary CSV (method x grid x metric P50/P90/P95/mean) ----------------
    summary_csv = args.out_dir / "ex04_full_summary.csv"
    with open(summary_csv, "w", newline="") as fh:
        w = csv.writer(fh)
        w.writerow(["method", "grid", "n"] + [f"{m}_{s}" for m in ALL_METRICS for s in ("mean", "p50", "p90", "p95")])
        for (mid, g), rs in sorted(by_mg.items()):
            row = [mid, g, len(rs)]
            for m in ALL_METRICS:
                s = summary_row(rs, m)
                row += [s["mean"], s["p50"], s["p90"], s["p95"]]
            w.writerow(row)

    print(f"[analyze_ex04] wrote {report_path}")
    print(f"[analyze_ex04] wrote {summary_csv}")
    print(f"[analyze_ex04] {len(files)} files, methods={methods_present}, grids={grids_present}")


if __name__ == "__main__":
    main()
