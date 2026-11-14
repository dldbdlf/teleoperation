"""Reproducible analysis for the EX04 canonical-solver experiment (R1-R6, G1).

Inputs (not committed, outputs/ is gitignored):
  outputs/baselines_P/per_traj.csv          B0-B8, P  (894-trajectory run, 2026-11-14)
  outputs/baselines_canonical/per_traj.csv  B7c, Pc   (894-trajectory run, 2026-11-14)
  outputs/synthetic/manifest.json

Output (committed): share/11.14_exp/canonical_summary.csv -- the small, derived numbers the
results report is written from, so the report's tables are reproducible from this script plus
the two CSVs above (regenerable via run_baselines.py; see the report for the exact commands).

Usage: python analyze_canonical.py [--ablation-sample N]
"""
from __future__ import annotations

import argparse
import csv
import json
from collections import defaultdict
from pathlib import Path

import numpy as np

METRICS = ["wrist_mm_p50", "wrist_deg_p50", "psi_feasible_pct", "psi_p90", "regret_p50",
           "psi_jumps", "jerk_rms", "recovery_jump_deg", "recovery_path_deg",
           "recovery_capped_pct", "held_pct", "runtime_p95_ms"]


def load(path):
    rows = defaultdict(dict)  # (group, file) -> {method: {metric: value}}
    with open(path) as fh:
        for r in csv.DictReader(fh):
            key = (r["group"], r["file"])
            rows[key][r["method"]] = {m: float(r[m]) if r[m] not in ("", "nan") else float("nan") for m in METRICS}
    return rows


def bootstrap_ci(x, n=2000, seed=0):
    x = np.asarray(x, float)
    x = x[np.isfinite(x)]
    if len(x) == 0:
        return float("nan"), float("nan"), float("nan")
    rng = np.random.default_rng(seed)
    boot = [np.mean(rng.choice(x, len(x), replace=True)) for _ in range(n)]
    return float(np.mean(x)), float(np.percentile(boot, 2.5)), float(np.percentile(boot, 97.5))


def paired_table(old, new, methods_old, methods_new, n_worst=5):
    """old, new: dict[(group,file)] -> dict[method] -> dict[metric] -> value, same key space."""
    common = sorted(set(old) & set(new))
    out = {}
    for mo, mn in zip(methods_old, methods_new):
        per_metric = {}
        for metric in METRICS:
            deltas, groups = [], []
            for key in common:
                a = old[key].get(mo, {}).get(metric, float("nan"))
                b = new[key].get(mn, {}).get(metric, float("nan"))
                if np.isfinite(a) and np.isfinite(b):
                    deltas.append(b - a)
                    groups.append(key[0])
            mean, lo, hi = bootstrap_ci(deltas)
            worst = sorted(zip(deltas, common), key=lambda t: -abs(t[0]) if np.isfinite(t[0]) else 0)[:n_worst]
            per_metric[metric] = dict(n=len(deltas), mean_delta=mean, ci95=(lo, hi),
                                       worst=[(g, float(d)) for d, (g, f) in zip([w[0] for w in worst], [w[1] for w in worst])])
        out[f"{mn}-{mo}"] = per_metric
    return out, common


def branch_ambiguity(manifest_path, sample_n, seed=0):
    """Ablation (a) support: how often does the canonical search find >1 feasible branch at the
    chosen psi (i.e. how often does the home-distance tie-break actually decide something, vs.
    being moot because only one branch exists)? Sampled across random (file, camera-frame) pairs."""
    import canonical as can
    import kinematics as k
    import baselines as bl

    m = k.Nero.from_urdf()
    manifest = json.loads(Path(manifest_path).read_text())
    rng = np.random.default_rng(seed)
    files = rng.choice(manifest["files"], size=min(sample_n, len(manifest["files"])), replace=False)
    n_total, n_tied, n_infeasible = 0, 0, 0
    by_group = defaultdict(lambda: [0, 0, 0])
    for fmeta in files:
        d = np.load(Path(manifest_path).parent / fmeta["file"])
        i = int(rng.integers(0, len(d["belief_t"])))
        Tb = d["belief_wrist_T"][i] @ np.linalg.inv(m.M)
        T = d["belief_wrist_T"][i]
        ct = can.canonical_feasible_map(m, T, float(d["belief_psi_mean"][i]), k.READY)
        group = Path(fmeta["file"]).parent.name
        n_total += 1
        by_group[group][0] += 1
        if not ct.feasible:
            n_infeasible += 1
            by_group[group][2] += 1
        elif ct.n_candidates > 1:
            n_tied += 1
            by_group[group][1] += 1
    return dict(n_total=n_total, n_tied=n_tied, n_infeasible=n_infeasible,
                tied_pct=100 * n_tied / n_total if n_total else float("nan"),
                by_group={g: dict(n=v[0], tied=v[1], infeasible=v[2]) for g, v in sorted(by_group.items())})


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--old", default="outputs/baselines_P/per_traj.csv")
    ap.add_argument("--new", default="outputs/baselines_canonical/per_traj.csv")
    ap.add_argument("--manifest", default="outputs/synthetic/manifest.json")
    ap.add_argument("--ablation-sample", type=int, default=300)
    ap.add_argument("--out", default="../11.14_exp/canonical_summary.csv")
    a = ap.parse_args()

    old, new = load(a.old), load(a.new)
    table, common = paired_table(old, new, ["B7", "P"], ["B7c", "Pc"])
    print(f"{len(common)} trajectories common to both runs")

    rows = []
    for pair, metrics in table.items():
        for metric, v in metrics.items():
            rows.append(dict(comparison=pair, metric=metric, n=v["n"],
                              mean_delta=round(v["mean_delta"], 4),
                              ci95_lo=round(v["ci95"][0], 4), ci95_hi=round(v["ci95"][1], 4)))
    Path(a.out).parent.mkdir(parents=True, exist_ok=True)
    with open(a.out, "w", newline="") as fh:
        w = csv.DictWriter(fh, fieldnames=list(rows[0]))
        w.writeheader()
        w.writerows(rows)
    print(f"wrote {a.out}")

    amb = branch_ambiguity(a.manifest, a.ablation_sample)
    print(f"branch ambiguity (ablation a): {amb['n_tied']}/{amb['n_total']} frames had >1 feasible "
          f"branch at the chosen psi ({amb['tied_pct']:.1f}%), {amb['n_infeasible']} pose_relaxed")
    Path("../11.14_exp/canonical_branch_ambiguity.json").write_text(json.dumps(amb, indent=2))

    for pair, metrics in table.items():
        print(f"\n=== {pair} (new - old), n={metrics['wrist_mm_p50']['n']} ===")
        for metric in ["wrist_mm_p50", "wrist_deg_p50", "psi_p90", "jerk_rms", "recovery_jump_deg", "runtime_p95_ms"]:
            v = metrics[metric]
            print(f"  {metric}: mean delta {v['mean_delta']:+.2f}  95% CI [{v['ci95'][0]:+.2f}, {v['ci95'][1]:+.2f}]  n={v['n']}")


if __name__ == "__main__":
    main()
