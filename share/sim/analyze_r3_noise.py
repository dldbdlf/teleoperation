"""R3 Noise Robustness (04_proposed_solver.md EX04-R table): same true human pose, independent
observation noise draws at several levels -> how much does the canonical target q_ref move?

This is deliberately a fixture-level check (no synthetic_human.py dataset, no run_baselines.py
control loop): R3 is a question about canonical_feasible_map's sensitivity to input noise, which
is cheap to answer directly with many repeated draws, rather than needing the expensive 894-
trajectory harness (that is R4/R5's job). "Noise level" here is injected directly on the wrist
position/orientation and on psi, in the same units and (for position/rotation) the same default
magnitude synthetic_human.Noise uses at full visibility (15 mm / 5 deg), scaled by a multiplier.

Usage: python analyze_r3_noise.py [--trials 200] [--poses 8]
Output (committed): share/11.14_exp/r3_noise_summary.csv
"""
from __future__ import annotations

import argparse
import csv
from pathlib import Path

import numpy as np

import baselines as bl
import canonical as can
import kinematics as k

NOISE_MULT = [0.0, 0.33, 1.0, 2.0, 4.0, 8.0]  # x (15mm pos, 5deg rot, ~5-15deg psi depending on arm extension)


def one_pose(m, rng, mult, n_trials):
    q_true = rng.uniform(m.lower * 0.6, m.upper * 0.6)
    W, R = bl.wrist_pose(m, q_true)
    psi_true = float(m.psi(q_true))
    T_true = bl.tool_pose(m, W, R)
    ref_true = can.canonical_feasible_map(m, T_true, psi_true, k.READY)
    if not ref_true.feasible:
        return None
    sp, sr, spsi = 0.015 * mult, np.deg2rad(5.0) * mult, np.deg2rad(5.0) * mult
    dq, wrist_err_mm, psi_dev_deg, infeasible = [], [], [], 0
    for _ in range(n_trials):
        Wn = W + rng.normal(scale=sp, size=3) if sp > 0 else W
        Rn = can_small_rot(R, rng, sr) if sr > 0 else R
        psin = float(k.wrap(psi_true + rng.normal(scale=spsi))) if spsi > 0 else psi_true
        Tn = bl.tool_pose(m, Wn, Rn)
        ctn = can.canonical_feasible_map(m, Tn, psin, k.READY)
        if not ctn.feasible:
            infeasible += 1
            continue
        dq.append(np.rad2deg(np.max(np.abs(k.wrap(ctn.q_ref - ref_true.q_ref)))))
        Wfk, _ = bl.wrist_pose(m, ctn.q_ref)
        wrist_err_mm.append(1000 * np.linalg.norm(Wfk - W))
        psi_dev_deg.append(np.rad2deg(abs(k.wrap(m.psi(ctn.q_ref) - psi_true))))
    return dict(dq=dq, wrist_err_mm=wrist_err_mm, psi_dev_deg=psi_dev_deg, infeasible=infeasible, n=n_trials)


def can_small_rot(R, rng, sigma_rad):
    import synthetic_human as sh
    return sh.expm_so3(rng.normal(scale=sigma_rad, size=3)) @ R


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--trials", type=int, default=200)
    ap.add_argument("--poses", type=int, default=8)
    ap.add_argument("--out", default="../11.14_exp/r3_noise_summary.csv")
    a = ap.parse_args()

    m = k.Nero.from_urdf()
    rng = np.random.default_rng(42)
    rows = []
    for mult in NOISE_MULT:
        dq_all, w_all, p_all, infeas_all, n_all = [], [], [], 0, 0
        for pi in range(a.poses):
            res = one_pose(m, np.random.default_rng(1000 * pi + 7), mult, a.trials)
            if res is None:
                continue
            dq_all += res["dq"]; w_all += res["wrist_err_mm"]; p_all += res["psi_dev_deg"]
            infeas_all += res["infeasible"]; n_all += res["n"]
        row = dict(noise_mult=mult, pos_mm=15 * mult, rot_deg=5 * mult, n=n_all,
                   infeasible_pct=100 * infeas_all / n_all if n_all else float("nan"))
        for label, arr in [("dq_deg", dq_all), ("wrist_err_mm", w_all), ("psi_dev_deg", p_all)]:
            a_ = np.array(arr)
            row[f"{label}_p50"] = float(np.percentile(a_, 50)) if len(a_) else float("nan")
            row[f"{label}_p95"] = float(np.percentile(a_, 95)) if len(a_) else float("nan")
            row[f"{label}_p99"] = float(np.percentile(a_, 99)) if len(a_) else float("nan")
        rows.append(row)
        print(f"noise x{mult} ({15*mult:.0f}mm/{5*mult:.0f}deg): dq P50/P95 {row['dq_deg_p50']:.2f}/{row['dq_deg_p95']:.2f} deg, "
              f"wrist err P50/P95 {row['wrist_err_mm_p50']:.1f}/{row['wrist_err_mm_p95']:.1f} mm, "
              f"infeasible {row['infeasible_pct']:.1f}%", flush=True)

    Path(a.out).parent.mkdir(parents=True, exist_ok=True)
    with open(a.out, "w", newline="") as fh:
        w = csv.DictWriter(fh, fieldnames=list(rows[0]))
        w.writeheader()
        w.writerows(rows)
    print(f"wrote {a.out}")


if __name__ == "__main__":
    main()
