"""Roadmap 1.4: how often is a wrist pose infeasible for NERO, and which joint limit causes it?

For each sampled link7 pose we solve IK on a psi grid (all branches, no limits), then
check the solutions against
  - the nominal URDF limits,
  - the nominal limits with one joint j relaxed to [-pi, pi] (attribution), and
  - no limits at all (pure reachability).
A pose is feasible if at least one psi on the grid has a solution within the limits.

Sampling (assumed, not measured from humans):
  wrist centre uniform in a box in front of the robot (-x is forward), kept to
  d_min + 0.02 <= |W - S| <= 0.56 m, where d_min is the shortest |W - S| allowed by the
  J4 limits (computed, printed and saved) so that position reach does not mix into the result;
  orientation either uniform on SO(3) or a random rotation of the ready tool orientation
  by an angle uniform in [0, cone] about a uniform axis.

Usage: py analysis_wrist_feasibility.py [--n 1000] [--grid 180] [--seed 0]
Writes results/wrist_feasibility.{md,json,png}.
"""

import argparse
import json
from multiprocessing import Pool
from pathlib import Path

import numpy as np

import kinematics as k

OUT = Path(__file__).parent / "results"
BOX = np.array([[-0.60, -0.10], [-0.40, 0.40], [0.00, 0.70]])  # x, y, z of the wrist centre
CONES_DEG = [15, 30, 45, 60, 90]
ELBOW_DOWN_DEG = 30  # 'elbow down' band |psi| <= this (psi = 0 is elbow straight down)

_m = None


def _init():
    global _m
    _m = k.Nero.from_urdf()


def random_rotation(rng):
    q = rng.normal(size=4)
    q /= np.linalg.norm(q)
    w, x, y, z = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def reach_bounds(m):
    """|W - S| range allowed by the J4 limits (|W - S| does not depend on the other joints)."""
    d = [np.linalg.norm(k.screw(m.axes[3], m.points[3], t) @ np.r_[m.W0, 1] - np.r_[m.S, 1])
         for t in np.linspace(m.lower[3], m.upper[3], 2001)]
    return min(d), max(d)


def sample_poses(m, n, cone_deg, rng, reach):
    R_ready = m.fk(k.READY)[:3, :3]
    tool_from_wrist = (np.linalg.inv(m.M) @ np.r_[m.W0, 1])[:3]  # W in the link7 frame
    poses = []
    while len(poses) < n:
        W = rng.uniform(BOX[:, 0], BOX[:, 1])
        if not reach[0] <= np.linalg.norm(W - m.S) <= reach[1]:
            continue
        if cone_deg is None:
            R = random_rotation(rng)
        else:
            axis = rng.normal(size=3)
            R = k.rot(axis / np.linalg.norm(axis), np.deg2rad(rng.uniform(0, cone_deg))) @ R_ready
        T = np.eye(4)
        T[:3, :3] = R
        T[:3, 3] = W - R @ tool_from_wrist
        poses.append(T)
    return poses


def limit_sets(m):
    full = np.full(7, np.pi)
    sets = {"nominal": (m.lower, m.upper), "none": (-full, full)}
    for j in range(7):
        lo, hi = m.lower.copy(), m.upper.copy()
        lo[j], hi[j] = -np.pi, np.pi
        sets[f"relax_J{j + 1}"] = (lo, hi)
    return sets


def evaluate(args):
    T, grid = args
    sets = limit_sets(_m)
    count = {name: 0 for name in sets}
    down = {name: False for name in sets}  # any feasible psi with |psi| <= ELBOW_DOWN_DEG
    for psi in grid:
        sols = _m.ik(T, psi)
        for name, (lo, hi) in sets.items():
            if any(np.all(q >= lo) and np.all(q <= hi) for q in sols):
                count[name] += 1
                down[name] |= abs(np.rad2deg(psi)) <= ELBOW_DOWN_DEG
    step = np.rad2deg(2 * np.pi / len(grid))
    return {name: c * step for name, c in count.items()}, down  # feasible psi measure (deg), elbow-down flag


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--n", type=int, default=1000)
    ap.add_argument("--grid", type=int, default=180)
    ap.add_argument("--seed", type=int, default=0)
    a = ap.parse_args()

    m = k.Nero.from_urdf()
    d_min, d_max = reach_bounds(m)
    reach = (d_min + 0.02, min(d_max, 0.56))
    print(f"|W - S| allowed by J4: [{d_min:.3f}, {d_max:.3f}] m; sampling {reach[0]:.3f}-{reach[1]:.3f} m")
    rng = np.random.default_rng(a.seed)
    grid = np.linspace(-np.pi, np.pi, a.grid, endpoint=False)
    names = list(limit_sets(m))
    rows = {}
    with Pool(initializer=_init) as pool:
        for cone in CONES_DEG + [None]:
            label = "uniform SO(3)" if cone is None else f"ready ±{cone}°"
            arcs = pool.map(evaluate, [(T, grid) for T in sample_poses(m, a.n, cone, rng, reach)], chunksize=8)
            A = {nm: np.array([r[0][nm] for r in arcs]) for nm in names}
            D = {nm: np.array([r[1][nm] for r in arcs]) for nm in names}
            rows[label] = {
                "feasible_pct": {nm: float(100 * np.mean(A[nm] > 0)) for nm in names},
                "elbow_down_pct": {nm: float(100 * np.mean(D[nm])) for nm in names},
                "median_arc_deg_nominal": float(np.median(A["nominal"][A["nominal"] > 0]))
                if np.any(A["nominal"] > 0) else 0.0,
            }
            print(label, {nm: round(v, 1) for nm, v in rows[label]["feasible_pct"].items()},
                  "elbow-down:", {nm: round(v, 1) for nm, v in rows[label]["elbow_down_pct"].items()}, flush=True)

    OUT.mkdir(exist_ok=True)
    meta = {"elbow_down_deg": ELBOW_DOWN_DEG, "n_per_set": a.n, "psi_grid": a.grid, "seed": a.seed, "box": BOX.tolist(), "reach_J4_m": [d_min, d_max], "sampled_reach_m": list(reach)}
    (OUT / "wrist_feasibility.json").write_text(json.dumps({"meta": meta, "rows": rows}, indent=2))

    cols = ["nominal"] + [f"relax_J{j}" for j in range(1, 8)] + ["none"]
    lines = [f"N = {a.n} poses per row, psi grid {a.grid} ({360 / a.grid:.0f}° step), seed {a.seed}. "
             "Values: % of poses with a non-empty feasible psi set.", "",
             "| orientation | " + " | ".join(cols) + " | median feasible psi (nominal, °) |",
             "|---" * (len(cols) + 2) + "|"]
    for label, r in rows.items():
        lines.append(f"| {label} | " + " | ".join(f"{r['feasible_pct'][c]:.1f}" for c in cols)
                     + f" | {r['median_arc_deg_nominal']:.0f} |")
    lines += ["", f"Elbow down: % of poses with some feasible psi in |psi| <= {ELBOW_DOWN_DEG}°.", "",
              "| orientation | " + " | ".join(cols) + " |", "|---" * (len(cols) + 1) + "|"]
    for label, r in rows.items():
        lines.append(f"| {label} | " + " | ".join(f"{r['elbow_down_pct'][c]:.1f}" for c in cols) + " |")
    (OUT / "wrist_feasibility.md").write_text("\n".join(lines) + "\n", encoding="utf-8")

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots(figsize=(6, 3.5))
    cone_rows = [f"ready ±{c}°" for c in CONES_DEG]
    for key, kind in [("feasible_pct", "any psi"), ("elbow_down_pct", f"|psi|<={ELBOW_DOWN_DEG}°")]:
        for nm, style in [("nominal", "o-"), ("relax_J6", "s--")]:
            ax.plot(CONES_DEG, [rows[r][key][nm] for r in cone_rows], style, label=f"{kind}, {nm}",
                    alpha=1.0 if key == "feasible_pct" else 0.6)
    ax.set_xlabel("max rotation from ready tool orientation (deg)")
    ax.set_ylabel("poses with feasible psi (%)")
    ax.set_ylim(0, 102)
    ax.grid(alpha=0.3)
    ax.legend()
    fig.tight_layout()
    fig.savefig(OUT / "wrist_feasibility.png", dpi=150)


if __name__ == "__main__":
    main()
