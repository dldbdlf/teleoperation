"""Evaluate an IKFlow model trained on NERO.

  static  : random reachable poses -> accuracy, joint-limit rate, psi coverage of the feasible arcs, runtime
  harness : IKFlow as baseline B9 in share/sim's EX04 harness (same frames, shield, metrics as B0-B8)

Usage: eval_ikflow.py <weights.pkl> static|harness [--n N] [--out DIR]
"""
import argparse
import json
import sys
import time
from pathlib import Path

import numpy as np
import torch

SIM = Path(__file__).resolve().parents[2]  # share/sim
sys.path.insert(0, str(SIM))
sys.path.insert(0, str(Path(__file__).resolve().parent))
import nero_robot  # noqa: E402,F401
from ikflow.ikflow_solver import IKFlowSolver  # noqa: E402
from ikflow.model import IkflowModelParameters  # noqa: E402
from jrl.robots import get_robot  # noqa: E402
from scipy.spatial.transform import Rotation  # noqa: E402

import baselines as bl  # noqa: E402
import kinematics as k  # noqa: E402
import run_baselines as rb  # noqa: E402
from synthetic_human import tool_pose  # noqa: E402

DEV = "cuda"


def load(path, dim_latent=8):
    hp = IkflowModelParameters()
    # as printed by scripts/train.py for our run (its defaults differ from IkflowModelParameters')
    hp.__dict__.update(coupling_layer="glow", nb_nodes=12, dim_latent_space=dim_latent, coeff_fn_config=3,
                       coeff_fn_internal_size=1024, permute_random_enabled=True, sigmoid_on_output=False,
                       rnvp_clamp=2.5, y_noise_scale=1e-7, zeros_noise_scale=1e-3, softflow_noise_scale=1e-3,
                       softflow_enabled=True)
    s = IKFlowSolver(hp, get_robot("nero"))
    s.load_state_dict(path)
    s.nn_model.eval()
    return s


def pose7(T):
    x, y, z, w = Rotation.from_matrix(T[:3, :3]).as_quat()
    return np.r_[T[:3, 3], w, x, y, z]


def sample(solver, T, n, scale=1.0):
    """scale: latent multiplier; the paper reports accuracy at 0.25 (less diverse, ~30% lower error)."""
    y = torch.tensor(np.tile(pose7(T), (n, 1)), dtype=torch.float32, device=DEV)
    with torch.inference_mode():
        q = solver.generate_ik_solutions(y, latent_scale=scale, clamp_to_joint_limits=True)
    return q.double().cpu().numpy()


def pose_err(m, q, T):
    F = np.array([m.fk(x) for x in q])
    pe = np.linalg.norm(F[:, :3, 3] - T[:3, 3], axis=1)
    re = np.array([np.linalg.norm(Rotation.from_matrix(f[:3, :3] @ T[:3, :3].T).as_rotvec()) for f in F])
    return pe, re


def project(m, q, T):
    """Exact, in-limit solution at each sample's psi (closest branch); None where that psi is infeasible."""
    return [bl.closest(m.ik_feasible(T, m.psi(x)), x) for x in q]


def static(solver, m, n_poses, out):
    rng = np.random.default_rng(0)
    BIN = np.deg2rad(5.0)
    centres = np.arange(-np.pi, np.pi, BIN) + BIN / 2
    rows = []
    for i in range(n_poses):
        T = m.fk(rng.uniform(m.lower, m.upper))
        t0 = time.perf_counter()
        q = sample(solver, T, 256)
        torch.cuda.synchronize()
        rt = (time.perf_counter() - t0) * 1e3
        pe, re = pose_err(m, q, T)
        pe25, re25 = pose_err(m, sample(solver, T, 256, 0.25), T)
        arcs = m.feasible_arcs(T, n=180)
        feas_bins = np.array([k.in_arcs(c, arcs) for c in centres])
        psis = np.array([m.psi(x) for x in q])
        good = (pe < 0.01) & (re < np.deg2rad(2))
        hit = np.zeros(len(centres), bool)
        hit[((psis[good] + np.pi) // BIN).astype(int) % len(centres)] = True
        proj = project(m, q, T)
        pp = np.array([m.psi(x) for x in proj if x is not None])
        hit_p = np.zeros(len(centres), bool)
        hit_p[((pp + np.pi) // BIN).astype(int) % len(centres)] = True
        nf = feas_bins.sum()
        rows.append(dict(pos25_mm_mean=np.mean(pe25) * 1e3, rot25_deg_mean=np.rad2deg(np.mean(re25)),
                         pos_mm_p50=np.median(pe) * 1e3, pos_mm_p95=np.percentile(pe, 95) * 1e3,
                         rot_deg_p50=np.rad2deg(np.median(re)), good_pct=100 * good.mean(),
                         proj_ok_pct=100 * np.mean([x is not None for x in proj]),
                         feas_arc_deg=np.rad2deg(k.arcs_length(arcs)),
                         cover_raw_pct=100 * (hit & feas_bins).sum() / nf if nf else np.nan,
                         cover_proj_pct=100 * (hit_p & feas_bins).sum() / nf if nf else np.nan,
                         outside_raw_pct=100 * np.mean([not k.in_arcs(p, arcs) for p in psis[good]]) if good.any() else np.nan,
                         runtime_ms=rt))
    keys = rows[0].keys()
    summ = {kk: dict(p50=float(np.nanmedian([r[kk] for r in rows])), mean=float(np.nanmean([r[kk] for r in rows])))
            for kk in keys}
    rt = np.array([r["runtime_ms"] for r in rows[5:]])  # skip warm-up
    summ["runtime_ms"]["p95"] = float(np.percentile(rt, 95))
    out.mkdir(parents=True, exist_ok=True)
    (out / "static.json").write_text(json.dumps(summ, indent=2))
    for kk, v in summ.items():
        print(f"{kk:16s} " + "  ".join(f"{a}={b:.1f}" for a, b in v.items()))


class IKFlowIK(bl.Method):
    """B9. IKFlow (normalizing flow) samples n solutions for the belief wrist pose; each is projected to the
    exact in-limit solution at its own psi (best n_proj by cost only); pick the one closest to the belief psi
    (then to q). raw=True skips
    the projection and uses the flow sample directly (joint-clamped)."""
    per_frame = True

    def __init__(self, solver, n=64, raw=False, w_q=0.1, n_proj=8):
        self.solver, self.n, self.raw, self.w_q, self.n_proj = solver, n, raw, w_q, n_proj
        self.scale = 0.25 if raw else 1.0  # raw needs accuracy; projected needs psi diversity
        self.name = "B9 IKFlow" + (" raw" if raw else " + 투영")

    def update(self, f, q):
        if not np.all(np.isfinite(f.b_W)):
            return
        T = tool_pose(self.m, f.b_W, f.b_R)
        psi_t = f.b_psi if np.isfinite(f.b_psi) else self.m.psi(q)
        cost = lambda c: np.abs(k.wrap(np.array([self.m.psi(x) for x in c]) - psi_t)) \
            + self.w_q * np.linalg.norm(c - q, axis=1)
        cand = sample(self.solver, T, self.n, self.scale)
        cand = cand[np.argsort(cost(cand))]
        if not self.raw:  # project only the best few: the analytic IK is the slow part
            cand = [x for x in project(self.m, cand[:self.n_proj], T) if x is not None]
            if not cand:
                return
            cand = np.asarray(cand)
            cand = cand[np.argsort(cost(cand))]
        self.q_des = cand[0]


def harness(solver, m, data, out, kind="mirror"):
    files = sorted(Path(data).glob("*/*.npz"))
    rows = []
    for raw in (False, True):
        for p in files:
            d = np.load(p)
            meta = json.loads(str(d["meta"]))
            tgt, feas, best = rb.targets(m, d, kind)
            meth = IKFlowIK(solver, raw=raw)
            Q, Qreq, held, rt = rb.run(m, d, meth, kind, np.pi, 0.5, True)
            rows.append({"group": p.parent.name, "file": p.name, "method": "B9r" if raw else "B9",
                         **rb.metrics(m, d, Q, Qreq, held, rt, tgt, feas, best, meta, np.pi)})
            print(rows[-1]["group"], p.name, rows[-1]["method"], f"{rows[-1]['wrist_mm_p50']:.0f}mm",
                  f"psi90={rows[-1]['psi_p90']:.0f}", flush=True)
    import csv
    out.mkdir(parents=True, exist_ok=True)
    with open(out / "per_traj.csv", "w", newline="") as fh:
        w = csv.DictWriter(fh, fieldnames=list(rows[0]))
        w.writeheader()
        w.writerows(rows)


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("weights")
    ap.add_argument("mode", choices=["static", "harness"])
    ap.add_argument("--n", type=int, default=300)
    ap.add_argument("--data", default=str(SIM / "outputs/syn_down"))
    ap.add_argument("--out", default=str(Path(__file__).resolve().parent / "out"))
    ap.add_argument("--dim-latent", type=int, default=8, help="8 for our AdamW runs, 7 for the paper-config run")
    a = ap.parse_args()
    torch.manual_seed(0)
    solver, m = load(a.weights, a.dim_latent), k.Nero.from_urdf()
    (static(solver, m, a.n, Path(a.out)) if a.mode == "static" else harness(solver, m, a.data, Path(a.out)))
