"""EX04 / G1: synthetic human trajectories with an oracle belief, for testing the solver
without perception.

Each trajectory gives, in the NERO base frame (see kinematics.py conventions):
  - ground truth at the control rate: link7 target pose, wrist centre W, SEW angle psi, elbow E;
  - observations at the camera rate (what B0-B4 see): noisy W, tool orientation, elbow,
    psi computed from them, NaN while occluded;
  - an oracle HumanBelief at the camera rate (what B7 / P see): wrist mean + 6x6 tangent
    covariance, psi mean + von Mises concentration, occlusion state, age.

Oracle belief: the belief is not computed from the observations. Its error is drawn as
sigma(t) * eps(t), where eps is unit-variance AR(1) noise (time constant `belief_tau_s`) and
sigma(t) is exactly the reported standard deviation, so the reported covariance is
calibrated by construction. sigma grows linearly with the time since the part was last
visible. This is the "oracle Sigma" upper bound of EX04, not a learned belief.

Assumptions (not measured from humans):
  - The human arm is already mapped onto the robot: same S, |SE| = L_se, |EW| = L_ew.
  - Motion is a sum of slow sinusoids around the ready wrist pose; psi centres are uniform
    on the circle unless `--psi-centre-deg` narrows them.
  - Unimodal psi belief (single von Mises); no mixture yet.

Scenarios (experiment_roadmap.md EX04):
  S1 psi sweep, wrist fixed at ready               S4 elbow-only occlusion
  S2 near-straight elbow (|W - S| -> J4 reach)      S5 wrist dropout (wrist and elbow hidden)
  S3 tool rotation into a pose infeasible for all psi  S6 elbow occlusion with psi reversal inside it
The occlusion window starts at `--occl-start` and lasts each of `--dropouts` (S4-S6 only).

Usage:
    python synthetic_human.py --n-traj 20 --seeds 0 1 2 --plot
Writes outputs/synthetic/<scenario>/<name>.npz, manifest.json and plots/ (outputs/ is git-ignored).
"""

from __future__ import annotations

import argparse
import json
import subprocess
from dataclasses import asdict, dataclass, field
from multiprocessing import Pool
from pathlib import Path

import numpy as np

import kinematics as k

SCENARIOS = ["S1", "S2", "S3", "S4", "S5", "S6"]
OCCLUDING = {"S4", "S5", "S6"}
VISIBLE, ELBOW_HIDDEN, WRIST_HIDDEN = 0, 1, 2  # occlusion_state codes (WRIST_HIDDEN implies elbow hidden)


@dataclass
class Noise:
    obs_pos_m: float = 0.015         # wrist position, per axis
    obs_rot_deg: float = 5.0         # tool orientation, per tangent axis
    obs_elbow_m: float = 0.02        # elbow position, per axis
    belief_tau_s: float = 0.3        # correlation time of the belief error
    grow_pos_m_s: float = 0.10       # belief std growth while the wrist is hidden
    grow_rot_deg_s: float = 20.0
    grow_psi_deg_s: float = 60.0     # belief std growth while the elbow is hidden
    max_psi_sigma_deg: float = 120.0
    min_circle_r_m: float = 0.01     # below this elbow-circle radius psi is marked undefined


@dataclass
class Config:
    duration_s: float = 8.0
    gt_hz: float = 100.0
    cam_hz: float = 15.0
    occl_start_s: float = 3.0
    psi_centre_deg: tuple = (-180.0, 180.0)
    wrist_amp_m: float = 0.06        # total sinusoid amplitude per axis
    wrist_offset_m: float = 0.08     # random offset of the motion centre from ready
    rot_amp_deg: float = 10.0
    feas_grid: int = 36              # psi grid for the any-psi wrist feasibility label
    noise: Noise = field(default_factory=Noise)


# ---------------------------------------------------------------- helpers
def expm_so3(v):
    th = np.linalg.norm(v)
    return np.eye(3) if th < 1e-12 else k.rot(v / th, th)


def logm_so3(R):
    c = np.clip((np.trace(R) - 1) / 2, -1.0, 1.0)
    th = np.arccos(c)
    if th < 1e-9:
        return np.zeros(3)
    w = np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]])
    if np.pi - th < 1e-6:  # near pi: axis from the symmetric part
        A = (R + np.eye(3)) / 2
        ax = A[:, np.argmax(np.diag(A))]
        return th * ax / np.linalg.norm(ax)
    return th * w / (2 * np.sin(th))


def wrap(a):
    return (np.asarray(a) + np.pi) % (2 * np.pi) - np.pi


def sinusoids(rng, n, amp, f_lo=0.1, f_hi=0.4, k_terms=2):
    """Random smooth function R -> R^n with per-axis amplitude <= amp and value 0 at t = 0."""
    A = rng.uniform(0.3, 1.0, (k_terms, n))
    A *= amp / A.sum(axis=0)
    f = rng.uniform(f_lo, f_hi, (k_terms, n))
    ph = rng.uniform(0, 2 * np.pi, (k_terms, n))
    return lambda t: np.sum(A * (np.sin(2 * np.pi * f * np.asarray(t)[..., None, None] + ph) - np.sin(ph)), axis=-2)


def ar1_unit(rng, n, dim, dt, tau):
    """Unit-variance AR(1) noise, (n, dim)."""
    a = np.exp(-dt / tau)
    x = np.empty((n, dim))
    x[0] = rng.normal(size=dim)
    for i in range(1, n):
        x[i] = a * x[i - 1] + np.sqrt(1 - a * a) * rng.normal(size=dim)
    return x


def reach_bounds(m):
    d = [np.linalg.norm(k.screw(m.axes[3], m.points[3], t) @ np.r_[m.W0, 1] - np.r_[m.S, 1])
         for t in np.linspace(m.lower[3], m.upper[3], 2001)]
    return min(d), max(d)


def tool_pose(m, W, Rg):
    """link7 pose whose wrist centre is W and whose wrist-frame rotation is Rg."""
    g = np.eye(4)
    g[:3, :3] = Rg
    g[:3, 3] = W - Rg @ m.W0
    return g @ m.M


def wrist_feasible(m, T, grid):
    return any(m.ik_feasible(T, s) for s in grid)


# ---------------------------------------------------------------- trajectory definitions
class Trajectory:
    """Closed-form ground truth: W(t), Rg(t), psi(t) and a visibility schedule."""

    def __init__(self, m, scenario, rng, cfg, dropout_s, reach):
        self.m, self.cfg = m, cfg
        T0 = m.fk(k.READY)
        g0 = T0 @ np.linalg.inv(m.M)
        R0, W0 = g0[:3, :3], g0[:3, :3] @ m.W0 + g0[:3, 3]
        dur, S = cfg.duration_s, m.S
        lo, hi = np.deg2rad(cfg.psi_centre_deg)
        psi_c = rng.uniform(lo, hi)
        psi_amp, psi_f, psi_ph = np.deg2rad(rng.uniform(20, 60)), rng.uniform(0.1, 0.3), rng.uniform(0, 2 * np.pi)
        self.psi = lambda t: wrap(psi_c + psi_amp * (np.sin(2 * np.pi * psi_f * t + psi_ph) - np.sin(psi_ph)))

        margin = 0.03 + np.sqrt(3) * cfg.wrist_amp_m  # per-axis amplitude -> worst-case 3D excursion
        while True:  # motion centre inside the reachable shell with margin for the motion
            Wc = W0 + rng.uniform(-1, 1, 3) * cfg.wrist_offset_m
            if reach[0] + margin <= np.linalg.norm(Wc - S) <= reach[1] - margin:
                break
        dW = sinusoids(rng, 3, cfg.wrist_amp_m)
        dR = sinusoids(rng, 3, np.deg2rad(cfg.rot_amp_deg))
        self.W = lambda t: Wc + dW(t)
        self.Rg = lambda t: expm_so3(dR(t)) @ R0
        self.hidden = (np.inf, np.inf)  # occlusion window [t0, t1)
        self.extra = {}

        if scenario == "S1":
            psi0 = rng.uniform(-np.pi, np.pi)
            self.W, self.Rg = (lambda t: W0), (lambda t: R0)
            self.psi = lambda t: wrap(psi0 + 2 * np.pi * t / dur)
        elif scenario == "S2":
            e = (W0 - S) / np.linalg.norm(W0 - S)
            d0, d1 = np.linalg.norm(W0 - S), reach[1] - 0.003
            self.W = lambda t: S + e * (d0 + (d1 - d0) * (1 - np.cos(2 * np.pi * t / dur)) / 2)
            self.Rg = lambda t: R0
            self.extra["reach_m"] = d1
        elif scenario == "S3":
            grid = np.linspace(-np.pi, np.pi, cfg.feas_grid, endpoint=False)
            for _ in range(200):  # rotation axis/peak whose peak pose is infeasible for every psi
                ax = rng.normal(size=3) + [0, -1.5, 0]
                ax /= np.linalg.norm(ax)
                peak = np.deg2rad(rng.uniform(80, 150))
                if not wrist_feasible(m, tool_pose(m, W0, k.rot(ax, peak) @ R0), grid):
                    break
            else:
                raise RuntimeError("S3: no infeasible rotation found")
            self.W = lambda t: W0 + 0.3 * dW(t)
            self.Rg = lambda t: k.rot(ax, peak * (1 - np.cos(2 * np.pi * t / dur)) / 2) @ R0
            self.extra.update(rot_axis=ax.tolist(), rot_peak_deg=float(np.rad2deg(peak)))
        elif scenario in OCCLUDING:
            t0 = cfg.occl_start_s
            self.hidden = (t0, t0 + dropout_s)
            if scenario == "S6":  # psi moves one way, reverses at the middle of the occlusion
                tm, v = t0 + dropout_s / 2, np.deg2rad(rng.uniform(40, 80)) * rng.choice([-1, 1])
                w, eps = max(dropout_s, 0.5) * 1.5, 0.05
                soft = lambda x: np.sqrt(x * x + eps * eps)
                self.psi = lambda t: wrap(psi_c - v * np.minimum(soft(np.asarray(t) - tm), w))
                self.extra.update(reversal_s=tm, psi_speed_deg_s=float(np.rad2deg(v)))

        self.wrist_hidden = scenario == "S5"

    def elbow(self, W, psi):
        return k.elbow_from_psi(self.m.S, W, psi, self.m.L_se, self.m.L_ew)

    def visibility(self, t):
        h = (t >= self.hidden[0]) & (t < self.hidden[1])
        return (~h if self.wrist_hidden else np.ones_like(h)), ~h  # wrist_visible, elbow_visible


# ---------------------------------------------------------------- generation
def ages(t, visible):
    """Seconds since the part was last visible (0 while visible)."""
    out, last = np.zeros_like(t), t[0]
    for i, (ti, v) in enumerate(zip(t, visible)):
        last = ti if v else last
        out[i] = ti - last
    return out


def generate(m, scenario, seed, idx, dropout_s, cfg, reach):
    rng = np.random.default_rng([seed, SCENARIOS.index(scenario), idx, int(round(dropout_s * 1000))])
    tr = Trajectory(m, scenario, rng, cfg, dropout_s, reach)
    nz = cfg.noise
    out = {}
    S = m.S
    grid = np.linspace(-np.pi, np.pi, cfg.feas_grid, endpoint=False)

    def sample(t, prefix, labels):
        W = np.array([tr.W(ti) for ti in t])
        R = np.array([tr.Rg(ti) for ti in t])
        psi = np.array([float(tr.psi(ti)) for ti in t])
        E = np.array([tr.elbow(w, p) for w, p in zip(W, psi)])
        D = np.linalg.norm(W - S, axis=1)
        a = (m.L_se ** 2 - m.L_ew ** 2 + D ** 2) / (2 * D)
        r = np.sqrt(np.maximum(m.L_se ** 2 - a ** 2, 0))
        T = np.array([tool_pose(m, w, Rg) for w, Rg in zip(W, R)])
        out.update({f"{prefix}t": t, f"{prefix}T": T, f"{prefix}W": W, f"{prefix}psi": psi, f"{prefix}E": E,
                    f"{prefix}circle_r": r, f"{prefix}psi_defined": r >= nz.min_circle_r_m})
        if labels:
            out[f"{prefix}psi_feasible"] = np.array([bool(m.ik_feasible(Ti, p)) for Ti, p in zip(T, psi)])
            out[f"{prefix}wrist_feasible"] = np.array(
                [f or wrist_feasible(m, Ti, grid) for Ti, f in zip(T, out[f"{prefix}psi_feasible"])])
        return W, R, psi, E, r

    sample(np.arange(0, cfg.duration_s, 1 / cfg.gt_hz), "gt_", labels=False)
    t = np.arange(0, cfg.duration_s, 1 / cfg.cam_hz)
    W, R, psi, E, r = sample(t, "cam_gt_", labels=True)
    n = len(t)
    wv, ev = tr.visibility(t)
    ev = ev & wv

    # observations: independent white noise per frame, NaN while hidden
    oW = W + rng.normal(scale=nz.obs_pos_m, size=(n, 3))
    oR = np.array([expm_so3(rng.normal(scale=np.deg2rad(nz.obs_rot_deg), size=3)) @ Ri for Ri in R])
    oE = E + rng.normal(scale=nz.obs_elbow_m, size=(n, 3))
    opsi = np.array([k.sew_angle(S, e, w) for e, w in zip(oE, oW)])
    oW[~wv], oR[~wv], oE[~ev], opsi[~ev] = np.nan, np.nan, np.nan, np.nan
    out.update(obs_t=t, obs_wrist_visible=wv, obs_elbow_visible=ev, obs_W=oW, obs_R=oR, obs_E=oE, obs_psi=opsi)

    # oracle belief: error = sigma(t) * unit AR(1) noise
    aw, ae = ages(t, wv), ages(t, ev)
    sp = nz.obs_pos_m + nz.grow_pos_m_s * aw
    sr = np.deg2rad(nz.obs_rot_deg + nz.grow_rot_deg_s * aw)
    s_vis = np.minimum(nz.obs_elbow_m / np.maximum(r, nz.min_circle_r_m), np.pi)  # tangential elbow noise / circle radius
    spsi = np.minimum(np.sqrt(s_vis ** 2 + np.deg2rad(nz.grow_psi_deg_s * ae) ** 2), np.deg2rad(nz.max_psi_sigma_deg))
    eps = ar1_unit(rng, n, 7, 1 / cfg.cam_hz, nz.belief_tau_s)
    bW = W + sp[:, None] * eps[:, :3]
    bR = np.array([expm_so3(s * e) @ Ri for s, e, Ri in zip(sr, eps[:, 3:6], R)])
    bpsi = wrap(psi + spsi * eps[:, 6])
    cov = np.zeros((n, 6, 6))
    cov[:, [0, 1, 2], [0, 1, 2]] = sp[:, None] ** 2
    cov[:, [3, 4, 5], [3, 4, 5]] = sr[:, None] ** 2
    state = np.where(~wv, WRIST_HIDDEN, np.where(~ev, ELBOW_HIDDEN, VISIBLE))
    out.update(belief_t=t, belief_wrist_T=np.array([tool_pose(m, w, Rb) for w, Rb in zip(bW, bR)]),
               belief_W=bW, belief_wrist_cov=cov, belief_psi_mean=bpsi, belief_psi_sigma=spsi,
               belief_psi_kappa=1.0 / spsi ** 2, belief_occlusion_state=state,
               belief_age_wrist_ms=1000 * aw, belief_age_elbow_ms=1000 * ae)

    meta = {"scenario": scenario, "seed": seed, "index": idx, "dropout_s": dropout_s if scenario in OCCLUDING else None,
            "hidden_window_s": list(tr.hidden) if np.isfinite(tr.hidden[0]) else None, **tr.extra}
    return out, meta


# ---------------------------------------------------------------- CLI
_m = _reach = None


def _init():
    global _m, _reach
    _m = k.Nero.from_urdf()
    _reach = reach_bounds(_m)


def _job(args):
    scenario, seed, idx, drop, cfg, out_dir = args
    data, meta = generate(_m, scenario, seed, idx, drop, cfg, _reach)
    tag = f"{scenario}" + (f"_drop{int(drop * 1000)}ms" if scenario in OCCLUDING else "")
    path = Path(out_dir) / tag / f"{tag}_s{seed}_{idx:03d}.npz"
    path.parent.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(path, meta=json.dumps(meta), **data)
    return str(path.relative_to(out_dir)), meta, data


def git_sha():
    try:
        return subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=Path(__file__).parent, text=True).strip()
    except Exception:
        return None


def plot(path, data, meta):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    t = data["belief_t"]
    fig, ax = plt.subplots(3, 1, figsize=(9, 7), sharex=True)
    for a in ax:
        if meta["hidden_window_s"]:
            a.axvspan(*meta["hidden_window_s"], color="0.85", label="hidden")
    d = np.rad2deg
    gt = np.unwrap(data["cam_gt_psi"])  # plot unwrapped; obs/belief as GT + wrapped error
    ax[0].plot(data["gt_t"], d(np.unwrap(data["gt_psi"])), "k", lw=1.2, label="GT ψ (unwrapped)")
    with np.errstate(invalid="ignore"):  # obs_psi is NaN while hidden
        ax[0].plot(t, d(gt + wrap(data["obs_psi"] - data["cam_gt_psi"])), ".", ms=3, label="obs ψ")
    m, s = gt + wrap(data["belief_psi_mean"] - data["cam_gt_psi"]), data["belief_psi_sigma"]
    ax[0].plot(t, d(m), "C1", lw=1, label="belief mean")
    ax[0].fill_between(t, d(m - 2 * s), d(m + 2 * s), color="C1", alpha=0.2, label="belief ±2σ")
    ax[0].set_ylabel("ψ [deg]")
    ax[0].legend(fontsize=7, ncol=5, loc="upper left")
    err = np.linalg.norm(data["belief_W"] - data["cam_gt_W"], axis=1)
    ax[1].plot(t, 1000 * err, label="|belief W − GT| (3D)")
    ax[1].plot(t, 1000 * 2.8 * np.sqrt(data["belief_wrist_cov"][:, 0, 0]), "--", label="95% bound (2.8σ, 3D)")
    ax[1].set_ylabel("wrist [mm]")
    ax[1].legend(fontsize=7)
    ax[2].step(t, data["cam_gt_psi_feasible"], where="post", label="GT ψ feasible for NERO")
    ax[2].step(t, data["cam_gt_wrist_feasible"] + 0.03, where="post", label="wrist feasible for some ψ")
    ax[2].set_yticks([0, 1], ["no", "yes"])
    ax[2].set_xlabel("t [s]")
    ax[2].legend(fontsize=7)
    fig.suptitle(f"{meta['scenario']} seed {meta['seed']} #{meta['index']}" +
                 (f", dropout {meta['dropout_s']} s" if meta["dropout_s"] else ""))
    fig.tight_layout()
    fig.savefig(path, dpi=110)
    plt.close(fig)


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--scenarios", nargs="+", default=SCENARIOS, choices=SCENARIOS)
    ap.add_argument("--n-traj", type=int, default=20, help="trajectories per scenario, dropout and seed")
    ap.add_argument("--seeds", type=int, nargs="+", default=[0, 1, 2])
    ap.add_argument("--dropouts", type=float, nargs="+", default=[0.2, 0.5, 1.0, 2.0], help="occlusion lengths [s], S4-S6")
    ap.add_argument("--duration", type=float, default=Config.duration_s)
    ap.add_argument("--occl-start", type=float, default=Config.occl_start_s)
    ap.add_argument("--psi-centre-deg", type=float, nargs=2, default=Config.psi_centre_deg)
    ap.add_argument("--out", default="outputs/synthetic")
    ap.add_argument("--plot", action="store_true", help="one figure per scenario/dropout (first trajectory)")
    a = ap.parse_args()
    if a.occl_start + max(a.dropouts) >= a.duration:
        ap.error("occlusion window must end before --duration")

    cfg = Config(duration_s=a.duration, occl_start_s=a.occl_start, psi_centre_deg=tuple(a.psi_centre_deg))
    out = Path(a.out)
    jobs = [(s, seed, i, d, cfg, str(out)) for s in a.scenarios for d in (a.dropouts if s in OCCLUDING else [0.0])
            for seed in a.seeds for i in range(a.n_traj)]
    print(f"{len(jobs)} trajectories -> {out}")
    files, plotted = [], set()
    with Pool(initializer=_init) as pool:
        for rel, meta, data in pool.imap(_job, jobs, chunksize=2):
            files.append({"file": rel, **meta})
            group = Path(rel).parent.name
            if a.plot and group not in plotted:
                plotted.add(group)
                (out / "plots").mkdir(parents=True, exist_ok=True)
                plot(out / "plots" / f"{group}.png", data, meta)
    m = k.Nero.from_urdf()
    manifest = {"git_sha": git_sha(), "urdf": str(k.URDF_PATH.name), "reach_J4_m": list(reach_bounds(m)),
                "config": asdict(cfg), "args": vars(a),
                "occlusion_state_codes": {"visible": VISIBLE, "elbow_hidden": ELBOW_HIDDEN, "wrist_hidden": WRIST_HIDDEN},
                "files": files}
    (out / "manifest.json").write_text(json.dumps(manifest, indent=2))
    print(f"wrote {len(files)} files, manifest.json" + (f", {len(plotted)} plots" if a.plot else ""))


if __name__ == "__main__":
    main()
