"""Run EX04 baselines B0-B8 (baselines.py) on synthetic_human.py trajectories, same harness for all.

Harness (identical for every method):
  - camera frames at their own rate (15 Hz); last valid wrist / elbow held while hidden, with age;
    the human psi mapping (--psi-map) is applied to every elbow / psi input first;
  - control loop at the ground-truth rate (100 Hz), q0 = READY;
  - safety shield: per-joint speed cap (--qd-max, default pi rad/s, the 2610.16855 setting,
    below the URDF 5 rad/s), joint limits, hold if the wrist input is older than --hold-s.

Metrics per trajectory (EX04 list in experiment_roadmap.md):
  wrist position / orientation error vs GT (median, P95) at 100 Hz; robot psi error P90 vs the
  mapped GT psi on camera frames where that psi is feasible; on frames where it is not but the
  wrist is, regret = |robot psi - nearest feasible psi| (median); RMS joint jerk; psi jumps
  (> 60 deg change within 0.2 s, counted as events); recovery in the 1 s after the occlusion
  ends: largest requested joint jump before the shield, joint path length (sum over joints),
  and % of ticks at the speed cap; fraction of ticks held; method runtime P95 per control tick.
  Per-frame methods (B1, B5-B8) get an output interpolation layer (new target reached linearly
  over one camera period) so jerk compares the methods, not the 15 Hz steps; --no-interp disables.

Usage:
    python synthetic_human.py --scenarios S3 S4 S6 --dropouts 1.0 --n-traj 10 --seeds 0 \
        --psi-centre-deg -30 30 --out outputs/syn_down
    python run_baselines.py outputs/syn_down --psi-map mirror --out outputs/baselines_mirror
Writes per_traj.csv, summary.md and config.json under --out (outputs/ is git-ignored).
"""

from __future__ import annotations

import argparse
import csv
import json
import time
from collections import defaultdict
from multiprocessing import Pool
from pathlib import Path

import numpy as np

import baselines as bl
import kinematics as k
import synthetic_human as sh

METHODS = ["B0", "B1", "B2", "B3", "B4", "B5", "B6", "B7", "B8"]
_m = _net = None


def make(mid, net):
    return {"B0": bl.WristDLS, "B1": bl.SEWMimic, "B2": bl.AEACJParse, "B3": bl.OneEuroDLS,
            "B4": bl.KalmanDLS, "B5": bl.AKC, "B6": lambda: bl.LearnedPsi(net),
            "B7": bl.BeliefWeightedIK, "B8": bl.SequentialBeliefIK}[mid]()


# ---------------------------------------------------------------- inputs
def frames(m, d, pmap):
    """Camera frames as methods see them (held values, ages, mapped elbow / psi)."""
    out, W, R, E = [], np.full(3, np.nan), np.full((3, 3), np.nan), np.full(3, np.nan)
    for i, t in enumerate(d["obs_t"]):
        wv, ev = bool(d["obs_wrist_visible"][i]), bool(d["obs_elbow_visible"][i])
        if wv:
            W, R = d["obs_W"][i], d["obs_R"][i]
        if ev:
            E = pmap.elbow(m.S, d["obs_E"][i], d["obs_W"][i])
        Tb = d["belief_wrist_T"][i] @ np.linalg.inv(m.M)
        out.append(bl.Frame(
            t=float(t), wrist_visible=wv, elbow_visible=ev, W=W.copy(), R=R.copy(), E=E.copy(),
            wrist_age=float(d["belief_age_wrist_ms"][i]) / 1000, elbow_age=float(d["belief_age_elbow_ms"][i]) / 1000,
            b_W=d["belief_W"][i], b_R=Tb[:3, :3], b_cov=d["belief_wrist_cov"][i],
            b_psi=pmap.psi(float(d["belief_psi_mean"][i])), b_kappa=float(d["belief_psi_kappa"][i]),
            b_state=int(d["belief_occlusion_state"][i])))
    return out


# ---------------------------------------------------------------- B6 training data
def train_psi_net(cfg, kind, n_traj=60, seed0=1000):
    """GT wrist pose -> mapped GT psi on trajectories disjoint from the test seeds."""
    m = k.Nero.from_urdf()
    reach = sh.reach_bounds(m)
    X, Y = [], []
    for i in range(n_traj):
        d, _ = sh.generate(m, "S4", seed0, i, 0.0, cfg, reach)
        pm = bl.PsiMap(kind)
        p0 = d["gt_psi"][0]
        pm.psi(p0)  # offset map: psi0 = first value of each trajectory
        sel = slice(None, None, 4)
        for T, p in zip(d["gt_T"][sel], d["gt_psi"][sel]):
            g = T @ np.linalg.inv(m.M)
            X.append(bl.psi_features(m, g[:3, :3] @ m.W0 + g[:3, 3], g[:3, :3]))
            Y.append(pm.psi(p))
    return bl.PsiMLP().fit(np.array(X), np.array(Y))


# ---------------------------------------------------------------- evaluation targets (shared by methods)
def targets(m, d, kind):
    pm = bl.PsiMap(kind)
    first = np.flatnonzero(d["obs_elbow_visible"])
    if len(first):  # offset map anchors on the first observed (noisy) psi, as the methods do
        pm.psi(float(d["obs_psi"][first[0]]))
    tgt = np.array([pm.psi(p) for p in d["cam_gt_psi"]])
    feas, best = np.zeros(len(tgt), bool), np.full(len(tgt), np.nan)
    for i, (T, p) in enumerate(zip(d["cam_gt_T"], tgt)):
        if not d["cam_gt_psi_defined"][i]:
            continue
        if m.ik_feasible(T, p):
            feas[i], best[i] = True, p
            continue
        for s in bl.PSI_GRID[np.argsort(np.abs(k.wrap(bl.PSI_GRID - p)))]:
            if m.ik_feasible(T, s):
                best[i] = s
                break
    return tgt, feas, best


# ---------------------------------------------------------------- one run
def run(m, d, method, pmap_kind, qd_max, hold_s, interp=True):
    """Returns q (after the shield), the request before the shield, hold flags, runtimes.
    Per-frame methods: the new q_des is linearly interpolated from the previous command over
    one camera period (output interpolation layer, as in 2610.16855), unless interp=False."""
    pmap = bl.PsiMap(pmap_kind)
    fr = frames(m, d, pmap)
    t_ctrl = d["gt_t"]
    dt = float(t_ctrl[1] - t_ctrl[0])
    cam_t = np.array([f.t for f in fr])
    T_cam = float(np.median(np.diff(cam_t)))
    q = k.READY.copy()
    method.reset(m, q)
    n_t = len(t_ctrl)
    Q, Qreq, held, rt, j_last = np.empty((n_t, 7)), np.empty((n_t, 7)), np.zeros(n_t, bool), [], -1
    seg = None  # (q_from, q_to, t_start) for per-frame interpolation
    for n, t in enumerate(t_ctrl):
        t0 = time.perf_counter()
        j = int(np.searchsorted(cam_t, t + 1e-9) - 1)
        if j > j_last:
            j_last = j
            prev = method.q_des
            method.update(fr[j], q)
            if method.per_frame and method.q_des is not None and method.q_des is not prev:
                start = q if seg is None else seg[0] + (seg[1] - seg[0]) * min(1.0, (t - seg[2]) / T_cam)
                seg = (start, method.q_des.copy(), t)
        q_des = method.step(q, dt)
        rt.append(time.perf_counter() - t0)
        if method.per_frame and interp and seg is not None and q_des is not None:
            q_des = seg[0] + (seg[1] - seg[0]) * min(1.0, (t - seg[2]) / T_cam)
        age = fr[j].wrist_age + (t - fr[j].t)
        if q_des is None or not np.all(np.isfinite(q_des)) or not np.isfinite(fr[j].W).all() or age > hold_s:
            q_des, held[n] = q, True
        Qreq[n] = q_des
        step = np.clip(q_des - q, -qd_max * dt, qd_max * dt)
        q = np.clip(q + step, m.lower, m.upper)
        Q[n] = q
    return Q, Qreq, held, np.array(rt)


def metrics(m, d, Q, Qreq, held, rt, tgt, feas, best, meta, qd_max):
    t, dt = d["gt_t"], float(d["gt_t"][1] - d["gt_t"][0])
    pose = [bl.wrist_pose(m, q) for q in Q]
    G = [T @ np.linalg.inv(m.M) for T in d["gt_T"]]
    e_pos = 1000 * np.array([np.linalg.norm(W - g[:3, :3] @ m.W0 - g[:3, 3]) for (W, _), g in zip(pose, G)])
    e_rot = np.rad2deg([np.linalg.norm(sh.logm_so3(R @ g[:3, :3].T)) for (_, R), g in zip(pose, G)])
    idx = np.clip(np.searchsorted(t, d["cam_gt_t"] - 1e-9), 0, len(t) - 1)
    psi_r = np.array([m.psi(Q[i]) for i in idx])
    err = np.rad2deg(np.abs(k.wrap(psi_r - tgt)))
    reg_sel = ~feas & np.isfinite(best)
    regret = np.rad2deg(np.abs(k.wrap(psi_r - best)))[reg_sel]
    psi_all = np.array([m.psi(q) for q in Q])
    lag = int(round(0.2 / dt))
    dpsi = np.rad2deg(np.abs(k.wrap(psi_all[lag:] - psi_all[:-lag])))
    over = dpsi > 60
    jumps = int(np.sum(over[1:] & ~over[:-1]) + over[0]) if len(over) else 0
    jerk = np.diff(Q, 3, axis=0) / dt ** 3
    qd = np.abs(np.diff(Q, axis=0)) / dt
    rec = dict(recovery_jump_deg=np.nan, recovery_path_deg=np.nan, recovery_capped_pct=np.nan)
    win = meta.get("hidden_window_s")
    if win and np.searchsorted(t, win[1]) < len(qd):  # first 1 s after the occlusion ends
        a = np.searchsorted(t, win[1])
        b = min(a + int(round(1 / dt)), len(qd))
        req = np.abs(Qreq[a:b] - np.r_[Q[a - 1:a], Q[a:b - 1]])  # request vs q at that tick, before the shield
        rec = dict(recovery_jump_deg=float(np.rad2deg(req.max())),
                   recovery_path_deg=float(np.rad2deg(qd[a:b].sum() * dt)),
                   recovery_capped_pct=float(100 * np.mean(qd[a:b].max(1) >= 0.999 * qd_max)))
    return dict(
        wrist_mm_p50=np.median(e_pos), wrist_mm_p95=np.percentile(e_pos, 95),
        wrist_deg_p50=np.median(e_rot), wrist_deg_p95=np.percentile(e_rot, 95),
        psi_feasible_pct=100 * feas.mean(),
        psi_p90=np.percentile(err[feas], 90) if feas.any() else np.nan,
        regret_p50=np.median(regret) if len(regret) else np.nan,
        psi_jumps=jumps, jerk_rms=float(np.sqrt(np.mean(jerk ** 2))),
        **rec, held_pct=100 * held.mean(), runtime_p95_ms=1000 * np.percentile(rt, 95))


# ---------------------------------------------------------------- pool
def _init(net):
    global _m, _net
    _m, _net = k.Nero.from_urdf(), net


def _job(args):
    path, mids, kind, qd_max, hold_s, interp = args
    d = np.load(path)
    meta = json.loads(str(d["meta"]))
    tgt, feas, best = targets(_m, d, kind)
    rows = []
    for mid in mids:
        Q, Qreq, held, rt = run(_m, d, make(mid, _net), kind, qd_max, hold_s, interp)
        group = Path(path).parent.name
        rows.append({"group": group, "file": Path(path).name, "method": mid,
                     **metrics(_m, d, Q, Qreq, held, rt, tgt, feas, best, meta, qd_max)})
    return rows


COLS = [("wrist_mm_p50", "손목 mm P50", "{:.0f}"), ("wrist_deg_p50", "손목 ° P50", "{:.1f}"),
        ("psi_feasible_pct", "목표 ψ 가능 %", "{:.0f}"), ("psi_p90", "ψ 오차 P90 °", "{:.0f}"),
        ("regret_p50", "불가능 시 regret °", "{:.0f}"), ("psi_jumps", "ψ 점프", "{:.1f}"),
        ("jerk_rms", "jerk RMS", "{:.0f}"), ("recovery_jump_deg", "복귀 요청 점프 °", "{:.0f}"),
        ("recovery_path_deg", "복귀 관절 이동 °", "{:.0f}"), ("recovery_capped_pct", "복귀 중 속도제한 %", "{:.0f}"),
        ("held_pct", "멈춤 %", "{:.0f}"), ("runtime_p95_ms", "계산 P95 ms", "{:.2f}")]


def summary(rows, names):
    by = defaultdict(list)
    for r in rows:
        by[(r["group"], r["method"])].append(r)
    lines = []
    for g in sorted({r["group"] for r in rows}):
        n = len({r["file"] for r in rows if r["group"] == g})
        lines += [f"### {g} (궤적 {n}개 평균)", "", "| 방법 | " + " | ".join(c[1] for c in COLS) + " |",
                  "|---" * (len(COLS) + 1) + "|"]
        for mid in METHODS:
            rs = by.get((g, mid))
            if not rs:
                continue
            cells = []
            for key, _, fmt in COLS:
                v = np.array([r[key] for r in rs], float)
                cells.append("–" if np.all(np.isnan(v)) else fmt.format(np.nanmean(v)))
            lines.append(f"| {names[mid]} | " + " | ".join(cells) + " |")
        lines.append("")
    return "\n".join(lines)


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("data", help="synthetic_human.py output directory (with manifest.json)")
    ap.add_argument("--methods", nargs="+", default=METHODS, choices=METHODS)
    ap.add_argument("--psi-map", default="none", choices=bl.PsiMap.KINDS)
    ap.add_argument("--qd-max", type=float, default=np.pi, help="shield joint speed cap [rad/s]")
    ap.add_argument("--hold-s", type=float, default=0.5, help="hold if the wrist input is older than this [s]")
    ap.add_argument("--no-interp", action="store_true", help="no output interpolation for per-frame methods")
    ap.add_argument("--groups", nargs="*", help="only these subfolders, e.g. S4_drop1000ms")
    ap.add_argument("--out", default="outputs/baselines")
    a = ap.parse_args()

    data = Path(a.data)
    manifest = json.loads((data / "manifest.json").read_text())
    cfg = sh.Config(**{**manifest["config"], "noise": sh.Noise(**manifest["config"]["noise"]),
                       "psi_centre_deg": tuple(manifest["config"]["psi_centre_deg"])})
    files = sorted(str(data / f["file"]) for f in manifest["files"]
                   if not a.groups or Path(f["file"]).parent.name in a.groups)
    net = train_psi_net(cfg, a.psi_map) if "B6" in a.methods else None
    jobs = [(f, a.methods, a.psi_map, a.qd_max, a.hold_s, not a.no_interp) for f in files]
    print(f"{len(files)} trajectories x {len(a.methods)} methods, psi map '{a.psi_map}'")
    with Pool(initializer=_init, initargs=(net,)) as pool:
        rows = [r for rs in pool.imap_unordered(_job, jobs) for r in rs]
    rows.sort(key=lambda r: (r["group"], r["file"], METHODS.index(r["method"])))

    out = Path(a.out)
    out.mkdir(parents=True, exist_ok=True)
    with open(out / "per_traj.csv", "w", newline="") as fh:
        w = csv.DictWriter(fh, fieldnames=list(rows[0]))
        w.writeheader()
        w.writerows(rows)
    names = {mid: make(mid, net).name for mid in a.methods}
    (out / "summary.md").write_text(f"ψ map: {a.psi_map}, speed cap {a.qd_max:.2f} rad/s, hold {a.hold_s} s, "
                                    f"output interpolation {'off' if a.no_interp else 'on'}\n\n"
                                    + summary(rows, names))
    (out / "config.json").write_text(json.dumps({"args": vars(a), "data_manifest_sha": manifest.get("git_sha"),
                                                 "git_sha": sh.git_sha()}, indent=2))
    print((out / "summary.md").read_text())


if __name__ == "__main__":
    main()
