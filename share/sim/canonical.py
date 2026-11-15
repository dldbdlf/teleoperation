"""Canonical reference solver (share/docs/04_proposed_solver.md Part 1A) and the separate
execution/tracking layer (Part 1B), used by baselines.py's B7-canonical / P-canonical.

R-CANON requirement this module exists to satisfy: for a *fixed* robot model / joint limits /
tie-break policy, the canonical target q_ref is a pure function of the human belief (b_W, b_R,
mu_psi, kappa) -- it must not depend on q_prev, call order, process, or any random seed. All
state (occlusion smoothing, recovery) lives in `track_safe`, not here.

Scope note (see share/11.14_exp/EX04_joint_consistency_results.md "Scope and simplifications"):
04_proposed_solver.md's formal objective jointly optimizes psi and branch against
E[l_wrist+l_psi] + lam_h*||q-q_home||^2 over the *entire* psi domain. A full grid x branch scan
of that objective costs ~1.2s per camera frame on this Jetson (measured), i.e. days for the
894-trajectory sweep, so we use the home-prior only as the *tie-break* among branches at the
psi nearest to the belief mean -- the psi search itself stays a deterministic nearest-feasible
scan (no q_prev used anywhere), matching the role lam_h/kh already plays elsewhere in
baselines.py (B0's posture homing). This is a narrower claim than the full joint optimization
and is reported as such, not as a complete implementation of 04_proposed_solver.md.
"""

from __future__ import annotations

import hashlib
from dataclasses import dataclass, field

import numpy as np

import kinematics as k

GRID_DEG = 10.0  # ASSUMPTION: coarser than baselines.py's PSI_GRID (2 deg) -- measured on this
# Jetson: 2 deg costs ~1.1s/frame when nothing is feasible nearby (S3: scans most of the grid
# before giving up), vs ~0.2s/frame at 10 deg; a 894-trajectory sweep needs the cheaper one. This
# only trades off precision in the already-degraded pose_relaxed/far-fallback case -- the common
# (feasible-near-mu_psi) case hits the direct check or an early grid point either way.
PSI_GRID = np.deg2rad(np.arange(-180.0, 180.0, GRID_DEG))
CONFIG_HASH = hashlib.sha1(f"canonical_v1 grid_deg={GRID_DEG} n_grid={len(PSI_GRID)}".encode()).hexdigest()[:12]


@dataclass
class CanonicalTarget:
    q_ref: np.ndarray
    psi_ref: float            # nan if no psi term was used (undefined belief psi and no feasible branch found)
    branch_id: int            # index into m.ik(T, psi_ref)'s own deterministic enumeration order, -1 if none
    mode_id: int              # oracle belief is unimodal -> always 0 (kept for the 04_proposed_solver.md API)
    feasible: bool            # some branch at psi_ref satisfies joint limits
    pose_relaxed: bool        # no psi anywhere in the grid had a feasible branch; q_ref falls back to q_home
    cost_gap: float           # ||q_best-q_home||^2 - ||q_second-q_home||^2 among branches tied at psi_ref (nan if <2)
    n_candidates: int         # number of feasible branches found at psi_ref
    solver_config_hash: str = CONFIG_HASH


def _branches_at(m, T, psi, q_home):
    """All m.ik(T, psi) solutions within joint limits, as (branch_id, q, home_dist2), branch_id
    = index in m.ik's own (deterministic, warm-start-free) enumeration order."""
    out = []
    for i, q in enumerate(m.ik(T, psi)):
        if np.all(q >= m.lower) and np.all(q <= m.upper):
            out.append((i, q, float(np.sum((q - q_home) ** 2))))
    return out


def _grid_for(psi_grid_deg):
    """Build a PSI_GRID-shaped grid at an arbitrary step size. `psi_grid_deg=None` reproduces the
    module default (GRID_DEG) -- grid resolution is otherwise read per-call, not fixed at import
    time, so the same process can compare several grid resolutions (EX04 grid ablation) without
    restarting workers."""
    if psi_grid_deg is None:
        return PSI_GRID
    return np.deg2rad(np.arange(-180.0, 180.0, float(psi_grid_deg)))


def canonical_feasible_map(m, T, mu_psi, q_home, psi_grid=None, psi_grid_deg=None):
    """Deterministic canonical target for wrist pose T and belief psi mean mu_psi (nan if the
    belief's psi is undefined). No q_prev, no RNG: same (m, T, mu_psi, q_home) always returns
    the same CanonicalTarget (R1). Candidate psi order = direct mu_psi first, then the grid
    sorted by angular distance to mu_psi (or, if mu_psi is not finite, the grid in its fixed
    index order) -- never by distance to any robot state. Branch tie-break at the chosen psi:
    smallest ||q-q_home||^2, then branch_id, then lexicographic q (deterministic, matches
    04_proposed_solver.md's "cost tolerance bucket -> preset branch priority -> home distance ->
    q lexicographic", collapsed here since all evaluated branches already share the same psi
    cost term -- see module docstring)."""
    if psi_grid is None:
        psi_grid = _grid_for(psi_grid_deg)
    order = [mu_psi] if np.isfinite(mu_psi) else []
    order += list(psi_grid[np.argsort(np.abs(k.wrap(psi_grid - mu_psi)))] if np.isfinite(mu_psi) else psi_grid)
    for psi in order:
        cands = _branches_at(m, T, float(psi), q_home)
        if cands:
            cands.sort(key=lambda c: (round(c[2], 9), c[0], tuple(np.round(c[1], 9))))
            gap = cands[1][2] - cands[0][2] if len(cands) > 1 else float("nan")
            return CanonicalTarget(q_ref=cands[0][1], psi_ref=float(psi), branch_id=cands[0][0],
                                    mode_id=0, feasible=True, pose_relaxed=False, cost_gap=gap,
                                    n_candidates=len(cands))
    return CanonicalTarget(q_ref=q_home.copy(), psi_ref=float("nan"), branch_id=-1, mode_id=0,
                            feasible=False, pose_relaxed=True, cost_gap=float("nan"), n_candidates=0)


def track_safe(q_track, q_ref, dt, tau):
    """First-order execution filter toward the canonical target: q_track_new = q_track +
    (1-exp(-dt/tau)) * wrap(q_ref-q_track). tau small -> track q_ref within ~tau; tau large ->
    hold near the previous track state. This is the *only* place state (q_prev) and occlusion
    confidence are allowed to matter -- q_ref itself must stay canonical (see module docstring).
    `k.wrap` so a branch that reports e.g. +179 deg vs -179 deg is not read as a 358 deg move."""
    alpha = 1.0 - np.exp(-dt / max(tau, 1e-6))
    return k.wrap(q_track + alpha * k.wrap(q_ref - q_track))
