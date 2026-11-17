"""NERO exact, accelerated analytic IK candidate enumeration.

Two rigorously separated optimizations:

1. FastIKPose: hoist terms of the original kinematics.Nero.ik() which do not
   depend on psi out of the per-psi loop, and invert rigid transforms with
   R.T. Returns the same joint solutions (up to floating-point tolerance).
2. exact_canonical_e1: *exact* E0 canonical q_ref AND E1 tracked q_des via
   certified branch-and-bound over psi groups. The certificate is
      L(psi) = 0.5 * wrap(psi - mu)**2 / sigma**2
   Since E0 and E1 scores have only nonnegative extras, an unvisited psi
   group with L > BOTH current best scores cannot change either optimum.

This is NOT an optimization for E2 or E3: their entropy / transition
posterior depends on every candidate, so they must keep the full bank.

No goal/initial-state approximation, no tolerance-based cache keys.
"""
from __future__ import annotations

from dataclasses import dataclass
from time import perf_counter

import numpy as np
import kinematics as k


def _inverse_point(T, p):
    """Inverse *rigid* transform on a point; avoids np.linalg.inv(4x4)."""
    return T[:3, :3].T @ (p - T[:3, 3])


def _inverse_rigid_multiply(A, B):
    """inv(A) @ B when A is a rigid transform."""
    Rt = A[:3, :3].T
    result = np.eye(4)
    result[:3, :3] = Rt @ B[:3, :3]
    result[:3, 3] = Rt @ (B[:3, 3] - A[:3, 3])
    return result


class FastIKPose:
    """Precompute the IK geometry that is constant for one target SE(3) pose."""

    def __init__(self, m, T):
        self.m = m
        self.T = np.asarray(T)
        self.g = self.T @ np.linalg.inv(m.M)
        self.W = (self.g @ np.r_[m.W0, 1])[:3]
        a, P = m.axes, m.points
        self.p7 = m.W0 + 0.1 * a[6]
        cross = np.cross(a[6], a[6] + [0.3, 0.5, 0.7])
        self.p = m.W0 + 0.1 * (cross / np.linalg.norm(cross))
        self.arm = []
        for t4 in k.sp3(a[3], P[3], m.W0, m.S, np.linalg.norm(self.W - m.S)):
            T4 = k.screw(a[3], P[3], t4)
            W4 = (T4 @ np.r_[m.W0, 1])[:3]
            self.arm.append((t4, W4, np.linalg.norm(W4 - m.E0), T4))

    def ik(self, psi):
        m, W, g = self.m, self.W, self.g
        a, P = m.axes, m.points
        sols = []
        for t4, W4, forearm, T4 in self.arm:
            E = k.elbow_from_psi(m.S, W, psi, m.L_se, forearm)
            if E is None:
                continue
            for t1, t2 in k.sp2(a[0], a[1], m.S, m.E0, E):
                T12 = k.screw(a[0], P[0], t1) @ k.screw(a[1], P[1], t2)
                t3 = k.sp1(a[2], P[2], W4, _inverse_point(T12, W))
                T14 = T12 @ k.screw(a[2], P[2], t3) @ T4
                h = _inverse_rigid_multiply(T14, g)
                for t5, t6 in k.sp2(a[4], a[5], m.W0, self.p7,
                                     (h @ np.r_[self.p7, 1])[:3]):
                    T56 = k.screw(a[4], P[4], t5) @ k.screw(a[5], P[5], t6)
                    target = (h @ np.r_[self.p, 1])[:3]
                    t7 = k.sp1(a[6], P[6], self.p, _inverse_point(T56, target))
                    sols.append(k.wrap(np.array([t1, t2, t3, t4, t5, t6, t7])))
        return sols

    def feasible(self, psi):
        m = self.m
        return [q for q in self.ik(psi)
                if np.all(q >= m.lower) and np.all(q <= m.upper)]


@dataclass
class ExactE1Result:
    q_ref: np.ndarray | None
    q_des: np.ndarray | None
    feasible_candidates: int
    psi_evaluated: int
    psi_total: int
    candidate_ms: float
    selection_ms: float


def exact_canonical_e1(m, T, mu_psi, kappa, q_track, psi_grid_deg=15.0):
    """Exact legacy E0 q_ref and E1 q_des, skipping provably noncompetitive psi.

    The reference objective omits the logsumexp constant: normalization is
    identical across candidates, so argmin is unchanged.

    E0: minimize U(psi) + 0.055 * ||q - READY||^2
    E1: minimize E0_score + 0.85 * ||q - q_track||^2
    U = 0.5 * (wrap(psi - mu) / sigma)^2 >= 0

    Both extra costs are nonnegative. Hence U is a lower bound for BOTH.
    At each iteration, visit the unprocessed psi with smallest U; after both
    solutions exist, stop only when every remaining U is strictly larger
    than both best achieved costs. The tiny epsilon avoids boundary ties.
    Original psi-grid/IK-solution order resolves exact ties reproducibly.
    """
    begin = perf_counter()
    grid = np.deg2rad(np.arange(-180, 180, float(psi_grid_deg)))
    if np.isfinite(mu_psi):
        grid = np.r_[mu_psi, grid]
    sigma = float(np.sqrt(1 / max(float(kappa), 1e-9)))
    sigma = float(np.clip(sigma, np.deg2rad(1), np.deg2rad(180)))
    delta = np.zeros(len(grid)) if not np.isfinite(mu_psi) else k.wrap(grid - float(mu_psi))
    lower = 0.5 * (delta / sigma) ** 2
    order = np.argsort(lower, kind="stable")

    context = FastIKPose(m, T)
    best_ref = (float("inf"), float("inf"), float("inf"))
    best_track = (float("inf"), float("inf"), float("inf"))
    q_ref, q_des = None, None
    n_evaluated, n_candidates, candidate_s = 0, 0, 0.0
    for j in order:
        j = int(j)
        if lower[j] > max(best_ref[0], best_track[0]) + 1e-12:
            break

        t0 = perf_counter()
        possible = context.feasible(float(grid[j]))
        n_evaluated += 1
        for branch_idx, q in enumerate(possible):
            # Exactly the same per-candidate FK gate as legacy ProbJointBranch._bank.
            if not np.all(np.isfinite(q)) or np.linalg.norm(m.fk(q) - T) >= 1e-4:
                continue
            n_candidates += 1
            candidate_s += 0.0  # explicit: FK gate time included below in candidate_s
            r = float(lower[j] + .055 * np.sum((q - k.READY) ** 2))
            t = float(r + .85 * np.sum((q - q_track) ** 2))
            # Legacy's np.argmax/min selects the first equal-scoring candidate.
            kr = (r, j, branch_idx)
            kt = (t, j, branch_idx)
            if kr < best_ref:
                best_ref = kr
                q_ref = q.copy()
            if kt < best_track:
                best_track = kt
                q_des = q.copy()
        candidate_s += perf_counter() - t0

    elapsed_s = perf_counter() - begin
    return ExactE1Result(q_ref, q_des, n_candidates, n_evaluated, len(grid),
                         1000 * candidate_s,
                         1000 * max(0., elapsed_s - candidate_s))
