"""E0-E3 probabilistic joint-branch policies for the original EX04 harness.

DEFINITIONS:
  E0  canonical MAP branch, current robot state ignored during target selection
  E1  branch-aware tracking with fixed switch/continuity cost
  E2  categorical IK-branch posterior + entropy-modulated continuity cost
  E3  temporal posterior from previous IK modes + transition kernel

These are exploratory ablations, NOT original P/Pc methods. Human posterior
is never filtered by robot feasibility; feasible robot candidates are used for
the *decision policy* only.

Wrapper: run_894_methods.py. Dependencies: existing share/sim baseline modules.
"""
from __future__ import annotations

import time
import numpy as np
from scipy.special import logsumexp
import baselines as bl
import kinematics as k
import synthetic_human as sh

METHODS = ("E0", "E1", "E2", "E3")
METHOD_NAMES = {
    "E0": "E0 canonical branch MAP",
    "E1": "E1 branch-aware deterministic tracking",
    "E2": "E2 instantaneous joint-branch probabilities",
    "E3": "E3 temporal joint-branch posterior",
}

class ProbJointBranch(bl.Method):
    per_frame = True

    def __init__(self, kind: str, psi_grid_deg: float = 15.0):
        if kind not in METHODS:
            raise ValueError(kind)
        if not (2 <= psi_grid_deg <= 45):
            raise ValueError("psi_grid_deg must be in [2,45]")
        self.kind = kind
        self.name = METHOD_NAMES[kind]
        self.psi_grid_deg = float(psi_grid_deg)

    def reset(self, m, q0):
        super().reset(m, q0)
        self.q_ref = None
        self.prev_q_bank = None
        self.prev_p = None
        self.last_candidate_count = 0
        self.fallback_count = 0
        self.selection_latency_ms = []
        self.bank_latency_ms = []

    def _bank(self, f):
        # Fixed ordering and no q_previous dependence -> deterministic canonical q_ref
        T = sh.tool_pose(self.m, f.b_W, f.b_R)
        grid = np.deg2rad(np.arange(-180, 180, self.psi_grid_deg))
        if np.isfinite(f.b_psi):
            # Include exact psi (not only a coarse grid)
            grid = np.r_[f.b_psi, grid]
        bank, psis = [], []
        for psi in grid:
            for q in self.m.ik_feasible(T, float(psi)):
                if not np.all(np.isfinite(q)):
                    continue
                if np.linalg.norm(self.m.fk(q) - T) > 1.e-4:
                    continue
                bank.append(q)
                psis.append(float(psi))
        if not bank:
            return np.empty((0,7)), np.empty(0)
        return np.asarray(bank), np.asarray(psis)

    def update(self, f, q):
        start = time.perf_counter()
        if not np.all(np.isfinite(f.b_W)) or not np.all(np.isfinite(f.b_R)):
            self.q_des = q.copy()
            self.fallback_count += 1
            return
        bank, psis = self._bank(f)
        self.bank_latency_ms.append(1000*(time.perf_counter()-start))
        self.last_candidate_count = len(bank)
        if not len(bank):
            # Hold instead of jumping to home; explicitly count this condition.
            self.q_des = q.copy()
            self.q_ref = None
            self.prev_q_bank = None
            self.prev_p = None
            self.fallback_count += 1
            return
        s = time.perf_counter()
        # kappa in synthetic belief is inverse circular variance
        sigma = float(np.sqrt(1 / max(float(f.b_kappa), 1.e-9)))
        sigma = float(np.clip(sigma, np.deg2rad(1), np.deg2rad(180)))
        if not np.isfinite(f.b_psi):
            dpsi = np.zeros(len(psis))
        else:
            dpsi = k.wrap(psis - float(f.b_psi))
        # Immutable, predetermined home prior; no temporal robot state.
        logw = -0.5*(dpsi/sigma)**2 - .055*np.sum((bank-k.READY)**2,axis=1)
        logw -= logsumexp(logw)
        i_ref = int(np.argmax(logw))
        self.q_ref = bank[i_ref].copy()
        if self.kind == "E0":
            i = i_ref
        elif self.kind == "E1":
            score = -logw + .85*np.sum((bank-q)**2,axis=1)
            i = int(np.argmin(score))
        elif self.kind == "E2":
            p = np.exp(logw)
            entropy = -float(np.sum(p*logw))
            score = -logw + (.45+.23*entropy)*np.sum((bank-q)**2,axis=1)
            i = int(np.argmin(score))
        else:
            if self.prev_p is None or self.prev_q_bank is None:
                filtered = np.exp(logw)
            else:
                d2 = np.sum((bank[:,None,:]-self.prev_q_bank[None,:,:])**2,axis=-1)
                transition = np.exp(-.5*d2/(.60**2))+.0005
                transition /= transition.sum(axis=0,keepdims=True)
                predicted = transition @ self.prev_p
                lp = logw + np.log(np.maximum(predicted,1e-300))
                filtered = np.exp(lp-logsumexp(lp))
            score = -np.log(np.maximum(filtered,1e-300)) + .22*np.sum((bank-q)**2,axis=1)
            i = int(np.argmin(score))
            self.prev_q_bank = bank.copy()
            self.prev_p = filtered.copy()
        self.q_des = bank[i].copy()
        self.selection_latency_ms.append(1000*(time.perf_counter()-s))
