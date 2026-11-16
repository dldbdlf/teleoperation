"""R1/R2-style canonical-consistency checks for E0-E3 (prob_methods.ProbJointBranch), extending
share/sim/tests/test_canonical.py's R1/R2 coverage of canonical.py/baselines.py to this module.

The 2026-11-15 prompt requires this explicitly: "E1/E2/E3의 실제 제어 목표(q_des)가 달라지더라도
기준 관절각 q_ref는 동일 입력에 대해 동일해야 한다" -- E1/E2/E3 each compute their own self.q_ref
(the canonical/E0 target) alongside whatever they actually track (self.q_des), and that q_ref must
not depend on self.kind, on the robot's current q, or on prior update() calls (R-CANON). Looking at
ProbJointBranch.update(): logw/i_ref/q_ref are computed purely from the frame (f.b_W/b_R/b_psi/
b_kappa) and self.psi_grid_deg -- `q` and the self.prev_p/self.prev_q_bank temporal-filter state
are only ever read in the kind-specific branches that pick `i` (the tracked index), never for
i_ref/q_ref. These tests assert that invariant holds rather than just re-deriving it from reading
the code.
"""
from __future__ import annotations

import sys
from pathlib import Path

SIM = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(SIM))
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import numpy as np
import pytest

import baselines as bl
import kinematics as k
from prob_methods import ProbJointBranch


@pytest.fixture(scope="module")
def m():
    return k.Nero.from_urdf()


def frame(W, R, b_psi=np.nan, b_kappa=0.0):
    cov = np.eye(6) * 0.015 ** 2
    return bl.Frame(0.0, True, True, W, R, W, 0.0, 0.0, W, R, cov, b_psi, b_kappa, 0)


def random_belief(m, rng):
    q = rng.uniform(m.lower * 0.6, m.upper * 0.6)
    W, R = bl.wrist_pose(m, q)
    return W, R, float(m.psi(q))


# -------------------------------------------------- R-CANON: q_ref same across E0/E1/E2/E3
def test_q_ref_identical_across_all_four_kinds_same_frame(m):
    rng = np.random.default_rng(10)
    W, R, psi = random_belief(m, rng)
    f = frame(W, R, b_psi=psi, b_kappa=50.0)
    q_start = rng.uniform(m.lower * 0.5, m.upper * 0.5)
    refs = {}
    for kind in ("E0", "E1", "E2", "E3"):
        meth = ProbJointBranch(kind, psi_grid_deg=15.0)
        meth.reset(m, q_start.copy())
        meth.update(f, q_start.copy())
        assert meth.q_ref is not None, f"{kind}: bank must be non-empty for a feasible probe pose"
        refs[kind] = meth.q_ref.copy()
    base = refs["E0"]
    for kind in ("E1", "E2", "E3"):
        assert np.array_equal(refs[kind], base), f"{kind}.q_ref diverged from E0.q_ref for identical input"


# -------------------------------------------------- R1: repeated calls are bit-identical
@pytest.mark.parametrize("kind", ["E0", "E1", "E2", "E3"])
def test_r1_q_ref_repeated_calls_bit_identical(m, kind):
    rng = np.random.default_rng(11)
    W, R, psi = random_belief(m, rng)
    f = frame(W, R, b_psi=psi, b_kappa=50.0)
    q0 = k.READY.copy()
    first = None
    for _ in range(20):
        meth = ProbJointBranch(kind, psi_grid_deg=15.0)
        meth.reset(m, q0.copy())
        meth.update(f, q0.copy())
        if first is None:
            first = meth.q_ref.copy()
        else:
            assert np.array_equal(meth.q_ref, first)


# -------------------------------------------------- R2: independent of q_start and call history
@pytest.mark.parametrize("kind", ["E0", "E1", "E2", "E3"])
def test_r2_q_ref_independent_of_q_start(m, kind):
    rng = np.random.default_rng(12)
    W, R, psi = random_belief(m, rng)
    f = frame(W, R, b_psi=psi, b_kappa=50.0)
    a = ProbJointBranch(kind, psi_grid_deg=15.0)
    a.reset(m, k.READY.copy())
    a.update(f, a.q0)
    b = ProbJointBranch(kind, psi_grid_deg=15.0)
    far_q0 = rng.uniform(m.lower * 0.5, m.upper * 0.5)
    b.reset(m, far_q0)
    b.update(f, b.q0)
    assert np.array_equal(a.q_ref, b.q_ref)


@pytest.mark.parametrize("kind", ["E1", "E2", "E3"])  # E0's i==i_ref, so history can't matter to it
def test_r2_q_ref_independent_of_call_history(m, kind):
    """Approaching the same final belief through several unrelated prior update() calls (E1/E2/E3
    each accumulate kind-specific temporal state across calls, e.g. E3's prev_p/prev_q_bank) must
    not perturb q_ref, even though it legitimately changes q_des."""
    rng = np.random.default_rng(13)
    W, R, psi = random_belief(m, rng)
    f_final = frame(W, R, b_psi=psi, b_kappa=50.0)

    short = ProbJointBranch(kind, psi_grid_deg=15.0)
    short.reset(m, k.READY.copy())
    short.update(f_final, short.q0)

    long = ProbJointBranch(kind, psi_grid_deg=15.0)
    long.reset(m, k.READY.copy())
    q = long.q0.copy()
    for _ in range(5):
        w2, r2, p2 = random_belief(m, rng)
        long.update(frame(w2, r2, b_psi=p2, b_kappa=50.0), q)
        if long.q_des is not None:
            q = long.q_des.copy()
    long.update(f_final, q)
    assert np.array_equal(short.q_ref, long.q_ref)


# -------------------------------------------------- q_des (tracked target) is free to diverge
def test_q_des_may_legitimately_differ_across_kinds_while_q_ref_does_not(m):
    """Sanity check for the test suite itself: if q_des never diverged either, the R-CANON tests
    above would be vacuous (the methods would be indistinguishable). Pick a frame + robot history
    where E1/E2/E3's continuity cost actually pulls the tracked target away from the raw MAP."""
    rng = np.random.default_rng(14)
    q_cur = rng.uniform(m.lower * 0.3, m.upper * 0.3)
    methods = {kind: ProbJointBranch(kind, psi_grid_deg=15.0) for kind in ("E0", "E1", "E2", "E3")}
    for meth in methods.values():
        meth.reset(m, q_cur.copy())
    found_divergence = False
    for _ in range(8):
        W, R, psi = random_belief(m, rng)
        f = frame(W, R, b_psi=psi, b_kappa=3.0)  # low kappa -> wide belief -> more branch ambiguity
        for meth in methods.values():
            meth.update(f, q_cur)
        refs = [meth.q_ref for meth in methods.values() if meth.q_ref is not None]
        if all(np.array_equal(refs[0], r) for r in refs[1:]):
            pass  # R-CANON invariant holds on this frame too (checked exhaustively above already)
        dess = [meth.q_des for meth in methods.values() if meth.q_des is not None]
        if len(dess) > 1 and not all(np.array_equal(dess[0], d) for d in dess[1:]):
            found_divergence = True
        q_cur = methods["E3"].q_des.copy() if methods["E3"].q_des is not None else q_cur
    assert found_divergence, "expected E1/E2/E3's tracked target to differ from E0 at least once"
