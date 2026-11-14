"""R1/R2/R6 unit-level checks for canonical.py (share/docs/04_proposed_solver.md).

These are the cheap, fixture-based half of R1/R2/R6 -- determinism and path-independence of
canonical_feasible_map() itself, checkable in seconds without the 894-trajectory dataset. The
full-dataset R3-R6 numbers (noise sensitivity, occlusion recovery, feasibility boundaries) are
in share/11.14_exp/EX04_joint_consistency_results.md, produced by run_baselines.py.
"""

import subprocess
import sys

import numpy as np
import pytest

import baselines as bl
import canonical as can
import kinematics as k


@pytest.fixture(scope="module")
def m():
    return k.Nero.from_urdf()


def frame(W, R, b_cov=None, b_psi=np.nan, b_kappa=0.0):
    cov = np.eye(6) * 0.015 ** 2 if b_cov is None else b_cov
    return bl.Frame(0.0, True, True, W, R, W, 0.0, 0.0, W, R, cov, b_psi, b_kappa, 0)


def random_belief(m, rng):
    """A random feasible wrist pose (so psi has at least one feasible branch) and a psi mean."""
    q = rng.uniform(m.lower * 0.6, m.upper * 0.6)
    W, R = bl.wrist_pose(m, q)
    return W, R, float(m.psi(q))


# ---------------------------------------------------------------- R1: same-distribution determinism
def test_r1_repeated_calls_are_bit_identical(m):
    rng = np.random.default_rng(0)
    for _ in range(5):
        W, R, psi = random_belief(m, rng)
        T = bl.tool_pose(m, W, R)
        first = can.canonical_feasible_map(m, T, psi, k.READY)
        for _ in range(100):
            again = can.canonical_feasible_map(m, T, psi, k.READY)
            assert np.max(np.abs(again.q_ref - first.q_ref)) == 0.0
            assert again.branch_id == first.branch_id
            assert again.psi_ref == first.psi_ref
            assert again.feasible == first.feasible


def test_r1_call_order_independent(m):
    """No call-order / shared-state leakage: evaluating belief B right after belief A gives the
    same result as evaluating B alone, in either order."""
    rng = np.random.default_rng(1)
    beliefs = [random_belief(m, rng) for _ in range(6)]
    targets = [bl.tool_pose(m, W, R) for W, R, _ in beliefs]
    forward = [can.canonical_feasible_map(m, T, p, k.READY) for T, (_, _, p) in zip(targets, beliefs)]
    backward = [can.canonical_feasible_map(m, T, p, k.READY)
                for T, (_, _, p) in reversed(list(zip(targets, beliefs)))][::-1]
    for fwd, bwd in zip(forward, backward):
        assert np.array_equal(fwd.q_ref, bwd.q_ref)
        assert fwd.branch_id == bwd.branch_id


def test_r1_independent_of_process(m):
    """Same belief, a fresh python process: still bit-identical (no env/global-state dependence)."""
    rng = np.random.default_rng(2)
    W, R, psi = random_belief(m, rng)
    T = bl.tool_pose(m, W, R)
    here = can.canonical_feasible_map(m, T, psi, k.READY)
    script = (
        "import numpy as np, kinematics as k, canonical as can\n"
        "m = k.Nero.from_urdf()\n"
        f"T = np.array({T.tolist()!r})\n"
        f"ct = can.canonical_feasible_map(m, T, {psi!r}, k.READY)\n"
        "print(repr(ct.q_ref.tolist()), ct.branch_id)\n"
    )
    out = subprocess.run([sys.executable, "-c", script], cwd=".", capture_output=True, text=True, timeout=60)
    assert out.returncode == 0, out.stderr
    q_ref_str, branch_str = out.stdout.strip().rsplit(" ", 1)
    assert np.array_equal(np.array(eval(q_ref_str)), here.q_ref)
    assert int(branch_str) == here.branch_id


# ---------------------------------------------------------------- R2: cross-path target consistency
@pytest.mark.parametrize("cls", [bl.BeliefCanonicalIK, bl.FeasibleMAPCanonical])
def test_r2_q_ref_independent_of_q_start(m, cls):
    """Two robots that arrived from totally different q0 but see the same belief frame must
    compute the identical canonical q_ref (R-CANON); q_track (tracking layer) may legitimately
    differ, which this test also records rather than asserting away."""
    rng = np.random.default_rng(3)
    W, R, psi = random_belief(m, rng)
    f = frame(W, R, b_kappa=50.0, b_psi=psi)
    meth_a, meth_b = cls(), cls()
    meth_a.reset(m, k.READY.copy())
    meth_b.reset(m, rng.uniform(m.lower * 0.5, m.upper * 0.5))
    meth_a.update(f, meth_a.q0)
    meth_b.update(f, meth_b.q0)
    assert np.array_equal(meth_a.target.q_ref, meth_b.target.q_ref)
    assert meth_a.target.branch_id == meth_b.target.branch_id
    # q_track legitimately starts off different -- that's the tracking layer's job, not q_ref's.
    assert not np.array_equal(meth_a.q_track, meth_b.q_track) or np.array_equal(meth_a.q0, meth_b.q0)


def test_r2_q_ref_independent_of_call_history(m):
    """Approaching the same final belief through a different number of prior update() calls
    (a stand-in for 'different past trajectory') must not change the canonical q_ref."""
    rng = np.random.default_rng(4)
    W, R, psi = random_belief(m, rng)
    f_final = frame(W, R, b_kappa=50.0, b_psi=psi)
    short = bl.BeliefCanonicalIK()
    short.reset(m, k.READY.copy())
    short.update(f_final, short.q0)

    long = bl.BeliefCanonicalIK()
    long.reset(m, k.READY.copy())
    q = long.q0.copy()
    for i in range(5):  # a few unrelated prior frames before the same final belief
        w2, r2, p2 = random_belief(m, rng)
        long.update(frame(w2, r2, b_kappa=50.0, b_psi=p2), q)
        qd = long.step(q, 0.01)
        q = np.clip(q + np.clip(qd - q, -np.pi * 0.01, np.pi * 0.01), m.lower, m.upper)
    long.update(f_final, q)
    assert np.array_equal(short.target.q_ref, long.target.q_ref)
    assert short.target.branch_id == long.target.branch_id


# ---------------------------------------------------------------- R6: same wrist, different elbow intent
def test_r6_different_psi_is_not_collapsed_to_one_q(m):
    """Same 6D wrist target, two different (feasible) human elbow intents -> different q_ref.
    The canonical solver must NOT force the same joint configuration just because the wrist
    matches (04_proposed_solver.md Part 0 item 1)."""
    rng = np.random.default_rng(5)
    q_probe = rng.uniform(m.lower * 0.5, m.upper * 0.5)
    W, R = bl.wrist_pose(m, q_probe)
    T = bl.tool_pose(m, W, R)
    arcs = m.feasible_arcs(T)
    assert arcs, "probe pose should be reachable"
    lo, hi = arcs[0]
    psi_a, psi_b = lo + 0.1 * (hi - lo), lo + 0.9 * (hi - lo)
    ct_a = can.canonical_feasible_map(m, T, psi_a, k.READY)
    ct_b = can.canonical_feasible_map(m, T, psi_b, k.READY)
    assert ct_a.feasible and ct_b.feasible
    assert not np.allclose(ct_a.q_ref, ct_b.q_ref, atol=1e-3)
    assert abs(k.wrap(ct_a.psi_ref - psi_a)) < abs(k.wrap(psi_b - psi_a))
    assert abs(k.wrap(ct_b.psi_ref - psi_b)) < abs(k.wrap(psi_a - psi_b))


# ---------------------------------------------------------------- pose-relaxed fallback
def test_pose_relaxed_falls_back_to_home_and_flags_infeasible(m):
    unreachable_T = bl.tool_pose(m, np.array([10.0, 10.0, 10.0]), np.eye(3))
    ct = can.canonical_feasible_map(m, unreachable_T, np.nan, k.READY)
    assert ct.feasible is False and ct.pose_relaxed is True
    assert np.array_equal(ct.q_ref, k.READY)
