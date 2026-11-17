"""Certified accelerated E1 vs unmodified legacy E1 on the same NERO URDF.

R-CANON: reference must not depend on current q or tracking history.
Correctness: in all tested feasible and infeasible psi situations, q_ref/q_des
must equal full enumeration to ~1e-8 radians.
"""
from __future__ import annotations

import sys
from pathlib import Path

import numpy as np
import pytest

SIM = Path(__file__).resolve().parents[3]
PROB = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(SIM))
sys.path.insert(0, str(PROB))

import baselines as bl
import kinematics as k
import synthetic_human as sh
from fast_ik_bank import FastIKPose, exact_canonical_e1
from prob_methods import ProbJointBranch


@pytest.fixture(scope="module")
def robot():
    return k.Nero.from_urdf()


def random_frame(robot, seed=51):
    rng = np.random.default_rng(seed)
    q = np.clip(k.READY + rng.normal(0, .17, 7), robot.lower + .06, robot.upper - .06)
    W, R = bl.wrist_pose(robot, q)
    psi = float(robot.psi(q))
    return W, R, psi, q


def frame(W, R, mu, sigma):
    cov = np.eye(6) * 0.015 ** 2
    kappa = float(1.0 / sigma ** 2)
    return bl.Frame(0., True, True, W, R, W, 0., 0., W, R,
                    cov, float(mu), kappa, 0)


@pytest.mark.parametrize("grid_deg", [10., 15., 20.])
def test_same_all_ik_candidates_in_order(robot, grid_deg):
    rng = np.random.default_rng(122)
    W, R, psi, _ = random_frame(robot)
    T = sh.tool_pose(robot, W, R)
    new = FastIKPose(robot, T)
    psis = np.r_[psi, np.deg2rad(np.arange(-180, 180, grid_deg))]
    for p in psis:
        legacy = robot.ik_feasible(T, p)
        optimized = new.feasible(p)
        assert len(legacy) == len(optimized), (p, len(legacy), len(optimized))
        for a, b in zip(legacy, optimized):
            np.testing.assert_allclose(k.wrap(a-b), np.zeros(7), atol=1e-8, rtol=0)


@pytest.mark.parametrize("grid_deg", [10., 15., 20.])
@pytest.mark.parametrize("noise_deg", [5., 10., 20., 35., 65., 120.])
def test_fast_e1_qref_qdes_match_legacy(robot, grid_deg, noise_deg):
    W, R, psi, original_q = random_frame(robot, seed=72)
    rng = np.random.default_rng(2026)
    sigma = np.deg2rad(noise_deg)
    mu = float(k.wrap(psi + rng.normal(0, np.deg2rad(18))))
    q_start = np.clip(original_q + rng.normal(0, .3, 7), robot.lower, robot.upper)
    f = frame(W, R, mu, sigma)

    legacy = ProbJointBranch("E1", grid_deg, fast_e1=False)
    accelerated = ProbJointBranch("E1", grid_deg, fast_e1=True)
    legacy.reset(robot, q_start)
    accelerated.reset(robot, q_start)
    legacy.update(f, q_start)
    accelerated.update(f, q_start)

    assert (legacy.q_ref is None) == (accelerated.q_ref is None)
    if legacy.q_ref is not None:
        np.testing.assert_allclose(k.wrap(legacy.q_ref-accelerated.q_ref), 0, atol=1e-8, rtol=0)
    np.testing.assert_allclose(k.wrap(legacy.q_des-accelerated.q_des), 0, atol=1e-8, rtol=0)


def test_fast_e1_reference_independent_of_current_q(robot):
    W, R, psi, _ = random_frame(robot, seed=93)
    f = frame(W, R, psi + .13, np.deg2rad(20))
    rng = np.random.default_rng(221)
    refs = []
    for _ in range(8):
        q = rng.uniform(robot.lower + .05, robot.upper - .05)
        m = ProbJointBranch("E1", 15., fast_e1=True)
        m.reset(robot, q)
        m.update(f, q)
        assert m.q_ref is not None
        refs.append(m.q_ref.copy())
    for candidate in refs[1:]:
        np.testing.assert_array_equal(candidate, refs[0])


def test_infeasible_pose_does_not_invent_solution(robot):
    T = robot.fk(k.READY)
    T[:3, 3] += [3., 0., 0.]
    r = exact_canonical_e1(robot, T, 0., 25., k.READY, 20.)
    assert r.q_ref is None
    assert r.q_des is None
    assert r.psi_evaluated == r.psi_total


def test_fast_e1_does_not_mutate_reference_or_robot(robot):
    W, R, psi, q = random_frame(robot, seed=18)
    T = sh.tool_pose(robot, W, R)
    before = q.copy()
    first = exact_canonical_e1(robot, T, psi, 40., q, 15.)
    second = exact_canonical_e1(robot, T, psi, 40., q, 15.)
    assert np.array_equal(first.q_ref, second.q_ref)
    assert np.array_equal(first.q_des, second.q_des)
    np.testing.assert_array_equal(q, before)
