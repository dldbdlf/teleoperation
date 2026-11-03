import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import kinematics as k  # noqa: E402

RNG_SEED = 0


@pytest.fixture(scope="module")
def m():
    return k.Nero.from_urdf()


def urdf_chain_fk(q):
    """Independent FK: multiply URDF joint origins and axis rotations in order."""
    root = ET.parse(k.URDF_PATH).getroot()
    joints = {j.get("name"): j for j in root.iter("joint")}
    T = np.eye(4)
    for name, qi in zip(k.JOINTS, q):
        j = joints[name]
        o = j.find("origin")
        Tj = np.eye(4)
        Tj[:3, :3] = k.rpy_to_R(*map(float, o.get("rpy").split()))
        Tj[:3, 3] = list(map(float, o.get("xyz").split()))
        Rj = np.eye(4)
        Rj[:3, :3] = k.rot(np.array(list(map(float, j.find("axis").get("xyz").split()))), qi)
        T = T @ Tj @ Rj
    return T


def random_q(m, rng):
    return rng.uniform(m.lower, m.upper)


def test_srs_structure_and_link_lengths(m):
    m.check_srs()
    assert m.L_se == pytest.approx(0.310, abs=1e-6)
    assert m.L_ew == pytest.approx(0.270, abs=2e-5)  # URDF has 0.27001


def test_fk_matches_urdf_chain(m):
    rng = np.random.default_rng(RNG_SEED)
    for _ in range(200):
        q = random_q(m, rng)
        np.testing.assert_allclose(m.fk(q), urdf_chain_fk(q), atol=1e-10)


def test_ready_pose_is_elbow_up(m):
    assert abs(abs(m.psi(k.READY)) - np.pi) < 1e-6


def test_ik_recovers_random_configurations(m):
    rng = np.random.default_rng(RNG_SEED)
    for _ in range(1000):
        q = random_q(m, rng)
        T, psi = m.fk(q), m.psi(q)
        sols = m.ik(T, psi)
        assert sols, q
        for s in sols:  # every branch reaches the pose at the requested psi
            np.testing.assert_allclose(m.fk(s), T, atol=1e-6)  # arccos/sqrt round-off, max ~3e-7 seen
            assert abs(k.wrap(m.psi(s) - psi)) < 1e-5
        # completeness: the generating configuration is one of the branches
        assert min(np.abs(k.wrap(s - q)).max() for s in sols) < 1e-4


def test_feasible_arcs_inside_and_outside(m):
    rng = np.random.default_rng(RNG_SEED)
    margin = 1e-3
    for _ in range(15):
        q = random_q(m, rng)
        T = m.fk(q)
        arcs = m.feasible_arcs(T, n=180)
        assert k.in_arcs(m.psi(q), arcs) or min(
            min(abs(k.wrap(m.psi(q) - lo)), abs(k.wrap(m.psi(q) - hi))) for lo, hi in arcs) < margin
        for psi in rng.uniform(-np.pi, np.pi, 100):
            near_edge = any(min(abs(psi - lo), abs(psi - hi)) < margin for lo, hi in arcs)
            if near_edge:
                continue
            feas = m.ik_feasible(T, psi)
            if k.in_arcs(psi, arcs):
                assert feas, (q, psi, arcs)
                assert all(m.within_limits(s) for s in feas)
            else:
                assert not feas, (q, psi, arcs)
