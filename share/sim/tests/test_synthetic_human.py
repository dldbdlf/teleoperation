import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import kinematics as k  # noqa: E402
import synthetic_human as sh  # noqa: E402


@pytest.fixture(scope="module")
def m():
    return k.Nero.from_urdf()


@pytest.fixture(scope="module")
def reach(m):
    return sh.reach_bounds(m)


def gen(m, reach, scenario, seed=0, idx=0, drop=1.0):
    return sh.generate(m, scenario, seed, idx, drop, sh.Config(), reach)


def test_deterministic(m, reach):
    a, _ = gen(m, reach, "S4")
    b, _ = gen(m, reach, "S4")
    for key in a:
        np.testing.assert_array_equal(a[key], b[key], err_msg=key)


@pytest.mark.parametrize("scenario", sh.SCENARIOS)
def test_ground_truth_geometry(m, reach, scenario):
    d, _ = gen(m, reach, scenario)
    W, E, psi = d["gt_W"], d["gt_E"], d["gt_psi"]
    np.testing.assert_allclose(np.linalg.norm(E - m.S, axis=1), m.L_se, atol=1e-9)
    np.testing.assert_allclose(np.linalg.norm(W - E, axis=1), m.L_ew, atol=1e-9)
    ok = d["gt_psi_defined"]
    back = np.array([k.sew_angle(m.S, e, w) for e, w in zip(E[ok], W[ok])])
    np.testing.assert_allclose(sh.wrap(back - psi[ok]), 0, atol=1e-6)
    Wc = np.array([m.wrist_center(T) for T in d["gt_T"]])  # link7 target carries the same wrist centre
    np.testing.assert_allclose(Wc, W, atol=1e-9)


def test_occlusion_masks(m, reach):
    d, meta = gen(m, reach, "S5", drop=0.5)
    t0, t1 = meta["hidden_window_s"]
    t = d["obs_t"]
    hidden = (t >= t0) & (t < t1)
    assert np.array_equal(~d["obs_wrist_visible"], hidden)
    assert np.all(np.isnan(d["obs_W"][hidden])) and not np.any(np.isnan(d["obs_W"][~hidden]))
    assert np.all(d["belief_occlusion_state"][hidden] == sh.WRIST_HIDDEN)
    assert d["belief_age_wrist_ms"][hidden].max() == pytest.approx(1000 * (t[hidden][-1] - t[hidden][0] + 1 / 15), abs=1e-6)

    d, _ = gen(m, reach, "S4", drop=0.5)
    assert d["obs_wrist_visible"].all() and np.isnan(d["obs_psi"][hidden]).all()
    assert np.all(d["belief_occlusion_state"][hidden] == sh.ELBOW_HIDDEN)


def test_s3_reaches_infeasible_wrist(m, reach):
    d, _ = gen(m, reach, "S3")
    assert d["cam_gt_wrist_feasible"][0] and not d["cam_gt_wrist_feasible"].all()


def test_s6_reverses_inside_occlusion(m, reach):
    d, meta = gen(m, reach, "S6", drop=1.0)
    t, psi = d["gt_t"], np.unwrap(d["gt_psi"])
    i = np.argmax(np.abs(psi - psi[0]))
    assert meta["hidden_window_s"][0] < t[i] < meta["hidden_window_s"][1]


def test_belief_is_calibrated(m, reach):
    """Oracle belief: normalized errors ~ N(0, 1), so about 95% fall within 2 sigma."""
    zp, zr, zpsi = [], [], []
    for i in range(30):
        d, _ = gen(m, reach, "S4", seed=1, idx=i)
        sp = np.sqrt(d["belief_wrist_cov"][:, 0, 0])
        zp.append(((d["belief_W"] - d["cam_gt_W"]) / sp[:, None]).ravel())
        zpsi.append(sh.wrap(d["belief_psi_mean"] - d["cam_gt_psi"]) / d["belief_psi_sigma"])
        R_err = [sh.logm_so3(Tb[:3, :3] @ Tg[:3, :3].T) for Tb, Tg in zip(d["belief_wrist_T"], d["cam_gt_T"])]
        zr.append((np.array(R_err) / np.sqrt(d["belief_wrist_cov"][:, 3, 3])[:, None]).ravel())
    for z in (zp, zr, zpsi):
        frac = np.mean(np.abs(np.concatenate(z)) < 2)
        assert 0.92 < frac < 0.98, frac
