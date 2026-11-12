import numpy as np
import pytest

import baselines as bl
import kinematics as k
import run_baselines as rb
import synthetic_human as sh


@pytest.fixture(scope="module")
def m():
    return k.Nero.from_urdf()


def random_q(m, rng):
    return rng.uniform(m.lower * 0.8, m.upper * 0.8)


def frame(W, R, E, b_cov=np.eye(6), b_psi=np.nan, b_kappa=0.0):
    return bl.Frame(0.0, True, True, W, R, E, 0.0, 0.0, W, R, b_cov, b_psi, b_kappa, 0)


def drive(m, meth, f, n=300, dt=0.01):
    q = k.READY.copy()
    meth.reset(m, q)
    for i in range(n):
        if i % 7 == 0:
            f.t = i * dt
            meth.update(f, q)
        qd = meth.step(q, dt)
        if qd is not None:
            q = np.clip(q + np.clip(qd - q, -np.pi * dt, np.pi * dt), m.lower, m.upper)
    return q


def test_jacobian_matches_finite_differences(m):
    rng = np.random.default_rng(0)
    for _ in range(5):
        q = random_q(m, rng)
        J = bl.jacobian(m, q)
        W0, R0 = bl.wrist_pose(m, q)
        for i in range(7):
            dq = np.zeros(7)
            dq[i] = 1e-6
            W1, R1 = bl.wrist_pose(m, q + dq)
            num = np.r_[(W1 - W0), sh.logm_so3(R1 @ R0.T)] / 1e-6
            assert np.allclose(J[:, i], num, atol=1e-4)


@pytest.mark.parametrize("kind,f", [("neg", lambda p: -p), ("flip", lambda p: p + np.pi), ("mirror", lambda p: np.pi - p)])
def test_psi_map_moves_elbow_on_its_circle(m, kind, f):
    rng = np.random.default_rng(1)
    q = random_q(m, rng)
    S, E, W = m.sew_points(q)
    E2 = bl.PsiMap(kind).elbow(S, E, W)
    assert np.isclose(np.linalg.norm(E2 - S), np.linalg.norm(E - S))
    assert np.isclose(np.linalg.norm(E2 - W), np.linalg.norm(E - W))
    assert np.isclose(k.wrap(k.sew_angle(S, E2, W) - f(k.sew_angle(S, E, W))), 0, atol=1e-9)


def test_psi_map_offset_anchors_first_value():
    pm = bl.PsiMap("offset")
    assert np.isclose(pm.psi(0.3), k.wrap(np.pi))
    assert np.isclose(pm.psi(0.5), k.wrap(np.pi + 0.2))


def test_sew_mimic_reproduces_exact_keypoints(m):
    rng = np.random.default_rng(2)
    for _ in range(10):
        q = random_q(m, rng)
        S, E, W = m.sew_points(q)
        _, R = bl.wrist_pose(m, q)
        meth = bl.SEWMimic()
        meth.reset(m, k.READY)
        meth.update(frame(W, R, E), q)
        W2, R2 = bl.wrist_pose(m, meth.q_des)
        assert np.allclose(W2, W, atol=1e-6) and np.allclose(R2, R, atol=1e-6)
        assert np.isclose(k.wrap(m.psi(meth.q_des) - m.psi(q)), 0, atol=1e-6)


def test_common_ik_returns_feasible_nearest(m):
    q = k.READY.copy()
    W, R = bl.wrist_pose(m, q)
    out = bl.common_ik(m, W, R, 0.0, q)  # psi = 0 is infeasible at the ready wrist
    assert out is not None and m.within_limits(out)
    assert np.allclose(bl.wrist_pose(m, out)[0], W, atol=1e-6)
    assert np.isclose(abs(np.rad2deg(m.psi(out))), 70.1, atol=2.5)  # edge of the feasible arc


@pytest.mark.parametrize("cls", [bl.WristDLS, bl.AEACJParse, bl.OneEuroDLS, bl.KalmanDLS])
def test_differential_methods_converge_on_a_reachable_wrist(m, cls):
    q_t = k.READY + np.array([0.2, 0.15, -0.1, -0.2, 0.1, 0.15, -0.1])
    W, R = bl.wrist_pose(m, q_t)
    q = drive(m, cls(), frame(W, R, m.sew_points(q_t)[1]), n=400)
    W2, R2 = bl.wrist_pose(m, q)
    assert np.linalg.norm(W2 - W) < 2e-3 and np.linalg.norm(sh.logm_so3(R2 @ R.T)) < np.deg2rad(1)


def test_aeac_pulls_elbow_to_human_direction(m):
    W, R = bl.wrist_pose(m, k.READY)
    E = k.elbow_from_psi(m.S, W, np.deg2rad(120), m.L_se, m.L_ew)
    q = drive(m, bl.AEACJParse(), frame(W, R, E))
    assert abs(np.rad2deg(m.psi(q)) - 120) < 5
    assert np.linalg.norm(bl.wrist_pose(m, q)[0] - W) < 1e-3


def test_aeac_releases_guidance_for_straight_arm(m):
    meth = bl.AEACJParse()
    meth.reset(m, k.READY)
    W = m.S + np.array([-0.57, 0, 0])
    E = m.S + np.array([-0.31, 0, 0.005])  # 5 mm offset < 2 cm deadband
    meth.update(frame(W, np.eye(3), E), k.READY)
    assert meth.w_a == 0


def test_akc_corrects_forearm_length(m):
    meth = bl.AKC()
    meth.reset(m, k.READY)
    S, E, W = m.sew_points(k.READY)
    E_bad = E + np.array([0.08, 0, 0])  # depth error along the camera axis
    E_c = meth.correct(W, E_bad)
    assert np.isclose(np.linalg.norm(E_c - W), m.L_ew, atol=1e-9)
    assert np.allclose(E_c, E, atol=1e-6)  # the right depth sign is the one near the KF elbow


def test_psi_mlp_learns_a_function():
    rng = np.random.default_rng(3)
    X = rng.normal(size=(2000, 9))
    psi = np.arctan2(X[:, 1], X[:, 0])
    net = bl.PsiMLP().fit(X, psi, epochs=60)
    err = np.abs(k.wrap(net(X) - psi))
    assert np.median(err) < np.deg2rad(10)


def test_belief_weighted_ik_tracks_a_sharp_belief(m):
    q_t = k.READY + np.array([0.1, 0.1, 0.1, -0.1, 0.05, 0.1, 0.1])
    W, R = bl.wrist_pose(m, q_t)
    meth = bl.BeliefWeightedIK()
    meth.reset(m, k.READY)
    q = k.READY.copy()
    for _ in range(10):
        meth.update(frame(W, R, np.full(3, np.nan), b_cov=np.diag([1e-6] * 3 + [1e-4] * 3),
                          b_psi=m.psi(q_t), b_kappa=1e3), q)
        q = meth.q_des
    assert np.linalg.norm(bl.wrist_pose(m, q)[0] - W) < 2e-3
    assert abs(k.wrap(m.psi(q) - m.psi(q_t))) < np.deg2rad(3)


def test_harness_shield_and_all_methods_run(m):
    cfg = sh.Config(duration_s=2.0, occl_start_s=0.8)
    d, meta = sh.generate(m, "S4", 0, 0, 0.5, cfg, sh.reach_bounds(m))
    tgt, feas, best = rb.targets(m, d, "mirror")
    net = bl.PsiMLP().fit(np.random.default_rng(0).normal(size=(50, 9)), np.zeros(50), epochs=2)
    for mid in rb.METHODS:
        Q, Qreq, held, rt = rb.run(m, d, rb.make(mid, net), "mirror", np.pi, 0.5)
        assert np.all(Q >= m.lower - 1e-12) and np.all(Q <= m.upper + 1e-12), mid
        assert np.all(np.abs(np.diff(Q, axis=0)) <= np.pi * 0.01 + 1e-12), mid
        r = rb.metrics(m, d, Q, Qreq, held, rt, tgt, feas, best, meta, np.pi)
        assert np.isfinite(r["wrist_mm_p50"]), mid
        assert np.isfinite(r["recovery_jump_deg"]) and 0 <= r["recovery_capped_pct"] <= 100, mid


def test_output_interpolation_spreads_a_frame_step(m):
    cfg = sh.Config(duration_s=1.0, occl_start_s=0.5)
    d, _ = sh.generate(m, "S1", 0, 0, 0.0, cfg, sh.reach_bounds(m))
    meth = rb.make("B8", None)
    _, req_on, _, _ = rb.run(m, d, meth, "none", np.pi, 0.5, interp=True)
    _, req_off, _, _ = rb.run(m, d, rb.make("B8", None), "none", np.pi, 0.5, interp=False)
    # without interpolation the request is piecewise constant between frames; with it, it ramps
    assert np.abs(np.diff(req_on, axis=0)).max() < np.abs(np.diff(req_off, axis=0)).max()
