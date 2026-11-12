"""EX04 baselines B0-B8 (roadmap.md "Baselines") for the NERO arm, on synthetic_human.py data.

Every method sees the same inputs and runs inside the same harness (run_baselines.py):
camera frames at 15 Hz (last valid value held while hidden, with its age), a 100 Hz control
loop, and one safety shield (joint limits, per-joint speed cap, hold when the wrist input is
too old). Methods only return a desired configuration q_des; the shield moves q toward it.

| ID | Method                                   | Source                                  | Code          |
|----|------------------------------------------|-----------------------------------------|---------------|
| B0 | wrist DLS IK + posture home              | standard damped least squares           | reimplemented |
| B1 | SEW direction retargeting (SEW-Mimic)    | arXiv 2602.01632, Alg. 1-3              | reimplemented (code "after review") |
| B2 | adaptive elbow offset (AEAC) + J-PARSE   | arXiv 2610.16855 Sec. III               | J-PARSE: `jparse-robotics` (MIT); rest reimplemented |
| B3 | B0 + One Euro filter on the wrist target | Casiez et al., CHI 2012                 | `OneEuroFilter` (BSD) |
| B4 | B0 + constant-velocity Kalman filter     | standard CV KF                          | reimplemented |
| B5 | AKC bone-length correction + common IK   | arXiv 2606.19240 Sec. II-C/D            | reimplemented (no code found) |
| B6 | learned psi (wrist pose -> psi MLP) + common IK | Biomimetics 11(6):408, Polimi IEEE Access 2019 | reimplemented (no code found) |
| B7 | unimodal uncertainty-weighted constrained IK | roadmap definition                  | reimplemented |
| B8 | belief MAP mode -> ordinary constrained IK (sequential) | roadmap definition       | reimplemented |

"Common IK" (B5, B6, B8) = analytic NERO IK at the requested psi; if that psi is infeasible,
the nearest feasible psi on a 2 deg grid; among solutions the one closest to the current q;
hold if the wrist pose is infeasible for every psi.

Assumptions we had to add are marked ASSUMPTION; gains the papers do not give are marked
UNTUNED (chosen once, not tuned on test data).
"""

from __future__ import annotations

import warnings
from dataclasses import dataclass

import numpy as np
from jparse_robotics import JParseCore
from OneEuroFilter import OneEuroFilter
from scipy.optimize import minimize

import kinematics as k
from synthetic_human import expm_so3, logm_so3, tool_pose, wrap

warnings.filterwarnings("ignore", message="the matrix subclass", category=PendingDeprecationWarning)  # jparse uses np.matrix
PSI_GRID = np.deg2rad(np.arange(-180.0, 180.0, 2.0))


# ---------------------------------------------------------------- shared kinematics
def wrist_pose(m, q):
    """Wrist centre W and wrist-frame rotation Rg (the g in T = g M) of configuration q."""
    g = m.fk(q) @ np.linalg.inv(m.M)
    return g[:3, :3] @ m.W0 + g[:3, 3], g[:3, :3]


def jacobian(m, q):
    """6x7 world-frame Jacobian of [W velocity; angular velocity]."""
    W = wrist_pose(m, q)[0]
    J = np.zeros((6, 7))
    T = np.eye(4)
    for i, (a, p, t) in enumerate(zip(m.axes, m.points, q)):
        w = T[:3, :3] @ a
        r = T[:3, :3] @ p + T[:3, 3]
        J[:3, i] = np.cross(w, W - r)
        J[3:, i] = w
        T = T @ k.screw(a, p, t)
    return J


def pose_error(m, q, W_d, R_d):
    W, R = wrist_pose(m, q)
    return np.r_[W_d - W, logm_so3(R_d @ R.T)]


def closest(cands, q_ref):
    return min(cands, key=lambda q: np.abs(wrap(q - q_ref)).max()) if cands else None


def common_ik(m, W, R, psi, q_ref):
    """Nearest feasible psi (2 deg grid) to `psi`, then the solution closest to q_ref; None if none."""
    T = tool_pose(m, W, R)
    order = [psi] if np.isfinite(psi) else []
    base = psi if np.isfinite(psi) else m.psi(q_ref)
    order += list(PSI_GRID[np.argsort(np.abs(wrap(PSI_GRID - base)))])
    for s in order:
        q = closest(m.ik_feasible(T, s), q_ref)
        if q is not None:
            return q
    return None


# ---------------------------------------------------------------- human -> robot psi mapping
class PsiMap:
    """Applied to every human elbow / psi input before any method sees it (decision pending).
    none: psi; neg: -psi; flip: psi + pi; mirror: pi - psi; offset: pi + (psi - psi0), psi0 = first seen."""

    KINDS = ("none", "neg", "flip", "mirror", "offset")

    def __init__(self, kind="none"):
        assert kind in self.KINDS, kind
        self.kind, self.psi0 = kind, None

    def psi(self, p):
        if self.kind == "offset" and self.psi0 is None and np.isfinite(p):
            self.psi0 = p
        f = {"none": lambda x: x, "neg": lambda x: -x, "flip": lambda x: x + np.pi,
             "mirror": lambda x: np.pi - x, "offset": lambda x: np.pi + x - self.psi0}[self.kind]
        return float(wrap(f(p))) if np.isfinite(p) else np.nan

    def elbow(self, S, E, W):
        """Elbow moved on its own S-W circle to the mapped psi (arm lengths kept)."""
        if self.kind == "none" or not np.all(np.isfinite(E)):
            return E
        p = self.psi(k.sew_angle(S, E, W))
        out = k.elbow_from_psi(S, W, p, np.linalg.norm(E - S), np.linalg.norm(W - E))
        return E if out is None else out


# ---------------------------------------------------------------- inputs
@dataclass
class Frame:
    """One camera frame as every method sees it. W/R/E are the last valid values (held while
    hidden; NaN before the first sighting). Elbow / psi are already mapped by PsiMap."""
    t: float
    wrist_visible: bool
    elbow_visible: bool
    W: np.ndarray
    R: np.ndarray
    E: np.ndarray
    wrist_age: float
    elbow_age: float
    b_W: np.ndarray       # oracle belief (B7, B8)
    b_R: np.ndarray
    b_cov: np.ndarray
    b_psi: float
    b_kappa: float
    b_state: int


class Method:
    name = "?"
    per_frame = False  # True: q_des is set once per camera frame (the harness interpolates it)

    def reset(self, m, q0):
        self.m, self.q0, self.q_des = m, q0.copy(), None

    def update(self, f: Frame, q):
        """New camera frame."""

    def step(self, q, dt):
        """Every control tick; returns q_des, or None to hold."""
        return self.q_des


# ---------------------------------------------------------------- B0 / B3 / B4: wrist DLS IK + posture home
class WristDLS(Method):
    """B0. dq = J^T (J J^T + lam^2 I)^-1 v + (I - J^+ J)(-k_h (q - q0)), v = [kp e_W; kR e_R].
    UNTUNED: kp = kR = 10 /s, lam = 0.05, k_h = 1 /s."""
    name = "B0 wrist DLS"

    def __init__(self, kp=10.0, kr=10.0, lam=0.05, kh=1.0):
        self.kp, self.kr, self.lam, self.kh = kp, kr, lam, kh

    def reset(self, m, q0):
        super().reset(m, q0)
        self.target = None

    def filter(self, f):
        return f.W, f.R

    def update(self, f, q):
        if np.all(np.isfinite(f.W)):
            self.target = self.filter(f)

    def step(self, q, dt):
        if self.target is None:
            return None
        e = pose_error(self.m, q, *self.target)
        v = np.r_[self.kp * e[:3], self.kr * e[3:]]
        J = jacobian(self.m, q)
        dq = J.T @ np.linalg.solve(J @ J.T + self.lam ** 2 * np.eye(6), v)
        dq += (np.eye(7) - np.linalg.pinv(J) @ J) @ (-self.kh * (q - self.q0))
        return q + dq * dt


class OneEuroDLS(WristDLS):
    """B3. One Euro filter on wrist position and on the rotation vector relative to the first
    orientation. UNTUNED: position mincutoff 1 Hz, beta 10; rotation mincutoff 1 Hz, beta 1."""
    name = "B3 DLS + One Euro"

    def reset(self, m, q0):
        super().reset(m, q0)
        self.fp = [OneEuroFilter(15.0, mincutoff=1.0, beta=10.0) for _ in range(3)]
        self.fr = [OneEuroFilter(15.0, mincutoff=1.0, beta=1.0) for _ in range(3)]
        self.R_ref = None

    def filter(self, f):
        if self.R_ref is None:
            self.R_ref = f.R
        r = logm_so3(self.R_ref.T @ f.R)
        W = np.array([fi(x, f.t) for fi, x in zip(self.fp, f.W)])
        r = np.array([fi(x, f.t) for fi, x in zip(self.fr, r)])
        return W, self.R_ref @ expm_so3(r)


class CVKalman:
    """Constant-velocity Kalman filter on a 3-vector; `step(z or None, dt)` returns the position."""

    def __init__(self, sig_meas, sig_acc):
        self.sig_meas, self.sig_acc, self.x, self.P = sig_meas, sig_acc, None, None

    def step(self, z, dt):
        if self.x is None:
            if z is None:
                return None
            self.x, self.P = np.r_[z, np.zeros(3)], np.diag([self.sig_meas ** 2] * 3 + [1.0] * 3)
            return z
        F = np.block([[np.eye(3), dt * np.eye(3)], [np.zeros((3, 3)), np.eye(3)]])
        G = np.r_[0.5 * dt ** 2 * np.ones(3), dt * np.ones(3)]
        self.x, self.P = F @ self.x, F @ self.P @ F.T + self.sig_acc ** 2 * np.outer(G, G) * np.kron(np.ones((2, 2)), np.eye(3))
        if z is not None:
            H = np.c_[np.eye(3), np.zeros((3, 3))]
            K = self.P @ H.T @ np.linalg.inv(H @ self.P @ H.T + self.sig_meas ** 2 * np.eye(3))
            self.x = self.x + K @ (z - H @ self.x)
            self.P = (np.eye(6) - K @ H) @ self.P
        return self.x[:3]


class KalmanDLS(WristDLS):
    """B4. Constant-velocity Kalman filters on W and on the rotation vector relative to the first
    orientation; they predict through wrist dropouts (the shield's age hold still applies).
    Measurement noise = generator noise (15 mm, 5 deg). UNTUNED: white-acceleration
    sigma 1 m/s^2 and 5 rad/s^2."""
    name = "B4 DLS + Kalman"

    def reset(self, m, q0):
        super().reset(m, q0)
        self.kw, self.kr_ = CVKalman(0.015, 1.0), CVKalman(np.deg2rad(5.0), 5.0)
        self.t_last, self.R_ref = None, None

    def update(self, f, q):
        dt = 0.0 if self.t_last is None else f.t - self.t_last
        self.t_last = f.t
        if f.wrist_visible and self.R_ref is None:
            self.R_ref = f.R
        W = self.kw.step(f.W if f.wrist_visible else None, dt)
        r = self.kr_.step(logm_so3(self.R_ref.T @ f.R) if f.wrist_visible else None, dt)
        if W is not None:
            self.target = (W, self.R_ref @ expm_so3(r))


# ---------------------------------------------------------------- B1: SEW-Mimic
class SEWMimic(Method):
    """B1, SEW-Mimic (arXiv 2602.01632) Alg. 1-3 on NERO: Subproblem 2 on (q1, q2) aligns the
    upper-arm axis (joint 3) with the human upper arm, then (q3, q4) align the forearm axis
    (joint 5) with the human forearm, then (q5, q6, q7) match the hand orientation. Each step
    drops out-of-limit pairs and keeps the pair closest to the current q. Wrist position is
    not tracked (directions only), as in the paper.
    ASSUMPTION: if no pair is within limits, the closest pair is clipped to the limits
    (the paper's BoundJoints is only described for q7)."""
    name = "B1 SEW-Mimic"
    per_frame = True

    def _pick(self, pairs, q, idx):
        if not pairs:
            return None
        pairs = [wrap(np.array(p)) for p in pairs]
        lo, hi = self.m.lower[idx], self.m.upper[idx]
        ok = [p for p in pairs if np.all(p >= lo) and np.all(p <= hi)]
        best = min(ok or pairs, key=lambda p: np.abs(wrap(p - q[idx])).sum())
        return np.clip(best, lo, hi)

    def update(self, f, q):
        if not (np.all(np.isfinite(f.W)) and np.all(np.isfinite(f.E))):
            return
        m, a, P = self.m, self.m.axes, self.m.points
        S = m.S
        u, l = k._unit(f.E - S), k._unit(f.W - f.E)
        q12 = self._pick(k.sp2(a[0], a[1], S, m.E0, S + m.L_se * u), q, [0, 1])
        if q12 is None:
            return
        T12 = k.screw(a[0], P[0], q12[0]) @ k.screw(a[1], P[1], q12[1])
        E_r = S + m.L_se * u
        x = (np.linalg.inv(T12) @ np.r_[E_r + m.L_ew * l, 1])[:3]
        q34 = self._pick(k.sp2(a[2], a[3], m.E0, m.W0, x), q, [2, 3])
        if q34 is None:
            return
        T14 = T12 @ k.screw(a[2], P[2], q34[0]) @ k.screw(a[3], P[3], q34[1])
        W_r = (T14 @ np.r_[m.W0, 1])[:3]
        g = tool_pose(m, W_r, f.R) @ np.linalg.inv(m.M)
        h = np.linalg.inv(T14) @ g
        p7 = m.W0 + 0.1 * a[6]
        q56 = self._pick(k.sp2(a[4], a[5], m.W0, p7, (h @ np.r_[p7, 1])[:3]), q, [4, 5])
        if q56 is None:
            return
        T56 = k.screw(a[4], P[4], q56[0]) @ k.screw(a[5], P[5], q56[1])
        p = m.W0 + 0.1 * k._unit(np.cross(a[6], a[6] + [0.3, 0.5, 0.7]))
        q7 = k.sp1(a[6], P[6], p, (np.linalg.inv(T56) @ h @ np.r_[p, 1])[:3])
        q7 = np.clip(wrap(q7), m.lower[6], m.upper[6])
        self.q_des = np.r_[q12, q34, q56, q7]


# ---------------------------------------------------------------- B2: AEAC + J-PARSE (arXiv 2610.16855)
def elbow_offset(S, E, W, eps=1e-6):
    """Eq. (7)-(10): unit perpendicular elbow direction n and its magnitude h."""
    d = (W - S) / max(np.linalg.norm(W - S), eps)
    v = E - (S + d * (d @ (E - S)))
    h = np.linalg.norm(v)
    return v / max(h, eps), h


class AEACJParse(Method):
    """B2. Hierarchical velocity solver: dq* = J#_JPARSE v_d + N (dq_home + dq_aeac),
    dq_aeac = -k_e grad C, C = w_a(h) |n_r(q) - n_h|^2, w_a = w_max clip((h - delta)/(l_h - delta), 0, 1),
    then the inf-norm speed scaling of Eq. (28). Target position One Euro filtered
    (mincutoff 5, beta 10), orientation smoothed toward the new frame with alpha 0.9, as in the
    paper. J-PARSE (gamma 0.1) from the official `jparse-robotics` package.
    UNTUNED (not in the paper): kp = kR = 10 /s, k_h = 0.5 /s, w_max = 1, delta = 2 cm; k_e = 50 chosen
    so a 60 deg elbow step on the static ready wrist settles in ~0.6 s (speed-cap limited above 50).
    The gradient of C is by central differences (the paper uses autodiff)."""
    name = "B2 AEAC + J-PARSE"

    def __init__(self, kp=10.0, kr=10.0, kh=0.5, ke=50.0, w_max=1.0, delta=0.02, gamma=0.1, qd_max=np.pi):
        self.kp, self.kr, self.kh, self.ke = kp, kr, kh, ke
        self.w_max, self.delta, self.qd_max = w_max, delta, qd_max
        self.jp = JParseCore(gamma=gamma)

    def reset(self, m, q0):
        super().reset(m, q0)
        self.fp = [OneEuroFilter(15.0, mincutoff=5.0, beta=10.0) for _ in range(3)]
        self.target, self.R_f, self.n_h, self.w_a = None, None, None, 0.0

    def update(self, f, q):
        if np.all(np.isfinite(f.W)):
            W = np.array([fi(x, f.t) for fi, x in zip(self.fp, f.W)])
            self.R_f = f.R if self.R_f is None else self.R_f @ expm_so3(0.9 * logm_so3(self.R_f.T @ f.R))
            self.target = (W, self.R_f)
        if np.all(np.isfinite(f.E)) and np.all(np.isfinite(f.W)):
            self.n_h, h = elbow_offset(self.m.S, f.E, f.W)
            l_h = np.linalg.norm(f.W - self.m.S)
            self.w_a = self.w_max * np.clip((h - self.delta) / (l_h - self.delta), 0, 1)

    def cost(self, q):
        n_r, _ = elbow_offset(*self.m.sew_points(q))
        return self.w_a * np.sum((n_r - self.n_h) ** 2)

    def step(self, q, dt):
        if self.target is None:
            return None
        e = pose_error(self.m, q, *self.target)
        v = np.r_[self.kp * e[:3], self.kr * e[3:]]
        J = jacobian(self.m, q)
        Jp, N = self.jp.compute(J, position_dimensions=3, angular_dimensions=3, return_nullspace=True)
        sec = -self.kh * (q - self.q0)
        if self.n_h is not None and self.w_a > 0:
            eps = 1e-5
            grad = np.array([(self.cost(q + eps * ei) - self.cost(q - eps * ei)) / (2 * eps) for ei in np.eye(7)])
            sec = sec - self.ke * grad
        dq = Jp @ v + N @ sec
        dq *= min(1.0, self.qd_max / (np.abs(dq).max() + 1e-9))
        return q + dq * dt


# ---------------------------------------------------------------- B5: AKC (arXiv 2606.19240) + common IK
class AKC(Method):
    """B5. Stage 3: constant-velocity KF per landmark (wrist, elbow), predicting missing ones.
    Stage 4: elbow depth from the KF wrist by Eq. (7), p_e,z = p_w,z +- sqrt(l_f^2 - dx^2 - dy^2),
    candidate chosen by w1 |E - E_kf| + w3 |E - E_prev| + m5 (elbow angle outside [40, 180] deg),
    paper weights W_c (w1 = 1, w3 = 20, s = 0.9); negative radicand -> Eq. (9) projection.
    Then psi = SEW angle of (S, E_akc, W_kf) -> common IK, wrist target = KF wrist + observed R.
    ASSUMPTIONS: the shoulder is the known robot shoulder, so the shoulder terms (w2, w4) drop
    out; the camera depth axis is base x; the synthetic noise is isotropic so the KF uses one
    sigma for x, y, z; the shrink factor picks the sign, the final depth uses the full l_f."""
    name = "B5 AKC + IK"
    per_frame = True

    def __init__(self, w1=1.0, w3=20.0, s=0.9, k5=1.0):
        self.w1, self.w3, self.s, self.k5 = w1, w3, s, k5

    def reset(self, m, q0):
        super().reset(m, q0)
        self.kf_w, self.kf_e = CVKalman(0.015, 1.0), CVKalman(0.02, 1.0)  # UNTUNED sig_acc 1 m/s^2
        self.t_last, self.E_prev = None, None

    def correct(self, W, E_kf):
        l_f = self.m.L_ew
        dyz = E_kf[1:] - W[1:]
        cands = []
        if self.s ** 2 * l_f ** 2 - dyz @ dyz >= 0:
            rad = max(l_f ** 2 - dyz @ dyz, 0.0)
            cands = [np.r_[W[0] + sg * np.sqrt(rad), E_kf[1:]] for sg in (1, -1)]
        if not cands:  # Eq. (9) fallback
            return W + k._unit(E_kf - W) * l_f

        def cost(E):
            c = self.w1 * np.linalg.norm(E - E_kf)
            if self.E_prev is not None:
                c += self.w3 * np.linalg.norm(E - self.E_prev)
            u, v = self.m.S - E, W - E
            ang = np.degrees(np.arccos(np.clip(u @ v / (np.linalg.norm(u) * np.linalg.norm(v)), -1, 1)))
            return c + (self.k5 if not 40 <= ang <= 180 else 0.0)
        return min(cands, key=cost)

    def update(self, f, q):
        dt = 0.0 if self.t_last is None else f.t - self.t_last
        self.t_last = f.t
        W = self.kf_w.step(f.W if f.wrist_visible else None, dt)
        E = self.kf_e.step(f.E if f.elbow_visible else None, dt)
        if W is None or E is None or not np.all(np.isfinite(f.R)):
            return
        E = self.correct(W, E)
        self.E_prev = E
        qd = common_ik(self.m, W, f.R, k.sew_angle(self.m.S, E, W), q)
        if qd is not None:
            self.q_des = qd


# ---------------------------------------------------------------- B6: learned psi + common IK
def psi_features(m, W, R):
    return np.r_[W - m.S, R[:, 0], R[:, 1]]


class PsiMLP:
    """Small numpy MLP (2 x 64 tanh) from wrist pose features to (cos psi, sin psi), Adam, MSE."""

    def __init__(self, n_in=9, hidden=64, seed=0):
        rng = np.random.default_rng(seed)
        sz = [n_in, hidden, hidden, 2]
        self.p = []
        for a, b in zip(sz[:-1], sz[1:]):
            self.p += [rng.normal(scale=1 / np.sqrt(a), size=(a, b)), np.zeros(b)]
        self.mu, self.sd = np.zeros(n_in), np.ones(n_in)

    def _fwd(self, X):
        h = [X]
        for i in range(0, len(self.p) - 2, 2):
            h.append(np.tanh(h[-1] @ self.p[i] + self.p[i + 1]))
        return h, h[-1] @ self.p[-2] + self.p[-1]

    def fit(self, X, psi, epochs=300, lr=3e-3, batch=256, seed=0):
        rng = np.random.default_rng(seed)
        self.mu, self.sd = X.mean(0), X.std(0) + 1e-9
        X, Y = (X - self.mu) / self.sd, np.c_[np.cos(psi), np.sin(psi)]
        mom = [np.zeros_like(p) for p in self.p]
        vel = [np.zeros_like(p) for p in self.p]
        it = 0
        for _ in range(epochs):
            for idx in np.array_split(rng.permutation(len(X)), max(1, len(X) // batch)):
                h, out = self._fwd(X[idx])
                d = 2 * (out - Y[idx]) / len(idx)
                grads = [None] * len(self.p)
                for i in range(len(self.p) - 2, -1, -2):
                    grads[i], grads[i + 1] = h[i // 2].T @ d, d.sum(0)
                    if i:
                        d = (d @ self.p[i].T) * (1 - h[i // 2] ** 2)
                it += 1
                for j, g in enumerate(grads):
                    mom[j] = 0.9 * mom[j] + 0.1 * g
                    vel[j] = 0.999 * vel[j] + 0.001 * g * g
                    self.p[j] -= lr * (mom[j] / (1 - 0.9 ** it)) / (np.sqrt(vel[j] / (1 - 0.999 ** it)) + 1e-8)
        return self

    def __call__(self, x):
        out = self._fwd(((np.atleast_2d(x) - self.mu) / self.sd))[1]
        return np.arctan2(out[:, 1], out[:, 0])


class LearnedPsi(Method):
    """B6. psi target predicted from the (held) wrist pose by a PsiMLP trained offline on
    separate trajectories (GT wrist pose -> mapped GT psi); the human elbow is never used.
    The paper's network size and features are not accessible (MDPI 403); ours is ASSUMED."""
    name = "B6 learned psi + IK"
    per_frame = True

    def __init__(self, net: PsiMLP):
        self.net = net

    def update(self, f, q):
        if not np.all(np.isfinite(f.W)):
            return
        psi = float(self.net(psi_features(self.m, f.W, f.R))[0])
        qd = common_ik(self.m, f.W, f.R, psi, q)
        if qd is not None:
            self.q_des = qd


# ---------------------------------------------------------------- B7 / B8: belief inputs
class BeliefWeightedIK(Method):
    """B7. q* = argmin 1/2 e^T Sigma^-1 e + kappa (1 - cos(psi(q) - mu_psi)) + lam/2 |q - q_prev|^2
    within joint limits (L-BFGS-B from the current q), e = belief wrist mean minus wrist(q) in
    the belief's tangent space. Unimodal belief, weights = belief precision.
    UNTUNED: lam = 1 (a small continuity regulariser)."""
    name = "B7 belief-weighted IK"
    per_frame = True

    def __init__(self, lam=1.0):
        self.lam = lam

    def update(self, f, q):
        Wi = np.linalg.inv(f.b_cov)
        mu_psi = f.b_psi
        kap = f.b_kappa if np.isfinite(mu_psi) else 0.0

        def cost(x):
            e = pose_error(self.m, x, f.b_W, f.b_R)
            c = 0.5 * e @ Wi @ e + 0.5 * self.lam * np.sum((x - q) ** 2)
            p = self.m.psi(x)
            if kap > 0 and np.isfinite(p):
                c += kap * (1 - np.cos(p - mu_psi))
            return c
        r = minimize(cost, q, method="L-BFGS-B", bounds=list(zip(self.m.lower, self.m.upper)),
                     options={"maxiter": 60})
        self.q_des = r.x


class SequentialBeliefIK(Method):
    """B8. Take the belief's MAP mode (wrist mean, highest-weight psi mode) and hand it to the
    common IK, which ignores the uncertainty. Our oracle belief is unimodal, so the mode is the
    mean; a `b_psi_modes` field with weights would be used if present."""
    name = "B8 belief MAP -> IK"
    per_frame = True

    def update(self, f, q):
        psi = f.b_psi
        modes = getattr(f, "b_psi_modes", None)
        if modes:
            psi = max(modes, key=lambda mw: mw[1])[0]
        qd = common_ik(self.m, f.b_W, f.b_R, psi, q)
        if qd is not None:
            self.q_des = qd
