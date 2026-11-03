"""NERO 7-DoF (SRS) kinematics: URDF FK, analytic IK parameterized by the SEW angle psi,
and psi feasible arcs under joint limits.

Joint axes are read from the URDF at the zero configuration and written in
product-of-exponentials form; IK is solved with Paden-Kahan subproblems, so
nothing here depends on hand-derived DH parameters.

Conventions
- Tool frame = link7. Positions in base_link, metres; angles in radians.
- S = shoulder (J1/J2/J3 axes meet), E = elbow (J4 origin), W = wrist centre (J5/J6/J7 axes meet).
- NERO's "forward" is -x (the ready pose puts the tool at x = -0.30 m).
- psi = stereographic SEW angle (Elias & Wen, arXiv 2307.13122) with singular direction
  E_T = -z (straight down into the base) and reference E_R = -x: when reaching forward
  (W - S along -x) psi = 0 means elbow down and psi = +pi/2 means elbow toward -y.
  The ready pose (upper arm vertical, forearm forward) has psi = +-pi.
"""

from __future__ import annotations

import itertools
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path

import numpy as np

URDF_PATH = Path(__file__).parent / "assets" / "nero_description.urdf"
JOINTS = [f"joint{i}" for i in range(1, 8)]
E_T = np.array([0.0, 0.0, -1.0])
E_R = np.array([-1.0, 0.0, 0.0])
_EPS = 1e-9


# ---------------------------------------------------------------- SO(3)/SE(3) helpers
def rpy_to_R(r, p, y):
    cr, sr, cp, sp, cy, sy = np.cos(r), np.sin(r), np.cos(p), np.sin(p), np.cos(y), np.sin(y)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def rot(axis, theta):
    """Rodrigues rotation about a unit axis."""
    k = np.array([[0, -axis[2], axis[1]], [axis[2], 0, -axis[0]], [-axis[1], axis[0], 0]])
    return np.eye(3) + np.sin(theta) * k + (1 - np.cos(theta)) * k @ k


def screw(axis, point, theta):
    """4x4 rigid motion: rotation by theta about the line through `point` along `axis`."""
    T = np.eye(4)
    R = rot(axis, theta)
    T[:3, :3] = R
    T[:3, 3] = point - R @ point
    return T


def wrap(a):
    return (np.asarray(a) + np.pi) % (2 * np.pi) - np.pi


def _unit(v):
    return v / np.linalg.norm(v)


# ---------------------------------------------------------------- Paden-Kahan subproblems
def sp1(w, r, p, q):
    """theta with rot(w, theta) about line (w, r) taking p to q (q, p equidistant from the axis)."""
    u, v = p - r, q - r
    u_, v_ = u - w * (w @ u), v - w * (w @ v)
    return np.arctan2(w @ np.cross(u_, v_), u_ @ v_)


def sp2(w1, w2, r, p, q):
    """All (theta1, theta2) with e^{w1 theta1} e^{w2 theta2} p = q, axes intersecting at r."""
    u, v = p - r, q - r
    c = w1 @ w2
    den = c * c - 1
    alpha = (c * (w2 @ u) - w1 @ v) / den
    beta = (c * (w1 @ v) - w2 @ u) / den
    n = np.cross(w1, w2)
    g2 = (u @ u - alpha ** 2 - beta ** 2 - 2 * alpha * beta * c) / (n @ n)
    if g2 < -1e-9:
        return []
    out = []
    for g in ({0.0} if g2 < 1e-12 else {np.sqrt(g2), -np.sqrt(g2)}):
        z = alpha * w1 + beta * w2 + g * n + r
        out.append((sp1(w1, r, z, q), sp1(w2, r, p, z)))
    return out


def sp3(w, r, p, q, delta):
    """All theta with |rot(w, theta) p - q| = delta, rotation about line (w, r)."""
    u, v = p - r, q - r
    u_, v_ = u - w * (w @ u), v - w * (w @ v)
    d2 = delta ** 2 - (w @ (p - q)) ** 2
    nu, nv = np.linalg.norm(u_), np.linalg.norm(v_)
    c = (nu ** 2 + nv ** 2 - d2) / (2 * nu * nv)
    if abs(c) > 1 + 1e-9:
        return []
    t0 = np.arctan2(w @ np.cross(u_, v_), u_ @ v_)
    a = np.arccos(np.clip(c, -1, 1))
    return [t0] if a < 1e-12 else [t0 + a, t0 - a]


# ---------------------------------------------------------------- model
@dataclass
class Nero:
    axes: np.ndarray    # (7, 3) joint axes in base frame at q = 0
    points: np.ndarray  # (7, 3) a point on each axis at q = 0
    M: np.ndarray       # link7 pose at q = 0
    lower: np.ndarray
    upper: np.ndarray
    S: np.ndarray
    E0: np.ndarray
    W0: np.ndarray

    @property
    def L_se(self):
        return float(np.linalg.norm(self.E0 - self.S))

    @property
    def L_ew(self):
        return float(np.linalg.norm(self.W0 - self.E0))

    @classmethod
    def from_urdf(cls, path=URDF_PATH):
        root = ET.parse(path).getroot()
        joints = {j.get("name"): j for j in root.iter("joint")}
        T = np.eye(4)
        axes, points, lo, hi = [], [], [], []
        for name in JOINTS:
            j = joints[name]
            o = j.find("origin")
            Tj = np.eye(4)
            Tj[:3, :3] = rpy_to_R(*map(float, o.get("rpy").split()))
            Tj[:3, 3] = list(map(float, o.get("xyz").split()))
            T = T @ Tj
            axes.append(T[:3, :3] @ np.array(list(map(float, j.find("axis").get("xyz").split()))))
            points.append(T[:3, 3].copy())
            lim = j.find("limit")
            lo.append(float(lim.get("lower")))
            hi.append(float(lim.get("upper")))
        axes, points = np.array(axes), np.array(points)
        # joint1 origin is not at the J1/J2 intersection; use joint2's origin (it lies on both axes)
        m = cls(axes, points, T, np.array(lo), np.array(hi), points[1], points[3], points[4])
        m.check_srs()
        return m

    def check_srs(self, tol=1e-6):
        """Raise if the URDF is not SRS (axes 1-3 meet at S, 5-7 meet at W, axis 4 through E)."""
        def dist(i, x):
            d = x - self.points[i]
            return np.linalg.norm(d - self.axes[i] * (self.axes[i] @ d))
        bad = [(i, c) for i, c in [(0, self.S), (1, self.S), (2, self.S), (2, self.E0), (3, self.E0),
                                   (4, self.W0), (5, self.W0), (6, self.W0)] if dist(i, c) > tol]
        if bad:
            raise ValueError(f"not an SRS chain: {bad}")

    # ---------------------------------------------------------------- forward
    def fk(self, q):
        T = np.eye(4)
        for w, p, t in zip(self.axes, self.points, q):
            T = T @ screw(w, p, t)
        return T @ self.M

    def sew_points(self, q):
        """S, E, W in base frame for configuration q."""
        T4 = np.eye(4)
        for w, p, t in zip(self.axes[:4], self.points[:4], q[:4]):
            T4 = T4 @ screw(w, p, t)
        return self.S, (T4 @ np.r_[self.E0, 1])[:3], (T4 @ np.r_[self.W0, 1])[:3]

    def psi(self, q):
        return sew_angle(*self.sew_points(q))

    def within_limits(self, q, tol=0.0):
        q = np.asarray(q)
        return bool(np.all(q >= self.lower - tol) and np.all(q <= self.upper + tol))

    # ---------------------------------------------------------------- inverse
    def wrist_center(self, T):
        return (T @ np.linalg.inv(self.M) @ np.r_[self.W0, 1])[:3]

    def ik(self, T, psi):
        """All solutions (up to 8, angles wrapped to [-pi, pi)) for link7 pose T at SEW angle psi.
        Joint limits are not applied; see `ik_feasible`."""
        a, P = self.axes, self.points
        g = T @ np.linalg.inv(self.M)
        W = (g @ np.r_[self.W0, 1])[:3]
        sols = []
        for t4 in sp3(a[3], P[3], self.W0, self.S, np.linalg.norm(W - self.S)):
            W4 = (screw(a[3], P[3], t4) @ np.r_[self.W0, 1])[:3]
            E = elbow_from_psi(self.S, W, psi, self.L_se, np.linalg.norm(W4 - self.E0))
            if E is None:
                continue
            for t1, t2 in sp2(a[0], a[1], self.S, self.E0, E):
                T12 = screw(a[0], P[0], t1) @ screw(a[1], P[1], t2)
                t3 = sp1(a[2], P[2], W4, (np.linalg.inv(T12) @ np.r_[W, 1])[:3])
                T14 = T12 @ screw(a[2], P[2], t3) @ screw(a[3], P[3], t4)
                h = np.linalg.inv(T14) @ g
                p7 = self.W0 + 0.1 * a[6]
                for t5, t6 in sp2(a[4], a[5], self.W0, p7, (h @ np.r_[p7, 1])[:3]):
                    T56 = screw(a[4], P[4], t5) @ screw(a[5], P[5], t6)
                    p = self.W0 + 0.1 * _unit(np.cross(a[6], a[6] + [0.3, 0.5, 0.7]))
                    t7 = sp1(a[6], P[6], p, (np.linalg.inv(T56) @ h @ np.r_[p, 1])[:3])
                    sols.append(wrap(np.array([t1, t2, t3, t4, t5, t6, t7])))
        return sols

    def ik_feasible(self, T, psi, lower=None, upper=None):
        lower = self.lower if lower is None else lower
        upper = self.upper if upper is None else upper
        return [q for q in self.ik(T, psi) if np.all(q >= lower) and np.all(q <= upper)]

    # ---------------------------------------------------------------- feasible arcs
    def feasible_arcs(self, T, n=360, refine_tol=1e-4, lower=None, upper=None):
        """Feasible psi set for pose T as a list of (lo, hi) arcs in [-pi, pi], lo < hi.
        Grid of n samples, then each boundary is bisected to `refine_tol`.
        Arcs narrower than one grid step can be missed."""
        ok = lambda s: bool(self.ik_feasible(T, s, lower, upper))
        grid = np.linspace(-np.pi, np.pi, n, endpoint=False)
        f = np.array([ok(s) for s in grid])
        if f.all():
            return [(-np.pi, np.pi)]
        if not f.any():
            return []

        def edge(a, b):  # ok(a) != ok(b); returns the boundary between them
            fa = ok(a)
            while abs(b - a) > refine_tol:
                m = 0.5 * (a + b)
                a, b = (m, b) if ok(m) == fa else (a, m)
            return 0.5 * (a + b)

        step = 2 * np.pi / n
        start = int(np.argmin(f))  # begin scanning from an infeasible sample
        arcs, lo = [], None
        for k in range(1, n + 1):
            i, j = (start + k - 1) % n, (start + k) % n
            if f[i] != f[j]:
                x = edge(grid[i], grid[i] + step)
                if f[j]:
                    lo = x
                else:
                    arcs.append((lo, x))
        out = []
        for lo, hi in arcs:  # unwrap arcs that cross +-pi into two pieces
            lo, hi = wrap(lo), wrap(hi)
            out += [(lo, hi)] if lo < hi else [(lo, np.pi), (-np.pi, hi)]
        return sorted(out)


# ---------------------------------------------------------------- SEW angle
def _sew_frame(e_sw):
    k_r = np.cross(e_sw - E_T, E_R)
    k_x = np.cross(k_r, e_sw)
    nx = np.linalg.norm(k_x)
    if nx < _EPS:
        return None
    e_x = k_x / nx
    return e_x, np.cross(e_sw, e_x)


def sew_angle(S, E, W):
    e_sw = _unit(W - S)
    fr = _sew_frame(e_sw)
    if fr is None:
        return np.nan
    d = E - S
    d = d - e_sw * (e_sw @ d)
    return float(np.arctan2(fr[1] @ d, fr[0] @ d))


def elbow_from_psi(S, W, psi, L1, L2):
    """Elbow position for SEW angle psi given |SE| = L1, |EW| = L2; None if unreachable."""
    D = np.linalg.norm(W - S)
    if D < _EPS or D > L1 + L2 + 1e-7 or D < abs(L1 - L2) - 1e-7:
        return None
    D = np.clip(D, abs(L1 - L2), L1 + L2)  # absorb round-off at full stretch / full fold
    e_sw = _unit(W - S)
    fr = _sew_frame(e_sw)
    if fr is None:
        return None
    a = (L1 ** 2 - L2 ** 2 + D ** 2) / (2 * D)
    r = np.sqrt(max(L1 ** 2 - a ** 2, 0.0))
    return S + a * e_sw + r * (np.cos(psi) * fr[0] + np.sin(psi) * fr[1])


def arcs_length(arcs):
    return float(sum(hi - lo for lo, hi in arcs))


def in_arcs(psi, arcs):
    return any(lo <= psi <= hi for lo, hi in arcs)


READY = np.array([0.0, 0.1, 0.0, 1.5, -1.5708, 0.0, 0.0])
