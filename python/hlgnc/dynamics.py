"""Quaternion-based 6-DOF rigid-body flight dynamics.

State vector (14 elements):
    x[0:3]   p_ned   position in local NED frame [m]
    x[3:6]   v_b     velocity in body axes [m/s]
    x[6:10]  q_nb    attitude quaternion (scalar first), rotates
                     body -> NED
    x[10:13] w_b     body angular rates p, q, r [rad/s]
    x[13]    reserved (0) -- keeps the vector length even for
                     alignment with the C implementation.

Flat-earth, constant gravity. WGS84 gravity/round-earth kinematics are
a documented roadmap item; for terminal-area flight (< 30 km altitude,
< 100 km range) flat-earth errors are small relative to the aerodynamic
model uncertainty.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Callable

import numpy as np

GRAV = 9.80665

STATE_DIM = 14


def quat_normalize(q: np.ndarray) -> np.ndarray:
    return q / np.linalg.norm(q)


def quat_to_dcm(q: np.ndarray) -> np.ndarray:
    """Direction cosine matrix C_nb (body -> NED) from quaternion."""
    w, x, y, z = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
    ])


def quat_from_euler(phi: float, theta: float, psi: float) -> np.ndarray:
    """Quaternion from 3-2-1 Euler angles [rad]."""
    cph, sph = np.cos(phi / 2), np.sin(phi / 2)
    cth, sth = np.cos(theta / 2), np.sin(theta / 2)
    cps, sps = np.cos(psi / 2), np.sin(psi / 2)
    return np.array([
        cph * cth * cps + sph * sth * sps,
        sph * cth * cps - cph * sth * sps,
        cph * sth * cps + sph * cth * sps,
        cph * cth * sps - sph * sth * cps,
    ])


def quat_to_euler(q: np.ndarray) -> tuple[float, float, float]:
    """3-2-1 Euler angles (phi, theta, psi) [rad] from quaternion."""
    w, x, y, z = q
    phi = np.arctan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    s = np.clip(2 * (w * y - z * x), -1.0, 1.0)
    theta = np.arcsin(s)
    psi = np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return float(phi), float(theta), float(psi)


def wind_angles(v_b: np.ndarray) -> tuple[float, float, float]:
    """(V, alpha [rad], beta [rad]) from body velocity."""
    V = float(np.linalg.norm(v_b))
    if V < 1e-6:
        return 0.0, 0.0, 0.0
    alpha = float(np.arctan2(v_b[2], v_b[0]))
    beta = float(np.arcsin(np.clip(v_b[1] / V, -1.0, 1.0)))
    return V, alpha, beta


@dataclass
class RigidBody:
    """Rigid body with constant mass properties."""

    mass: float
    inertia: np.ndarray  # 3x3 [kg m^2]

    def __post_init__(self):
        self._inertia_inv = np.linalg.inv(self.inertia)

    def derivatives(self, x: np.ndarray, F_b: np.ndarray,
                    M_b: np.ndarray) -> np.ndarray:
        """State derivative given body force (incl. aero, excl. gravity)
        and body moment about the CG."""
        v_b = x[3:6]
        q = x[6:10]
        w = x[10:13]

        C_nb = quat_to_dcm(q)
        g_b = C_nb.T @ np.array([0.0, 0.0, GRAV])

        xdot = np.zeros(STATE_DIM)
        xdot[0:3] = C_nb @ v_b
        xdot[3:6] = F_b / self.mass + g_b - np.cross(w, v_b)
        # Quaternion kinematics: qdot = 0.5 * Omega(w) * q
        p, qr, r = w
        omega = np.array([
            [0.0, -p, -qr, -r],
            [p, 0.0, r, -qr],
            [qr, -r, 0.0, p],
            [r, qr, -p, 0.0],
        ])
        xdot[6:10] = 0.5 * omega @ q
        xdot[10:13] = self._inertia_inv @ (M_b - np.cross(w, self.inertia @ w))
        return xdot


def rk4_step(f: Callable[[float, np.ndarray], np.ndarray],
             t: float, x: np.ndarray, dt: float) -> np.ndarray:
    """Single classical Runge-Kutta step with quaternion renormalization."""
    k1 = f(t, x)
    k2 = f(t + dt / 2, x + dt / 2 * k1)
    k3 = f(t + dt / 2, x + dt / 2 * k2)
    k4 = f(t + dt, x + dt * k3)
    xn = x + dt / 6.0 * (k1 + 2 * k2 + 2 * k3 + k4)
    xn[6:10] = quat_normalize(xn[6:10])
    return xn
