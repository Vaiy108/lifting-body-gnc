"""HL-20 vehicle simulation: plant assembly, trim, and time stepping.

Combines the TM-4302 aerodynamic model, US76 atmosphere, actuator bank
and rigid-body dynamics into a single vehicle plant with a longitudinal
steady-glide trim solver.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Callable, Optional

import numpy as np
from scipy.optimize import fsolve

from .aero import HL20Aero, HL20Geometry, ControlDeflections
from .atmosphere import atmosphere
from .dynamics import (RigidBody, rk4_step, quat_from_euler, quat_to_dcm,
                       wind_angles, STATE_DIM, GRAV)
from .actuators import ActuatorBank, allocate


@dataclass
class HL20Vehicle:
    """HL-20 plant: dynamics + aero + atmosphere + actuators."""

    geom: HL20Geometry = field(default_factory=HL20Geometry)
    aero: HL20Aero = None
    body: RigidBody = None
    actuators: ActuatorBank = field(default_factory=ActuatorBank)
    wind_ned: np.ndarray = field(default_factory=lambda: np.zeros(3))

    def __post_init__(self):
        if self.aero is None:
            self.aero = HL20Aero(self.geom)
        if self.body is None:
            self.body = RigidBody(self.geom.mass, self.geom.inertia)

    # -- forces ---------------------------------------------------------
    def air_relative(self, x: np.ndarray) -> tuple[float, float, float, float]:
        """(V, alpha_deg, beta_deg, qbar) including wind."""
        C_nb = quat_to_dcm(x[6:10])
        v_air_b = x[3:6] - C_nb.T @ self.wind_ned
        V, alpha, beta = wind_angles(v_air_b)
        alt = -x[2]
        _, _, rho, _ = atmosphere(max(alt, 0.0))
        qbar = 0.5 * rho * V * V
        return V, np.rad2deg(alpha), np.rad2deg(beta), qbar

    def aero_forces(self, x: np.ndarray, defl: ControlDeflections
                    ) -> tuple[np.ndarray, np.ndarray]:
        V, alpha_deg, beta_deg, qbar = self.air_relative(x)
        p, q, r = x[10:13]
        return self.aero.forces_moments(alpha_deg, beta_deg, defl, qbar,
                                        p, q, r, V)

    def derivatives(self, t: float, x: np.ndarray,
                    defl: ControlDeflections) -> np.ndarray:
        F_b, M_b = self.aero_forces(x, defl)
        return self.body.derivatives(x, F_b, M_b)

    def specific_force(self, x: np.ndarray, defl: ControlDeflections
                       ) -> np.ndarray:
        """True specific force in body axes (what an ideal IMU measures)."""
        F_b, _ = self.aero_forces(x, defl)
        return F_b / self.geom.mass

    # -- trim -----------------------------------------------------------
    def trim_glide(self, V: float, alt: float,
                   alpha_guess_deg: float = 12.0,
                   de_guess_deg: float = -5.0
                   ) -> tuple[float, float, float]:
        """Longitudinal steady wings-level glide trim at (V, alt).

        Solves for (alpha [deg], elevator de [deg], flight-path angle
        gamma [rad]) such that body-axis force residuals and pitching
        moment vanish for an unpowered steady glide.
        Returns (alpha_deg, de_deg, gamma_rad).
        """
        _, _, rho, _ = atmosphere(alt)
        qbar = 0.5 * rho * V * V
        m, S = self.geom.mass, self.geom.S_ref

        def residuals(u):
            alpha_deg, de_deg, gamma = u
            theta = gamma + np.deg2rad(alpha_deg)
            c = self.aero.coefficients(alpha_deg, 0.0,
                                       ControlDeflections(de=de_deg),
                                       V=V)
            fx = qbar * S * c["CX"] - m * GRAV * np.sin(theta)
            fz = qbar * S * c["CZ"] + m * GRAV * np.cos(theta)
            mm = c["Cm"]
            return [fx / (m * GRAV), fz / (m * GRAV), mm]

        sol, info, ier, msg = fsolve(residuals, [alpha_guess_deg,
                                                 de_guess_deg, -0.1],
                                     full_output=True)
        if ier != 1:
            raise RuntimeError(f"Trim did not converge: {msg}")
        alpha_deg, de_deg = float(sol[0]), float(sol[1])
        if not (-10.0 <= alpha_deg <= 30.0):
            raise RuntimeError(
                f"Trim alpha {alpha_deg:.1f} deg outside TM-4302 validity "
                "envelope [-10, 30] deg")
        if abs(de_deg) > 30.0:
            raise RuntimeError(
                f"Trim elevator {de_deg:.1f} deg exceeds wing-flap "
                "authority (+/-30 deg)")
        return alpha_deg, de_deg, float(sol[2])

    def initial_state(self, V: float, alt: float, alpha_deg: float,
                      gamma: float, psi: float = 0.0,
                      north: float = 0.0, east: float = 0.0) -> np.ndarray:
        """Build a trimmed-glide initial state vector."""
        x = np.zeros(STATE_DIM)
        alpha = np.deg2rad(alpha_deg)
        x[0:3] = [north, east, -alt]
        x[3:6] = V * np.array([np.cos(alpha), 0.0, np.sin(alpha)])
        theta = gamma + alpha
        x[6:10] = quat_from_euler(0.0, theta, psi)
        return x


@dataclass
class SimResult:
    t: np.ndarray
    x: np.ndarray          # (n, STATE_DIM)
    defl: np.ndarray       # (n, 6) aero deflections [deg]

    def altitude(self) -> np.ndarray:
        return -self.x[:, 2]


def simulate(vehicle: HL20Vehicle, x0: np.ndarray, t_end: float,
             dt: float = 0.005,
             control_law: Optional[Callable] = None,
             stop_at_ground: bool = True) -> SimResult:
    """Run the simulation.

    control_law(t, x, vehicle) -> aero-effect command
    [da, de, dr, dfp, dfn, ddf] in degrees; None holds zero deflection.
    """
    n = int(np.ceil(t_end / dt)) + 1
    ts = np.zeros(n)
    xs = np.zeros((n, STATE_DIM))
    ds = np.zeros((n, 6))
    x = x0.copy()
    for k in range(n):
        t = k * dt
        aero_cmd = (control_law(t, x, vehicle) if control_law is not None
                    else np.zeros(6))
        surf_cmd = allocate(np.asarray(aero_cmd, dtype=float))
        vehicle.actuators.step(surf_cmd, dt)
        d6 = vehicle.actuators.aero_deflections()
        defl = ControlDeflections(*d6)
        ts[k], xs[k], ds[k] = t, x, d6
        if stop_at_ground and -x[2] <= 0.0 and k > 0:
            return SimResult(ts[: k + 1], xs[: k + 1], ds[: k + 1])
        x = rk4_step(lambda tt, xx: vehicle.derivatives(tt, xx, defl),
                     t, x, dt)
    return SimResult(ts, xs, ds)
