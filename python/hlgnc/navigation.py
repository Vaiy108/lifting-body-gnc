"""15-state error-state Kalman filter (ESKF) for strapdown inertial
navigation, aided by GNSS position/velocity and barometric altitude.

Nominal state (16 elements, propagated by strapdown mechanization):
    p_ned (3), v_ned (3), q_nb (4, attitude), b_g (3, gyro bias),
    b_a (3, accel bias)

Error state (15 elements, propagated/estimated by the filter):
    dp (3), dv (3), dtheta (3, small-angle attitude error),
    db_g (3), db_a (3)

This is the standard aerospace/robotics ESKF formulation (see e.g.
Sola, "Quaternion kinematics for the error-state Kalman filter").
Reference frame: flat-earth NED, consistent with hlgnc.dynamics.

Documented approximations:
    * First-order (Euler) discretization of the error-state transition
      and process-noise matrices, valid for dt << 1/wn of the fastest
      modeled error dynamic. Adequate at the ~200 Hz IMU rate used
      here; a Van Loan / matrix-exponential discretization is a
      roadmap item if higher fidelity is needed.
    * No error-state reset Jacobian is applied after attitude
      injection (standard small-angle approximation, error negligible
      for the attitude uncertainties expected here).
    * Flat, non-rotating earth (consistent with the rest of the
      simulation); no Coriolis/transport-rate terms.
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

from .dynamics import (quat_normalize, quat_to_dcm, quat_mult,
                       quat_from_rotvec, skew, GRAV)

N_ERR = 15


@dataclass
class ImuNoiseParams:
    accel_noise_std: float = 0.02       # [m/s^2], per-sample (predict-rate)
    gyro_noise_std: float = np.deg2rad(0.01)   # [rad/s]
    gyro_bias_walk_std: float = 1e-6    # [rad/s / sqrt(s)]
    accel_bias_walk_std: float = 1e-5   # [m/s^2 / sqrt(s)]


@dataclass
class NavState:
    p: np.ndarray = field(default_factory=lambda: np.zeros(3))
    v: np.ndarray = field(default_factory=lambda: np.zeros(3))
    q: np.ndarray = field(default_factory=lambda: np.array([1.0, 0, 0, 0]))
    bg: np.ndarray = field(default_factory=lambda: np.zeros(3))
    ba: np.ndarray = field(default_factory=lambda: np.zeros(3))

    def copy(self) -> "NavState":
        return NavState(self.p.copy(), self.v.copy(), self.q.copy(),
                        self.bg.copy(), self.ba.copy())


@dataclass
class ESKF:
    """Error-state Kalman filter for strapdown INS/GNSS/baro fusion."""

    noise: ImuNoiseParams = field(default_factory=ImuNoiseParams)
    state: NavState = field(default_factory=NavState)
    P: np.ndarray = field(default_factory=lambda: np.eye(N_ERR) * 1.0)

    def initialize(self, p0: np.ndarray, v0: np.ndarray, q0: np.ndarray,
                  pos_std: float = 5.0, vel_std: float = 1.0,
                  att_std: float = np.deg2rad(2.0),
                  bg_std: float = np.deg2rad(0.5),
                  ba_std: float = 0.1) -> None:
        self.state = NavState(p0.copy(), v0.copy(), q0.copy())
        self.P = np.diag(np.concatenate([
            np.full(3, pos_std**2), np.full(3, vel_std**2),
            np.full(3, att_std**2), np.full(3, bg_std**2),
            np.full(3, ba_std**2),
        ]))

    # -- prediction -------------------------------------------------
    def predict(self, f_meas: np.ndarray, w_meas: np.ndarray,
               dt: float) -> None:
        """Strapdown mechanization + error-covariance propagation.

        f_meas, w_meas: raw IMU specific-force [m/s^2] and body-rate
        [rad/s] measurements (bias + noise not yet removed).
        """
        s = self.state
        f_b = f_meas - s.ba
        w_b = w_meas - s.bg
        C_nb = quat_to_dcm(s.q)

        a_ned = C_nb @ f_b + np.array([0.0, 0.0, GRAV])
        s.p = s.p + s.v * dt + 0.5 * a_ned * dt * dt
        s.v = s.v + a_ned * dt
        dq = quat_from_rotvec(w_b * dt)
        s.q = quat_normalize(quat_mult(s.q, dq))
        # bg, ba: random walk, no deterministic propagation.

        F = np.eye(N_ERR)
        F[0:3, 3:6] = np.eye(3) * dt
        F[3:6, 6:9] = -C_nb @ skew(f_b) * dt
        F[3:6, 12:15] = -C_nb * dt
        F[6:9, 6:9] = np.eye(3) - skew(w_b) * dt
        F[6:9, 9:12] = -np.eye(3) * dt

        G = np.zeros((N_ERR, 12))
        G[3:6, 0:3] = C_nb
        G[6:9, 3:6] = -np.eye(3)
        G[9:12, 6:9] = np.eye(3)
        G[12:15, 9:12] = np.eye(3)
        Qc = np.diag(np.concatenate([
            np.full(3, self.noise.accel_noise_std**2),
            np.full(3, self.noise.gyro_noise_std**2),
            np.full(3, self.noise.gyro_bias_walk_std**2),
            np.full(3, self.noise.accel_bias_walk_std**2),
        ]))
        Qd = G @ Qc @ G.T * dt

        self.P = F @ self.P @ F.T + Qd

    # -- updates ------------------------------------------------------
    def _inject_and_reset(self, dx: np.ndarray, P_new: np.ndarray) -> None:
        s = self.state
        s.p = s.p + dx[0:3]
        s.v = s.v + dx[3:6]
        s.q = quat_normalize(quat_mult(s.q, quat_from_rotvec(dx[6:9])))
        s.bg = s.bg + dx[9:12]
        s.ba = s.ba + dx[12:15]
        self.P = P_new

    def _joseph_update(self, H: np.ndarray, R: np.ndarray,
                       innov: np.ndarray) -> None:
        S = H @ self.P @ H.T + R
        K = self.P @ H.T @ np.linalg.solve(S, np.eye(S.shape[0]))
        dx = K @ innov
        IKH = np.eye(N_ERR) - K @ H
        P_new = IKH @ self.P @ IKH.T + K @ R @ K.T
        self._inject_and_reset(dx, P_new)

    def update_gnss(self, pos_meas: np.ndarray, vel_meas: np.ndarray,
                    pos_std: np.ndarray, vel_std: float) -> None:
        H = np.zeros((6, N_ERR))
        H[0:3, 0:3] = np.eye(3)
        H[3:6, 3:6] = np.eye(3)
        R = np.diag(np.concatenate([pos_std**2, np.full(3, vel_std**2)]))
        innov = np.concatenate([pos_meas - self.state.p,
                                vel_meas - self.state.v])
        self._joseph_update(H, R, innov)

    def update_baro(self, alt_meas: float, alt_std: float) -> None:
        H = np.zeros((1, N_ERR))
        H[0, 2] = -1.0    # altitude = -p_z
        R = np.array([[alt_std**2]])
        innov = np.array([alt_meas - (-self.state.p[2])])
        self._joseph_update(H, R, innov)

    # -- convenience ----------------------------------------------------
    def pos_error_std(self) -> np.ndarray:
        return np.sqrt(np.diag(self.P)[0:3])

    def att_error_std(self) -> np.ndarray:
        return np.sqrt(np.diag(self.P)[6:9])
