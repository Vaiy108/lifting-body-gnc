"""Sensor models: IMU, GNSS, barometric altimeter, air data.

Error models are deliberately simple and parameterized (constant bias +
white noise, optional random walk on gyro bias) -- sufficient for EKF
development and Monte Carlo dispersion studies. All randomness flows
through a caller-supplied numpy Generator for reproducibility.
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

from .dynamics import quat_to_dcm, wind_angles, GRAV


@dataclass
class ImuConfig:
    accel_bias: np.ndarray = field(default_factory=lambda: np.array([0.05, -0.03, 0.08]))   # [m/s^2]
    accel_noise_std: float = 0.02          # [m/s^2 / sample]
    gyro_bias: np.ndarray = field(default_factory=lambda: np.deg2rad(np.array([0.1, -0.15, 0.05]) / 3600 * 100))  # [rad/s]
    gyro_noise_std: float = np.deg2rad(0.01)   # [rad/s / sample]
    gyro_bias_walk_std: float = 0.0        # [rad/s / sqrt(sample)]


@dataclass
class Imu:
    cfg: ImuConfig = field(default_factory=ImuConfig)

    def sample(self, x: np.ndarray, accel_body_specific: np.ndarray,
               rng: np.random.Generator) -> tuple[np.ndarray, np.ndarray]:
        """Return (specific force meas [m/s^2], body rate meas [rad/s]).

        accel_body_specific: true specific force = (F_ext/m) in body axes
        (i.e., acceleration minus gravity, which is what an IMU measures).
        """
        if self.cfg.gyro_bias_walk_std > 0.0:
            self.cfg.gyro_bias = self.cfg.gyro_bias + rng.normal(
                0.0, self.cfg.gyro_bias_walk_std, 3)
        f_meas = (accel_body_specific + self.cfg.accel_bias
                  + rng.normal(0.0, self.cfg.accel_noise_std, 3))
        w_meas = (x[10:13] + self.cfg.gyro_bias
                  + rng.normal(0.0, self.cfg.gyro_noise_std, 3))
        return f_meas, w_meas


@dataclass
class GnssConfig:
    pos_noise_std: np.ndarray = field(default_factory=lambda: np.array([1.5, 1.5, 3.0]))  # NED [m]
    vel_noise_std: float = 0.1            # [m/s]
    rate_hz: float = 5.0


@dataclass
class Gnss:
    cfg: GnssConfig = field(default_factory=GnssConfig)

    def sample(self, x: np.ndarray, rng: np.random.Generator
               ) -> tuple[np.ndarray, np.ndarray]:
        C_nb = quat_to_dcm(x[6:10])
        p = x[0:3] + rng.normal(0.0, 1.0, 3) * self.cfg.pos_noise_std
        v = C_nb @ x[3:6] + rng.normal(0.0, self.cfg.vel_noise_std, 3)
        return p, v


@dataclass
class BaroConfig:
    alt_noise_std: float = 1.0            # [m]
    alt_bias: float = 2.0                 # [m]


@dataclass
class Baro:
    cfg: BaroConfig = field(default_factory=BaroConfig)

    def sample(self, x: np.ndarray, rng: np.random.Generator) -> float:
        alt = -x[2]
        return float(alt + self.cfg.alt_bias
                     + rng.normal(0.0, self.cfg.alt_noise_std))


@dataclass
class AirDataConfig:
    v_noise_std: float = 0.5              # [m/s]
    alpha_noise_std: float = np.deg2rad(0.2)
    beta_noise_std: float = np.deg2rad(0.2)


@dataclass
class AirData:
    cfg: AirDataConfig = field(default_factory=AirDataConfig)

    def sample(self, x: np.ndarray, rng: np.random.Generator
               ) -> tuple[float, float, float]:
        V, alpha, beta = wind_angles(x[3:6])
        return (V + rng.normal(0.0, self.cfg.v_noise_std),
                alpha + rng.normal(0.0, self.cfg.alpha_noise_std),
                beta + rng.normal(0.0, self.cfg.beta_noise_std))
