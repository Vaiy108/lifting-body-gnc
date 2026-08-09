"""Pitch flight control: rate-damped attitude hold on the elevator.

This is the Phase 2 inner loop: a PD attitude controller with body
pitch-rate feedback, commanding the symmetric wing flap (elevator)
about a trim deflection. It is deliberately single-axis and
single-gain-set (no scheduling over Mach/qbar yet) -- gain scheduling
and the outer-loop energy-managed glide-path/TAEM guidance are Phase 5
items that command this loop's theta_cmd input.

Architecture matches the eventual embedded target: pure function of
(commanded pitch, estimated pitch, estimated pitch rate) -> elevator
command, with explicit output clipping. This is the function that gets
ported to C in Phase 3.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

DE_LIMIT_DEG = 30.0


@dataclass
class PitchHoldGains:
    """Default gains stabilize the vehicle at the 160 m/s / 8 km trim
    point, where the bare airframe is statically unstable in pitch
    (dCm/dalpha > 0 at trim -- see docs/pitch_stability_note.md).
    Tuned by gain sweep (python/scripts/tune_pitch_gains.py) for
    < 2 deg overshoot and < 0.05 deg steady-state error on a 2 deg
    attitude step, without saturating the elevator (max ~17 deg of
    30 deg authority used). Not a final flight-quality tuning -- gain
    scheduling across the flight envelope is a Phase 5 item.
    """

    kp: float = 250.0    # [deg elevator / rad attitude error]
    kd: float = 70.0      # [deg elevator / (rad/s) pitch rate]


@dataclass
class PitchAttitudeHold:
    """PD pitch-attitude-hold controller about a trim deflection."""

    gains: PitchHoldGains = None
    de_trim_deg: float = 0.0

    def __post_init__(self):
        if self.gains is None:
            self.gains = PitchHoldGains()

    def command(self, theta_cmd_rad: float, theta_est_rad: float,
               q_est_rad_s: float) -> np.ndarray:
        """Return the 6-element aero-effect command
        [da, de, dr, dfp, dfn, ddf] with only elevator active."""
        err = theta_cmd_rad - theta_est_rad
        # Sign convention: positive elevator deflection (de) produces a
        # nose-down pitching moment (dCm/dde < 0, verified in Phase 1's
        # aero tests). To increase theta, de must decrease; to damp a
        # positive pitch rate, de must increase.
        de = self.de_trim_deg - self.gains.kp * err + self.gains.kd * q_est_rad_s
        de = float(np.clip(de, -DE_LIMIT_DEG, DE_LIMIT_DEG))
        return np.array([0.0, de, 0.0, 0.0, 0.0, 0.0])
