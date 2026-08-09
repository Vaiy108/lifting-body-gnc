"""HL-20 control-surface actuator models and mixing.

Seven physical surfaces (reference-simulation configuration):
    [dwfl, dwfr, dbfll, dbflr, dbful, dbfur, dr]
    (left/right wing flaps; lower-left/lower-right/upper-left/upper-right
     body flaps; all-movable rudder)

Six aero-effect deflections used by the aerodynamic model:
    [da, de, dr, dfp, dfn, ddf]

Each surface is a second-order servo (wn = 44 rad/s, zeta = 0.707)
with position limits per the reference simulation.
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

WN_ACT = 44.0
ZETA_ACT = 1.0 / np.sqrt(2.0)

DWF_MAX = 30.0
DBF_MAX = 60.0
DR_MAX = 60.0

MAX_LIM = np.array([DWF_MAX, DWF_MAX, DBF_MAX, DBF_MAX, 0.0, 0.0, DR_MAX])
MIN_LIM = np.array([-DWF_MAX, -DWF_MAX, 0.0, 0.0, -DBF_MAX, -DBF_MAX, -DR_MAX])

# Surfaces -> aero effects: [da de dr df+ df- ddf]^T = ACT2AERO @ surfaces
ACT2AERO = 0.5 * np.array([
    [1, -1, 0, 0, 0, 0, 0],
    [1,  1, 0, 0, 0, 0, 0],
    [0,  0, 0, 0, 0, 0, 2],
    [0,  0, 1, 1, 0, 0, 0],
    [0,  0, 0, 0, 1, 1, 0],
    [0,  0, 1, -1, 1, -1, 0],
])
AERO2ACT = np.linalg.pinv(ACT2AERO)

N_SURF = 7


@dataclass
class ActuatorBank:
    """Bank of second-order servos with position limiting."""

    wn: float = WN_ACT
    zeta: float = ZETA_ACT
    pos: np.ndarray = field(default_factory=lambda: np.zeros(N_SURF))
    vel: np.ndarray = field(default_factory=lambda: np.zeros(N_SURF))

    def step(self, cmd_deg: np.ndarray, dt: float) -> np.ndarray:
        """Advance the servo states by dt toward commanded deflections."""
        cmd = np.clip(cmd_deg, MIN_LIM, MAX_LIM)
        # Semi-implicit Euler at actuator rate; adequate for wn*dt << 1.
        acc = self.wn**2 * (cmd - self.pos) - 2.0 * self.zeta * self.wn * self.vel
        self.vel += acc * dt
        self.pos += self.vel * dt
        low = self.pos < MIN_LIM
        high = self.pos > MAX_LIM
        self.pos = np.clip(self.pos, MIN_LIM, MAX_LIM)
        self.vel[low | high] = 0.0
        return self.pos.copy()

    def aero_deflections(self) -> np.ndarray:
        """Current [da, de, dr, dfp, dfn, ddf] in degrees."""
        return ACT2AERO @ self.pos


def allocate(aero_cmd_deg: np.ndarray) -> np.ndarray:
    """Map aero-effect commands [da de dr dfp dfn ddf] to surface
    commands via the pseudo-inverse allocator, then clip to limits."""
    return np.clip(AERO2ACT @ aero_cmd_deg, MIN_LIM, MAX_LIM)
