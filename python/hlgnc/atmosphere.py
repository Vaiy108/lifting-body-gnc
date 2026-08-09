"""U.S. Standard Atmosphere 1976 (COESA), 0 to 84.852 km geopotential.

Provides temperature, pressure, density and speed of sound as functions
of geometric altitude. Matches the COESA model used by standard
aerospace simulation toolchains, enabling direct cross-validation
against the reference Simulink implementation.
"""

from __future__ import annotations

import numpy as np

G0 = 9.80665          # standard gravity [m/s^2]
R_AIR = 287.0528      # specific gas constant, air [J/(kg K)]
GAMMA = 1.4           # ratio of specific heats
R_EARTH = 6356766.0   # effective earth radius for geopotential [m]

# Layer base geopotential altitude [m], base temperature [K],
# lapse rate [K/m], base pressure [Pa].
_H_BASE = np.array([0.0, 11000.0, 20000.0, 32000.0, 47000.0, 51000.0, 71000.0])
_T_BASE = np.array([288.15, 216.65, 216.65, 228.65, 270.65, 270.65, 214.65])
_LAPSE = np.array([-6.5e-3, 0.0, 1.0e-3, 2.8e-3, 0.0, -2.8e-3, -2.0e-3])
_P_BASE = np.array([101325.0, 22632.06, 5474.889, 868.0187,
                    110.9063, 66.93887, 3.956420])
H_TOP = 84852.0


def geopotential(z: float) -> float:
    """Geopotential altitude [m] from geometric altitude [m]."""
    return R_EARTH * z / (R_EARTH + z)


def atmosphere(z: float) -> tuple[float, float, float, float]:
    """Return (T [K], p [Pa], rho [kg/m^3], a [m/s]) at geometric altitude z.

    Altitude is clamped to the model's validity range [0, ~86 km geometric].
    """
    h = float(np.clip(geopotential(z), 0.0, H_TOP))
    i = int(np.clip(np.searchsorted(_H_BASE, h, side="right") - 1,
                    0, len(_H_BASE) - 1))
    dh = h - _H_BASE[i]
    L, Tb, pb = _LAPSE[i], _T_BASE[i], _P_BASE[i]
    if abs(L) > 1e-12:
        T = Tb + L * dh
        p = pb * (T / Tb) ** (-G0 / (L * R_AIR))
    else:
        T = Tb
        p = pb * np.exp(-G0 * dh / (R_AIR * Tb))
    rho = p / (R_AIR * T)
    a = np.sqrt(GAMMA * R_AIR * T)
    return float(T), float(p), float(rho), float(a)


def density(z: float) -> float:
    return atmosphere(z)[2]


def speed_of_sound(z: float) -> float:
    return atmosphere(z)[3]
