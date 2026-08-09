"""Propulsion model: turbojet-class demonstrator ferry/ascent mode.

The HL-20 itself is unpowered (deorbit-and-glide only), so this module
targets the powered subsonic/transonic demonstrator configuration that
precedes the unpowered glide phase in a staged flight-test program
(cf. turbojet-powered subscale spaceplane demonstrators) -- the same
propulsion class used to ferry to altitude/speed before an unpowered
glide test point, or to fly powered UAV mission legs.

Model: first-order thrust-response lag (spool dynamics) with density
and Mach-based thrust lapse, plus a fuel-mass integrator so vehicle
mass and inertia are consistent with fuel burned. Deliberately simple
(no compressor maps, no turbine thermodynamics) -- the intent is a
control-oriented model adequate for 6-DOF integration and throttle-loop
design, not a propulsion-system design tool.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .atmosphere import atmosphere


@dataclass
class PropulsionConfig:
    thrust_max_sl: float = 18_000.0     # max sea-level static thrust [N]
    tau_spool: float = 1.2              # spool-up/down time constant [s]
    tsfc: float = 1.8e-5                # thrust-specific fuel consumption [kg/(N s)]
    rho_sl: float = 1.225                # sea-level density, for lapse ref [kg/m^3]
    mach_lapse_a: float = 0.30           # thrust lapse coefficient vs Mach
    mount_offset_x: float = 0.0          # thrust-line offset from CG, body x [m]
    mount_offset_z: float = 0.0          # thrust-line offset from CG, body z [m]


@dataclass
class PropulsionState:
    throttle_cmd: float = 0.0            # commanded throttle [0, 1]
    thrust: float = 0.0                  # current thrust [N]
    fuel_mass: float = 500.0             # remaining fuel [kg]


@dataclass
class Propulsion:
    """First-order-lag turbojet-class thrust model."""

    cfg: PropulsionConfig = None
    state: PropulsionState = None

    def __post_init__(self):
        if self.cfg is None:
            self.cfg = PropulsionConfig()
        if self.state is None:
            self.state = PropulsionState()

    def available_thrust(self, alt: float, mach: float) -> float:
        """Static thrust available at the given altitude and Mach
        number, before spool-lag dynamics.

        Density lapse: thrust scales with ambient density ratio
        (typical dry-turbojet approximation). Mach lapse: linear
        de-rating with an installation-representative coefficient;
        this is a control-oriented approximation, not an engine-cycle
        model, and is documented as such.
        """
        _, _, rho, _ = atmosphere(max(alt, 0.0))
        density_ratio = rho / self.cfg.rho_sl
        mach_factor = max(0.0, 1.0 - self.cfg.mach_lapse_a * mach)
        return self.cfg.thrust_max_sl * density_ratio * mach_factor

    def step(self, throttle_cmd: float, alt: float, mach: float,
             dt: float) -> tuple[float, float]:
        """Advance spool and fuel state by dt.

        Returns (thrust [N], fuel_mass [kg]).
        """
        throttle_cmd = float(np.clip(throttle_cmd, 0.0, 1.0))
        self.state.throttle_cmd = throttle_cmd
        thrust_target = throttle_cmd * self.available_thrust(alt, mach)

        # First-order spool lag (semi-implicit Euler).
        alpha = dt / (self.cfg.tau_spool + dt)
        self.state.thrust += alpha * (thrust_target - self.state.thrust)
        self.state.thrust = max(self.state.thrust, 0.0)

        # Fuel burn, non-negative.
        burn = self.cfg.tsfc * self.state.thrust * dt
        self.state.fuel_mass = max(0.0, self.state.fuel_mass - burn)
        if self.state.fuel_mass <= 0.0:
            self.state.thrust = 0.0

        return self.state.thrust, self.state.fuel_mass

    def force_moment_body(self) -> tuple[np.ndarray, np.ndarray]:
        """Body-axis thrust force [N] and moment about CG [N m],
        assuming thrust acts along body +x from the mount offset."""
        F_b = np.array([self.state.thrust, 0.0, 0.0])
        r = np.array([self.cfg.mount_offset_x, 0.0, self.cfg.mount_offset_z])
        M_b = np.cross(r, F_b)
        return F_b, M_b
