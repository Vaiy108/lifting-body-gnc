"""Propulsion model demonstration: throttle step response and thrust
lapse with altitude/Mach. Output: docs/demo_propulsion.png
"""

import sys
import pathlib

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))

from hlgnc.propulsion import Propulsion, PropulsionConfig


def main():
    # --- Throttle step response at sea level, zero Mach ---
    p = Propulsion()
    dt = 0.01
    t_end = 8.0
    n = int(t_end / dt)
    t = np.arange(n) * dt
    thrust = np.zeros(n)
    fuel = np.zeros(n)
    for k in range(n):
        cmd = 1.0 if t[k] > 0.5 else 0.0
        thrust[k], fuel[k] = p.step(cmd, alt=0.0, mach=0.0, dt=dt)

    # --- Thrust lapse maps ---
    alts = np.linspace(0, 12000, 100)
    p2 = Propulsion()
    lapse_alt = [p2.available_thrust(a, mach=0.3) for a in alts]
    machs = np.linspace(0, 1.0, 100)
    lapse_mach = [p2.available_thrust(2000.0, m) for m in machs]

    fig, ax = plt.subplots(1, 3, figsize=(14, 4))
    ax[0].plot(t, thrust / 1000.0)
    ax[0].set_xlabel("t [s]"), ax[0].set_ylabel("Thrust [kN]")
    ax[0].set_title(f"Spool step response (tau={p.cfg.tau_spool}s)")
    ax[1].plot(alts / 1000.0, np.array(lapse_alt) / 1000.0)
    ax[1].set_xlabel("Altitude [km]"), ax[1].set_ylabel("Available thrust [kN]")
    ax[1].set_title("Density lapse (Mach 0.3)")
    ax[2].plot(machs, np.array(lapse_mach) / 1000.0)
    ax[2].set_xlabel("Mach"), ax[2].set_ylabel("Available thrust [kN]")
    ax[2].set_title("Mach lapse (2 km alt)")
    for a in ax:
        a.grid(alpha=0.3)
    fig.suptitle("Propulsion model: spool dynamics and thrust lapse")
    fig.tight_layout()
    out = pathlib.Path(__file__).resolve().parents[2] / "docs" / "demo_propulsion.png"
    fig.savefig(out, dpi=130)
    print(f"Saved {out}")
    print(f"Fuel burned in 8 s at max throttle: {500.0 - fuel[-1]:.2f} kg")


if __name__ == "__main__":
    main()
