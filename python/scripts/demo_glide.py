"""Open-loop trimmed-glide demonstration.

Trims the HL-20 at 160 m/s / 8 km, flies the trim open loop, and plots
the trajectory and key states. Output: docs/demo_glide.png
"""

import sys
import pathlib

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))

from hlgnc.sim import HL20Vehicle, simulate
from hlgnc.actuators import allocate
from hlgnc.dynamics import wind_angles, quat_to_euler


def main():
    veh = HL20Vehicle()
    V0, alt0 = 160.0, 8000.0
    alpha_deg, de_deg, gamma = veh.trim_glide(V=V0, alt=alt0)
    print(f"Trim @ V={V0} m/s, h={alt0} m: alpha={alpha_deg:.2f} deg, "
          f"de={de_deg:.2f} deg, gamma={np.rad2deg(gamma):.2f} deg "
          f"(L/D ~ {1/np.tan(-gamma):.2f})")

    x0 = veh.initial_state(V0, alt0, alpha_deg, gamma)
    veh.actuators.pos = allocate(np.array([0, de_deg, 0, 0, 0, 0], float))
    hold = lambda t, x, v: np.array([0.0, de_deg, 0.0, 0.0, 0.0, 0.0])
    res = simulate(veh, x0, t_end=60.0, dt=0.005, control_law=hold)

    V = np.array([wind_angles(v)[0] for v in res.x[:, 3:6]])
    alpha = np.rad2deg([wind_angles(v)[1] for v in res.x[:, 3:6]])
    theta = np.rad2deg([quat_to_euler(q)[1] for q in res.x[:, 6:10]])

    fig, ax = plt.subplots(2, 2, figsize=(11, 7))
    ax[0, 0].plot(res.x[:, 0] / 1000.0, res.altitude() / 1000.0)
    ax[0, 0].set_xlabel("Range North [km]")
    ax[0, 0].set_ylabel("Altitude [km]")
    ax[0, 0].set_title("Glide trajectory")
    ax[0, 1].plot(res.t, V)
    ax[0, 1].set_xlabel("t [s]"), ax[0, 1].set_ylabel("V [m/s]")
    ax[0, 1].set_title("Airspeed")
    ax[1, 0].plot(res.t, alpha, label=r"$\alpha$")
    ax[1, 0].plot(res.t, theta, label=r"$\theta$")
    ax[1, 0].set_xlabel("t [s]"), ax[1, 0].set_ylabel("[deg]")
    ax[1, 0].legend(), ax[1, 0].set_title("Angle of attack / pitch")
    ax[1, 1].plot(res.t, res.defl[:, 1])
    ax[1, 1].set_xlabel("t [s]"), ax[1, 1].set_ylabel("de [deg]")
    ax[1, 1].set_title("Elevator deflection")
    for a in ax.flat:
        a.grid(alpha=0.3)
    fig.suptitle("HL-20 open-loop trimmed glide (NASA TM-4302 aero model)")
    fig.tight_layout()
    out = pathlib.Path(__file__).resolve().parents[2] / "docs" / "demo_glide.png"
    fig.savefig(out, dpi=130)
    print(f"Saved {out}")


if __name__ == "__main__":
    main()
