"""Closed-loop demo: ESKF navigation estimates drive the pitch-
attitude-hold controller (not the true simulation state) -- this is
the integration test that matters, since a controller that only works
on perfect truth state is not a flight-software architecture.

IMU runs at the simulation rate (200 Hz); GNSS updates at 5 Hz; baro
at 20 Hz. Output: docs/demo_closed_loop.png
"""

import sys
import pathlib

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))

from hlgnc.sim import HL20Vehicle
from hlgnc.actuators import allocate
from hlgnc.control import PitchAttitudeHold, PitchHoldGains
from hlgnc.navigation import ESKF
from hlgnc.sensors import Imu, Gnss, Baro
from hlgnc.dynamics import quat_to_euler, rk4_step, quat_to_dcm
from hlgnc.aero import ControlDeflections


def main():
    rng = np.random.default_rng(7)
    veh = HL20Vehicle()
    V0, alt0 = 160.0, 8000.0
    alpha_deg, de_deg, gamma = veh.trim_glide(V=V0, alt=alt0)
    theta_trim = gamma + np.deg2rad(alpha_deg)
    x = veh.initial_state(V0, alt0, alpha_deg, gamma)
    veh.actuators.pos = allocate(np.array([0, de_deg, 0, 0, 0, 0], float))

    eskf = ESKF()
    C_nb0 = quat_to_dcm(x[6:10])
    eskf.initialize(x[0:3].copy(), C_nb0 @ x[3:6], x[6:10].copy())

    imu, gnss, baro = Imu(), Gnss(), Baro()
    ctrl = PitchAttitudeHold(PitchHoldGains(), de_trim_deg=de_deg)
    theta_cmd = theta_trim + np.deg2rad(2.0)

    dt = 0.005
    gnss_period = round(1 / gnss.cfg.rate_hz / dt)
    baro_period = round(1 / 20.0 / dt)
    n = int(30.0 / dt)

    t_hist = np.zeros(n)
    theta_true = np.zeros(n)
    theta_est = np.zeros(n)
    pos_err = np.zeros((n, 3))
    de_hist = np.zeros(n)

    for k in range(n):
        f_true = veh.specific_force(x, ControlDeflections(*veh.actuators.aero_deflections()))
        w_true = x[10:13]
        f_meas, w_meas = imu.sample(x, f_true, rng)
        eskf.predict(f_meas, w_meas, dt)

        if k % gnss_period == 0:
            p_meas, v_meas = gnss.sample(x, rng)
            eskf.update_gnss(p_meas, v_meas, gnss.cfg.pos_noise_std,
                             gnss.cfg.vel_noise_std)
        if k % baro_period == 0:
            alt_meas = baro.sample(x, rng)
            eskf.update_baro(alt_meas, baro.cfg.alt_noise_std)

        theta_hat = quat_to_euler(eskf.state.q)[1]
        C_nb_hat = quat_to_dcm(eskf.state.q)
        q_hat = (C_nb_hat.T @ (w_meas - eskf.state.bg))[1]
        aero_cmd = ctrl.command(theta_cmd, theta_hat, q_hat)

        surf_cmd = allocate(aero_cmd)
        veh.actuators.step(surf_cmd, dt)
        d6 = veh.actuators.aero_deflections()
        defl = ControlDeflections(*d6)

        t_hist[k] = k * dt
        theta_true[k] = np.rad2deg(quat_to_euler(x[6:10])[1])
        theta_est[k] = np.rad2deg(theta_hat)
        pos_err[k] = eskf.state.p - x[0:3]
        de_hist[k] = d6[1]

        x = rk4_step(lambda tt, xx: veh.derivatives(tt, xx, defl), 0.0, x, dt)

    print(f"Final theta true={theta_true[-1]:.3f} deg, "
          f"est={theta_est[-1]:.3f} deg, target={np.rad2deg(theta_cmd):.3f} deg")
    print(f"Final nav position error: {pos_err[-1]} m "
          f"(|.|={np.linalg.norm(pos_err[-1]):.2f} m)")

    fig, ax = plt.subplots(2, 2, figsize=(11, 7))
    ax[0, 0].plot(t_hist, theta_true, label="true")
    ax[0, 0].plot(t_hist, theta_est, "--", label="ESKF estimate")
    ax[0, 0].axhline(np.rad2deg(theta_cmd), color="gray", ls=":", label="command")
    ax[0, 0].set_xlabel("t [s]"), ax[0, 0].set_ylabel("theta [deg]")
    ax[0, 0].set_title("Pitch attitude: true vs. estimated")
    ax[0, 0].legend()

    ax[0, 1].plot(t_hist, de_hist)
    ax[0, 1].set_xlabel("t [s]"), ax[0, 1].set_ylabel("de [deg]")
    ax[0, 1].set_title("Elevator command (closed loop on ESKF estimate)")

    ax[1, 0].plot(t_hist, pos_err[:, 0], label="N")
    ax[1, 0].plot(t_hist, pos_err[:, 1], label="E")
    ax[1, 0].plot(t_hist, pos_err[:, 2], label="D")
    ax[1, 0].set_xlabel("t [s]"), ax[1, 0].set_ylabel("nav error [m]")
    ax[1, 0].set_title("ESKF position error vs. truth")
    ax[1, 0].legend()

    ax[1, 1].plot(t_hist, np.linalg.norm(pos_err, axis=1))
    ax[1, 1].set_xlabel("t [s]"), ax[1, 1].set_ylabel("|error| [m]")
    ax[1, 1].set_title("ESKF position error norm")

    for a in ax.flat:
        a.grid(alpha=0.3)
    fig.suptitle("Closed loop: ESKF navigation -> pitch-attitude-hold control")
    fig.tight_layout()
    out = pathlib.Path(__file__).resolve().parents[2] / "docs" / "demo_closed_loop.png"
    fig.savefig(out, dpi=130)
    print(f"Saved {out}")


if __name__ == "__main__":
    main()
