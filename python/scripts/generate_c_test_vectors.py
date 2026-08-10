"""Generate SIL cross-validation test vectors from the Python reference.

Runs the ESKF and pitch-attitude-hold controller through representative
scenarios and dumps every intermediate input/output to CSV. The C
implementation (c/test/test_main.c) replays the same inputs and its
outputs are diffed against these files -- this is the same
"reference implementation + dependency-free C, cross-validated on
shared test vectors" pattern used in the radar and S32R45 projects.

Output: c/test_vectors/*.csv
"""

import sys
import pathlib

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))

from hlgnc.navigation import ESKF
from hlgnc.control import PitchAttitudeHold, PitchHoldGains
from hlgnc.dynamics import quat_from_euler

OUT = pathlib.Path(__file__).resolve().parents[2] / "c" / "test_vectors"


def gen_eskf_predict_vectors():
    """Sequence of predict() calls with varying IMU inputs; dump the
    nominal state and a P diagonal sample after each step."""
    rng = np.random.default_rng(123)
    kf = ESKF()
    kf.initialize(np.array([0., 0., -8000.]), np.array([160., 0., 0.]),
                  quat_from_euler(0, -0.05, 0.02))
    rows = []
    dt = 0.005
    for k in range(500):
        f = np.array([0.5 * np.sin(0.1 * k), 0.2, -9.80665 + 0.3 * np.cos(0.05 * k)])
        w = np.array([0.01 * np.sin(0.02 * k), 0.005, -0.008 * np.cos(0.03 * k)])
        kf.predict(f, w, dt)
        rows.append(np.concatenate([
            f, w, [dt],
            kf.state.p, kf.state.v, kf.state.q, kf.state.bg, kf.state.ba,
            np.diag(kf.P),
        ]))
    header = ("fx,fy,fz,wx,wy,wz,dt,"
             "p0,p1,p2,v0,v1,v2,q0,q1,q2,q3,bg0,bg1,bg2,ba0,ba1,ba2,"
             + ",".join(f"Pdiag{i}" for i in range(15)))
    np.savetxt(OUT / "eskf_predict_vectors.csv", rows, delimiter=",",
              header=header, comments="")


def gen_eskf_update_vectors():
    """Predict-then-update sequence exercising both GNSS and baro
    updates, dumping post-update state and P diagonal."""
    rng = np.random.default_rng(7)
    kf = ESKF()
    kf.initialize(np.array([0., 0., -8000.]), np.array([160., 0., 0.]),
                  quat_from_euler(0, -0.05, 0.0), pos_std=8.0)
    rows = []
    dt = 0.005
    for k in range(300):
        f = np.array([0.1, 0.0, -9.80665])
        w = np.array([0.0, 0.002, 0.0])
        kf.predict(f, w, dt)
        update_type = 0
        pos_meas = vel_meas = np.zeros(3)
        alt_meas = 0.0
        if k % 40 == 0:
            update_type = 1
            pos_meas = kf.state.p + rng.normal(0, 1.0, 3)
            vel_meas = kf.state.v + rng.normal(0, 0.1, 3)
            kf.update_gnss(pos_meas, vel_meas, np.array([1.5, 1.5, 3.0]), 0.1)
        elif k % 10 == 0:
            update_type = 2
            alt_meas = -kf.state.p[2] + rng.normal(0, 1.0)
            kf.update_baro(alt_meas, 1.0)
        rows.append(np.concatenate([
            f, w, [dt, update_type],
            pos_meas, vel_meas, [alt_meas],
            kf.state.p, kf.state.v, kf.state.q, kf.state.bg, kf.state.ba,
            np.diag(kf.P),
        ]))
    header = ("fx,fy,fz,wx,wy,wz,dt,update_type,"
             "posx,posy,posz,velx,vely,velz,alt,"
             "p0,p1,p2,v0,v1,v2,q0,q1,q2,q3,bg0,bg1,bg2,ba0,ba1,ba2,"
             + ",".join(f"Pdiag{i}" for i in range(15)))
    np.savetxt(OUT / "eskf_update_vectors.csv", rows, delimiter=",",
              header=header, comments="")


def gen_control_vectors():
    """Sweep of (theta_cmd, theta_est, q_est, de_trim) -> elevator
    command, including saturation cases."""
    rng = np.random.default_rng(0)
    ctrl = PitchAttitudeHold(PitchHoldGains(), de_trim_deg=10.28)
    rows = []
    cases = []
    for _ in range(200):
        theta_cmd = rng.uniform(-0.5, 0.5)
        theta_est = rng.uniform(-0.5, 0.5)
        q_est = rng.uniform(-0.3, 0.3)
        cases.append((theta_cmd, theta_est, q_est))
    # explicit saturation-triggering cases
    cases += [(1.5, -1.5, 0.0), (-1.5, 1.5, 0.0), (0.0, 0.0, 5.0), (0.0, 0.0, -5.0)]
    for theta_cmd, theta_est, q_est in cases:
        cmd = ctrl.command(theta_cmd, theta_est, q_est)
        rows.append([theta_cmd, theta_est, q_est, cmd[1]])
    np.savetxt(OUT / "control_vectors.csv", rows, delimiter=",",
              header="theta_cmd,theta_est,q_est,de_expected", comments="")


if __name__ == "__main__":
    OUT.mkdir(parents=True, exist_ok=True)
    gen_eskf_predict_vectors()
    gen_eskf_update_vectors()
    gen_control_vectors()
    print(f"Wrote test vectors to {OUT}")
