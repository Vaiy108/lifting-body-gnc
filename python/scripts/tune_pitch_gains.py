"""Pitch-attitude-hold gain sweep at the 160 m/s / 8 km trim point.

Documents the tuning process referenced by PitchHoldGains' default
values. The bare airframe is statically unstable in pitch at this trim
point (dCm/dalpha > 0), so low gains fail to stabilize a 2-degree
attitude step; this script sweeps (kp, kd) and reports overshoot,
steady-state error, and peak elevator usage.
"""

import sys
import pathlib

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))

from hlgnc.sim import HL20Vehicle, simulate
from hlgnc.actuators import allocate
from hlgnc.control import PitchAttitudeHold, PitchHoldGains
from hlgnc.dynamics import quat_to_euler


def run(kp: float, kd: float, V0=160.0, alt0=8000.0, step_deg=2.0,
       t_end=20.0):
    veh = HL20Vehicle()
    alpha_deg, de_deg, gamma = veh.trim_glide(V=V0, alt=alt0)
    theta_trim = gamma + np.deg2rad(alpha_deg)
    x0 = veh.initial_state(V0, alt0, alpha_deg, gamma)
    veh.actuators.pos = allocate(np.array([0, de_deg, 0, 0, 0, 0], float))
    ctrl = PitchAttitudeHold(PitchHoldGains(kp=kp, kd=kd), de_trim_deg=de_deg)
    theta_cmd = theta_trim + np.deg2rad(step_deg)

    def law(t, x, v):
        theta = quat_to_euler(x[6:10])[1]
        return ctrl.command(theta_cmd, theta, x[11])

    res = simulate(veh, x0, t_end=t_end, dt=0.005, control_law=law)
    theta_hist = np.rad2deg([quat_to_euler(q)[1] for q in res.x[:, 6:10]])
    tgt = np.rad2deg(theta_cmd)
    return dict(
        overshoot=theta_hist.max() - tgt,
        final_err=theta_hist[-1] - tgt,
        de_max=np.abs(res.defl[:, 1]).max(),
    )


if __name__ == "__main__":
    print(f"{'kp':>6} {'kd':>6} {'overshoot':>10} {'final_err':>10} {'de_max':>8}")
    for kp, kd in [(100, 30), (150, 50), (200, 60), (250, 70), (300, 80), (400, 100)]:
        r = run(kp, kd)
        print(f"{kp:6.0f} {kd:6.0f} {r['overshoot']:10.3f} "
              f"{r['final_err']:10.4f} {r['de_max']:8.2f}")
