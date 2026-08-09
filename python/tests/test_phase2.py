"""Phase 2 verification: ESKF navigation and pitch-attitude-hold control.

Run with: pytest python/tests -v   (includes Phase 1 + Phase 2)
or:       python tests/run_tests.py
"""

import sys
import pathlib

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))

from hlgnc.navigation import ESKF, NavState
from hlgnc.control import PitchAttitudeHold, PitchHoldGains
from hlgnc.dynamics import quat_from_euler, quat_to_euler, quat_to_dcm
from hlgnc.sim import HL20Vehicle, simulate
from hlgnc.actuators import allocate
from hlgnc.aero import HL20Aero, ControlDeflections


# --------------------------------------------------------------------- ESKF

def test_eskf_stationary_predict_holds_attitude():
    """With zero angular rate and specific force exactly cancelling
    gravity, a stationary vehicle's attitude estimate must not drift."""
    eskf = ESKF()
    eskf.initialize(np.zeros(3), np.zeros(3), quat_from_euler(0, 0, 0))
    f = np.array([0.0, 0.0, -9.80665])
    w = np.zeros(3)
    for _ in range(1000):
        eskf.predict(f, w, 0.005)
    phi, theta, psi = quat_to_euler(eskf.state.q)
    assert abs(phi) < 1e-6 and abs(theta) < 1e-6 and abs(psi) < 1e-6


def test_eskf_covariance_grows_during_predict_only():
    """Position uncertainty must monotonically grow with no
    measurement updates (information can only be lost, not gained)."""
    eskf = ESKF()
    eskf.initialize(np.zeros(3), np.array([160.0, 0, 0]),
                    quat_from_euler(0, 0, 0))
    f = np.array([0.0, 0.0, -9.80665])
    w = np.zeros(3)
    stds = []
    for k in range(200):
        eskf.predict(f, w, 0.005)
        if k % 20 == 0:
            stds.append(eskf.pos_error_std()[0])
    assert all(b >= a - 1e-9 for a, b in zip(stds, stds[1:]))


def test_eskf_gnss_update_reduces_uncertainty():
    eskf = ESKF()
    eskf.initialize(np.zeros(3), np.zeros(3), quat_from_euler(0, 0, 0))
    for _ in range(200):
        eskf.predict(np.array([0, 0, -9.80665]), np.zeros(3), 0.005)
    std_before = eskf.pos_error_std().copy()
    eskf.update_gnss(np.array([1.0, 0.5, -2.0]), np.zeros(3),
                     np.array([1.5, 1.5, 3.0]), 0.1)
    std_after = eskf.pos_error_std()
    assert np.all(std_after < std_before)


def test_eskf_converges_to_true_position_with_bias_and_noise():
    """With biased, noisy IMU and periodic GNSS aiding, the filter must
    track true position to within a few meters despite sensor errors
    it was never told the true value of."""
    rng = np.random.default_rng(3)
    from hlgnc.sensors import Imu, Gnss

    true_pos = np.zeros(3)
    true_vel = np.array([160.0, 0.0, 0.0])
    q_true = quat_from_euler(0, 0, 0)

    eskf = ESKF()
    eskf.initialize(true_pos + np.array([5, -5, 3]), true_vel,
                    q_true, pos_std=8.0)

    imu, gnss = Imu(), Gnss()
    dt = 0.005
    n = int(10.0 / dt)
    gnss_period = round(1 / gnss.cfg.rate_hz / dt)
    for k in range(n):
        true_pos = true_pos + true_vel * dt
        C_nb = quat_to_dcm(q_true)
        f_specific = C_nb.T @ np.array([0.0, 0.0, -9.80665])  # level flight
        x_dummy = np.zeros(14)
        x_dummy[10:13] = 0.0
        f_meas, w_meas = imu.sample(x_dummy, f_specific, rng)
        eskf.predict(f_meas, w_meas, dt)
        if k % gnss_period == 0:
            x_dummy2 = np.zeros(14)
            x_dummy2[0:3] = true_pos
            x_dummy2[3:6] = true_vel  # body == NED here (level, unrotated)
            x_dummy2[6:10] = q_true
            p_meas, v_meas = gnss.sample(x_dummy2, rng)
            eskf.update_gnss(p_meas, v_meas, gnss.cfg.pos_noise_std,
                             gnss.cfg.vel_noise_std)

    err = np.linalg.norm(eskf.state.p - true_pos)
    assert err < 5.0, f"nav position error {err:.2f} m too large"


# ------------------------------------------------------------------ control

def test_pitch_hold_sign_convention():
    """A positive attitude error (want more nose-up) must command a
    NEGATIVE elevator (since positive de is nose-down, per the Phase 1
    aero model): verifies the closed-loop sign is correct, not just
    that *some* control is produced."""
    ctrl = PitchAttitudeHold(PitchHoldGains(kp=100, kd=0), de_trim_deg=0.0)
    cmd = ctrl.command(theta_cmd_rad=np.deg2rad(5.0), theta_est_rad=0.0,
                       q_est_rad_s=0.0)
    assert cmd[1] < 0.0


def test_pitch_hold_saturates_within_limits():
    ctrl = PitchAttitudeHold(PitchHoldGains(kp=1000, kd=0), de_trim_deg=0.0)
    cmd = ctrl.command(theta_cmd_rad=np.deg2rad(90.0), theta_est_rad=0.0,
                       q_est_rad_s=0.0)
    assert abs(cmd[1]) <= 30.0 + 1e-9


def test_pitch_hold_zero_error_zero_rate_holds_trim():
    ctrl = PitchAttitudeHold(PitchHoldGains(), de_trim_deg=7.5)
    cmd = ctrl.command(theta_cmd_rad=0.1, theta_est_rad=0.1, q_est_rad_s=0.0)
    assert abs(cmd[1] - 7.5) < 1e-9


# --------------------------------------------------------- closed-loop plant

def test_closed_loop_stabilizes_unstable_trim():
    """The 160 m/s / 8 km trim point is statically unstable in pitch
    (verified below); the closed-loop controller must nonetheless hold
    a commanded attitude step within a small band."""
    veh = HL20Vehicle()
    V0, alt0 = 160.0, 8000.0
    alpha_deg, de_deg, gamma = veh.trim_glide(V=V0, alt=alt0)
    theta_trim = gamma + np.deg2rad(alpha_deg)
    x0 = veh.initial_state(V0, alt0, alpha_deg, gamma)
    veh.actuators.pos = allocate(np.array([0, de_deg, 0, 0, 0, 0], float))

    ctrl = PitchAttitudeHold(PitchHoldGains(), de_trim_deg=de_deg)
    theta_cmd = theta_trim + np.deg2rad(2.0)

    def law(t, x, v):
        theta = quat_to_euler(x[6:10])[1]
        return ctrl.command(theta_cmd, theta, x[11])

    res = simulate(veh, x0, t_end=20.0, dt=0.005, control_law=law)
    theta_final = np.rad2deg(quat_to_euler(res.x[-1, 6:10])[1])
    assert abs(theta_final - np.rad2deg(theta_cmd)) < 0.5


def test_trim_point_is_statically_unstable_in_pitch():
    """Documents the reason the controller needs meaningful gain: at
    the nominal trim point dCm/dalpha > 0 (destabilizing)."""
    veh = HL20Vehicle()
    alpha_deg, de_deg, gamma = veh.trim_glide(V=160.0, alt=8000.0)
    aero = HL20Aero()
    d = ControlDeflections(de=de_deg)
    h = 0.01
    c1 = aero.coefficients(alpha_deg - h, 0.0, d, V=160.0)["Cm"]
    c2 = aero.coefficients(alpha_deg + h, 0.0, d, V=160.0)["Cm"]
    dcm_dalpha = (c2 - c1) / (2 * h)
    assert dcm_dalpha > 0.0


def test_closed_loop_uses_only_bounded_control_effort():
    """Sanity bound: a well-tuned loop should not ride the actuator
    limit continuously for a modest 2 deg step."""
    veh = HL20Vehicle()
    V0, alt0 = 160.0, 8000.0
    alpha_deg, de_deg, gamma = veh.trim_glide(V=V0, alt=alt0)
    theta_trim = gamma + np.deg2rad(alpha_deg)
    x0 = veh.initial_state(V0, alt0, alpha_deg, gamma)
    veh.actuators.pos = allocate(np.array([0, de_deg, 0, 0, 0, 0], float))
    ctrl = PitchAttitudeHold(PitchHoldGains(), de_trim_deg=de_deg)
    theta_cmd = theta_trim + np.deg2rad(2.0)

    def law(t, x, v):
        theta = quat_to_euler(x[6:10])[1]
        return ctrl.command(theta_cmd, theta, x[11])

    res = simulate(veh, x0, t_end=20.0, dt=0.005, control_law=law)
    assert np.abs(res.defl[:, 1]).max() < 25.0
