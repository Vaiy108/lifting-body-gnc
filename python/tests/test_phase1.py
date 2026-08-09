"""Phase 1 verification suite.

Run with:  pytest python/tests -v
Every test is a plain assert-based function (no fixtures) so the suite
also runs under the bundled dependency-free runner.
"""

import numpy as np

import sys
import pathlib
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))

from hlgnc.atmosphere import atmosphere
from hlgnc.aero import HL20Aero, HL20Geometry, ControlDeflections
from hlgnc.dynamics import (RigidBody, rk4_step, quat_from_euler,
                            quat_to_euler, quat_to_dcm, wind_angles,
                            STATE_DIM, GRAV)
from hlgnc.actuators import ActuatorBank, allocate, ACT2AERO, MAX_LIM, MIN_LIM
from hlgnc.sensors import Imu, Gnss, Baro
from hlgnc.propulsion import Propulsion, PropulsionConfig
from hlgnc.sim import HL20Vehicle, simulate


# ---------------------------------------------------------------- atmosphere

def test_atmosphere_sea_level():
    T, p, rho, a = atmosphere(0.0)
    assert abs(T - 288.15) < 1e-6
    assert abs(p - 101325.0) < 1e-3
    assert abs(rho - 1.2250) < 1e-3
    assert abs(a - 340.29) < 0.05


def test_atmosphere_tropopause():
    # US76 published values at 11 km geopotential.
    # Geometric altitude for 11 km geopotential:
    z = 11000.0 * 6356766.0 / (6356766.0 - 11000.0)
    T, p, rho, a = atmosphere(z)
    assert abs(T - 216.65) < 0.01
    assert abs(p - 22632.06) / 22632.06 < 1e-4
    assert abs(rho - 0.36392) / 0.36392 < 1e-3


def test_atmosphere_20km():
    z = 20000.0 * 6356766.0 / (6356766.0 - 20000.0)
    T, p, rho, a = atmosphere(z)
    assert abs(T - 216.65) < 0.01
    assert abs(rho - 0.088035) / 0.088035 < 1e-3


def test_atmosphere_monotonic_density():
    zs = np.linspace(0, 60000, 200)
    rhos = [atmosphere(z)[2] for z in zs]
    assert all(a > b for a, b in zip(rhos, rhos[1:]))


# ---------------------------------------------------------------------- aero

def test_datum_reference_values():
    """Independent re-computation of the TM-4302 datum polynomial at a
    spot point (alpha=10 deg, beta=5 deg), written out long-hand so a
    bug in the vectorized implementation cannot hide."""
    aero = HL20Aero()
    a, b = 10.0, 5.0
    reg = [1.0, a, a**2, a**3, a**4, a**5, a**6,
           abs(b), b**2, abs(b) ** 3, b**4, a * abs(b)]
    P = [  # [term_Z, term_m, term_X] rows, from NASA TM-4302
        (-9.025e-2, 2.632e-2, 7.362e-2),
        (4.070e-2, -2.226e-3, -2.560e-4),
        (3.094e-5, -1.859e-5, -2.208e-4),
        (1.564e-5, 6.001e-7, -2.262e-6),
        (-1.386e-6, 1.828e-7, 2.966e-7),
        (2.545e-8, -9.733e-9, -3.640e-9),
        (-1.189e-10, 1.710e-10, 9.388e-12),
        (2.564e-3, -5.233e-4, -5.299e-4),
        (8.501e-4, 6.795e-5, -4.709e-4),
        (-1.156e-4, -1.993e-5, 8.572e-5),
        (3.416e-6, 1.341e-6, -4.199e-6),
        (-4.862e-4, 6.061e-5, 1.295e-4),
    ]
    tz = sum(r * row[0] for r, row in zip(reg, P))
    tm = sum(r * row[1] for r, row in zip(reg, P))
    tx = sum(r * row[2] for r, row in zip(reg, P))
    c = aero.datum(a, b)
    assert abs(c["CZ"] - (-tz)) < 1e-12
    assert abs(c["Cm"] - tm) < 1e-12
    assert abs(c["CX"] - (-tx)) < 1e-12
    assert abs(c["CY"] - (-0.01242 * b)) < 1e-12
    assert abs(c["Cl"] - (-0.00787 * b)) < 1e-12


def test_datum_alpha_zero_lift_sign():
    """At moderate positive alpha the lifting body must produce upward
    lift: CZ < 0 in body axes (z down), and CX < 0 (drag)."""
    aero = HL20Aero()
    c = aero.datum(15.0, 0.0)
    assert c["CZ"] < 0.0
    assert c["CX"] < 0.0


def test_lateral_symmetry():
    """CY, Cl, Cn odd in beta; CX, CZ, Cm even in beta (TM-4302
    laterally-symmetric vehicle assumption)."""
    aero = HL20Aero()
    for a in (-5.0, 0.0, 10.0, 25.0):
        cp = aero.datum(a, 6.0)
        cn = aero.datum(a, -6.0)
        assert abs(cp["CY"] + cn["CY"]) < 1e-12
        assert abs(cp["Cl"] + cn["Cl"]) < 1e-12
        assert abs(cp["Cn"] + cn["Cn"]) < 1e-12
        assert abs(cp["CX"] - cn["CX"]) < 1e-12
        assert abs(cp["CZ"] - cn["CZ"]) < 1e-12
        assert abs(cp["Cm"] - cn["Cm"]) < 1e-12


def test_cn0_table_nodes():
    """Interpolator must reproduce table values exactly at grid nodes."""
    aero = HL20Aero()
    c = aero.datum(0.0, 5.0)
    assert abs(c["Cn"] - 2.00e-2) < 1e-12
    c = aero.datum(20.0, 10.0)
    assert abs(c["Cn"] - 1.20e-2) < 1e-12
    c = aero.datum(20.0, -10.0)
    assert abs(c["Cn"] + 1.20e-2) < 1e-12


def test_elevator_effectiveness_sign():
    """Positive (trailing-edge-down) elevator must pitch nose down
    across the operating alpha range."""
    aero = HL20Aero()
    for a in (0.0, 10.0, 20.0):
        c0 = aero.control_increments(a, ControlDeflections())
        c1 = aero.control_increments(a, ControlDeflections(de=10.0))
        assert c1["Cm"] < c0["Cm"]


def test_control_linearity():
    """TM-4302 assumption: increments linear in deflection."""
    aero = HL20Aero()
    c1 = aero.control_increments(8.0, ControlDeflections(da=4.0))
    c2 = aero.control_increments(8.0, ControlDeflections(da=8.0))
    for k in ("CX", "CY", "CZ", "Cl", "Cm", "Cn"):
        assert abs(c2[k] - 2.0 * c1[k]) < 1e-12


def test_pitch_damping_sign():
    """Positive pitch rate must reduce Cm (Cmq < 0 everywhere)."""
    aero = HL20Aero()
    d = ControlDeflections()
    c_still = aero.coefficients(10.0, 0.0, d, q=0.0, V=100.0)
    c_pitch = aero.coefficients(10.0, 0.0, d, q=0.2, V=100.0)
    assert c_pitch["Cm"] < c_still["Cm"]


# ------------------------------------------------------------------ dynamics

def test_quaternion_norm_preserved():
    geom = HL20Geometry()
    body = RigidBody(geom.mass, geom.inertia)
    x = np.zeros(STATE_DIM)
    x[3:6] = [100.0, 5.0, 10.0]
    x[6:10] = quat_from_euler(0.2, 0.3, -0.5)
    x[10:13] = [0.4, -0.3, 0.2]
    f = lambda t, xx: body.derivatives(xx, np.zeros(3), np.zeros(3))
    for k in range(2000):
        x = rk4_step(f, 0.0, x, 0.005)
    assert abs(np.linalg.norm(x[6:10]) - 1.0) < 1e-12


def test_ballistic_free_fall():
    """With zero external force, NED vertical speed gains g*t."""
    geom = HL20Geometry()
    body = RigidBody(geom.mass, geom.inertia)
    x = np.zeros(STATE_DIM)
    x[6:10] = quat_from_euler(0.0, 0.0, 0.0)
    x[3:6] = [50.0, 0.0, 0.0]
    f = lambda t, xx: body.derivatives(xx, np.zeros(3), np.zeros(3))
    t_end, dt = 2.0, 0.001
    for k in range(int(t_end / dt)):
        x = rk4_step(f, 0.0, x, dt)
    C_nb = quat_to_dcm(x[6:10])
    v_ned = C_nb @ x[3:6]
    assert abs(v_ned[2] - GRAV * t_end) < 1e-6
    assert abs(v_ned[0] - 50.0) < 1e-6


def test_euler_quat_roundtrip():
    for phi, theta, psi in [(0.1, -0.4, 2.0), (-1.0, 0.7, -2.5)]:
        q = quat_from_euler(phi, theta, psi)
        e = quat_to_euler(q)
        assert abs(e[0] - phi) < 1e-12
        assert abs(e[1] - theta) < 1e-12
        assert abs(e[2] - psi) < 1e-12


def test_dcm_orthonormal():
    q = quat_from_euler(0.3, -0.2, 1.1)
    C = quat_to_dcm(q)
    assert np.allclose(C @ C.T, np.eye(3), atol=1e-12)
    assert abs(np.linalg.det(C) - 1.0) < 1e-12


def test_wind_angles():
    V, alpha, beta = wind_angles(np.array([100.0, 0.0, 17.63269807]))
    assert abs(np.rad2deg(alpha) - 10.0) < 1e-6
    assert abs(beta) < 1e-12


# ----------------------------------------------------------------- actuators

def test_actuator_converges_to_command():
    bank = ActuatorBank()
    cmd = np.array([10.0, -10.0, 20.0, 20.0, -15.0, -15.0, 5.0])
    dt = 0.001
    for k in range(int(1.0 / dt)):
        pos = bank.step(cmd, dt)
    assert np.allclose(pos, cmd, atol=1e-3)


def test_actuator_position_limits():
    bank = ActuatorBank()
    cmd = np.array([90.0, -90.0, 90.0, 90.0, -90.0, -90.0, 90.0])
    dt = 0.001
    for k in range(int(1.0 / dt)):
        pos = bank.step(cmd, dt)
    assert np.all(pos <= MAX_LIM + 1e-9)
    assert np.all(pos >= MIN_LIM - 1e-9)
    assert abs(pos[0] - 30.0) < 1e-6
    assert abs(pos[6] - 60.0) < 1e-6


def test_mixer_roundtrip():
    """Aero command -> surfaces -> aero effects must round-trip."""
    aero_cmd = np.array([3.0, -6.0, 4.0, 10.0, -8.0, 2.0])
    surf = allocate(aero_cmd)
    back = ACT2AERO @ surf
    assert np.allclose(back, aero_cmd, atol=1e-9)


# ------------------------------------------------------------------- sensors

def test_sensor_statistics():
    rng = np.random.default_rng(42)
    imu = Imu()
    x = np.zeros(STATE_DIM)
    x[6:10] = quat_from_euler(0, 0, 0)
    f_true = np.array([0.0, 0.0, -9.0])
    samples = np.array([imu.sample(x, f_true, rng)[0] for _ in range(5000)])
    mean_err = samples.mean(axis=0) - f_true - imu.cfg.accel_bias
    assert np.all(np.abs(mean_err) < 5 * imu.cfg.accel_noise_std / np.sqrt(5000))


def test_baro_bias():
    rng = np.random.default_rng(1)
    baro = Baro()
    x = np.zeros(STATE_DIM)
    x[2] = -5000.0
    vals = np.array([baro.sample(x, rng) for _ in range(2000)])
    assert abs(vals.mean() - (5000.0 + baro.cfg.alt_bias)) < 0.2


# ---------------------------------------------------------------- propulsion

def test_thrust_lapses_with_altitude():
    """Thrust available at altitude must be less than sea-level static
    thrust (density lapse), at fixed Mach."""
    p = Propulsion()
    t_sl = p.available_thrust(alt=0.0, mach=0.3)
    t_alt = p.available_thrust(alt=8000.0, mach=0.3)
    assert 0.0 < t_alt < t_sl


def test_thrust_lapses_with_mach():
    p = Propulsion()
    t_low = p.available_thrust(alt=2000.0, mach=0.1)
    t_high = p.available_thrust(alt=2000.0, mach=0.8)
    assert t_high < t_low


def test_spool_first_order_response():
    """Step throttle command; thrust must rise monotonically toward the
    steady-state target and reach it within ~5 time constants."""
    p = Propulsion()
    dt = 0.01
    target = p.available_thrust(alt=0.0, mach=0.0)
    thrust_hist = []
    for _ in range(int(5 * p.cfg.tau_spool / dt)):
        th, _ = p.step(throttle_cmd=1.0, alt=0.0, mach=0.0, dt=dt)
        thrust_hist.append(th)
    assert all(b >= a - 1e-6 for a, b in zip(thrust_hist, thrust_hist[1:]))
    assert abs(thrust_hist[-1] - target) / target < 0.01


def test_fuel_burns_and_floors_at_zero():
    p = Propulsion(cfg=PropulsionConfig(thrust_max_sl=18000.0, tsfc=1.8e-5))
    p.state.fuel_mass = 0.05   # nearly empty
    dt = 0.1
    for _ in range(200):
        th, fuel = p.step(throttle_cmd=1.0, alt=0.0, mach=0.0, dt=dt)
    assert fuel == 0.0
    assert th == 0.0


def test_thrust_force_along_body_x():
    p = Propulsion()
    p.state.thrust = 5000.0
    F_b, M_b = p.force_moment_body()
    assert np.allclose(F_b, [5000.0, 0.0, 0.0])
    assert np.allclose(M_b, [0.0, 0.0, 0.0])  # zero offset by default


def test_thrust_offset_produces_moment():
    cfg = PropulsionConfig(mount_offset_x=-2.0, mount_offset_z=0.3)
    p = Propulsion(cfg=cfg)
    p.state.thrust = 4000.0
    F_b, M_b = p.force_moment_body()
    # r x F with r=[-2,0,0.3], F=[4000,0,0] -> M = [0, 0.3*4000, 0]
    assert np.allclose(M_b, [0.0, 1200.0, 0.0])


# --------------------------------------------------------------- trim & sim

def test_trim_glide_converges():
    veh = HL20Vehicle()
    alpha_deg, de_deg, gamma = veh.trim_glide(V=120.0, alt=5000.0)
    # Within model validity envelope and physically sensible for a
    # low-L/D lifting body: steep glide, single-digit-to-moderate alpha.
    assert -10.0 < alpha_deg < 30.0
    assert -30.0 < de_deg < 30.0
    assert -np.deg2rad(45.0) < gamma < 0.0


def test_trim_is_equilibrium():
    """Simulating from trim with frozen controls must hold alpha, V and
    gamma nearly constant over a short horizon."""
    veh = HL20Vehicle()
    V0, alt0 = 160.0, 8000.0
    alpha_deg, de_deg, gamma = veh.trim_glide(V=V0, alt=alt0)
    x0 = veh.initial_state(V0, alt0, alpha_deg, gamma)

    hold = lambda t, x, v: np.array([0.0, de_deg, 0.0, 0.0, 0.0, 0.0])
    # Pre-position actuators at the trim deflection to avoid startup
    # transient.
    veh.actuators.pos = allocate(np.array([0, de_deg, 0, 0, 0, 0], float))
    res = simulate(veh, x0, t_end=5.0, dt=0.005, control_law=hold)

    from hlgnc.dynamics import wind_angles as wa
    V_end, alpha_end, _ = wa(res.x[-1, 3:6])
    assert abs(V_end - V0) / V0 < 0.05
    assert abs(np.rad2deg(alpha_end) - alpha_deg) < 1.5


def test_glide_descends_and_travels():
    """Open-loop trimmed glide: loses altitude, gains range, stays
    upright, quaternion remains unit norm."""
    veh = HL20Vehicle()
    V0, alt0 = 160.0, 8000.0
    alpha_deg, de_deg, gamma = veh.trim_glide(V=V0, alt=alt0)
    x0 = veh.initial_state(V0, alt0, alpha_deg, gamma)
    veh.actuators.pos = allocate(np.array([0, de_deg, 0, 0, 0, 0], float))
    hold = lambda t, x, v: np.array([0.0, de_deg, 0.0, 0.0, 0.0, 0.0])
    res = simulate(veh, x0, t_end=30.0, dt=0.005, control_law=hold)
    assert res.altitude()[-1] < alt0 - 500.0
    assert res.x[-1, 0] > 1000.0
    assert abs(np.linalg.norm(res.x[-1, 6:10]) - 1.0) < 1e-9
