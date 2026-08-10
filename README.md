# Lifting-Body GNC: 6-DOF Simulation of the NASA HL-20

Nonlinear six-degree-of-freedom flight dynamics and GNC development
framework for a lifting-body reentry vehicle, built on the **published
NASA HL-20 aerodynamic database** (Jackson & Cruz, NASA TM-4302, 1992).

The HL-20 Personnel Launch System is an unpowered lifting body designed
to return from orbit and land horizontally on a runway — the same class
of vehicle and the same terminal-flight GNC problem faced by modern
reusable spaceplanes. A control-oriented propulsion model is included
for the powered ferry/ascent configuration that precedes an unpowered
glide test point in a staged flight-test program — the class of
propulsion used by turbojet-powered subscale demonstrators.

This project models all four physical-subsystem categories relevant to
GNC development: **aerodynamics**, **actuators**, **sensors**, and
**propulsion** — each as an independently created, tested, and (for
aerodynamics) externally cross-validated mathematical model.

![Open-loop trimmed glide](docs/demo_glide.png)

---

## Why this project

Terminal-area flight of an unpowered lifting body is one of the harder
GNC problems in atmospheric flight: low lift-to-drag ratio (~3), steep
glide path (−17° to −22°), no go-around, and tight touchdown
dispersion requirements. This repository builds the complete
simulation and GNC stack for that problem from first principles:

1. validated nonlinear 6-DOF plant using real published aero data,
2. navigation (EKF sensor fusion), control (gain-scheduled), and
   guidance (energy-managed approach) layers,
3. a dependency-free embedded C implementation of the flight software,
   cross-validated against the Python reference,
4. processor-in-the-loop execution on an STM32 target.

---

## Current status

| Module | Status |
|---|---|
| US Standard Atmosphere 1976 (0–86 km) | done |
| HL-20 aero model (TM-4302 polynomials, full control set, rate damping, CG transfer) | done |
| Quaternion 6-DOF rigid-body dynamics (RK4) | done |
| Actuator bank (2nd-order servos, limits) + 7-surface control allocation | done |
| Sensor models (IMU / GNSS / baro / air data) | done |
| Propulsion model (spool lag, density/Mach thrust lapse, fuel burn) | done |
| Steady-glide trim solver + validity-envelope guards | done |
| Test suite (30 tests: physics invariants, port integrity, symmetry, trim equilibrium) | done |
| MATLAB cross-validation of the aero data port | done (`matlab/cross_validation_output.txt`) |
| ESKF navigation (15-state error-state KF: IMU strapdown + GNSS + baro) | **done** |
| Pitch-attitude-hold flight control (PD, rate-damped) | **done** |
| Gain-scheduled flight control | in progress |
| TAEM-style energy-managed approach & landing guidance | in progress |
| Monte Carlo dispersion campaign | planned |
| Embedded C flight software + SIL cross-validation | planned |
| Processor-in-the-loop on STM32 (UART, then CAN via MCP2515) | planned |
| Supersonic/hypersonic aero extension (Mach-dependent tables) | planned |
| Simulink plant model (Aerospace Blockset, shared CSV tables) + trajectory-level cross-validation | planned |

---

## Project phases

| Phase | Content | Status |
|---|---|---|
| **1** | Plant: aerodynamics, atmosphere, rigid-body dynamics, actuators, sensors, propulsion | done |
| **2** | ESKF navigation, pitch-attitude-hold control, closed loop on estimated (not true) state | done |
| **3** | Dependency-free embedded C port of navigation + control, SIL cross-validation vs. Python | in progress |
| **4** | Processor-in-the-loop on STM32 Nucleo-F401RE | in progress |
| **5** | Gain-scheduled control, TAEM-style energy-managed glide-path guidance, Monte Carlo dispersion, Simulink plant cross-validation, CAN/MCP2515 PIL upgrade | roadmap -- landing as commits after initial release |

--- 

## Physical subsystem models

Four physical subsystems are independently modeled, each with stated
assumptions and a distinct validation method:

| Subsystem | Model | Key assumptions | Validated by |
|---|---|---|---|
| **Aerodynamics** | TM-4302 polynomial coefficients (datum + 6 control-surface effectiveness models + rate damping), transferred from the reference point to the CG | subsonic only; α ∈ [−10°, 30°], β ∈ [−10°, 10°]; control effectiveness linear in deflection | long-hand polynomial re-derivation in tests, lateral-symmetry checks, MATLAB cross-validation against the reference implementation |
| **Actuators** | 2nd-order servo (ωn = 44 rad/s, ζ = 0.71) with position limits; 7-surface → 6-aero-effect allocation via pseudo-inverse | rate saturation not modeled (roadmap) | step-response convergence, limit clamping, mixer round-trip test |
| **Sensors** | IMU (bias + noise + optional gyro-bias random walk), GNSS, baro, air data | linear-Gaussian error models; no scale-factor error (roadmap) | statistical recovery of bias/noise over Monte Carlo sample sets |
| **Propulsion** | 1st-order spool lag, density- and Mach-based thrust lapse, fuel-mass integration, thrust-line offset moment | control-oriented (no compressor/turbine thermodynamics); models the powered demonstrator/ferry configuration, since the HL-20 itself is an unpowered glider | spool time-constant recovery, Mach-lapse monotonicity, fuel-depletion floor, thrust-offset moment sign |

---


### Propulsion model

First-order spool-lag thrust response with density- and Mach-based
thrust lapse, plus fuel-mass integration — modeling the powered
demonstrator/ferry configuration (the HL-20 itself is an unpowered
glider).

![Propulsion model verification](docs/demo_propulsion.png)

---

## Navigation and control

**Navigation** is a 15-state error-state Kalman filter (position,
velocity, attitude, gyro bias, accel bias) fusing strapdown IMU
mechanization with GNSS (5 Hz) and barometric altitude (20 Hz)
updates. First-order (Euler) discretization of the error-state
transition; documented in `navigation.py`.

**Control** is a PD pitch-attitude-hold loop with body-rate damping,
commanding the elevator. A notable finding from closed-loop
verification: the vehicle's trim point at 160 m/s / 8 km altitude is
**statically unstable in pitch** (dCm/dalpha > 0), consistent with
lifting-body vehicles that rely on active stability augmentation
rather than inherent aerodynamic stability. `test_phase2.py` documents
this explicitly and verifies the controller stabilizes it. Gains were
tuned by sweep (`scripts/tune_pitch_gains.py`) for a 2 deg attitude step
with < 2 deg overshoot, < 0.05 deg steady-state error, and no actuator
saturation.

The `scripts/demo_closed_loop.py` demo is the integration test that
matters: the controller runs on the **ESKF's estimated** attitude and
rate, not the simulator's true state -- proving the navigation and
control layers compose correctly, which is the actual flight-software
architecture.

![Closed-loop demo](docs/demo_closed_loop.png)


This table is deliberate: it mirrors a common GNC job-ad requirement —
create mathematical models for aerodynamics, sensors, actuators, and
propulsion — with each subsystem backed by an explicit assumption set
and a named test, not just an implementation.

---


## Aerodynamic data provenance

All aerodynamic coefficients originate from:

> Jackson, E. B., and Cruz, C. L., *"Preliminary Subsonic Aerodynamic
> Model for Simulation Studies of the HL-20 Lifting Body,"*
> NASA TM-4302, August 1992.

The published polynomial fits (datum coefficients, six control-surface
effectiveness models, dynamic damping derivatives) are implemented in
`python/hlgnc/aero.py`. `matlab/cross_validate_hl20.m` verifies the
port bit-for-bit against the reference MATLAB/Simulink implementation
of the same database (Aerospace Blockset HL-20 example — MathWorks
example code is **not** redistributed here; only the public-domain NASA
coefficient data is used).

**Model validity envelope** (inherited from TM-4302 and enforced in
code): α ∈ [−10°, +30°], β ∈ [−10°, +10°], subsonic only
(compressibility neglected), control effectiveness linear in
deflection. The simulation scenario is therefore terminal-area flight:
subsonic glide, approach and landing. Mach-dependent table
infrastructure is in place for the published higher-Mach HL-20 data as
a roadmap item.

---

## Quick start (Windows / Anaconda or any Python ≥ 3.10)

```
pip install -r requirements.txt
pytest python/tests -v                        # 40 tests
python python/scripts/demo_glide.py            # open-loop trimmed-glide demo plot
python python/scripts/demo_closed_loop.py       # ESKF + pitch-hold closed-loop demo plot
python python/scripts/generate_aero_tables.py  # lookup tables for the C port
python python/scripts/demo_propulsion.py       # propulsion plot
```

---

## Architecture

```
lifting-body-gnc/
├── python/
│   ├── hlgnc/
│   │   ├── aero.py          TM-4302 aerodynamic model
│   │   ├── atmosphere.py    US76 / COESA atmosphere
│   │   ├── dynamics.py      quaternion 6-DOF EOM, RK4
│   │   ├── actuators.py     servo bank + control allocation
│   │   ├── sensors.py       IMU / GNSS / baro / air data
│   │   ├── propulsion.py    spool lag, thrust lapse, fuel burn
│   │   ├── navigation.py    15-state ESKF (IMU + GNSS + baro)
│   │   ├── control.py       pitch-attitude-hold flight control
│   │   └── sim.py           vehicle assembly, trim, sim loop
│   ├── scripts/             demos and table generation
│   └── tests/               pytest suite (+ dependency-free runner)
├── matlab/                  cross-validation vs. reference model
├── data/hl20_aero/          generated lookup tables (C port source)
├── c/                       embedded flight software (Phase 3)
└── docs/
```


---

## Verification approach

Every layer is tested against an independent reference:

- **Atmosphere** vs. published US76 table values (tropopause, 20 km).
- **Aero data port** vs. long-hand re-computation in the tests *and*
  vs. the reference MATLAB implementation (`matlab/`).
- **Physics invariants**: quaternion norm preservation over 2000 RK4
  steps, ballistic free-fall against closed-form kinematics, DCM
  orthonormality, lateral-symmetry properties required by TM-4302.
- **Trim** verified as a true equilibrium by simulation: a trimmed
  state flown open-loop holds airspeed within 5 % and α within 1.5°.
- Trim solutions outside actuator authority or the aero validity
  envelope are rejected, not returned.

  ---



## 👤 Author

**Vasan Iyer**  
GNC / Embedded Software Engineer  

Focus areas:
 
- Embedded systems (C++, Python) 
- GNC
- Flight dynamics & control  
- Sensor fusion & state estimation  
- Autonomous systems  
- UAV systems 


GitHub: https://github.com/Vaiy108
