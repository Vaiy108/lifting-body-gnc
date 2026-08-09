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

## Aerodynamic data provenance

All aerodynamic coefficients originate from:

> Jackson, E. B., and Cruz, C. L., *"Preliminary Subsonic Aerodynamic
> Model for Simulation Studies of the HL-20 Lifting Body,"*
> NASA TM-4302, August 1992.

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
