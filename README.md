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





## Aerodynamic data provenance

All aerodynamic coefficients originate from:

> Jackson, E. B., and Cruz, C. L., *"Preliminary Subsonic Aerodynamic
> Model for Simulation Studies of the HL-20 Lifting Body,"*
> NASA TM-4302, August 1992.


## Proposed Architecture and Repo Structure

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
│   │   └── sim.py           vehicle assembly, trim, sim loop
│   ├── scripts/             demos and table generation
│   └── tests/               pytest suite (+ dependency-free runner)
├── matlab/                  cross-validation vs. reference model
├── data/hl20_aero/          generated lookup tables (C port source)
├── c/                       embedded flight software (Phase 3)
└── docs/
```


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
