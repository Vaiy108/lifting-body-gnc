"""hlgnc: HL-20 lifting-body GNC simulation package."""

from .aero import HL20Aero, HL20Geometry, ControlDeflections
from .atmosphere import atmosphere
from .dynamics import (RigidBody, rk4_step, quat_from_euler, quat_to_euler,
                       quat_to_dcm, quat_normalize, wind_angles)
from .actuators import ActuatorBank, allocate, ACT2AERO, AERO2ACT
from .sensors import Imu, Gnss, Baro, AirData
from .propulsion import Propulsion, PropulsionConfig, PropulsionState
from .sim import HL20Vehicle, simulate, SimResult

__version__ = "0.1.0"
