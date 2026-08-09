"""HL-20 lifting-body aerodynamic model.

Nonlinear 6-DOF aerodynamic model of the HL-20 Personnel Launch System
lifting body. Coefficient data are the published polynomial fits of
wind-tunnel measurements:

    Jackson, E. B., and Cruz, C. L., "Preliminary Subsonic Aerodynamic
    Model for Simulation Studies of the HL-20 Lifting Body,"
    NASA TM-4302, August 1992.

Model validity envelope (per TM-4302):
    * alpha in [-10, +30] deg, beta in [-10, +10] deg
    * subsonic only -- compressibility (Mach) effects neglected
    * control effectiveness linear in deflection angle,
      independent of sideslip
    * rigid vehicle, laterally symmetric

Sign conventions (body axes: x forward, y right, z down):
    CX, CY, CZ : force coefficients along body x, y, z
    Cl, Cm, Cn : rolling, pitching, yawing moment coefficients
    Moments are produced about the TM-4302 moment reference point
    (x_ref = 0.54 * vehicle length from nose) and transferred to the
    current center of gravity in `coefficients()`.

Damping-derivative convention: the tabulated Cmq, Clp, Cnp, Clr, Cnr are
applied to non-dimensional body rates (rate * ref_length / (2 V)), the
standard TM-4302 / stability-derivative convention. Pitch uses the
longitudinal reference length d_ref; roll/yaw use the span b_ref.
This assumption is checked against the reference Simulink implementation
by matlab/cross_validate_hl20.m.
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

D2R = np.pi / 180.0
M2FT = 3.28084
KG2LB = 2.20462


# Geometry and mass properties (TM-4302 / reference simulation values)



@dataclass
class HL20Geometry:
    """HL-20 reference geometry and mass properties (SI units)."""

    S_ref: float = 286.45 / M2FT**2      # reference area [m^2]
    d_ref: float = 28.24 / M2FT          # longitudinal reference length [m]
    b_ref: float = 13.89 / M2FT          # reference span [m]
    len_veh: float = 27.31 / M2FT        # vehicle length [m]
    mass: float = (22932.0 + 2948.0) / KG2LB   # body + fuel [kg]
    x_ref_frac: float = 0.540            # moment reference point, frac of length from nose
    x_cg_frac: float = 0.575             # center of gravity, frac of length from nose

    @property
    def x_ref(self) -> float:
        return self.x_ref_frac * self.len_veh

    @property
    def x_cg(self) -> float:
        return self.x_cg_frac * self.len_veh

    @property
    def inertia(self) -> np.ndarray:
        """Principal inertia tensor [kg m^2].

        Simplified slender-body estimate used by the reference
        simulation (Ixx from disk of area S_ref, Iyy = Izz from rod of
        length d_ref).
        """
        ixx = self.mass * (self.S_ref / np.pi) / 8.0
        iyy = self.mass * (self.d_ref / 2.0) ** 2 / 3.0
        return np.diag([ixx, iyy, iyy])


# ----------------------------------------------------------------------
# NASA TM-4302 polynomial coefficient data
# ----------------------------------------------------------------------
# Datum (control-surfaces-neutral) coefficients.
# Regressor: [1, a, a^2, a^3, a^4, a^5, a^6, |b|, b^2, |b|^3, b^4, a|b|]
# with a = alpha [deg], b = beta [deg].
# Columns of _DATUM_POLY give [term_Z, term_m, term_X]; the published
# sign convention is CZ0 = -term_Z, Cm0 = +term_m, CX0 = -term_X.
_DATUM_POLY = np.array([
    [-9.025e-2,  2.632e-2,  7.362e-2],
    [ 4.070e-2, -2.226e-3, -2.560e-4],
    [ 3.094e-5, -1.859e-5, -2.208e-4],
    [ 1.564e-5,  6.001e-7, -2.262e-6],
    [-1.386e-6,  1.828e-7,  2.966e-7],
    [ 2.545e-8, -9.733e-9, -3.640e-9],
    [-1.189e-10, 1.710e-10, 9.388e-12],
    [ 2.564e-3, -5.233e-4, -5.299e-4],
    [ 8.501e-4,  6.795e-5, -4.709e-4],
    [-1.156e-4, -1.993e-5,  8.572e-5],
    [ 3.416e-6,  1.341e-6, -4.199e-6],
    [-4.862e-4,  6.061e-5,  1.295e-4],
])

# Datum yawing moment Cn0: tabulated over alpha x |beta|, odd in beta.
_ALPHA_CN0 = np.array([-10., -5., 0., 5., 10., 15., 20., 25., 30.])
_BETA_CN0 = np.array([0., 2., 5., 10.])
_CN0_TABLE = 1e-2 * np.array([
    [0.0, 0.46, 1.15, 2.30],
    [0.0, 0.56, 1.40, 2.80],
    [0.0, 1.00, 2.00, 4.00],
    [0.0, 0.50, 1.40, 3.00],
    [0.0, 0.80, 1.80, 3.00],
    [0.0, 0.60, 1.50, 3.00],
    [0.0, 0.70, 1.00, 1.20],
    [0.0, 0.17, 0.53, 0.99],
    [0.0, -0.29, -0.24, 0.02],
])

# Datum side-force / rolling-moment sideslip derivatives [per deg].
_CY_BETA = -0.01242
_CL_BETA = -0.00787

# Control-surface effectiveness polynomials (slope per degree of
# deflection, as polynomial in alpha [deg]).
# Symmetric wing flaps (elevator, de). Regressor [1,a,a^2,a^3,a^4].
# Columns: [term_Z, term_m, term_X]; CZde = -t1, Cmde = t2, CXde = -t3.
_ELEV_POLY = np.array([
    [ 5.140e-3, -1.903e-3, -1.854e-4],
    [ 3.683e-5, -1.593e-5,  2.830e-6],
    [-6.092e-6,  2.611e-6, -6.966e-7],
    [ 2.818e-9,  5.116e-8,  1.323e-7],
    [-2.459e-9, -1.626e-9, -2.758e-9],
])

# Differential wing flaps (ailerons, da). Regressor [1,a,a^2,a^3,a^4].
# Columns: [t_Z, t_m, t_X, t_Y, t_n, t_l];
# CZda=-t1, Cmda=t2, CXda=-t3, CYda=t4, Cnda=t5, Clda=t6.
_AIL_POLY = np.array([
    [-2.503e-4,  1.471e-4,  9.776e-4,  3.357e-3, -2.769e-3,  2.538e-3],
    [ 4.987e-5,  4.673e-5, -2.703e-5, -1.661e-5, -4.377e-5,  1.963e-5],
    [-2.274e-6, -8.282e-6, -8.303e-6, -3.280e-6,  9.952e-6, -3.725e-6],
    [-1.407e-7,  4.891e-7,  6.645e-7,  5.526e-8, -3.642e-7,  3.539e-8],
    [ 5.135e-9, -8.742e-9, -1.273e-8, -3.269e-10, 4.692e-9, -1.778e-10],
])

# Positive body flaps (dfp). Regressor [1, a^2, a^4] (even in alpha).
_BFP_POLY = np.array([
    [ 3.779e-3, -9.896e-4,  1.310e-4],
    [-7.017e-7, -1.494e-9,  1.565e-6],
    [ 1.400e-10, 6.303e-11, -1.542e-9],
])

# Negative body flaps (dfn). Regressor [1,a,a^2,a^3,a^4].
_BFN_POLY = np.array([
    [ 3.711e-3, -1.086e-3, -4.415e-4],
    [-3.547e-5,  1.570e-5, -4.056e-6],
    [-2.706e-6,  4.174e-7, -4.657e-7],
    [ 2.938e-7, -1.133e-7,  0.0],
    [-5.552e-9,  2.723e-9,  0.0],
])

# Differential body flaps (ddf). Regressor [1,a,a^2,a^3,a^4].
# Columns: [t_X, t_Y, t_n, t_l]; note CXddf carries NO sign flip
# (faithful to the published model).
_BFD_POLY = np.array([
    [-6.043e-4,  2.672e-5, -5.107e-5,  7.453e-4],
    [-1.858e-5, -3.849e-5,  1.108e-5, -1.811e-5],
    [ 8.000e-7,  4.564e-7, -1.547e-8, -1.264e-7],
    [-4.845e-8,  1.798e-8, -1.552e-8,  9.972e-8],
    [ 1.360e-9, -4.099e-10, 1.413e-10, -2.684e-9],
])

# All-movable rudder (dr). Regressor [1,a,a^2,a^3,a^4].
# Columns: [t_Z, t_m, t_X, t_Y, t_n, t_l].
_RUD_POLY = np.array([
    [ 5.173e-4, -5.116e-5,  5.812e-4,  1.855e-3, -1.278e-3,  2.260e-4],
    [ 7.359e-5, -1.516e-5,  1.410e-5,  1.128e-5,  1.320e-5, -1.299e-5],
    [-8.270e-7,  1.729e-6, -2.585e-6,  6.069e-6, -4.720e-6,  5.565e-6],
    [-6.034e-7, -2.481e-8,  3.051e-7, -1.780e-7,  2.371e-7, -3.382e-7],
    [ 2.016e-8, -7.867e-10, -8.161e-9, -1.886e-12, -3.340e-9,  6.461e-9],
])

# Dynamic (damping) derivatives, tabulated over alpha [deg], per rad of
# non-dimensional rate.
_ALPHA_DAMP = np.array([0., 5., 10., 12., 14., 16., 18., 20., 22., 25., 30.])
_CMQ = 1e-1 * np.array([-2.03, -1.58, -1.16, -1.76, -1.76, -1.75, -1.74, -2.52, -2.99, -5.71, -8.0])
_CNP = 1e-1 * np.array([3.81, 3.53, 2.19, 2.44, 1.79, 1.67, 2.01, 2.22, 3.07, 3.79, 8.50])
_CLP = 1e-1 * np.array([-4.98, -6.0, -3.98, -5.79, -4.25, -4.99, -6.49, -6.19, -8.10, -8.72, -24.10])
_CNR = 1e-1 * np.array([-7.94, -8.37, -9.21, -9.22, -8.67, -9.22, -9.46, -10.07, -10.90, -12.86, -20.41])
_CLR = 1e-1 * np.array([4.96, 5.98, 8.40, 8.06, 6.86, 5.72, 6.03, 8.99, 9.57, 13.57, 44.90])

ALPHA_MIN_DEG, ALPHA_MAX_DEG = -10.0, 30.0
BETA_MIN_DEG, BETA_MAX_DEG = -10.0, 10.0


@dataclass
class ControlDeflections:
    """Aero-effect control deflections [deg].

    da  : differential wing flap (aileron)
    de  : symmetric wing flap (elevator)
    dr  : all-movable rudder
    dfp : symmetric positive body flap (0 .. +60)
    dfn : symmetric negative body flap (-60 .. 0, passed as its
          deflection magnitude's signed value; effectiveness polynomial
          is defined for the deflection as commanded)
    ddf : differential body flap
    """

    da: float = 0.0
    de: float = 0.0
    dr: float = 0.0
    dfp: float = 0.0
    dfn: float = 0.0
    ddf: float = 0.0

    def as_array(self) -> np.ndarray:
        return np.array([self.da, self.de, self.dr, self.dfp, self.dfn, self.ddf])


def _datum_regressor(alpha_deg: float, beta_deg: float) -> np.ndarray:
    a, b = alpha_deg, beta_deg
    return np.array([
        1.0, a, a**2, a**3, a**4, a**5, a**6,
        abs(b), b**2, abs(b) ** 3, b**4, a * abs(b),
    ])


def _alpha_regressor(alpha_deg: float) -> np.ndarray:
    a = alpha_deg
    return np.array([1.0, a, a**2, a**3, a**4])


def _interp_cn0(alpha_deg: float, beta_deg: float) -> float:
    """Bilinear interpolation of the Cn0 table; odd extension in beta."""
    a = np.clip(alpha_deg, _ALPHA_CN0[0], _ALPHA_CN0[-1])
    babs = np.clip(abs(beta_deg), _BETA_CN0[0], _BETA_CN0[-1])
    ia = np.clip(np.searchsorted(_ALPHA_CN0, a) - 1, 0, len(_ALPHA_CN0) - 2)
    ib = np.clip(np.searchsorted(_BETA_CN0, babs) - 1, 0, len(_BETA_CN0) - 2)
    ta = (a - _ALPHA_CN0[ia]) / (_ALPHA_CN0[ia + 1] - _ALPHA_CN0[ia])
    tb = (babs - _BETA_CN0[ib]) / (_BETA_CN0[ib + 1] - _BETA_CN0[ib])
    c00 = _CN0_TABLE[ia, ib]
    c10 = _CN0_TABLE[ia + 1, ib]
    c01 = _CN0_TABLE[ia, ib + 1]
    c11 = _CN0_TABLE[ia + 1, ib + 1]
    val = (c00 * (1 - ta) * (1 - tb) + c10 * ta * (1 - tb)
           + c01 * (1 - ta) * tb + c11 * ta * tb)
    return float(np.sign(beta_deg) * val)


def _interp_damping(alpha_deg: float) -> tuple[float, float, float, float, float]:
    a = np.clip(alpha_deg, _ALPHA_DAMP[0], _ALPHA_DAMP[-1])
    return tuple(
        float(np.interp(a, _ALPHA_DAMP, tab))
        for tab in (_CMQ, _CNP, _CLP, _CNR, _CLR)
    )


@dataclass
class HL20Aero:
    """Evaluates HL-20 aerodynamic coefficients, forces and moments."""

    geom: HL20Geometry = field(default_factory=HL20Geometry)

    def datum(self, alpha_deg: float, beta_deg: float) -> dict:
        """Datum (controls-neutral) coefficients at the reference point."""
        t = _datum_regressor(alpha_deg, beta_deg) @ _DATUM_POLY
        return {
            "CX": -t[2],
            "CY": _CY_BETA * beta_deg,
            "CZ": -t[0],
            "Cl": _CL_BETA * beta_deg,
            "Cm": t[1],
            "Cn": _interp_cn0(alpha_deg, beta_deg),
        }

    def control_increments(self, alpha_deg: float, d: ControlDeflections) -> dict:
        """Control-surface coefficient increments at the reference point."""
        ra = _alpha_regressor(alpha_deg)

        te = ra @ _ELEV_POLY
        ta = ra @ _AIL_POLY
        tn = ra @ _BFN_POLY
        td = ra @ _BFD_POLY
        tr = ra @ _RUD_POLY
        a = alpha_deg
        tp = np.array([1.0, a**2, a**4]) @ _BFP_POLY

        dCX = (-te[2] * d.de) + (-ta[2] * d.da) + (-tp[2] * d.dfp) \
            + (-tn[2] * d.dfn) + (td[0] * d.ddf) + (-tr[2] * d.dr)
        dCY = (ta[3] * d.da) + (td[1] * d.ddf) + (tr[3] * d.dr)
        dCZ = (-te[0] * d.de) + (-ta[0] * d.da) + (-tp[0] * d.dfp) \
            + (-tn[0] * d.dfn) + (-tr[0] * d.dr)
        dCl = (ta[5] * d.da) + (td[3] * d.ddf) + (tr[5] * d.dr)
        dCm = (te[1] * d.de) + (ta[1] * d.da) + (tp[1] * d.dfp) \
            + (tn[1] * d.dfn) + (tr[1] * d.dr)
        dCn = (ta[4] * d.da) + (td[2] * d.ddf) + (tr[4] * d.dr)
        return {"CX": dCX, "CY": dCY, "CZ": dCZ, "Cl": dCl, "Cm": dCm, "Cn": dCn}

    def coefficients(
        self,
        alpha_deg: float,
        beta_deg: float,
        d: ControlDeflections,
        p: float = 0.0,
        q: float = 0.0,
        r: float = 0.0,
        V: float = 1.0,
    ) -> dict:
        """Total coefficients about the center of gravity.

        p, q, r are body rates [rad/s]; V is true airspeed [m/s].
        """
        c0 = self.datum(alpha_deg, beta_deg)
        dc = self.control_increments(alpha_deg, d)
        cx = c0["CX"] + dc["CX"]
        cy = c0["CY"] + dc["CY"]
        cz = c0["CZ"] + dc["CZ"]
        cl = c0["Cl"] + dc["Cl"]
        cm = c0["Cm"] + dc["Cm"]
        cn = c0["Cn"] + dc["Cn"]

        # Rate damping (non-dimensional rates).
        if V > 1.0:
            cmq, cnp, clp, cnr, clr = _interp_damping(alpha_deg)
            qhat = q * self.geom.d_ref / (2.0 * V)
            phat = p * self.geom.b_ref / (2.0 * V)
            rhat = r * self.geom.b_ref / (2.0 * V)
            cm += cmq * qhat
            cl += clp * phat + clr * rhat
            cn += cnp * phat + cnr * rhat

        # Transfer moments from the TM-4302 reference point to the CG.
        # Reference point is forward of the CG by dx (body +x forward).
        dx = self.geom.x_cg - self.geom.x_ref   # > 0: ref fwd of cg
        cm += -(dx / self.geom.d_ref) * cz
        cn += +(dx / self.geom.b_ref) * cy
        # (Sign detail: M_cg = M_ref + r_(cg->ref) x F, r = [+dx, 0, 0];
        #  dMy = -dx * Fz -> dCm = -dx*CZ/d_ref; dMz = +dx * Fy.)

        return {"CX": cx, "CY": cy, "CZ": cz, "Cl": cl, "Cm": cm, "Cn": cn}

    def forces_moments(
        self,
        alpha_deg: float,
        beta_deg: float,
        d: ControlDeflections,
        qbar: float,
        p: float = 0.0,
        q: float = 0.0,
        r: float = 0.0,
        V: float = 1.0,
    ) -> tuple[np.ndarray, np.ndarray]:
        """Body-axis aerodynamic force [N] and moment about CG [N m]."""
        c = self.coefficients(alpha_deg, beta_deg, d, p, q, r, V)
        S = self.geom.S_ref
        F = qbar * S * np.array([c["CX"], c["CY"], c["CZ"]])
        M = qbar * S * np.array([
            c["Cl"] * self.geom.b_ref,
            c["Cm"] * self.geom.d_ref,
            c["Cn"] * self.geom.b_ref,
        ])
        return F, M
