"""Generate lookup-table CSVs from the TM-4302 polynomial model.

The embedded C implementation (Phase 3) uses fixed-grid lookup tables
with linear interpolation instead of runtime polynomial evaluation.
This script generates those tables from the same polynomial source used
by the Python model, guaranteeing both implementations share one data
origin. Output: data/hl20_aero/*.csv
"""

import sys
import pathlib

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))

from hlgnc.aero import HL20Aero, ControlDeflections

OUT = pathlib.Path(__file__).resolve().parents[2] / "data" / "hl20_aero"

ALPHA = np.arange(-10.0, 30.0 + 0.5, 1.0)
BETA = np.arange(-10.0, 10.0 + 0.5, 1.0)


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    aero = HL20Aero()

    hdr = ("Generated from NASA TM-4302 polynomial model "
           "(Jackson & Cruz 1992) by generate_aero_tables.py")

    # Datum coefficient grids over (alpha, beta).
    for key in ("CX", "CY", "CZ", "Cl", "Cm", "Cn"):
        grid = np.array([[aero.datum(a, b)[key] for b in BETA] for a in ALPHA])
        path = OUT / f"datum_{key}.csv"
        np.savetxt(path, grid, delimiter=",",
                   header=f"{hdr}\nrows: alpha {ALPHA[0]}..{ALPHA[-1]} deg, "
                          f"cols: beta {BETA[0]}..{BETA[-1]} deg")

    # Control effectiveness (per degree of deflection) vs alpha.
    surfaces = {"de": "de", "da": "da", "dr": "dr",
                "dfp": "dfp", "dfn": "dfn", "ddf": "ddf"}
    rows = []
    for a in ALPHA:
        row = [a]
        for name in surfaces:
            d = ControlDeflections(**{name: 1.0})
            inc = aero.control_increments(a, d)
            row += [inc[k] for k in ("CX", "CY", "CZ", "Cl", "Cm", "Cn")]
        rows.append(row)
    cols = ["alpha_deg"] + [f"{s}_{c}" for s in surfaces
                            for c in ("CX", "CY", "CZ", "Cl", "Cm", "Cn")]
    np.savetxt(OUT / "control_effectiveness.csv", np.array(rows),
               delimiter=",", header=hdr + "\n" + ",".join(cols))

    np.savetxt(OUT / "breakpoints_alpha.csv", ALPHA, delimiter=",", header=hdr)
    np.savetxt(OUT / "breakpoints_beta.csv", BETA, delimiter=",", header=hdr)
    print(f"Wrote tables to {OUT}")


if __name__ == "__main__":
    main()
