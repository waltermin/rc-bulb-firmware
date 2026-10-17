"""Write web/cri_tables.json: the CIE data the tool page needs for CRI (CIE 13.3-1995),
taken from colour-science (pip install colour-science), 380-780 nm at 5 nm:

  cmf   CIE 1931 2 degree observer (x, y, z)
  tcs   test colour samples TCS01..TCS14 (CIE 1995 set: R1-R8 make Ra, R9 = saturated red)
  d     CIE D-series basis S0, S1, S2 (reference illuminant above 5000 K)

Also writes test/cri_cases.json (colour-science's CRI of standard spectra) for
web/check_cri.mjs. Run once; the outputs are checked in and used by build.py.
"""
import json
import os

import colour
import numpy as np
from colour.colorimetry.datasets.illuminants.sds_d_illuminant_series import \
    SDS_BASIS_FUNCTIONS_CIE_ILLUMINANT_D_SERIES as D_BASIS
from colour.quality.datasets.tcs import SDS_TCS

WL = np.arange(380, 781, 5)


def at(sd):
    return [round(float(v), 7) for v in sd[WL]]


cmf = colour.MSDS_CMFS["CIE 1931 2 Degree Standard Observer"]
out = {
    "source": f"colour-science {colour.__version__}",
    "wl": WL.tolist(),
    "cmf": [[round(float(v), 7) for v in row] for row in cmf[WL].T],
    "tcs": [at(SDS_TCS["CIE 1995"][f"TCS{i:02d}"]) for i in range(1, 15)],
    "d": [at(D_BASIS[k]) for k in ("S0", "S1", "S2")],
}
here = os.path.dirname(os.path.abspath(__file__))
path = os.path.join(here, "cri_tables.json")
json.dump(out, open(path, "w", newline="\n"), separators=(",", ":"))
print(f"wrote {path}")

# Reference answers for check_cri.mjs (run by build.py): colour-science's CRI of a few
# standard spectra on the same grid.
shape = colour.SpectralShape(380, 780, 5)
cases = []
sds = [(f"Planck {T}", colour.sd_blackbody(T, shape)) for T in (2700, 4000)]
sds += [(f"D {T}", colour.sd_CIE_illuminant_D_series(colour.temperature.CCT_to_xy_CIE_D(T)).align(shape))
        for T in (5000, 6500)]
sds += [(k, colour.SDS_ILLUMINANTS[k].copy().align(shape)) for k in ("FL2", "FL7", "FL11")]
for name, sd in sds:
    # rebuild from the 380-780 samples so colour sees exactly what the page sees
    # (passed whole, colour extrapolates the spd out to the TCS range, 360-830 nm)
    vals = [float(v) for v in sd[WL]]
    r = colour.colour_rendering_index(colour.SpectralDistribution(dict(zip(WL, vals))), additional_data=True)
    cases.append({"name": name, "spd": vals, "Ra": float(r.Q_a),
                  "R": [float(r.Q_as[k].Q_a) for k in sorted(r.Q_as)]})
cpath = os.path.join(here, "..", "test", "cri_cases.json")
json.dump(cases, open(cpath, "w", newline="\n"), separators=(",", ":"))
print(f"wrote {os.path.normpath(cpath)}")
