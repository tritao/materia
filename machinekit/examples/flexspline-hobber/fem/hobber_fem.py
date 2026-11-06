#!/usr/bin/env python3
"""Hobber FE checks: hand estimate vs finite element for the B motor cradle, the X carriage and the Z saddle (FRAME_PLAN.md).

Pipeline (all under the shared heavy-run queue; see README in the FRAME_PLAN log line):
  1. export     haxeon run --project fem/haxeon.json -- <dir>        STEP of the four parts + masses and centres (HobberFemExport.hx)
  2. cradle     python3 cradle_fem.py <dir> results/cradle-medium.json medium   (also `foot`, `fine`)
  3. carriage   python3 carriage_fem.py <dir> results/carriage.json 14 10         (+ results/carriage-primitives.json: plate-model geometry)
  4. saddle     python3 saddle_fem.py <dir> results/saddle.json 14 10
  5. report     python3 hobber_fem.py report                           -> results/hobber-fem-results.json and a Markdown table on stdout
Solver: fe.py (C3D10 tetrahedra, scipy sparse, SuperLU / AMG), meshes by gmsh 4.12.2; kernel verified by `tests/test_hobber_fem.py`.
"""
import json
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
R = HERE / "results"

# Hand estimates (FRAME_PLAN.md 2026-11-04 / 2026-11-05 log, HobberFrameChecks.spindleDrive, frame_stiffness.py) and gates.
HAND = {
    "cradle": {"axial_Hz": 1432, "vertical_Hz": 1870, "lateral_Hz": 3017, "unpreloaded_Hz": 940, "gate_Hz": 1248, "band_Hz": [199, 960]},
    # Plate model (frame_stiffness.py) steel 20 mm deck + 25 mm ribs; the sole tilt was 263923.9 before the ty sign fix of 2026-11-06 (see the FRAME_PLAN log).
    "carriage": {"front_tie_N_per_um": 28.9, "sole_tilt_kNm_per_rad": 11751.4, "sole_tilt_flat_kNm_per_rad": 7306.1, "welded_U_front_tie_N_per_um": 3.68, "welded_U_sole_tilt_kNm_per_rad": 6272.8},
    "saddle": {},
}


def load(name):
    p = R / name
    return json.loads(p.read_text()) if p.exists() else None


def cradle_summary():
    rows = load("cradle-medium.json")
    if not rows:
        return None
    out = {"frequencies_Hz": {}, "elements": rows[0]["elements"], "dofs": rows[0]["dofs"]}
    for r in rows:
        out["frequencies_Hz"][f'{r["support"]}/{r["flange"]}/{r["joint"]}'] = [round(x, 1) for x in r["frequencies_Hz"]]
    fine = load("cradle-fine.json")
    if fine:
        out["fine_mesh"] = {f'{r["support"]}/{r["flange"]}/{r["joint"]}': {"size": r["size"], "elements": r["elements"], "frequencies_Hz": [round(x, 1) for x in r["frequencies_Hz"]]} for r in fine}
    foot = load("cradle-foot.json")
    if foot:
        out["cradle_alone_floor_held"] = foot
    return out


def carriage_summary():
    rows = load("carriage.json")
    if not rows:
        return None
    out = {"step": rows, "primitives": load("carriage-primitives.json")}
    last = rows[-1]
    out["front_tie_N_per_um"] = last["front_tie_N_per_um"]
    out["sole_tilt_kNm_per_rad"] = {"pads_uz": last["sole_tilt_kNm_per_rad_pads_uz"], "pads_xyz": last["sole_tilt_kNm_per_rad_pads_xyz"]}
    return out


def build():
    result = {"hand": HAND, "cradle": cradle_summary(), "carriage": carriage_summary(), "saddle": load("saddle.json")}
    (R / "hobber-fem-results.json").write_text(json.dumps(result, indent=1))
    return result


if __name__ == "__main__":
    if sys.argv[1:2] == ["report"]:
        print(json.dumps(build(), indent=1)[:6000])
    else:
        print(__doc__)
