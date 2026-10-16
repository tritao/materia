# Archived arm experiments — do not extend

The user's frozen, trusted baseline is the supplier ZeroErr 5 kg STEP assembly:
110T at J1–J3, 70T at J4–J6. Its 5 kg rating is the premise for reconciling our
model. Geometry changes are stopped. CNC tending remains deferred.

These experiments remain in version control for historical reproducibility only:

| Experiment | Preserved sources | Last relevant comparison |
| --- | --- | --- |
| 80T wrist conversion | `revised-layout.json`, `revised-engineering-config.json`, `prepare-revised.py` | Existing revised build and earlier engineering reports |
| Straight tube / adapter / cover variants | `straight-link-config.json`, `straight-engineering-overrides.json`, `prepare-straight.py`, straight review scripts | Existing straight build / historical comparison |
| 142T / 170I resize concepts | `arm-options-config.json`, `arm-options-study.py`, `ARM-OPTIONS.md` | `c98a5f524` |

The larger-module and restricted-payload recommendations in those reports are
superseded. Their shell-distributed mass assumptions and universal rated-torque
screen do not disprove the supplier's arm. Do not regenerate or extend these
experiments as active design work. Source files/artifacts are retained rather
than moved, so historical links and source hashes continue to resolve.

Active work: `reference-baseline.json`, `reference_dynamics.py`,
`REFERENCE-RECONCILIATION.md`, supplier confirmation, build package, controls,
single-joint bench preparation and independent commercial-reference validation.
