# Sheet metal press brake

Open `materia.project.json` from Materia's Start page. The preview shows the assumed
40 t, 1600 mm WC67Y-style machine alone. Its ram, tooling and backgauge are ordinary
assembly members; edit joint coordinates to inspect their motion.

- `Y1` is downward ram travel, 0–100 mm. The torsion bar couples `Y2 = Y1`.
- `depth` positions the independent mechanical depth stop, 0–100 mm downward.
- `X` is backgauge distance behind the punch line, 0–500 mm.
- `R` raises the fingers from the lowest installed die top, 0–120 mm.

The default tooling is a 150 mm straight punch row over a 200 mm V12 die row.
`PressBrake` also accepts a typed `ToolLayout`; holders derive their height from
the installed punch and die heights, preserving an assumed 80 mm open tool gap.
That gap leaves 20 mm of penetration within the nominal ram stroke.

`PressGeometry.of(press)` rebuilds planning dimensions from connector poses,
part bounds and joint limits. Cylinder bodies, moving rods, frame envelopes,
servo ratings and the 40 t capacity are explicit assumptions pending vendor
drawings. Its assumed 48 V servo bus uses the low-voltage winding family,
32 A phase-RMS amplifiers and a 30 A DC source with a 1.5 kW regenerative
shunt. Assembly rebuild checks their simultaneous peak demand; the mechanical
torque-speed curves are unchanged. Frame and demo envelope checks do not rate the tools or certify a
bend sequence. Tool contact and bend-order collisions are checked by the later
single-bend and sequence-planning steps.

Both ram cylinders are bound to their existing guide joints through a shared
closed-centre valve block. Its pressure feed and tank return, and each cylinder's
A/B hoses through two distribution blocks, are assembly service connections included in the BOM. Regulated
pressure is derived from the two 160 mm bore areas to limit their combined
extension force to the assumed 40 t frame capacity.

The reference flow setup is 80/8/70 mm/s approach/working/return, changing at
68.5 mm extension: an explicitly assumed 80 mm gap, 1.5 mm blank and 10 mm
clearance. `PressBrake` accepts a `CylinderPhaseSpeeds` for other setups; it
never infers thickness. Agreeing valve coils close the centre.

Physical switches and the scripted demonstration stroke remain in SM8b. The
editor exposes nominal mechanical travel, including positions beyond a safe
installed-tool stroke; it does not execute a bending cycle.
