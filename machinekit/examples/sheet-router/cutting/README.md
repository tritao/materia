# SR13 cutting inputs

Run `machinekit/examples/sheet-router/cutting/check.sh` from the repository root.
The script reuses existing native libraries, including the retained SR9 compact
StockKit library; it does not rebuild OCCT. Results go below
`build/sheet-router-work/sr13`. Committed `validation.json` records the focused gate.

All chip loads and specific cutting energies below are planning assumptions,
not manufacturer recommendations. The chip-load bands are 3–6 and >6–12 mm;
the second band scales the first by 1.5. Do not extrapolate outside them.

| Material | 3–6 mm min/nominal/max, mm/tooth | Energy, J/mm³ |
|---|---:|---:|
| Birch plywood | .08/.15/.25 | .12 |
| MDF | .08/.15/.25 | .10 |
| PMMA acrylic | .05/.10/.18 | .30 |

The reference bit is a 6 mm, two-flute compression bit, 25 mm cutting length,
60 mm stick-out in an ISO30/ER32 holder, 120 mm gauge length. The simulated
profile includes a shank and a conservative 50 mm diameter holder envelope.
Helix sense is metadata, not a prediction of axial force or chip evacuation.
The assembly's original 12 mm reference cutter remains a clearance envelope;
this recipe does not silently replace the SR7 tool table or a measured H value.

Feed = rpm × flutes × chip load / 1000 (m/min), clamped to 20 m/min
and 24,000 rpm; actual chip load is recalculated after clamps. A value below
the minimum reports melting risk; one above the maximum reports a range warning.
Power = specific energy × axial depth × radial width × feed (mm/s).
Torque = power / angular speed. Mean tangential force = power / cutting speed;
the provisional resultant envelope is twice that mean, an explicit assumption.
It does not model chip thinning, runout, plunge force or local tooth peaks.

The admitted baseline uses 6 mm axial passes in 18 mm sheet, full 6 mm slot
width, 18,000 rpm: plywood/MDF 5.4 m/min; acrylic 3.6 m/min.
The worst assumed envelope is **229.183 N** (acrylic), supplied to SR4 and SR8.
A full-depth acrylic slot at its nominal chip load gives 687.549 N and fails
SR4 drive/preload/accuracy checks. The process admission refuses cuts above
300 N by operation name and asks for reduced depth/feed. This is a recipe
restriction, not a structural redesign. Other operations need their own admitted
engagement/force and hold-down check; no tool diameter alone qualifies a job.

The spindle recipe is 6 kW at 24,000 rpm. Until a supplier curve arrives we use
constant 2.387 Nm below that speed: 4.5 kW at 18,000 rpm. `PlanCheck.spindle`
checks mechanical power AND torque at the commanded speed and names the rejected
operation. Call it for each operation before admitting its motion plan; it does
not infer engagement from trajectory segments. A 25 mm deep acrylic slot at
6.48 m/min needs 4.86 kW and is refused at 18,000 rpm.

Only jerk-limited cutting acceptance remains held for MotionKit OM-R1 and
measured-mode commissioning SR6-C. Cutting acceptance must also qualify the
assumed energy/peak factor, retained tab/onion strength, actual tool/holder/H,
spoilboard skim/overcut reach, and measured spindle curve; numerical connectivity
and virtual overcut alone do not establish these physical properties.
