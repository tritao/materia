# Hobber mechanical layout

The machine uses a rear column, overhead Z/X saddle, side-mounted A cartridge,
underhung Y spindle saddle and a forward outboard bearing. Work and cutter datums
remain compatible with `HobbingAxes`; relocating supports must not move those datums.
All designed steel dimensions and structural properties are assumed engineering
inputs. This is a checked CAD layout, not a released manufacturing drawing set.

## Datums and load path

Coordinates are millimetres, with X radial infeed, Z parallel to the work axis,
and Y along the untilted hob spindle. The table centre is (0, -160); the tooth-band
start is Z=363, above a 40 mm fixture riser. A pivots about X through that cutter-centre line.

| Interface | Geometry |
|---|---|
| Bed | 650 × 750 × 120 mineral cast, with separately listed cast-in steel inserts |
| Column | 200 × 180 × 850, origin (0, 280, 140); front insert faces Y=180 |
| Z guides | Two HGR15 rails, X=±55, axis Y=165, Z=205..955; two blocks per rail, 110 mm longitudinal spacing |
| Z saddle | Block mounting plane Y=152; horizontal X rail mounting deck Z=695 |
| X guides | Two HGR15 rails, Y=±45, X=-80..220, axis Z=710; two blocks per rail, 100 mm longitudinal spacing |
| X bridge | Block mounting face Z=723; closed steel box crossbeam and side upright; swivel mounting plane X=200 before X translation |
| A yoke | Machined mounting face X=90 at the swivel head; overhead roof Z=515..535 |
| Y guide | Inverted HGR15 rail, axis Z=500, Y=-40..-280; two blocks separated by 100 mm |
| Y saddle | Block mounting plane Z=487; two bored Ø90 spindle saddles and an outboard-support hanger |
| Outboard | Housing Y=-245..-275; centred 6002 bearing, 15 × 32 × 9; Ø15 stepped arbor nose |

The rail base and block-top datums are different: HGR15 rail height is 15 mm,
assembled block height 28 mm. Mounts explicitly use these offsets. Four blocks on
Z and X, and two on Y, provide separated supports for carriage moments. Rail checks
include block half-length, 20 mm end margins and the declared 5 mm overtravel.

A 40 mm steel riser lifts the mandrel and the complete cutter/stage datums together.
It keeps the spindle and saddle clear of the table at the full ±15 degree tilt,
without changing the sleeve-to-tooth-band support or cutter-to-work kinematics.

The X bridge passes outside the fixed X deck. Its nut carrier passes through a
longitudinal deck slot. The A/Y swing sits below the overhead X drive tray. The
column sits behind the spindle's rear swept envelope. These are designed clear
lanes; there are no cuts made by subtracting arbitrary neighbouring parts.

## Drives and interfaces

Each screw has fixed/floating bearing seats, a motor mounting plate with shaft
passage, a nut flange seat with screw passage, and a backed linear-scale strip.
X and Y retain assumed 400 W servos. Z uses the assumed 750 W family for the larger
moving structure; its larger shaft has its own mounting passage, coupling clearance and 60 mm input
journal (50 mm on X/Y). `BallScrew` persists this journal dimension and uses it
in geometry and axial compliance. Drive ratings
must be evaluated against the generated physical masses, not the old box model.

The spindle remains at the original gauge line; only the arbor's forward extension
changes. Its 175 mm length ends in a 20 mm long Ø15 bearing journal. The hob stays
at Y=-140..-180. The outboard housing has a 9 mm bearing recess and Ø16.2 shaft
passage, rather than an empty Ø32 through bore. Bearing fits, retention and spindle
preload still require detailed manufacturing design.

Rails, table, column and Z-drive frame land on steel inserts. Finished insert
positions and tolerances remain explicit in the shared structural components.
The casting body is never the precision mounting surface. The larger bed/column
use shared `machinekit.structural` recipes. Legacy HB5 recipes retain their original
foundation dimensions and insert locations when loaded.

The electrical hardware is on a separate floor-supported backplate. Six drives
have 20 mm lateral gaps, with the rectifier below. This rack is an assembly-layout
item; enclosure, cooling, wiring, guards and service access are detailed design work.

## Verification and scope

Run `./machinekit/examples/flexspline-hobber/check-layout.sh` for physical contacts,
connected weldments, full assembly save/reload, guide alignment/end margins,
cutting kinematics and exact-solid audits. `initial` omits the travel grid.
The check reuses native providers without rebuilding them. Paths can be supplied
with `HOBBER_NATIVE_CACHE`, `HOBBER_STOCK_NATIVE` and `HOBBER_LAYOUT_EVIDENCE`.

The exact audit includes fixed neighbours and joint neighbours. Fixed rigid pairs
are intersected once and reuse their invariant common volume; they are never
excluded. The only permitted positive common volume is hob/blank cutting engagement.
Tolerance is 0.001 mm³. Travel samples cover the initial pose and all combinations
of lower/middle/upper X/Y/Z and A=-15/0/+15 degrees, for both gear configurations.
B/C remain at zero in this mechanical grid; rotating detail envelopes need separate
qualification. This is **sampled clearance**, not a continuous swept-volume or tolerance-stack proof.

The raised structure, longer arbor and changed moving masses require new physical
stiffness, gravity-load, modal and thermal assessment before machine acceptance.
Bolt patterns, weld preparation, ground-seat tolerances, support retention, guards,
homing and end stops must be detailed before fabrication. Phase D owns structural/
modal FEA, including the approximately 600 Hz gash excitation; HB14 owns lumped
thermal behaviour. The existing HB4/HB11 numerical cuts keep their frozen artifacts
and original scope; they do not certify this new structure's stiffness.
