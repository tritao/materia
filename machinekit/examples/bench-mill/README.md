# Bench mill and mineral-cast frame study

The working `BenchMill` has 250 mm X, 150 mm Y and 250 mm Z travel.
`EnclosedBenchMill` adds its stand, tray, guarding and tending interfaces.
`MillBasePreview` isolates that existing stand; it is not the machine frame.

## Structural frame and mechanical Z axis

`MillFramePreview` derives the working envelope from `BenchMill` and builds
`machinekit.milling.MillFrame`. The legacy machine keeps its independent,
working motion/controller model while the new frame develops separately.

The mill-specific frame uses shared `MineralFoundation` assemblies:

- A lower casting with full-height Y rail beams, a transverse rib, six
  underside-open pockets, a solid rear column support and steel mounting seats.
- A separate column with two rear-open chambers, a central front-open screw
  channel, a thick web behind that channel, and steel rail/bearing-mount seats.
- Four pairs of steel column mounting pads and four underside foot interfaces.
  Every insert has a matching casting recess, avoiding double-counted mass.

`MillFrameLayout` owns the mill layout. `MillZAxisLayout` sizes the central-drive
Z interfaces from the guide, nut, bearing and motor envelopes. The Z rail span
is now 180 mm, compared with the old machine's 110 mm. Its 250 mm stroke and
spindle/work datums are retained; the carriage and spindle head are redesigned.

`MillZAxis` is a mechanical assembly with a real `z` prismatic joint and a coupled
rotating screw. It includes:

- Two rails and four carriage blocks, with a broad steel carriage plate.
- A central ball nut on a gusseted shelf behind the carriage, inside the casting
  channel, with 3 mm nominal clearance to the channel sides and back.
- A ribbed fabricated steel spindle head projecting from the carriage.
- Upper fixed and lower floating bearing units on machined steel crossplates.
- A stationary motor on an open U stand, a coupling, shaft, and spring-applied
  shaft-brake hardware. The stand and bearing plates connect to steel anchors
  in the column; their contact interfaces are checked in CAD.

The new carriage, head, bearing plates and motor stand have registered component
recipes. The Z assembly rebuilds with the standard registry, including its
kinematic joints and lead-screw relation. Shared casting/insert code stays in
`machinekit.structural`; future lathe layouts belong in a separate machine package.

The base is initially 520 mm wide and 220 mm high, with depth determined by the
Y guides and rear column support. The column cross-section is 320 × 260 mm.
Steel seats finish 4 mm above/in front of their castings, with 16 mm embedded
thickness. These and the fabricated-part thicknesses are design assumptions,
not calculated stiffness requirements or validated casting-process minima.

## Inspect and check

From the repository root:

```sh
./haxeon/scripts/haxeon run --project=machinekit/examples/bench-mill/haxeon.frame.json
./app/run-built.sh --project="$PWD/machinekit/examples/bench-mill/build/mill-frame-concept.mtrg" --perspective
```

Outputs in `build/`:

- `mill-frame.mtrg`: castings and steel interfaces alone.
- `mill-z-axis.mtrg`: frame with the actual mechanical Z assembly.
- `mill-frame-concept.mtrg`: that same structure plus frozen X/Y reference
  hardware and the existing spindle drive attached to the moving Z head.
- `mill-frame-concept.json`: travel, dimensions, casting masses, clearance-audit
  status and remaining design work.

The full preview uses `zAxis/z` for the new joint. The spindle, tool and spindle
motor supports follow it; the Z motor stays stationary. X and Y remain frozen
reference arrangements, with their new mounting adapters still to be designed.
The spindle motor shelf/riser come from the existing machine, and their contacts
with the new head and motor are checked. The old Z head, rails, side screw and
mounts are omitted. Preview generation runs the Z structural clearance audit and
refuses output if it fails. This is not a powered CNC/controller integration.

```sh
./haxeon/scripts/haxeon run --project=machinekit/examples/bench-mill/haxeon.z-axis-tests.json
./haxeon/scripts/haxeon run --project=machinekit/examples/bench-mill/haxeon.frame-tests.json
./haxeon/scripts/haxeon run --project=machinekit/tests/casting/haxeon.json
```

`MillZAxisIntegrity` checks conservative CAD swept envelopes over the complete
nominal Z stroke against both castings, every insert, the rails, bearing mounts,
bearings and stationary motor/stand. The envelope decomposition includes the
carriage plate, nut shelf/gussets, nut, head and all four blocks. The head envelope
comes from its actual CAD bounds. This continuous check is separate from the
51-position joint-motion test. A negative control confirms a column without the
channel produces real swept-volume collisions.

Checks also cover connected solids, physical support contact without overlap,
steel/casting partitioning, guide planes and column-pad contact, recipe roundtrips,
coupled screw rotation, and spindle-payload motion after integration.

## Remaining design work

X/Y supports and carriage attachments still need mechanical design. Nominal
bracket holes are modeled, but vendor-specific mounting patterns, tapped insert
holes, installed fasteners, weld details and insert anchorage remain unfinished.
The generic motor does not supply a verified bolt pattern; the stand's explicit
40 mm square pitch is an assumed interface. The modeled brake is not yet bound
to electrical control or simulation interlocks.

The structural Z clearance audit does not qualify cutting-load capacity, bracket
or weld stiffness, whole-machine XYZ clearance, tool/work contact, or overrun
beyond nominal joint limits. Those checks, material qualification and static/modal
analysis precede manufacture. Chip drainage, lubrication access, guards, lifting,
molds and the separate enclosure follow the mechanical layout. Mineral-cast
properties remain the shared assumptions; this study does not claim the pictured
reference machine's UHPC grade.
