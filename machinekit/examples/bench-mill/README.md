# Bench mill and mineral-cast frame study

The working `BenchMill` has 250 mm X, 150 mm Y and 250 mm Z travel.
`EnclosedBenchMill` adds its stand, tray, guarding and tending interfaces.
`MillBasePreview` isolates that existing stand; it is not the machine frame.

## Structural frame and mechanical XYZ axes

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
is now 180 mm, compared with the old machine's 110 mm. Its 250 mm stroke is
retained. The new cross slide raises the table and spindle work datum by 58 mm;
the column and Z guides extend by the same amount to preserve the envelope.

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

## X/Y cross slide

`MillXYStageLayout` owns the horizontal stack and travel datums. `MillXYStage`
replaces the frozen reference hardware with two nested mechanical axes:

- Y guides on the base carry a 450 × 174 × 80 mm saddle. A 54 mm through-tunnel
  clears the central Y ball nut; a flange bracket attaches it to the rear face.
- X guides on the saddle are 124 mm apart. Four 48 mm block risers carry a
  400 × 200 × 40 mm T-slot table, leaving clearance above the central X drive.
  Its nut bracket has a top foot recessed into the table underside.
- Both screws have fixed/floating bearings on steel pedestals. Two-plate motor
  cages attach to the fixed pedestals; the Y pedestals land on dedicated steel
  casting inserts. X bearings and motor ride with the saddle.
- Both axes use generic 400 W servos and SFU1605 screws, with explicit couplings
  and rotating motor shafts. Generic motor bolt pitch is 30 mm square.

Travel remains 250 mm X and 150 mm Y, centered at zero. Positive work coordinates
move the table along negative world X/Y. Table top is 448 mm above the casting
bottom. Nominal structural clearance is 3 mm. Bearing/nut clearance limits the
modeled overrun reserve to 7 mm X and 5.5 mm Y, below the rail-only reserve.
These are geometric allowances; stops, homing and stopping-distance design are
still required. The 48 mm risers are a packaging choice, pending stiffness review.

The saddle, table, nut brackets, pedestals and motor cages have registered recipes.
Shared `AxisBuilder.slide` can cap guide overtravel for other physical hardware.

## Inspect and check

From the repository root:

```sh
./haxeon/scripts/haxeon run --project=machinekit/examples/bench-mill/haxeon.frame.json
./app/run-built.sh --project="$PWD/machinekit/examples/bench-mill/build/mill-frame-concept.mtrg" --perspective
```

Outputs in `build/`:

- `mill-frame.mtrg`: castings and steel interfaces alone.
- `mill-z-axis.mtrg`: frame with the actual mechanical Z assembly.
- `mill-xy-stage.mtrg`: isolated saddle/table assembly with both drives.
- `mill-frame-concept.mtrg`: the mechanical XYZ mill, with the existing spindle
  drive attached to the moving Z head.
- `mill-frame-concept.json`: travel, dimensions, casting masses, clearance-audit
  status and remaining design work.

The full preview exposes `xyStage/x`, `xyStage/y` and `zAxis/z`. The spindle, tool
and spindle motor supports follow Z. The X motor follows Y, and the Y motor stays
on the base. Preview generation requires both continuous structural audits to pass.
The legacy `BenchMill` controller model remains separate.

```sh
./haxeon/scripts/haxeon run --project=machinekit/examples/bench-mill/haxeon.xy-stage-tests.json
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

`MillXYStageIntegrity` checks continuous X sweeps against the saddle/drive, then
full Cartesian-product X/Y envelopes against the stationary Y drive and all
castings/inserts. Its saddle sweep retains the through-tunnel and is checked
against the actual Y shaft. The head's lower Z structural envelope stays above
the table. Intended guide contact and coaxial screw/nut engagement are treated
separately. A negative control adds an obstruction and must produce a collision.
Tests also exercise 121 X/Y coordinate pairs before and after recipe restoration.

Checks also cover connected solids, physical support contact without overlap,
steel/casting partitioning, guide planes and column-pad contact, recipe roundtrips,
coupled screw rotation, and spindle-payload motion after integration.

## Remaining design work

Nominal bracket holes are modeled, but guide/block attachment drilling,
vendor-specific mounting patterns, tapped insert
holes, installed fasteners, weld details and insert anchorage remain unfinished.
The generic motor does not supply a verified bolt pattern; the stand's explicit
40 mm square pitch is an assumed interface. The modeled brake is not yet bound
to electrical control or simulation interlocks.

The structural Z clearance audit does not qualify cutting-load capacity, bracket
or weld stiffness, spindle-payload clearance, tool/work contact, or overrun
beyond nominal joint limits. Those checks, material qualification and static/modal
analysis precede manufacture. Chip drainage, lubrication access, guards, lifting,
molds and the separate enclosure follow the mechanical layout. Mineral-cast
properties remain the shared assumptions; this study does not claim the pictured
reference machine's UHPC grade.
