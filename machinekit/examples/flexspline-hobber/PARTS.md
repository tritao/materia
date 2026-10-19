# Hobber reference parts (HB5)

The reusable parts are in `machinekit.motion`, `machinekit.hobbing` and
`machinekit.structural`. These are
reference engineering definitions with simplified CAD envelopes. All supplied rows
are **Assumed**, including dimensions, P4 precision targets, stiffness, runout,
winding constants and torque-speed curves. They are not sourced product ratings.
The reference dimensions can be edited through `ComponentValues` and the registered
recipes. Catalogue claims require an explicit source; error bounds have their own
`errorsBasis`, independently of `ratingsBasis`.

Parts extend `MachineComponent` directly: recipe values and typed parameter access
live there, while `ComponentEnvelopes` provides simple geometry. Engineering
provenance is an optional `EngineeringMetadata` facet; error bounds remain a
`PrecisionErrors` facet. Castings and inserts use the shared structural recipes.
Old HB5 casting recipe IDs load through input migration and save with shared IDs.

## Motion components

| Part | Reference rating | Reference envelope, mm |
|---|---|---|
| `TorqueMotor` | 10 N m continuous, 30 N m peak; 60 rpm rated, 120 rpm max; rotor inertia 0.003 kg m²; cogging amplitude 0.03 N m; 24 poles, assumed cogging order 48 | OD 150, bore 80, length 45 |
| `AngularContactBearing` | P4 target; radial/axial TIR 0.002 mm; moment stiffness 150,000 N m/rad; contact angle 25° | 25 × 62 × 17 |
| `CrossedRollerBearing` | P4 target; radial/axial TIR 0.002 mm; moment stiffness 1,000,000 N m/rad | 80 × 120 × 16 |
| `RotaryEncoderRing` | 2²³ absolute counts/rev; angular error bound 2 arcseconds; eccentricity amplitude 0.001 mm | OD 130, bore 100, thickness 8 |

Torques, speeds and inertias use SI units. CAD dimensions and linear error bounds
use mm. Encoder angular error is radians. Runout means total indicator range;
eccentricity and cogging mean amplitudes. These are bounds, not automatically
injected sinusoids. Their phases and runtime injection belong to HB11.

`TorqueMotor` implements the existing `MotorDrive` interface, including an explicit
assumed winding and torque-speed curve. Direct `jointTorque` is unity: no efficiency
or gearbox is silently inserted. `RotaryEncoderRing` implements `EncoderPart`.
The four `catalog()` methods return generic reference rows with unverified metadata
and empty verified-field lists.

## Hobbing components and packages

- `RotaryTable extends MachineAssembly`: stepped-bore housing, torque motor,
  crossed-roller bearing, ring encoder, clamp and platen. C is continuous and its
  initial effort/speed limits come from the installed motor. Exposes `mount`/`work`.
- `HobSpindle extends MachineAssembly`: housing, direct motor, opposed angular-contact
  pair, hollow shaft, drawbar, retainers, master encoder and removable arbor. The motor is 1.5 kW continuous at 4000 rpm, limited
  to 8000 rpm, with a 0.0002 kg m² rotor. The two bearings remain two instances of
  the same BOM item; opposing orientation is a placement, not another bearing model.
  Exposes `mount`, `gaugeLine`, `hobCentre`. B is continuous.
- `HobArbor`: 16 mm shaft, length 80 mm, 40 mm tool seat, support datum at the far end;
  radial/axial TIR bounds 0.002 mm.
- `OutboardSupport`: 60 × 60 × 75 mm support unit envelope, 32 mm bearing seat,
  16 mm shaft, 0.002 mm radial TIR, assumed 100 MN/m radial stiffness. Bearing
  internals are simplified to the seat bore.
- `Hob`: blank envelope of OD 32, bore 16, length 40 mm; its `design` is an actual
  GearKit conjugate rack/HobDesign. `Hob.fromDesign(design, target)` preserves the
  generating profile, starts, gashes, rake and root height and verifies the recreated
  rake points. Optional detailed CAD adds helical lands, gashes, assumed backing
  relief and an optional keyway. Its display is tessellated from the fused solid;
  the lightweight blank remains available. Cutting uses the unchanged rake regions.
  See [HOB_GEOMETRY.md](HOB_GEOMETRY.md) for assumptions and accuracy limits.
- `ExpandingMandrel`: OD 59, bore 12, length 50 mm, work shoulder at 30 mm, expansion
  0.05 mm, operating/maximum pressures 10/15 MPa. Eccentricity amplitude is 0.004 mm
  and clamping distortion bound 0.002 mm. The expansion is a capacity, not a solved
  deformation or hydraulic circuit. Other cup bores require another recipe size.
- `SwivelHead extends MachineAssembly`: direct servo, crossed-roller support, housing,
  moving head and clamp. A is ±45°, with 15°/s rated and 30°/s maximum speed.
  The static 100 N m clamp rating remains separate from the drive's joint effort.
- `MineralCastBed`: 1100 × 800 × 800 mm monolithic ribbed casting, with steel rail seats,
  a table plate, used drive seat and three levelling-foot anchors as child parts; lifting hardware is deferred.
- `MineralCastColumn`: 400 × 200 × 500 mm ribbed casting and separate steel base,
  rail seats, a side Z-drive seat, lifting inserts, 290 mm Z rail spacing. All precision mounts
  are insert faces, with 0.01 mm assumed position tolerance and ground-after-casting
  finish. Casting BOM rows show native-computed kg; inserts are separate items.
  Optional assumed bulk inputs are 35 GPa Young modulus, loss factor 0.02 (8x an
  assumed cast-iron 0.0025 reference), expansion 14e-6/K and conductivity 2 W/m/K.
  Density is assumed 2400 kg/m³. The hob blank's carbide density is assumed 14,500 kg/m³. Both material
  definitions are available through ProjectKit's material library.

`ASSUMED-TF20` is a custom taper-and-face spindle/arbor interface, replacing the
unimplemented HSK-E25 label. It includes a Ø25 hollow rotating spindle shaft,
front opposed bearing pair, housing shoulder and removable front gland, shaft
locknut, Ø40 nose/flange, 20 mm taper, Ø6 drawbar and two Ø4 drive pins. Shaft,
drawbar, gland and locknut have separate persistent recipes/BOM entries. B drives
the shaft; the removable arbor is fixed to its tool face. Thread helices, fits,
preload and strength qualification remain open manufacturing requirements.
See the dimensioned cartridge specification in MACHINE_DESIGN.md.

No vendor names appear in identifiers or reference row designations. Clamp state,
motor drivers/supplies, hydraulic services and complete X/Y/Z/C/B/A placement are
HB6/HB7 integration work. The packages define their local mechanical joints and
feedback encoders, without claiming an operational controller or clamping interlock.

## Persistence and checks

Each leaf has a registered `ComponentType`. Dimensions, ratings, error bounds and
provenance are normal recipe inputs, so the existing assembly description saves
and reconstructs them without a new parallel wire format. `PrecisionErrors.of(part)`
exposes typed bounds; `PrecisionErrors.inAssembly(assembly)` returns stable sorted
member paths with bounds and provenance, including nested packages. These facts
remain part-owned after save/load. ProjectKit's flattened geometry-only definition
is not an error-source export; runtime error consumption is later work.

`machinekit/tests/hobbing/haxeon.json` runs standalone checks. With no argument it
tests recipes, BOMs, direct-drive limits, winding/encoder data, generating rake
geometry, edited error bounds/provenance and assembly save/load. `geometry` checks
solid volumes, mineral-cast mass and housing-seat interference using native CAD.
The non-geometry group is registered in `MachineKitSmoke` as `HobberPartTests.run`.
It still needs CadKit because the built-in recipe registry constructs some standard
parts while registering their defaults.

Build with `./haxeon/scripts/haxeon build --compiler-only
--project=machinekit/tests/hobbing/haxeon.json`, then run the produced host `main.hl`
with a current CadKit library, OCCT and HashLink runtime on `LD_LIBRARY_PATH`.
Full workspace/native builds and heavy suites use the repository heavy-run wrapper.

## HB6 machine integration

`FlexsplineHobber` is the six-axis reference assembly. X is radial centre distance,
Y shifts along the tilted hob axis, Z feeds along the vertical work axis, A tilts
about the negative world X direction, and B/C rotate the hob/table independently.
The package-preserving IDs are `X`, `Y`, `Z`, `swivel/A`, `spindle/B`, `table/C`.
B has process-velocity ownership, C has a position servo, and A has a spring-applied
100 N m static clamp with release/feedback channels. Electronic gearing and the
program that releases/reapplies A remain HB7 work; there is no B/C mechanical coupling.

Three ground SFU1605 ball-screw/nut drives carry 400 W X/Y and 750 W Z servos, fixed/floating supports,
couplings and separate rotating shafts. The lead and support critical speed enter
the existing transmission resolver. Linear scales provide 10,000 counts/mm
(0.1 micrometre resolution). Six 20 A phase-RMS drives share an assumed 400 V
three-phase rectifier/bus; explicit winding ratings are checked by the existing
motor/driver power model. Ratings, fixtures and travel are design assumptions.

The reference's work shoulder is 115 mm above the table platen (including an integral 40 mm base), allowing end feed
and tilted-tool clearance. Its mandrel is sized from the cup's inner root/wall
geometry; other reference parts rebuild the fixture to match. X's lower soft bound
also accounts for that fixture radius. X retracts to 250 mm, Y is ±10 mm, Z is
−10..190 mm relative to the work shoulder. A's installed range is ±15°; the generic
swivel package still supports its original ±45° envelope. The work table remains fixed; the column moves on bed rails. An explicit +90°
rotation maps work-frame radial X to global Y and the hob axis to global X.

`hobToWork(state)` exports the actual solved mechanical frame in the hob designer's
CAD axes and the workpiece's cylindrical axes. It strips the connector convention
(+Y is the joint axis) rather than accidentally returning that connector rotation.
It is checked against HB3's independent `HobbingAxes.pose()` at all X/Y/Z/A corners
with nonzero B/C. The native bounds establish separating planes from the mandrel
and platen at the same corners. Scene export checks coupled startup coordinates,
separate part masses and all six drives. Precision foundation mounts identify their
steel insert parents. Paired rail faces are also checked directly for coplanarity,
including the case where each face passes its individual position tolerance.

Open `materia.project.json` in this directory, or the **Gear cutting** Start-page
card. It currently opens the mechanical twin; the cutting program is HB8 work.
`machinekit/tests/hobber/haxeon.json` runs the mechanical checks; `scene` also exports
and decodes the scene artifact. The registered app `flexspline-hobber` project-source
case checks project loading, source-derived axis limits and a coarse physical MuJoCo cutting cycle.

## Redesigned machine structure

The example uses registered `HobberSaddle` recipes for the X carriage, vertical
Z saddle, vertical swivel and Y plates, three drive support frames and electrical rack.
The old overhead A-yoke, Y-saddle, X-bridge and Z-saddle recipes are retired.
These extend `MachineComponent` directly. Designed steel weldments remain assumed;
physical contacts, connected solids and recipe/pose round trips are checked.
See [MACHINE_DESIGN.md](MACHINE_DESIGN.md) for datums, load paths and qualification scope.
The generic `HobberBracket` recipe is retained for saved descriptions, but is no
longer used to assemble the machine.

### Centred X transmission and supplier guide exteriors

X uses a central recessed screw deck and nut carrier, rear motor/fixed bearing,
and a separate cast-in channel-floor insert. The bed channel is a shared casting
void, not a machined mineral-cast surface. All twelve carriage blocks select pinned
HGH25CA (X) / HGH15CA (Y/Z) supplier exteriors. X rails use the matching HGR25R
section; Y/Z use the verified HGR15R generator. The supplier recipes persist source
hashes and share the existing rail/top/mount connectors. Assets are prepared by
`linear-rail-reference/prepare.py`; CAD-derived homogeneous-steel mass is assumed.

Rail-seat support revision: the bed has continuous casting beams (110 mm core-free bands, at least 50 mm at the well) below the
50 mm guide inserts, with bed guides spaced 440 mm apart and a matching wider
carriage. The 350 mm chip-well opening is preserved for tray installation. Bed
rail inserts have full-footprint support to the sole; column seats have at least
10 mm backing. The exposed 10 mm steel mounting land is intentional. Fit the
tray and align the rails before installing the moving column/head.


`FlexsplineCupBlank` is the single-piece physical workpiece recipe, including
its open toothed rim, thin body wall and closed end. `MandrelSleeve` is a separate
steel child/BOM part on the mandrel; the mandrel body includes the axial stop
and cavity/recess. The loading scene illustrates radial sleeve take-up only;
pressure deformation and seals remain unqualified.

HB-D11 installs a Ø220 × Ø180 × 20 mm C bearing (the generic component row
above retains its Ø120 reference default), Ø270 housing and Ø240 platen with
an integral hollow hub. Three `machinekit.structural.LevelingFoot` components
provide 47 mm support height; their 15 kN per-foot target is assumed. The
lightened column casting is 31.75 kg. The work-frame-to-machine transform also
raises the assembly 637 mm, putting the tooth band 1000 mm above the floor.
See MACHINE_DESIGN.md and LAYOUT_REPORT.json for current mass/stiffness evidence.
