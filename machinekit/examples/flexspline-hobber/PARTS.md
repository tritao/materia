# Hobber reference parts (HB5)

The reusable parts are in `machinekit.motion` and `machinekit.hobbing`. These are
reference engineering definitions with simplified CAD envelopes. All supplied rows
are **Assumed**, including dimensions, P4 precision targets, stiffness, runout,
winding constants and torque-speed curves. They are not sourced product ratings.
The reference dimensions can be edited through `ComponentValues` and the registered
recipes. Catalogue claims require an explicit source; error bounds have their own
`errorsBasis`, independently of `ratingsBasis`.

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
  pair, master encoder and arbor. The motor is 1.5 kW continuous at 4000 rpm, limited
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
  rake points. The CAD blank does not represent helical teeth, relief or sharpening;
  cutting continues to use GearKit's actual rake regions.
- `ExpandingMandrel`: OD 59, bore 12, length 50 mm, work shoulder at 30 mm, expansion
  0.05 mm, operating/maximum pressures 10/15 MPa. Eccentricity amplitude is 0.004 mm
  and clamping distortion bound 0.002 mm. The expansion is a capacity, not a solved
  deformation or hydraulic circuit. Other cup bores require another recipe size.
- `SwivelHead extends MachineAssembly`: direct servo, crossed-roller support, housing,
  moving head and clamp. A is ±45°, with 15°/s rated and 30°/s maximum speed.
  The static 100 N m clamp rating remains separate from the drive's joint effort.
- `MineralCastBed`: 650 × 550 × 120 mm, table and column datums.
- `Column`: 180 × 160 × 600 mm, 110 mm rail spacing. Mineral-cast density is assumed
  2400 kg/m³. The hob blank's carbide density is assumed 14,500 kg/m³. Both material
  definitions are available through ProjectKit's material library.

`HSK-E25` identifies the spindle/arbor gauge interface. Type E is covered by
[DIN 69893-5](https://www.dinmedia.de/en/standard/din-69893-5/378149482), and
[the manufacturer's interface listing](https://shop.haimer.com/de/Werkzeugaufnahmen/HSK-Hohlschaftkegel-DIN-69893/HSK-E25/)
confirms the E25 designation. These sources establish the interface name only.
The current cylinder envelopes and gauge frames do not implement a conforming taper,
drawbar, clamping mechanism or manufacturing interface. The choice of E25 is assumed.

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
