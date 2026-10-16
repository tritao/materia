# Robot screwdriver

Open the **Robot screwdriver** example in Materia for the whole-flange cell.
It carries a spindle on a sensed compliant feed, a replaceable hex socket bit
with a vacuum sleeve, a tightening controller, supported screw presenters and
four CAD-derived fastenings. The qualified mission picks, drives and releases
identified screws, with distinct replacement supplies and a reject bin.

The examples in this directory are:

- `materia.project.json`: equipment and idle tool feedback.
- `materia.drive.project.json`: one qualified M6 fastening.
- `materia.assembly.project.json`: all four flange fastenings with recovery.
- `materia.registered.project.json`: six touch measurements locate the flange
  from CAD reference faces before the four named fastenings run.
- `materia.force-search.project.json`: a commissioned force/spiral profile
  searches for screw entry after the CAD-derived approach.
- `materia.staged.project.json`: snug, then final tightening of installed screws.
- `materia.feeder.project.json`: a bowl and buffered track replenish one pickup
  nest for all four primary screws. Replacement presenters remain separate.
- `materia.blow-feed.project.json`: tube-to-tool delivery for the four CAD
  fastenings, with metering, an air pulse and measured receipt at the tool.
- `materia.threads.project.json` and `materia.fastenings.project.json`: cosmetic
  threads and CAD fastening discovery.
- `haxeon.drawing.json`: exports section, end and screw-side SVG drawing views
  from the same CAD artifact and named thread data.

The automatic feeder waits for geometry-backed presence before pickup. It
replenishes only after the tool holds the delivered identity clear of the nest.
Bulk transport is a behavioral simulation with a bounded buffer, explicit
transport/delivery delays and empty/jam diagnostics. It does not simulate
vibration-induced collisions among thousands of loose screws.

The feeder is a custom design envelope, not a vendor model. Its 120 mm bowl,
60 mm bowl height, 15 mm drive height, 115 mm track and 0.5 mm tooling clearance
are explicit design assumptions. The head-supported rail gap and guide height
come from the selected ISO 4762 screw's actual dimensions. The example's two
buffer positions, 0.2 s track delay, 0.3 s delivery delay and 3 s timeout are
simulation inputs, not measured production performance. Carlson and DEPRAG
references in [PLAN.md](PLAN.md) establish the feeding architecture only.

The CAD checks bit fit, torque capacity, on-axis reach and approach clearance.
Tightening guidance comes from the published fastener catalog; qualified angle
windows and joint-response inputs are separate. Process feedback reports
measured torque, angle, speed, stroke, phase, result and vacuum.

A hardware feeder uses `processkit.feeding.DiscreteScrewFeeder` with a
commissioned I/O profile and atomic timestamped acquisitions. Empty/jam inputs
are optional: missing sensors remain unknown. The I/O host polls independently
of planning and owns electrical polarity and its physical output watchdog.

The blow-feed variant adds metering and a separate air pulse, a receiving nosepiece
and measured receipt before direct travel to the hole. Presenter jobs remain
available through an explicit acquisition mode. The tube dimensions, local
mass, pressure, timing, receiving-speed limits and external dress-pack routing
are demonstration/commissioning assumptions recorded in [PLAN.md](PLAN.md).
Touch registration uses the screwdriver's digital TCP contact source and retains its
typed measurement provenance and remaining pose bounds. Vision is deferred
until a calibrated camera reference source is available. The scene format is
v41. The generated touch sensor assumes a noise-free simulation contact; a
hardware project must provide its commissioned calibration bound. The force-
search example carries simulation-only commissioned limits and requires a
fresh measured compression baseline followed by a confirmed force drop; it is
not a hardware safety recipe. SD0–SD14 acceptance evidence and the outstanding
project-wide gate failures are recorded in [PLAN.md](PLAN.md). The SD15 two-arm
pick, place and fasten cell is still being commissioned; its complete cycle and
fault/recovery acceptance remain open.

## Two-arm cell commissioning

`materia.pick-place.project.json` loads a plate and an unfitted flange housing
from sensed trays, fastens the four screws, and unloads the retained assembly
to the outfeed pocket. `materia.housing-handling.project.json` is the earlier
commissioning stage with the plate fixed in its fixture. The housing uses the
same bearing-derived CAD and screw pattern; its bearing is fitted at a later
station.

The handling robot has independently sized fixed A/B grippers for the housing
and plate. The fastening robot has its fixed screwdriver. Each robot has its
own cabinet and program; CAD wires carry `part-ready` and `fastened`. Handling
confirms plate presence and parks before asserting readiness. Fastening requires
an accepted result for every screw and parks before asserting completion.
Handling acknowledges completion before unloading. The accepted screws and
housing retain their identities while following the plate.

The pressure-driven grippers currently need MuJoCo. The deterministic test
backend stores joint effort without integrating it, so it cannot execute the
physical gripping cycle. Its both-backend acceptance requirement remains open
in [PLAN.md](PLAN.md); the missing-housing presence wait and local stop/retry
checks pass on both backends. The full cell remains under commissioning.

CAD and controller-program checks run with:

```sh
./haxeon/scripts/haxeon run --project machinekit/examples/robot-screwdriver/haxeon.pick-place.json
```

The app `screwdriver-pick-place` case runs the commissioning stages and fault
checks. During development, `PICK_PLACE_STAGE` selects `housing`, `full`,
`recovery`, `fixture`, `supply` or `interruption`; `PICK_PLACE_BACKEND` selects
`0` or `1`. The default selection exercises both backends. These selectors do
not turn an unsupported or incomplete scenario into an acceptance pass.
