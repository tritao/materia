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
- `materia.staged.project.json`: snug, then final tightening of installed screws.
- `materia.feeder.project.json`: a bowl and buffered track replenish one pickup
  nest for all four primary screws. Replacement presenters remain separate.
- `materia.threads.project.json` and `materia.fastenings.project.json`: cosmetic
  threads and CAD fastening discovery.

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
