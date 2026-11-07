# Linear module plan

**Goal.** A MachineKit component for a screw-driven precision linear module, in the style of the HZMotion HZM200:
one ground steel base with raised rail lands, the screw centred on the floor between them, an HZMotion motor
mount at one end and a floating support at the other, a carriage on four blocks, a cover, and a sealed linear
scale built in. The first user is the flexspline hobber's Y axis (hob shift). Status: plan, 2026-11-07.

## Why

The hobber's Y axis is built from loose parts: HGH15 rails on the 15 mm swivel plate, the HZMotion M60D-BK12 mount
and BF12 on two milled L brackets, a nut seat bar, and a JFT-40 on its own seat bar. The screw is off-centre. It
looks improvised, and it carries a known open item: FRAME_PLAN could not centre the Y screw because the M60D
reaches 22 mm below the rail plane and meets the plate. Raised rail lands remove that conflict, because the mount
and the BF then stand on a floor below the rail plane.

FRAME_PLAN rejected *bought* KK modules for Y (the shortest KK86 is 440 mm for a 30 mm need, +5.7 kg on the eRob,
stiffness unpublished). This plan does not reverse that: the module here is ours, sized to the stroke, and can be
machined into a part the host already has.

## Decisions

- **HZMotion drive parts.** The fixed end is an HZMotion integrated mount (`SupportedScrewDrive`: M60D-BK12-C or
  M80E-BK15-C), the floating end a BF unit. No custom motor housing.
- **Integral raised lands, not riser blocks.** The rail lands, the floor for the mount and BF, and the scale seat are
  ground in one setup of one part. A separate riser adds a bolted joint to the rail and screw alignment paths and
  adjusts nothing (parts rule: a part must be structural or a real adjustment/interface element).
- **Land height is derived:** the screw axis meets the nut in the carriage, from the mount's catalogue axis height
  and the nut housing; it is never typed in.
- **Host-integrated mode.** The module can either bring its own base or cut its lands, floor, seats and holes into a
  part the host supplies (for the hobber, the swivel plate). Same recipe and checks either way.
- **The scale belongs to the module.** Its seat, head bracket, travel window, cable and air are module features,
  checked against the vendor's mounting tolerances, not added afterwards by the host.
- **Z stays on the column** (2026-11-07). The Z screw is at y -82 because the eRob housing stands on the centre line,
  the rails sit on cast inserts with no added joint, and the column's insert rows and ground pad bar already act as
  lands and floor. Z gets a bellows cover only (see the hobber's FRAME_PLAN).

## Phases

One worktree, one commit per step, every step keeps the baselines (LM0) and the existing suites green.

### LM0. Baseline

Record the hobber's Y numbers before any change, in the plan's Log:
- Y moving mass, head mass and centre of mass, A-axis tilt moment and torque against the eRob figures;
- accuracy budget rows (`HobberAccuracyBudget`), the Y scale's Abbe offsets (`HobberFrameChecks.scales`);
- integrity, derived fasteners, collisions over Y travel and A ±15°, service removals;
- member count and BOM lines for the Y subtree; cradle FE result if the plate is in its model.

### LM1. Library component

`machinekit.motion.ScrewLinearModule` (name to confirm against the naming audit), with a spec:
- rail family and size (unified linear-rail recipe), rail spacing, blocks per rail, block spacing;
- stroke, overtravel, carriage length and width, carriage hole pattern;
- the `SupportedScrewDrive` (mount, BF, motor, coupling, screw and nut);
- base material and stock (ground steel plate), or a host part for the integrated mode;
- optional sealed scale (LM2) and cover (LM3).

Parts and features:
- **Base:** raised rail lands with a reference shoulder on one side; the floor between them carrying the mount and BF,
  each located by dowels; bolt and dowel pattern underneath; hard stops in line with the carriage.
- **Carriage:** plate on the blocks with the nut housing underneath; its top pattern is the host's interface.
- **Faces** declared so `JointFasteners` derives the screws, dowels and holes; a `ManufacturingRoute` per made part;
  BOM lines; service removals (motor, screw with nut, rails, carriage).

Checks, in a standalone test project with two or three sizes:
- integrity (no overlaps, every part supported) and collisions across the full travel;
- the screw axis meets the nut within the mount's tolerance; supports coaxial by construction;
- the nut and carriage reach the hard stops before the nut meets either support;
- derived fasteners resolve; BOM complete; service paths clear.

### LM2. Linear encoder integration

Using `SealedScaleData` (JFT-40 slim series; mounting tolerances: housing parallelism 0.1, head parallelism 0.05,
head lateral 0.2, gap 1 +- 0.3 mm):
- **Seat:** a face on the base's side, ground in the same setup as the rail lands, so its parallelism to the rails
  is a grinding tolerance, not an assembly adjustment. Its tolerance is checked against `housingParallelism`.
- **Head:** a bracket on the carriage with a declared adjustment (shim or slots) that covers `gap`, `headLateral`
  and `headParallelism`; the check reports the adjustment range against the stack of tolerances.
- **Measuring length:** the smallest standard ML (`standardLength`) for stroke plus 2 x overtravel; the hard stops
  are reached while the head is still inside the window, at both ends.
- **Abbe:** offsets of the scale's reading line from the module's declared functional point (for the hobber: the hob
  centre), reported for every tilt the host declares.
- **Cable and air:** cable exit with `CABLE_BEND_RADIUS` (75 mm) and the straight run respected into the host's
  moving cable path; the purge port (`airThread`) reachable and routed.
- **Signals:** the module exposes the scale as the axis's position feedback and the motor encoder as velocity
  feedback (dual loop over BiSS-C), as ports the host wires to its drive's second encoder input and its BiSS master.
- **Service:** the head and the housing can be removed without moving the rails or the screw.

### LM3. Cover

Pick the cover from the stroke. For short strokes (the hobber's +-15 mm), block wipers plus a fixed top plate that the
carriage skirt overlaps; telescopic or bellows covers only when the stroke needs them. Use the way-cover library
(`machinekit.cover`), its drain and service rules, and the existing wiper rows.

### LM4. Hobber Y conversion

- The swivel plate becomes the module's host (integrated mode): rail lands, a centred M60D-BK12 and BF12 on the floor,
  the scale seat face. If the plate must thicken, record the A-axis effect.
- The Y saddle stays as the carriage: it carries the spindle seat and becomes the module's carriage part, with the
  nut housing and the scale head bracket.
- Remove the two L brackets, the nut seat bar and the Y scale seat bar; their roles move into the module.
- Settle FRAME_PLAN's "Y screw centred between the rails" item with numbers.

### LM5. Verify on the hobber

Against the LM0 baseline:
- hobber checks, integrity, derived fasteners, collisions over Y travel and A ±15°;
- service removals (motor, screw and nut, scale housing and head);
- crash and hard stops;
- accuracy budget, Abbe offsets, A tilt moment and torque against the eRob figures;
- cradle FE if the plate or saddle changed;
- editor screenshots of the Y axis before and after.

### LM6. Documentation and manufacturing

FRAME_PLAN and PARTS updates, `MACHINE_DESIGN.md` Y section, and the machined-part drawings and RFQ rows in
`supplier-packages/machined/` for the new swivel plate and saddle.

## Open

- Whether the swivel plate keeps 15 mm or thickens to take the lands and floor (LM4 decides by numbers).
- The scale's mounting tolerances are datasheet values; the Y scale seat and head adjustment are only as good as them.
- NJPE drawings for the BK12 bracket and BF12 (already requested for X and Z) also cover Y.

## Log
