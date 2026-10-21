# Drill-tap VMC plan

**Goal.** A third configuration of the mill: a 30-taper drill-tap machining center
that can make our robot-joint housings in house, unattended and cobot-loaded. It
reuses the Compact VMC's mineral-cast frame, cross-slide and Z structure and
replaces what a production machine needs: a high-speed direct-drive spindle, a
tool changer, fast axis drives, a 4th axis, pneumatic workholding, an enclosure
with an automatic door, and probing.

In the editor it machines the reference housing the way the real machine would.
Every check runs against real or catalogue-class parts, so the same model gives
the build package and the Chinese parts list.

**Reference job.** The aluminium T housing of an eRob70T-class joint (Ø73 × 99 mm
drum with a perpendicular stem), about 1,000–2,000 a year, so about 9 a day.
Made from 90 × 90 mm 6061 square bar in two operations, both on the 4th axis:
- **Op 1, A across both axes:** the bar is held by its ends (chuck and tailstock).
  Front bore stack, rear cavity, link socket and most of the outside are cut in one
  clamping.
- **Op 2, A along the stem:** the drum is held by its front register. The 14 radial
  link-screw holes are cut at their index angles, and the two bar-end tabs are
  removed.

The supplier geometry is ZeroErr's `eRob70H××T.STEP`, pinned by the cobot-arm and
zeroerr-arm examples (`zeroerr-arm/reference-layout.json`).

**Scope (2026-10-21).** A modelled reference first; a physical build later. DT0–DT10
are modelling work. DT11 (sourcing and build decision) waits until the model is
complete.

**Not the only route.** Buying a used or Chinese drill-tap center
(Speedio/Robodrill class) is cheaper and quicker for the housings themselves
(DT-D7). This machine is worth building for its own sake: our control stack,
mineral-cast frames, and a repeatable design. A bought machine, if there is one,
is the benchmark it is measured against.

## Targets

| | Compact VMC today | Drill-tap target |
|---|---|---|
| Travel X × Y × Z | 400 × 300 × 380 | 400 × 300 × 380 working, plus Z above it for tool change |
| Table | 700 × 400 T-slot | same |
| Gauge-to-table | 150–530 mm, 100 mm fixture envelope | max ≥ about 550 mm, fixture stack ≤ about 300 mm (set in DT2 from real models) |
| Spindle | BT30 belt cartridge, 2.2 kW / 8,000 rpm study, 1.5 kW motor at 3:1 | **BT30 direct drive, 12–15k rpm, 3–4 kW continuous, orientation, rigid tapping** |
| Tool changer | drawbar and release study only | **about 21 tools** (DT-D4) |
| Axis drives | Z SFU2005 + ELM2H-0750, XY unfitted | 16–20 mm lead ground screws, 30 m/min rapids, about 5 m/s² to start |
| 4th axis | none | bought Ø170 class rotary with brake, air union, tailstock |
| Workholding | AR155-I on manual TS96-155, 60 mm jaw preset | 4th-axis chuck and tailstock (Op 1), front-register bracket (Op 2), AR96 pneumatic plates for other parts, seat sensing |
| Accuracy | not qualified | ±0.005 mm positioning, ±0.003 mm repeatability after pitch compensation; bores H7/K6, about 0.01 mm coaxial |
| Services | motor hoods, written requirements | enclosure, automatic door, way covers, coolant (DT-D5), lubrication, chips |
| Measurement | none | spindle probe, tool setter |
| Automation | vise air ports exposed | door, robot interface signals (machine-tending MT-D2), M-code air valves |

The spindle numbers replace the old belt-drive target. Aluminium needs surface
speed (feed scales with rpm), not power: 2–3 kW already removes more than a
30-taper structure can take.

## Decisions

**DT-D1. Refactor the mill, don't start a new machine.** The frame, casting, guided
cross-slide, Z carriage and head are already parametric. `MillCapacityLayout`
derives them from travel, and they carry continuous sweep, guide-margin, service
and persistence checks. What's missing is the selections that are hard-coded today:
- the Z nut, supports and motor (final fields in `MillZAxisLayout`);
- the XY drive set in `MillXYStageLayout`;
- the belt-only `MillSpindleHead`;
- the fixed head height, fixture height and nose range in `MillCapacityLayout`.

These become layout inputs. Small and Compact VMC keep today's values and outputs.

**DT-D2. Direct-drive BT30 spindle with a servo spindle drive.** Chosen over a belt
drive for:
- speed (12–15k rpm);
- encoder-to-spindle stiffness, which gives orientation for the tool changer and
  rigid-tapping synchronisation;
- smoothness, for the bore finish.

The cost is head height. An ATC electrospindle (18–24k rpm, inverter with encoder
feedback) is the alternative if orientation and tapping prove workable with it.

**DT-D3. 4th axis, not 5 (revised 2026-10-21).** An indexed rotary reaches every
direction perpendicular to its axis.
- **Op 1:** A lies across both the drum and the stem axis, so front, rear and the
  link socket are cut in one clamping. That holds the 39 ± 0.05 socket position
  and the socket-to-drum squareness.
- **Op 2:** A lies along the stem, so all of ZeroErr's radial link screws (11
  counterbored, 3 countersunk) are reachable by indexing.

The harmonic unit and rear cover use axial bolt circles. ZeroErr's link socket is
kept unchanged so the housing stays interchangeable (owner, 2026-10-21). 5-axis
returns only if a later housing needs compound angles.

**DT-D4. 21-tool drill-tap turret (decided 2026-10-21).** It's the fastest changer
and the one this machine class uses. It exchanges with Z motion above the working
stroke, so DT2 must add the exchange height to the column and Z rails. The
12–16-tool umbrella stays the fallback if the turret can't be packaged on the
mineral-cast column.

**DT-D5. MQL first, designed so flood can be added (decided 2026-10-21).**
- MQL + air blast keeps parts dry for the cobot and avoids a coolant-tight cell at
  first.
- The enclosure, way covers, seals, chip path, sump and drain must still be designed
  for flood: tank space, return path, splash height, and a spindle rotary union
  position for through-spindle coolant.
- Chips must leave the bore stack with MQL. DT9 checks that from part orientation
  at each index angle.

**DT-D6. Our own control stack, LinuxCNC as fallback (decided 2026-10-21).**
- The machine runs on the materia stack: CncKit, MotionKit, the RobotKit runtime,
  and EtherCAT servos.
- Everything that leaves the stack (G-code dialect, I/O map, EtherCAT drive
  configuration) stays LinuxCNC-compatible, so LinuxCNC + linuxcnc-ethercat can run
  the same hardware if our runtime isn't ready for a physical build.
- Syntec is no longer a target.

**DT-D7. Building does not block buying.** The plan doesn't decide whether housings
are first made on a bought machine. The targets above match a Speedio/Robodrill
class machine so the two compare directly.

**DT-D8.** One schema version, no compatibility code for old saved mill recipes
(as elsewhere in materia).

## What exists

Surveyed 2026-10-21 on local main c667adae4.

- **Layouts** (`machinekit/haxe/src/machinekit/milling/`):
  - `MillCapacityLayout`: travel-derived table, saddle, rails, column, head datums.
    Constants: `headHeight` 170, `fixtureHeight` 100, `noseMin` 150.
  - `MillFrameLayout`: casting datums.
  - `MillZAxisLayout`: SFU2005, BK15/BF15, ELM2H-0750LA80E, fixed in the class.
  - `MillXYStageLayout`: Small drives; Compact has none.
- **Assemblies:**
  - `MillFrame`, `MillGuidedXYStage`, `MillZAxis`;
  - `MillSpindleHead`: `Bt30Cartridge` (Ø90, 8,000 rpm class), power drawbar,
    compact release cylinder, HTD5M belt, `Bt30SpindleMotor` or `MillSpindleDrivePart`.
- **Checks:**
  - `MillZAxisIntegrity`, `MillXYStageIntegrity`, `MillSpindleIntegrity`;
  - `MillCapacityStudy.main`: 27 poses, continuous swept envelopes, guide margins,
    head-bolt access, motor extraction, persistence.
- **Sizing:**
  - `MillAxisSizing` / `AXIS_SIZING.md`: Leadshine catalogue, screw critical speed,
    rapid screens;
  - `spindle-drive-study/`: 55 Leadshine motors screened against
    `requirements.json` (2.2 kW / 8,000 rpm).
- **Workholding:**
  - `ZppWorkholding`: AR155-I + TS96-155, or CV15075 + TS52-108;
  - supplier STEPs pinned in `tools/suppliers/zppcnc-reference.json`;
  - `WORKHOLDING.md`: Z-stack audit; 60 mm jaw preset only.
  - Pneumatic actuation is unverified (`tools/suppliers/zppcnc-actuation-review.md`).
- **Project:** `materia.project.json` configurations `small` and `compact-vmc`
  (`MillProject.large`), and the `spindle-drive` study.
- **Elsewhere:**
  - `machinekit.hobbing.RotaryTable` (C-axis package, `PrecisionRotaryAxis`);
  - `PRECISION_ROTARY_PLAN.md` (own direct-drive table, later);
  - `examples/enclosure-kit` (extrusion and panel enclosure, sliding leaf);
  - `examples/machine-tending` (arm tends a bench mill; MT-D2 robot interface signal set);
  - `FEM_PLAN.md` and `fem/` (no assembled-machine stiffness result yet).
- **Not available:**
  - no tool changer;
  - no 4th axis on a mill;
  - ToolpathKit defers 4/5-axis tool orientation (`toolpathkit/docs/PLAN.md`);
  - no indexed rotary in CncKit or the stock cutter.

## Phases

Each phase ends with executable checks in the style of `MillCapacityStudy`. Small
and Compact VMC outputs must not change unless a phase says so.

### DT0 — Requirements and reference part
- `drill-tap/requirements.json`: the targets above, each with its source and status
  (assumed or measured).
- `ReferenceHousing`: a parametric machinable T housing. It is checked against the
  supplier eRob70T exterior for envelope and interfaces, and redesigned for
  machining:
  - bores stepping down front to rear;
  - radial holes only around the stem (DT-D3);
  - standard corner radii.
- The inside is **representative**: a size-14 harmonic stack sized from catalogue
  data and labelled assumptions (owner, 2026-10-21). It is a machining reference,
  not an actuator design; a real actuator gets its own plan.
- Setup plan for Op 1 / Op 2:
  - holding tab;
  - stock;
  - the tool list (roughly: face mill, roughing and finishing end mills, ball,
    chamfer, spot, drills, taps or thread mill, O-ring slot cutter, back-chamfer,
    fine-boring tools, probe).
- The spindle target (12–15k rpm, 3–4 kW continuous, direct drive) lives in
  `drill-tap/requirements.json`. `spindle-drive-study` keeps screening the belt
  head of the Small and Compact configurations; DT4 adds a drill-tap section to it.

**Acceptance:** the housing solid and the setup plan exist; the reachability of
every feature is listed per setup and per index angle.

**Findings (2026-10-21), supplier `eRob70H××T.STEP`.** The file has 22 solids in one
flattened product. The housing is solid 8: 275,750 mm³, bounds 73 × 85 × 85 mm.
Axes in the supplier frame:
- **Drum:** axis z, Ø73. Output flange (solid 10) at z 37.5–47; rear cover (solid 21)
  down to z −52, which gives the 99 mm length.
- **Stem:** axis −y, Ø73 with flat sides at x ±36.5.
  - Link socket face at y −39 (the drawing's 39 ± 0.05).
  - Socket Ø58, 9.5 deep, to y −48.5.
  - Radial holes at y −43: 11 × Ø3.3 counterbored Ø5.8, 3 × Ø3.4 at 45°.

The rear cavity (z −45 to −23, boards and brake) is modelled hollow. Everything from
z −23 to the output is a **filled placeholder** with only the Ø18 through bore. There
is no stator bore, gear register or bearing seat to read, so the internal bores have
to come from an internal stack we choose: size-14 strain-wave gear (ratios 50–120),
frameless torque motor, two encoders, friction brake, driver.

**Status (2026-10-21): reference housing and setup sheet done.**
- `ReferenceHousing.hx` holds the part. Each feature has its entry point, tool
  approach, cutter section, tolerance and source (drawing, STEP, catalogue or
  assumed).
- `drill-tap/requirements.json` holds the targets.
- `haxeon.housing.json` runs `ReferenceHousingChecks`, which prints the setup
  sheet. 150 assertions pass:
  - the envelope matches supplier solid 8, within 0.01 mm;
  - the link pattern is ZeroErr's: 11 counterbored and 3 countersunk;
  - every feature lies in its setup's rotary plane and has a clear tool path;
  - walls are at least 3 mm around bores and pockets, and at least 1.5 mm around
    tapped holes.
- **Part:** 195 cm³, about 527 g in 6061.

The inside sizes:

| Feature | Size | Source |
|---|---|---|
| Gear register | Ø66 H7 × 10 | Assumed. Harmonic Drive's CSF-14-2XH unit is Ø73 overall, so a size-14 gear set with its output bearing fits inside the drum. |
| Gear screws | 12 × M3 on Ø58 | Assumed |
| Stator bore | Ø50 H7 × 36.5 | Assumed |
| Rotor bearing seat | Ø42 K6 (6806) | Standard bearing size |
| Rear cavity | Ø56 × 25 | Assumed |
| Cover screws | 6 × M3 on Ø63.5 | Assumed |
| Stem pocket | Ø32 × 8 | Assumed |
| Cable hole to the rear cavity | Ø10 | Assumed |

The rear cavity is smaller than ZeroErr's (their boards are about Ø68), because
the cover screws need walls.

**Op 1** holds the bar by its ends: chuck on one, tailstock centre on the other,
with Ø30 tabs. It uses three index positions:
- **0°, front:** facing, the drum OD contour and the bore stack. The deepest reach
  is the bearing shoulder at 62.5 mm from the stock face; the Ø42 seat is at 59.5 mm.
- **90°, socket:** the socket face, Ø58 G7 socket, stem pocket, cable hole and stem
  contour.
- **180°, rear:** the rear face, cavity, pilot, cover screws and drum OD contour.

**Op 2** holds the drum on the angle bracket, with A along the stem. It uses 14
index positions: 0, 30, 60, 90, 120, 150, 180, 210, 225, 240, 270, 300, 315 and
330°. These cut the link holes and remove the tabs at 0° and 180°.

**Stock:** 90 × 90 × 155 bar, 1,256 cm³. 84 % of it becomes chips (1,060 cm³). DT9
should price this against a near-net sawn blank or a 3D-printed sand mould.

**Tools:**
- Ø50 face mill;
- Ø16 roughing and finishing end mills with 50 mm reach, which the 45 mm OD
  contours need;
- Ø8 finishing end mill;
- Ø6 ball;
- Ø6 90° spot/chamfer tool;
- Ø2.8 and Ø3.3 drills, and a Ø3.4 drill with a countersink;
- Ø5.8 counterbore;
- M3 form tap;
- fine-boring head for Ø42–66, 65 mm reach;
- spindle probe.

That is 12 tools, which fits the 21-tool turret with sister tools.

**DT0 leftovers for later phases:**
- DT6/DT7 check the Op 2 bracket's clearance to the hole tools at 90°.
- DT9 turns the contour and face entries into toolpaths.
- A real actuator would replace the assumed inside sizes.

### DT1 — Refactor: selections become layout inputs
Make these layout inputs, defaulting to today's values:
- `MillZAxisLayout`: screw/nut, supports, motor.
- `MillXYStageLayout` / `MillGuidedXYLayout`: X/Y drive sets (screw, supports, motor,
  coupling or belt).
- `MillCapacityLayout`:
  - head height and nose range derived from the selected spindle package, not constants;
  - fixture stack height as an input;
  - a tool-change height above the working Z.
- `MillSpindleHead`: split into a spindle package interface with today's belt
  package and a direct-drive package (DT4).

**Acceptance:** existing mill, Z, XY, spindle, capacity, workholding and machining
tests pass unchanged; scene and checks outputs for Small and Compact are identical.

### DT2 — Drill-tap configuration and envelope
- `DrillTapLayout` built on `MillCapacityLayout` with the DT targets.
- The `drill-tap` configuration in `materia.project.json`.
- Z stack from real models:
  - AR96-155 (confirm height and holes with ZPP);
  - AR155-I with an 80 mm jaw setup;
  - the 4th axis centre height;
  - the reference block;
  - the longest tool.
- From that, the gauge range, Z rail length, column height and tool-change height.

**Acceptance:**
- the capacity checks (poses, sweeps, guide margins, service access) pass for the
  new configuration;
- every Op 1 / Op 2 feature is reachable with the real tool lengths.

### DT3 — Axis drives
- Ground C5 screws, 16–20 mm lead. Add catalogue data: TBI or Chinese ground-screw
  makers, with nut and support patterns.
- Supports: fixed-fixed or BK/BF.
- XY nut and bearing integration (pending in `CAPACITY_STUDY.md`).
- Leadshine EtherCAT servos sized with `MillAxisSizing` for 30 m/min and the new
  moving masses; Z brake and holding.

**Acceptance:** sizing passes torque, inertia, critical speed and buckling; the
complete races are in the scene; drives are serviceable.

### DT4 — Direct-drive spindle
- BT30 direct-drive cartridge, 12–15k rpm (Taiwanese/Chinese drill-tap class).
- Coupling.
- Spindle servo 3–5.5 kW.
- Drawbar release arrangement: hollow-shaft motor with rear cylinder, or a cylinder
  in the coupling housing. Decide here.
- Orientation and clamp/release feedback (README ATC permissives).
- Rigid-tap synchronisation as a controller relation on the measured spindle angle,
  reusing the hobber/lathe electronic-gearbox work.

**Acceptance:**
- the head fits Z;
- motor removal and release service paths are clear;
- the payload passes `MillSpindleIntegrity`;
- the spindle-drive screen passes with a named motor/drive pair.

### DT5 — Tool changer
- Decide DT-D4.
- Magazine model with real pockets, a tool table for the DT0 tool list, and
  exchange kinematics.
- Sequence and interlocks: zero speed, orientation, exchange position, clamp
  feedback, air, guards.
- Tool-to-fixture and tool-to-magazine clearance for every tool.

**Acceptance:** a full tool change runs in the editor; interlocks reject each unsafe
state; tool-change time is reported.

### DT6 — 4th axis
- Bought rotary and tailstock from supplier STEP.
- A-axis joint with brake and air union.
- Mounting on the table; a TS96 round plate or L-plate on the faceplate.
- Kept separate from `RotaryTable`. Shared code is generalised to
  `machinekit.rotary` only if the precision-rotary plan adopts it.

**Acceptance:** Op 1 reach at every index angle, with swept clearance against
head, door and table travel.

### DT7 — Workholding and table layout
- AR96 pneumatic plates.
- AR155 jaw setup for the block.
- The Op 1 bar holding: rotary chuck on one bar end, tailstock centre in the other.
- The Op 2 fixture: an angle bracket on the faceplate with a spigot into the front
  register and a draw bolt through the drum, stem along A. Check the clearance to
  the hole tools at 90°, and add a bell centre on the socket if needed.
- Seat-sensing and blow-off ports.
- Table layout: the 4th axis carries both operations. A vise station stays as an
  option for other parts.
- Send the ZPP questions:
  - fail-safe on air loss;
  - AP96 studs;
  - sensing ports;
  - AR96 height and holes;
  - jaw lead time.

**Acceptance:** the WORKHOLDING-style Z-stack audit passes for both setups; the
Op 2 fixture locates on the Op 1 register.

### DT8 — Enclosure and services
- Decide DT-D5.
- Enclosure: extend enclosure-kit or a machine-specific sheet-metal design.
- Automatic door.
- Telescopic way covers with compressed and extended envelopes.
- Chip path and auger.
- Coolant or MQL.
- Central lubrication.
- Cabinet, door locking, safety relay.
- The MT-D2 robot interface signal set.

**Acceptance:**
- covers clear at every travel extreme;
- the door opening meets the cobot's loading envelope;
- service access checks pass.

### DT9 — Indexed 3+1 CAM and cutting
- A rotary axis in `MachineBinding`, CncKit (A word, rotated work offsets) and the
  stock cutter.
- An indexed `MillingJob` over setups and index angles.
- The housing program for Op 1 / Op 2.
- Real-time cut in the editor, with a virtual inspection of the bores (diameter,
  coaxiality) against the drawing.

**Acceptance:**
- the housing machines from the block in the editor;
- the cycle time is reported against the 20–40 min target.

### DT10 — Structure, thermal and error budget
- FEM (per `FEM_PLAN.md`) with the new head, spindle and table loads:
  tool-to-work static compliance, modes, the effect of the column height.
- Thermal growth of the spindle and screws.
- An error budget for the bore coaxiality and positioning targets.

**Acceptance:** the budget meets the bore targets, or names what limits them.

### DT11 — BOM, sourcing and build gate
- Decide DT-D6.
- Parts list with Chinese/Taiwanese options and quotes for: spindle and servo, ATC,
  screws and guides, servos, 4th axis, control, probe and setter, coolant/lube/covers,
  chiller, electrics and safety.
- Compare total cost and time against buying a machine (DT-D7).

**Acceptance:** a written build or buy decision with costed BOM.

### DT12 — Tending cell
Bring the drill-tap configuration into `machine-tending`: the cobot loads blocks
into Op 1, moves parts to Op 2, and unloads finished housings through the door,
using the MT-D2 handshake.

**Acceptance:** an unattended multi-part cycle runs in the editor.

## Order

```
DT0 → DT1 → DT2 → DT3 ┐
                  DT4 ┼→ DT5 → DT8 → DT10 → DT11
                  DT6 ┤
                  DT7 ┘
DT9 needs DT2, DT6 and DT7.
DT12 needs DT5, DT7, DT8 and DT9.
```

DT1 is pure refactoring and can start immediately. DT3, DT4, DT6 and DT7 are
independent once DT2 fixes the envelope.

## Open questions for the owner

None at the moment. The housing changes and the representative inside were
approved on 2026-10-21.
