# Bench mill concept review

Code review of the mineral-cast mill concept (`examples/bench-mill`, `milling/`, `structural/`,
`motion/GuideSupport.hx`). First pass against 765af4ed3 (2026-10-19); every finding re-verified at
7acbd6fb0 (2026-10-20). Paths are relative to `machinekit/`; `milling/`, `structural/` and `motion/`
mean `haxe/src/machinekit/...`, `bench/` means `examples/bench-mill/`. Line numbers are as of
7acbd6fb0.

Rail mounting (lateral references, replication allowance, metrology) is planned separately in
[../../RAIL_MOUNTING.md](../../RAIL_MOUNTING.md); B8 and T1 are fixed there (RM3).

## Structural

- **S1. The small mill is still derived from the legacy `BenchMill`.** Y now comes from `MillYTravel`,
  but Z and the column still come from `BenchMill` solved poses plus literals (+100, −60, +40, 220,
  520, 440, `BenchMill.RAIL`) in `bench/MillFramePreview.hx:20-54`. The large mill already uses the
  parametric `MillCapacityLayout`. Fix: absolute datums in the layouts, no `reference` input; then
  move tending, reference-arms, the app example catalogue and `MachineKitSmoke` onto the concept and
  delete `BenchMill` and its legacy-only parts (`MillCasting`, `SpindleCartridge`, `SpindleMotor`,
  `BenchMillGuideLoads`).
- **S2. No gate runs the concept checks.** None of the `Mill*Checks`, `MillDesignCheckTests` or
  `tests/casting` are in `materia.workspace.json` or `MachineKitSmoke`; `ProjectSourceTests` runs only
  the scene and the CNC job. `MillFemChecks` is called from `tests/SheetRouterFemChecks`, which is not
  in the workspace either. Five configs write to `build/frame-preview`, two to `build/fem-export`.
  Fix: one output dir per config and one gated entry point that runs every check.
- **S3. The checks validate a different mill than the project shows.** `MillProject.smallAssembly`
  (`bench/MillProject.hx:14-16`) uses the HGH15CA guide and the supplier Y servo; the frame, XY,
  Z, spindle, FEM and mold checks build the default HGR15 without it. Only `MillYServoChecks` and
  `MillProject.main` cover the shipped configuration. `MillProject.small()` also builds the layout
  twice. Fix: every check builds through `MillProject`.

## Bugs

| ID | Location | Problem | Fix |
| --- | --- | --- | --- |
| B1 | `milling/MillLevelingFoot.hx:17-23`, `milling/MillFrame.hx:127-153` | The foot nut (z 40–50) sits above a seat with an 18 mm clearance bore (z 20–40): it stops lift but does not carry the machine. `bench/MillFrameChecks.hx:105` only tests contact. `milling/MillCapacityLayout.hx:79` inherits it. | Use the hobber's `LevelingSeat`/`CastingFootPocket` (plate with support and lock nut), the planned single levelling-foot family; check load direction. |
| B2 | `milling/MillSpindleHead.hx:40,58`, `bench/MillSpindleChecks.hx:79-81` | The ±5 mm motor-adjustment endpoints rebuild the belt, and `centres()` rounds to whole teeth, so each endpoint checks a belt about ±2 teeth off the installed one. | Keep the installed belt, move only the motor, check slack within the tension window. |
| B3 | `milling/MillZAxisLayout.hx:131` (also :89) | The head clearance guard uses `SpindleCartridge.DIAMETER` (ER20, 70); the reach uses the BT30 body (90); neither uses the Ø125 flange. | Guard against the installed cartridge's largest radius. |
| B4 | `bench/MillDesignStudy.hx:133-134` | The 20 kg guide-study payload is a fixed world point, not on the table; `MillFemExport` puts it on the table. | Pose it in the table frame and sweep it over the table footprint. |
| B5 | `motion/GuideSupport.hx:58-62,73` | Block load is √(fx²+fy²); HIWIN/THK equivalent load is \|radial\|+\|lateral\| (up to 41 % higher). Reverse-radial and lateral ratings are not separated. | Use the catalogue formula and separate ratings. |
| B6 | `milling/MillSpindleHead.hx:29,31`, `bench/MillSpindleChecks.hx:57-60` | `drawbar-release` and `pistonRod-release` are independent; the rod can sit 4.5 mm inside the drawbar. The check is arithmetic on literals 285/287. | One release coordinate with the 2 mm dead band; derive the values from `MillDrawbarLayout`; check contact geometrically. |
| B7 | `milling/MillXYStageIntegrity.hx:31-34,64-65` | Parts are grouped by id substring; the X sweep omits saddle-carried Y hardware (nutY, nutBracketY and bolts, blockY, blockYBolt). `MillSpindleIntegrity.hx:62-63` reuses the predicates. | Classify by the joint each part descends from. |
| B8 | `milling/MillFrame.hx:89-90, 96-106, 221-223` | Bearing seats and column sole pads are flush with the casting; only the guide seats are proud. `bench/README.md:65` says all are 4 mm proud. | Replication allowance on every precision seat (RAIL_MOUNTING RM3). |

## Checks that prove less than claimed

| ID | Location | Problem |
| --- | --- | --- |
| C1 | `milling/MillXYStageIntegrity.hx:68,96,144` | Sweeps cover the nominal stroke; the declared `xOvertravel`/`yOvertravel` are never swept. `MillSpindleIntegrity.hx:58-63` inherits this. |
| C2 | `milling/MillXYStageIntegrity.hx:207`, `MillSpindleIntegrity.hx:127`, `bench/MillYServoChecks` | Any overlap ≤ 1e-5 mm³ passes, so 0 mm clearance is accepted; README "≥ 3 mm" claims are unchecked. Grow envelopes by the layout clearance or require a distance. |
| C3 | `bench/MillZAxisChecks.hx:140-144` | The negative control places two identical panels at one pose inside `spindle/`; they flag each other, so release space is never exercised. |
| C4 | `milling/MillSpindleIntegrity.hx:43-46, 88, 92, 113-119` | Thread exclusions skip whole part pairs (bolts vs parents, belt vs pulleys, now all `motorNut` parts and motor bolts in the removal check). Subtract the nominal thread volume instead. |
| C5 | `milling/MillZAxisIntegrity.hx:33-41, 58` | Sweep boxes are rebuilt from layout formulas, not posed-part bounds; the fixed list omits screw, coupling, brake and bolts. |
| T1 | `structural/FoundationIntegrity.hx:24-27`, `tests/casting/src/CastingTests.hx:116-120` | Only face normals are compared, so a seat rotated in plane (straightness/parallelism) passes; the negative control is a Z offset only. |

Also: several checks restate the code's own arithmetic (e.g. block-screw engagement 12 − 16); the XY
negative sweep control overlaps the table at rest, so a static check would pass it too.

## Engineering

- **E1. Spindle drive.** Now a Leadshine ELM2L-1500 (5000 rpm max) with a 60T/20T step-up. The belt
  is still 15 mm with unflanged pulleys (`motion/TimingPulley.hx:82-84`); a two-pulley drive needs at
  least one flange. Belt speed is about 13 m/s at the 8000 rpm spindle cap and 25 m/s at motor maximum.
  The rotor joint is capped at a literal 8000 rpm (`MillSpindleHead.hx:85`), so the motor never
  exceeds about 2667 rpm and its own limit is never reached: derive the spindle limit from the drive.
  Check belt power rating with a service factor.
- **E2.** `Bt30SpindleMotor` still has rated speed = maximum speed, but it now only feeds the dead
  `drive == null` path (see L1).
- **E4.** `MillDesignStudy` hard-codes ±1 m/s² (`:122-124,134`); the legacy study takes acceleration
  from `coupledLimits`. A static factor of 2 is low for interrupted milling (catalogues suggest about
  3–5 with impact).
- **E5.** The X motor-plate M6 × 25 tip lands exactly at the bottom of its tapped hole
  (`MillXYStage.hx:54-55` vs `MillCrossSaddle.hx:80-81`): no runout or drill-point allowance.
- **E6. Column.** Rear-open section (torsion roughly 1/35 of a closed box); the 46 mm clamp pocket
  over a 60 mm pad leaves 7 mm ledges carrying the M12 preload (about 70 MPa on mineral casting);
  the clamp pattern is only 140 mm fore/aft on an 833 mm column; base anchors are bond-only blocks in
  tension (`MillFrame.hx:198-208, 221-231`).
- **E7. Head–carriage joint.** 4 × M6 with 84 mm between rows; side windows start at the flange
  (`MillZHead.hx:56`). Bolt positions are now passed head → carriage, but the default 42 is still
  repeated in three places.
- **Resolved:** E3, X/Y servos are catalogue ELM2H-0400LA60F with a 66 mm square pocket and catalogue
  bolt circle.

## Cleanup (no-legacy rule)

- **L1. Dead or superseded code.** `MillYNutBridge` and `MillSpindleMotorSupport` registered but unused
  (`MillingComponents.hx:15,28`); the `drive == null` branch in `MillSpindleHead.hx:38-63` is dead
  (Bt30SpindleMotor, centred motor mount, belt guard, tension screw) because `MillFramePreview.hx:91-92`
  always passes `MillSpindleDrivePart.installed`, yet `MillSpindleChecks.hx:107` still tests the old
  guard; `compactCylinder=false` drawbar default; saddle `sidePockets` (`MillCrossSaddle.hx:23,88`);
  non-flared column and non-pocket feet (`MillFrame.hx:115,142-144`); `centeredMotor=false`
  (`MillZHead.hx:71`).
- **L2. Duplicated construction.** The layout → Z layout → XY layout → `MillFrame` sequence is
  repeated about 11 times (MillFemExport, MillFramePreview, MillFrameChecks, MillXYStageChecks,
  MillSpindleChecks, MillZAxisChecks, MillMotorStudy, MillMoldSetup, MillYServoChecks, MillProject,
  HybridMillPreview). Two guide-load studies (`tests/src/BenchMillGuideLoads.hx`,
  `MillDesignStudy.guideLoads`). Fix with S3: one builder in `MillProject`.
- Magic numbers are copied across files (X-drive coordinates in `MillXYStage`, `MillCrossSaddle` and
  the checks; spindle gauge offsets; the 53.5 bolt circle).

## Suggested order

1. B1–B7 and C1–C3: small, local fixes.
2. S2 and S3: per-config output dirs, every check built through `MillProject`, one gated entry point.
3. S1: absolute datums; then migrate the legacy users and delete `BenchMill`.
4. L1, L2 with S3.
5. B8 and T1 through RAIL_MOUNTING RM3; E1, E4–E7 before FEM results are trusted.
