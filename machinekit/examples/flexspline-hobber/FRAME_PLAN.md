# Hobber frames, drives and feedback: plan

Started 2026-11-03 from a review of the drive frames and weldments. Dimensions in mm.

## Why

A review of `HobberSaddle` and `FlexsplineHobber` found:

- The Y drive frame (`GE-Y-drive`) hangs on one 20 x 20 block resting on the swivel plate's top edge; the
  same block rests on the moving Y saddle.
- Moving parts sit flat on fixed ones with zero clearance: the X nut bracket bottom (z 155) on the X drive
  deck top, and the Y nut bracket on the Y drive plate (x 45) and the swivel plate top (z 95).
- The Y nut bracket hits the BF12 housing at Y = +10 (about 2 mm) and +15 (about 7 mm) after the 170 mm Y
  screw: `HobberYLayout` measured to the nut flange, not to the 17.5 mm bracket behind it.
- The rear spindle cradle and the Y nut bracket are joined by small face contacts; motor plates are 5 mm with
  only a shaft hole (no pilot bore, no bolt holes).
- `HobberInterferenceChecks` flags only overlap volume, so face contacts and tiny joins pass, and it already
  reports 412 known overlaps that hide new ones.
- The scales are exposed tapes sized to the screw length, with no reading head.

## Decisions

- **Construction:** bolted and dowel-pinned parts, no welding. Ground plate where a face only needs to be
  flat, ground flat bar for blocks and spacers, laser-cut plate for seats and covers. CNC milling from a
  machine shop (China) is acceptable where one milled part replaces several bolted plates or gives a precise
  face more simply. Rules for milled parts: within about 600 mm, at most two setups on a 3-axis mill, tight
  tolerances only on functional faces, no weld-then-machine.
- **Screw drives (X, Y, Z):** SFU1605, catalogue BK12 motor bracket (HZMotion M60D-BK12-C type) with BF12 and a
  Leadshine 400 W servo, braked on Z (`SupportedScrewDrive.m60dBk12()` parts). Screw supports aligned to the nut at
  assembly, then pinned; their seats need drilled and tapped holes only. Z falls back to BK15 + SFU2005 if the
  400 W sizing check fails.
- **Nuts:** a simple steel nut block (flat bar or small milled block) per axis. No DSG housings (usually
  aluminium, 20 mm centre height against the BK12's 25).
- **A axis:** ZeroErr eRob110I V6, revision D (MC2E.DX), EtherCAT, ratio 100, 20-bit output encoder
  (±7/±15″), brake, 48 V bus. CAD: `build/suppliers/zeroerr/` (git-ignored).
- **Scales:** Yuheng JFT-40 (slim, sealed, absolute, BiSS-C): X ML 120 ±3 µm, Z ML 270 ±5 µm, Y ML 70 ±5 µm,
  read by the controller. Datasheets: `build/suppliers/yuheng/` (git-ignored).
- **Spindle:** bought Ø80 g6 cartridge with the maker's outboard support. No standard flange-mount Ø80 cartridge was
  found (R0), so the seat is one milled split-clamp block for the Ø80 housing; ask the spindle maker for a flange anyway.
- **Y:** two rails + SFU1605 screw with the standard bracket and a nut block (R0). Bought KK modules rejected: the
  shortest KK86 is 440 mm long (210 stroke for a 30 mm need) on a 230 mm head, +5.7 kg on the eRob, stiffness
  unpublished; KK60 cannot take the 400 W servo.
- **X carriage:** one milled steel part, 20 mm deck with 25 mm ribs, U open at the front (H1: a front bar meets the Y
  guides at A ±15°; a thicker deck needs the X rails lower).

## Open

- eRob output bearing moment rating and brake holding torque (asked from ZeroErr).
- Servo voltage class: the ELM2H is a 220 V motor, the rack is 400 V; the model uses an assumed winding (R4).
- Thicker X carriage deck: needs the X rails 10-20 mm lower on the bed inserts (user decision; today 20 mm + ribs).

## Phase L: MachineKit library

- **L1. Integrity checks** (`AssemblyIntegrity`): minimum running gap between bodies that move relative to
  each other (default 1 mm, configurable) over the sampled sweep; minimum contact area for fixed mates; every
  flanged part fixed to a parent must find its `mountingCutout` in that parent. Unit tests reproduce a rubbing
  face, a 20 x 20 tab and a plate with only a shaft hole, and each fails.
- **L2. Screw drive in `motion/`:** move `MillScrewDrive` out of `milling/`, add placement (axis start,
  direction, mounting side) of bracket, BF12, ring, coupling, servo and nut from catalogue datums, with
  clearance holes and dowel locations. The bench mill keeps its poses.
- **L3. Ball-nut mounting:** `mountingCutout` for the ball nut; a flat-bar nut-block part.
- **L4. Manufacturing route per part:** ground plate and ground flat bar stock rows (standard thicknesses,
  grinding tolerances); each part tagged ground stock / cut plate / milled (setups, tolerance class) /
  turned, with machined faces flagged, so the parts list shows what goes to which supplier.
- **L5. eRob family:** flanges and spigots from the drawing, ratio rows, encoder grades, brake, 48 V. Bearing
  moment rating and brake holding torque marked assumed.
- **L6. Sealed scale family:** housing plus reading head, overall length, mounting tolerances, BiSS-C; JFT-40
  and JFT-10 rows from the datasheets.
- **L7. Servos:** confirm the braked `ELM2H-0400LA60E` row; catalogue servos the default over `GENERIC-*`.
- **L8. Electrical:** 48 V supply able to absorb regenerated energy; BiSS master on the controller.

## Phase H: hobber on today's head

- **H1. X and Z structure:** X carriage on one ground plate (closed U if it fits), Z saddle on ground plate or
  one milled part, drive seats on cut plate. Rough stiffness of the X carriage and the column base; Z 400 W
  sizing check.
- **H2. Drives:** L2 drives and L3 nut blocks on X, Y and Z, at least 2 mm running clearance.
- **H3. Y screw length:** set against the BF12 at ±15 mm overtravel; update `HobberYLayout`,
  `supplier-packages/ballscrew_package.py` and its README; regenerate the Y drawing.
- **H4. Scales:** JFT-40 on X and Z, housing on the fixed part, reading head on the carriage, Abbe offset
  reported per axis.
- **H5. Integrity:** `AssemblyIntegrity` on the hobber; clear the 412 known overlaps; fix "Collision hulls
  disagree… Xbridge"; reduce `HobberInterferenceChecks` to cutting-zone cases. Mechanical checks pass end to end.
- **H6. Minimum Y fixes** so today's Y head passes L1 (replaced in R).
- **H7. Assembly procedures:** align screw supports to the nut and pin; rails against an edge.
- **H8. Machined-parts package:** STEP and PDF drawing per milled or turned part (functional tolerances,
  material, finish), generated like the other `supplier-packages/`.

## Phase R: head redesign (after ZeroErr replies)

- **R0.** ~~Y module check~~ done: rails + screw (see Decisions).
- **R1.** eRob swivel: housing seated on the Z saddle, swivel plate on the output flange; torque motor,
  crossed roller and clamp removed.
- **R2.** Head: swivel plate (ground or milled with rail shoulder), Y as module or rails + drive, spindle seat
  for the flange cartridge, maker's outboard support, JFT-40 ML 70 on Y.
- **R3.** Head mass and centre of mass against the eRob bearing moment rating and brake holding torque.
- **R4.** Electrical: 48 V supply; A, scales and C table on EtherCAT/BiSS; remove A's EL8 row; layout and BOM.

## Phase F: follow-ups

- **F1.** Remaining generic parts (spindle servo, couplings) to catalogue rows as quotes arrive.
- **F2.** Docs: `MACHINE_DESIGN.md` and `LAYOUT_REPORT` (stale 836 kg), `Z_DRIVE_REPORT.json`, `PARTS.md`.

## Suppliers (outside the model)

- ZeroErr: bearing moment rating, brake holding torque, 48 V supply, configuration, price.
- NJPE: BK12 motor brackets (60-frame, 80-frame option), BF12, preloaded BK12 bearing pair, nut flange drawing;
  revised Y screw drawing after H3.
- Yuheng: JFT-40 at X 120 / Z 270 / Y 70, BiSS-C, air connection, STEP files.

## Order

L1-L4, H1-H8, L5-L8, then R0-R4 once ZeroErr replies, then F. One worktree, one commit per step, merged to local
main at the end of each phase.

## Log

- 2026-11-03: plan written.
- 2026-11-03: L1 done (worktree hobber-frames): `AssemblyIntegrity` running gap, contact area and flange cut-out checks, opt-in via `strict`; `MountingFace`; tests in `AssemblyIntegrityTests`.
- 2026-11-03: L2 done: `MillScrewDrive` moved to `motion/SupportedScrewDrive` (placement from axis start, direction and mounting side; seat holes and dowels); the bench mill's Y and Z drive poses are reproduced by `poses` (`MillScrewDrivePlacementChecks`).
- 2026-11-03: L4 done: `GroundStock` (ground plate and flat bar rows, standard thicknesses and grinding tolerances), `ManufacturingRoute` facet (ground stock / cut plate / milled / turned, setups, tolerance class, machined faces, supplier), `MachineAssembly.manufacturingRoutes()`; tests in `ManufacturingRouteTests`.
- 2026-11-03: L3 done (committed after L4, which it uses): `LeadScrewNut.mountingFace/mountingCutout` (barrel bore and tap-drill flange holes), `MountingFace.pose`, `NutBlock` in ground steel flat bar sized from the nut and the screw centre height; tests in `NutBlockTests`.
- 2026-11-03: L1/L2 follow-up: `MotorMount` and footed `ScrewSupportUnit` present their tapped seat as a `MountingFace` (`MountingFace.tappedFoot`), so the flange-cut-out check covers seats; bare vs cut slab checked in `MillScrewDrivePlacementChecks`.
- 2026-11-03: L5 done on hobber-catalog: eRob 110 V6 rev D family (ErobActuatorPart/ErobActuator, ErobActuatorData, DcBusConsumer, pinned STEP loader); moment rating, bearing stiffness and brake holding torque stay unset (Assumed once entered) until ZeroErr answers.
- 2026-11-03: L6 done on hobber-catalog: sealed absolute scale family (SealedScalePart housing + head, SealedScaleData JFT-40/JFT-10 rows, standardLength, BiSS-C EncoderPart).
- 2026-11-03: L7 done on hobber-catalog: ELM2H-0400LA60F/E and ELM2H-1000LA80F/E rows already in the catalogue; test now pins them. No brake holding torque/delay in the catalogue.
- 2026-11-03: H3 done: with the nut on a `NutBlock` (block behind the flange, flange outward) the Y nut flange ends 142.5 mm from the screw start at +15 mm against the BF12 housing face at 153 (10.5 mm clear); 160 would leave 0.5 mm, so 170 stays the shortest 10 mm step with at least 2 mm; no layout change, `ballscrew_package.py` already says 170, README corrected (it said Y 250), reasoning in `HobberYLayout`.
- 2026-11-04: H1 done (worktree hobber-frames). X carriage is one milled steel part (`GE-X-carriage`, 2 setups): 20 mm deck between the block plane (260) and the column sole plane (280), 25 mm ribs under it beside the rails (y +-120..140, +-160..180), open window x 60-140 over the swivel stack. **Closing the U does not work**: a front bar (x 60-80) meets the Y rails and blocks (x 57-85) when the head swings A +-15 deg at Y = -10 and Z = -10 (sweep: 5815 mm3 with railYLeft, 14676 mm3 with blockYLeftFront); it clears the table, chip tray, mandrel, cup and loading corridor, so the U stays open. **A 30-40 mm ground plate does not fit**: the deck is capped at 20 mm by the fixed block plane and column sole plane (only 25 mm ribs fit above the bed), so ground plate (aluminium in `GroundStock`) would be 20 mm; a thicker deck needs the X rails 10-20 mm lower (bed insert heights), a user decision. Rough stiffness (`frame_stiffness.py`, Mindlin plate FE; front tie = vertical stiffness at the front block pads with the column sole held; sole tilt = moment per rad for the column sole on the four pads, plate share only): before welded steel U 3.68 N/um, 188.9 MN m/rad, 14.8 kg; Al 20 flat 3.41 N/um, 60.8, 7.4 kg; Al 20 + ribs 9.49, 84.8, 9.4 kg; steel 20 flat 10.3, 189.3, 21.5 kg; **steel 20 + ribs 28.9 N/um, 263.9 MN m/rad, 27.4 kg (built)**: 7.9x on the front tie, 1.4x on the sole tilt. Z saddle is one milled plate (`GE-Z-saddle`, 13 mm: the swivel seat at x 135 and the block faces at 148 fix it, so no stiffness gain). Drive seats are bars under the two supports only (see H2); the 5 mm motor plates, the X side rib (scale seat) and the welded boxes are gone. Z at 400 W (ELM2H-0400LA60E, braked) passes `LinearDriveSizing` with a 1.5 reserve: moving 43.7 kg, peak 0.74 N m (rating 3.81), RMS 0.64 N m (rated 1.27), thrust margin 499 N, inertia ratio 1.17; X (172.5 kg) peak 0.42, RMS 0.32, margin 863 N; Y (17.5 kg) peak 0.39, RMS 0.27, margin 896 N.
- 2026-11-04: H2 done: `HobberDrives` places the X, Y and Z drives with `SupportedScrewDrive.poses` (M60D-BK12-C, BF12 + ring, Leadshine ELM2H-0400 on all three, braked LA60E on Z, `LeadshineServo.driven` for the assumed winding a driver needs), `NutBlock` per axis (80, 30 and 28 mm centre height) bolted to the carriage (X deck underside, Z saddle back face, Y on a ground nut-seat bar on the Y saddle edge), pads sawn from ground steel bar (X 15 mm on the bed insert, Z 40 mm against the column; the mount's hanging flange is pocketed 0.5 mm clear), Y on two milled L brackets on the swivel plate's top edge. Seat holes and dowels are cut from `seatHoles`. The guide joints are now rail -> block (the carriage is fixed to the block), the screw turns in the mount, the motor shaft sits in the coupling. Z axis moved to x 176 (40 mm bar pad). Nut block now has a recipe; `ScrewStretch` accepts the mount as the fixed bearing (assumed 200 kN/mm); screw senses X and Y were reversed (the drive check found it). Positions: tapes removed with the old frames, encoders on X/Y/Z are back with H4 (not yet done). TODO: brake release wiring on Z (`bindMotorBrake`), ELM2H is a 220 V motor on the 400 V rack (assumed winding), pads' own bolts to the bed/column inserts.
- 2026-11-04: H5 (done except the three open members): `MountingFace.hostTool` (flange with clearance holes finds a tapped host cut for tap drills), flange check skips mates that are not the flange's host, journal exemption from the model (`MachineComponent.carriesJournal`: a body that turns may sit next to a part that carries a journal; guide block on its rail), threaded fasteners overlap their hole (`ScrewFastener` on a fixed mate), parts held by form need no face area (`fastensByForm`: nuts, couplings, levelling feet), process contacts declared by the machine (`FlexsplineHobber.processContacts`: hob with blank, mandrel, sleeve), collision hulls compared number by number (`MachineAssemblyCodec`: `Equality.equals` compares arrays by reference, which broke every member with authored hulls). Strict integrity with the full sweep (28 states): 154 errors (109 overlaps, 15 support gaps, 15 ground) before -> 12, all on three members outside this plan: spindle cartridge (R2), electrical rack on its lifting seat, chip tray on the table plate (`HobberFrameChecks.OPEN_MEMBERS`). `HobberInterferenceChecks` is not reduced yet.
- 2026-11-04: H6 done: Y drive frame replaced by two milled L brackets (wall 10, bridge 5 mm over the Y saddle's top, foot 15 mm on the swivel plate edge, 89 and 56 mm long: contact above 625 mm2) and a nut-seat bar on the saddle edge (42 x 60 over the 12 mm edge); the old Y nut bracket is off the saddle; running clearance to the moving saddle verified by the sweep. The rear spindle cradle was not changed (spindle cartridge is open, R2).
- 2026-11-04: H7 done: `HobberAssemblyProcedures` (presentation procedures per axis: fit supports loose, run to both ends, tighten, ream and pin; rails against an edge); validated against the machine in `HobberFrameChecks`.
- 2026-11-04: checks run on hobber-frames: `haxeon.json` (default mechanical checks, now with `HobberFrameChecks.run`: procedures, 400 W sizing, strict rest-state integrity) 2078 assertions pass; `installation` pass (5888); `layout-metrics` thrust margins X 863 / Y 896 / Z 499 N (moving 171.6 / 17.5 / 43.7 kg; X was 128 kg in the older report); strict integrity with the full 28-state sweep at a 2 mm running gap (`haxeon.frames.json integrity sweep`): one finding left on a non-open member set, spindle/motor against the bed casting 1.79 mm at X min, Y -10, Z -10, A +15 deg (spindle cartridge, R2). Overtravel (+-5 mm beyond the limits) is not in the integrity sweep; the X reserve-travel checks (10 mm) and the Z saddle gaps in `HobberZDriveChecks` cover it. `z-drive-checks` fails before reaching the drive assertions, on the "125 mm casting backing behind the drive insert" check and, past that, on the report's `column.body.materialBoxes()` ("Unsupported non-box anchor material"): both are the column casting's own geometry, which H1-H7 do not touch, so the Z drive assertions in that file (coupling clearance, saddle gaps at overtravel, race margin, thread sign) were not executed against the new Z drive; the default run's `HobberDesignChecks` and the strict sweep cover the same geometry. Not done: H4 (sealed scales; the tapes left with the old frames), `HobberInterferenceChecks` not reduced, the Y drive's sweep margin at overtravel.
- 2026-11-04: R0 recorded: Y stays rails + screw; spindle seat is a milled split clamp (see Decisions). H branch rebased on main 4e1c9d8df; journal exemption narrowed to a body turning relative to the bearing's own body.
- 2026-11-04: H8 done (worktree hobber-catalog, branch hobber-parts): `supplier-packages/machined/` (exporter `MachinedPackageExport.hx`, `machined_package.py`, README): for the 9 routed `HobberSaddle` designs and the 3 nut blocks, a STEP in the part's own frame, an A4 PDF and a DXF, `parts.json`, `PARTS_RFQ.md`/`parts-rfq.csv`; checks (STEP volume within 0.1 %, every table hole probed in the solid, milled-part rules, sheet read-back) pass for all 12 parts, 41.3 kg. Model additions: `RouteHole`, `HobberSaddle.holes()`, `HobberDrives.seatRouteHoles()`, `NutBlock.holes()`. **Bug found and fixed:** the Z saddle's nut block bolt holes were cut at the work-frame position, 363 mm off the saddle (it has no holes in H1-H7's model); `withBlockBolts` takes the part frame's origin. Not modelled yet: the Y nut seat has no holes (nut block bolts, fixing to the Y saddle), the X carriage and Z saddle carry no fixing holes to the rail blocks or column. Open for the user: material grade, finish, tap run-out (see the README).
- 2026-11-04: H4 done: JFT-40 sealed scales on X, Y and Z (`HobberScales`; the smallest standard ML for stroke + 2 x 5 mm overtravel: X ML 120 +-3 um, Y ML 70 +-5 um, Z **ML 220** +-5 um, not 270: Z stroke is 200 + 10 = 210), housing on the fixed part, head on the moving part, encoder `<axis>position` on each head again. X: housing on a 10 x 20 ground bar on the flat bed beam top (z 210, y 178), head under the carriage's outer rib on that side (rib bottom z 245.7, 14.3 deep instead of 25 for that rib). Y: housing on the swivel plate's front face on the A axis line (M8 tapped in the plate), head on a 2.3 mm boss of the Y saddle. Z: the column front is a 90 mm deep channel (casting wall at y +-120, face at x 276 in the middle), so the housing sits on a ground bar on the channel wall (y 120, x 198) and the head on a 10 mm milled arm from the Z saddle's back face. Datasheet tolerances on the seat faces' `ManufacturingRoute` (housing seat 0.1 on the bar and the swivel plate, head seat 0.05 on rib, boss and arm). `AssemblyIntegrity.guided`: a sealed scale's head in its housing's slot is a designed running gap (like block on rail). Abbe offsets (housing line to hob centre, across the travel, work frame, printed by `HobberFrameChecks.scales`): X y 178 + z 129.5 mm (220.1), Y x 71.5 + z 0 (71.5), Z x 198 + y 96.5 (220.3). Nothing closer fits: the only fixed surfaces near the cut are the bed beam (z 210) and the column walls; X could drop to y ~46 in the screw channel but then z is 188, and the channel is full of the nut block and pads. Strict integrity with the sweep (28 states, 2 mm gap): the same 13 findings as before, none on a scale member. Also: the work seat height 363 is `HobberZLayout.SEAT_Z`.
- 2026-11-04: X moving mass (171.6 kg) explained, no modelling error. Per-member diff of the X subtree (everything below the X joint, built at 2784d2623^ and at this branch): **166.3 kg before H1-H6, 171.5 kg after (+5.2)**. Changes: X carriage 15.6 -> 25.2 (+9.6, the milled steel carriage with ribs against the welded U), Z drive frame 7.9 -> Z pads 3.1 (-4.8), Y drive frame 2.2 -> Y brackets 1.7 (-0.5), nut blocks and the Y nut seat +1.8, motors/Z saddle/Y saddle -1.2; the scales add 1.9 kg after (173.5 kg). The 128-135 kg of the older reports (LAYOUT_REPORT, 2026-10-19) predates the casting conversions of 2026-10-20 to 25: the bulk of the X group is the column package that rides on the carriage, **91.5 kg** (casting 69.9, drive seat insert 7.2, base plate 6.0, rail inserts 4.3, lifting inserts 4.1), then the carriage 25.2, the three saddles/plates 20.0 (Z 8.4, Y 6.4, swivel plate 5.2) and the head (spindle, swivel, hob, outboard) about 15 kg. Nothing is parented into the X subtree by mistake: every member is on the carriage or the column.
- 2026-11-04: Z brake wired: the built-in brake of an `E` Leadshine servo is now its `motorBrakeSpec()` (`LeadshineBrake`: the datasheet gives no holding torque or delays, so **assumed** 2 x rated torque = 2.54 N m for the 400 W, 50 ms release, 30 ms apply, listed on the actuator as an assumption), and `FlexsplineHobber.drive` binds it with `bindMotorBrake("Zdrive", "Zmotor", "Z.brakeRelease", "Z.brakeReleased", 250, 30)`. `HobberFrameChecks.sizing` checks the brake is bound and holds 1.5 x the weight's screw torque (2.54 N m against 0.351 N m for 45 kg). Open: the real holding torque from Leadshine.
