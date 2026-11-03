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
- **Spindle:** bought Ø80 cartridge, flange-mount version preferred, with the maker's outboard support.

## Open

- Y as a bought linear module (KK60/KK86 type) or rails + screw (R0).
- Whether a front cross-bar can close the X carriage's open U (H1).
- eRob output bearing moment rating and brake holding torque (asked from ZeroErr).

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

- **R0.** Y module check: KK60/KK86 moment rating, length and stiffness against the head load.
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
