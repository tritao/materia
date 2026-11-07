# Linear module plan

**Goal.** A MachineKit component for a screw-driven precision linear module, in the style of the HZMotion HZM200:
a ground steel base with raised rail seats, the screw centred on the floor between them, an HZMotion motor mount
at one end and a floating support at the other, a carriage on four blocks, a cover, and a sealed linear scale built
in. The first user is the flexspline hobber's Y axis (hob shift). The same round applies the structure rule below to
the hobber's X carriage and gives Z its cover. Status: plan, 2026-11-07.

## Why

The hobber's Y axis is built from loose parts: HGH15 rails on the 15 mm swivel plate, the HZMotion M60D-BK12 mount
and BF12 on two milled L brackets, a nut seat bar, and a JFT-40 on its own seat bar. The screw is off-centre. It
looks improvised, and it carries a known open item: FRAME_PLAN could not centre the Y screw because the M60D
reaches 22 mm below the rail plane and meets the plate. Raised rail seats remove that conflict, because the mount
and the BF then stand on a floor below the rail plane.

FRAME_PLAN rejected *bought* KK modules for Y (the shortest KK86 is 440 mm for a 30 mm need, +5.7 kg on the eRob,
stiffness unpublished). This plan does not reverse that: the module here is ours, sized to the stroke, and built on a
plate the host already has.

The X carriage (`GE-X-carriage`) is one milled steel part, a 20 mm deck with 25 mm ribs, because the deck is capped
at 20 mm between the block plane (z 260) and the column sole (z 280). It is the most expensive part to have made
(about 20 kg of chips, pad faces between ribs) for a stiffness a plate with bolted ribs can give.

## Structure rule (2026-11-07)

This is the long-term construction rule for the hobber, and the pattern the module expresses. It refines the
FRAME_PLAN parts rule (a part is a structural member or a real adjustment/interface element).

- **Castings and cast inserts** give mass, damping and coarse location. Nothing precise is machined into a casting;
  insert heights are outputs of the height stack.
- **Precision lives in ground flat stock.** Bodies are ground plates (carriages, saddles, bases). Bars are height
  spacers (rail risers, drive pads, scale seats); they exist only where the height stack needs them, can be reground
  or replaced, and disappear when the derived thickness is zero. Each part has two ground faces, drilled and tapped
  holes, and at most one ground reference edge; ribs, pockets or steps only where a number shows they are needed.
- **Heights are derived per axis** from block height, rail height, plate thickness and the mount's screw-axis height;
  bar thicknesses (from standard stock) and insert heights come out.
- **Location depends on the part:**
  - rails: pushed against a ground reference edge (push screws or clamps), never dowelled;
  - screw supports (HZMotion mount, BF): indicated to the rails at assembly, then match-reamed and pinned (two pins),
    so a service removal returns without re-indicating;
  - scales: housing on its seat face; the read head adjusted with shims or slots to the vendor's tolerances;
  - structural joints (plate and ribs, bars on plates): bolted, dowelled for assembly, and bonded where the joint
    must act as one part.
- **Buy ground where possible.** Precision-ground plate cut to size and ground flat bar are stock items; custom
  grinding only where stock cannot meet a checked tolerance. Where stock flatness fails a check, replicate only the
  failing faces (Moglice or SKC against a reference, as in `RAIL_MOUNTING.md`) before paying for custom grinding.

## Decisions

- **HZMotion drive parts.** The fixed end is an HZMotion integrated mount (`SupportedScrewDrive`: M60D-BK12-C or
  M80E-BK15-C), the floating end a BF unit. No custom motor housing.
- **Riser bars are the default base.** The module's base is a flat ground plate with two ground riser bars under the
  rails and the mount and BF on the plate between them. Integral lands (rail seats and floor ground into one stepped
  part) remain a library option for a host that justifies them.
- **Host-integrated mode.** The module either brings its own base plate or uses a plate the host supplies (for the
  hobber, the swivel plate): it adds its bars, holes, dowels and seats to that plate. Same recipe and checks.
- **The scale belongs to the module.** Its seat bar, head bracket, travel window, cable and air are module features,
  checked against the vendor's mounting tolerances, not added afterwards by the host.
- **X carriage:** a bought ground 20 mm steel plate with bright drawn flat-bar ribs underneath, bolted from below,
  dowelled and bonded. The bed, the X rail height and the stiffness target stay.
- **XC slip evidence:** retain derived assembly dowels, but prove the rib slip gate separately at every
  bolt from the local shear flow and that bolt's preload times assumed ground-steel friction 0.2.
  Credit neither adhesive nor dowels in that friction margin. `JointSlip` normally selects pin
  capacity for a dowelled joint; its aggregate pass therefore cannot substitute for XC's >=3 gate.
- **XC column load path and edge checks (user addition, 2026-11-07).** Replace the rear-half
  column sole interface with either a full-footprint steel sole (about 380 x 180 mm) or four
  corner pads with the front pair clear of the casting's front recess. Choose from the actual
  load path and clearances; carriage contact pads and joint-derived bolts must span that base.
  Report before/after column-to-carriage tilt stiffness from contact area and bolt pattern
  (about 3x expected, must not regress), and check that joint for slip.
  `JointFasteners` must check every derived hole in every machine: nearest actual part edge at
  least 1.5 screw diameters from its centre, with the counterbore or head entirely supported
  and material wall left to the edge. Report findings by part and joint. Fix the hobber's
  violations, gate both carriage and column-sole joints, and list other failing examples
  without silently modifying those machines.
- **Z stays on the column** (2026-11-07). The Z screw is at y -82 because the eRob housing stands on the centre line,
  the rails sit on cast inserts with no added joint, and the column's insert rows and ground pad bar already follow
  the rule. Z gets a bellows cover only.

## Phases

One worktree, one commit per step, every step keeps the baselines (LM0) and the existing suites green.
Order: LM0, XC, LM1, LM2, LM3, LM4, LM5, ZB, LM6.

### LM0. Baseline

Record before any change, in this plan's Log:
- Y moving mass, head mass and centre of mass, A-axis tilt moment and torque against the eRob figures;
- accuracy budget rows (`HobberAccuracyBudget`), the scales' Abbe offsets (`HobberFrameChecks.scales`);
- integrity, derived fasteners, collisions over Y travel and A ±15°, service removals;
- member count and BOM lines for the X carriage and the Y subtree;
- X carriage plate FE (`fem/frame_stiffness.py`): today 28.9 N/um front tie, 263.9 MN m/rad sole tilt, 27.4 kg.

### XC. X carriage: ground plate with bolted ribs

- Replace `GE-X-carriage` with a 20 mm ground steel plate (stock: precision-ground plate cut to size) and bright drawn
  flat-bar ribs in today's rib positions (y ±120..140, ±160..180), screwed from below into tapped holes so the column
  sole face stays clean, two dowels per rib, a bonded joint (adhesive declared as a process with its assumed data).
- Joints declared as faces: `JointFasteners` derives the screws and dowels; assembly order with `fittedBefore`.
- Gates:
  1. plate FE with ribs tied (upper bound) and with independent bars (lower bound); the design must land near the
     upper bound and at least match the LM0 values;
  2. slip check at every rib bolt: shear flow under the worst case (column weight, process load, X acceleration)
     against preload × friction (ground steel 0.2, assumed), margin ≥ 3, ignoring the bond;
  3. plate flatness of the stock row against HIWIN's allowable height difference between HG25 blocks for the
     declared preload; if it fails, replicate the four block pads and record why;
  4. collisions: ribs clear the blocks, the X nut block, the chip skirt and the shields over the full travel;
  5. hobber checks, integrity, fasteners, service removals unchanged or better.

Additional XC gates (user, 2026-11-07):
6. Column-to-carriage steel contact and derived bolt pattern span the column footprint; compare
   contact/bolt tilt stiffness before/after, expect about 3x and require no regression; slip passes.
7. Every carriage and column-sole derived hole passes the library-wide actual-edge rule above.
   Run it for the other machine examples and report their violations separately.

### LM1. Library component

`machinekit.motion.ScrewLinearModule` (name to confirm against the naming audit), with a spec:
- rail family and size (unified linear-rail recipe), rail spacing, blocks per rail, block spacing;
- stroke, overtravel, carriage length and width, carriage hole pattern;
- the `SupportedScrewDrive` (mount, BF, motor, coupling, screw and nut);
- base: own plate (stock row) or a host plate; riser bars (default) or integral lands;
- optional sealed scale (LM2) and cover (LM3).

Parts and features:
- **Riser bars:** ground flat stock; one ground reference edge per rail; bar thickness derived so the screw axis meets
  the nut (from the mount's catalogue axis height and the nut housing), from standard stock thicknesses.
- **Plate:** the mount and BF on the plate between the bars, each match-pinned; bolt and dowel pattern underneath when
  the module brings its own plate; hard stops in line with the carriage.
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
head lateral 0.2, gap 1 ± 0.3 mm):
- **Seat:** a ground seat bar located from the same reference edge as the rail bars, so its parallelism to the rails
  is a grinding and edge tolerance; checked against `housingParallelism`.
- **Head:** a bracket on the carriage with a declared adjustment (shims or slots) that covers `gap`, `headLateral` and
  `headParallelism`; the check reports the adjustment range against the tolerance stack.
- **Measuring length:** the smallest standard ML (`standardLength`) for stroke plus 2 × overtravel; the hard stops are
  reached while the head is still inside the window, at both ends.
- **Abbe:** offsets of the scale's reading line from the module's declared functional point (for the hobber: the hob
  centre), reported for every tilt the host declares.
- **Cable and air:** cable exit with `CABLE_BEND_RADIUS` (75 mm) and the straight run respected into the host's moving
  cable path; the purge port (`airThread`) reachable and routed.
- **Signals:** the module exposes the scale as the axis's position feedback and the motor encoder as velocity feedback
  (dual loop over BiSS-C), as ports the host wires to its drive's second encoder input and its BiSS master.
- **Service:** the head and the housing can be removed without moving the rails or the screw.

### LM3. Cover

Pick the cover from the stroke. For short strokes (the hobber's ±15 mm), block wipers plus a fixed top plate that the
carriage skirt overlaps; telescopic or bellows covers only when the stroke needs them. Use the way-cover library
(`machinekit.cover`), its drain and service rules, and the existing wiper rows.

### LM4. Hobber Y conversion

- The swivel plate stays a flat ground steel plate and becomes the module's host plate: two riser bars under the
  HGH15 rails, the M60D-BK12 and BF12 centred between them on the plate, the JFT-40 seat bar.
- The Y saddle stays as the carriage: it carries the spindle seat and becomes the module's carriage part, with the
  nut housing and the scale head bracket.
- Remove the two L brackets, the nut seat bar and the old Y scale seat bar; their roles move into the module.
- Swivel plate thickness: keep 15 mm unless a number requires more; record the A-axis effect of any change.
- Settle FRAME_PLAN's "Y screw centred between the rails" item with numbers.

### LM5. Verify on the hobber

Against the LM0 baseline:
- hobber checks, integrity, derived fasteners, collisions over Y travel and A ±15°;
- service removals (motor, screw and nut, scale housing and head);
- crash and hard stops;
- accuracy budget, Abbe offsets, A tilt moment and torque against the eRob figures;
- cradle FE if the plate or saddle changed;
- editor screenshots of the X carriage and the Y axis before and after.

### ZB. Z bellows

A bellows cover over the Z rails and screw on the column, from the way-cover library, with its drain and service
rules; no structural change to Z.

### LM6. Documentation and manufacturing

FRAME_PLAN, PARTS and `MACHINE_DESIGN.md` updates; drawings and RFQ rows in `supplier-packages/machined/` for the X
plate and ribs, the Y swivel plate and riser bars, the scale seat bar and any changed saddle, each with the stock it is
bought as (ground plate, ground flat bar, bright drawn bar) so quotes replace the estimates.

## Rough cost (estimates, not quotes)

X carriage at about 370 × 370 mm: one-piece milled with ribs about €400–700 (China) / €900–1,500 (EU); 20 mm plate with
ground bars €250–450 / €500–900; bought ground plate with bright drawn ribs €120–250 / €250–450. LM6 replaces these
with quotes.

## Open

- Swivel plate thickness (LM4 decides by numbers).
- The scale's mounting tolerances are datasheet values; the Y scale seat and head adjustment are only as good as them.
- The bond's strength and creep are assumed until a maker's data sheet is chosen; the slip gate ignores it.
- NJPE drawings for the BK12 bracket and BF12 (already requested for X and Z) also cover Y.

## Log

- 2026-11-07 — **LM0 complete, before design changes**, baseline `aeb4b4b44` on `linear-module`.
  Read-only native inventory and accuracy output is preserved in
  [`LINEAR_MODULE_BASELINE.json`](examples/flexspline-hobber/LINEAR_MODULE_BASELINE.json).
  Y-moving subtree (`blockYLeft`, including its four guide blocks): **28.132885 kg**, 34 members,
  work-frame CoM **(55.811, 107.181, 392.242) mm** at initial X. Y fixed-plus-moving subtree
  (`swivelPlate`, excluding adapter/output): **38.328895 kg**, 47 members, CoM
  **(73.971, 86.794, 399.869) mm**; 41 component purchase lines / 60 with clamped-member-owned
  fasteners, 82 screws and 12 internal dowels. X carriage alone: **24.586824 kg**, one member /
  one component purchase line, three lines with its 18 clamped-face screws (the nut-base pins
  cross this inventory boundary). These native member masses exclude BOM-only hardware, as do
  the existing frame reports. The full A-head/report, suite counts, fasteners, services and strict
  sweep are pending; **LM0 is not complete and XC has not started**.
  Current `examples/flexspline-hobber/frame_stiffness.py --json`: steel 20 + 25 ribs **28.895723 N/um**
  front tie, **11.751364 MN m/rad** sole tilt, **27.3808 kg** plate-model mass; flat deck
  **10.297279 N/um**, **7.306075 MN m/rad**, **21.4933 kg**. The plan's older 263.9 MN m/rad
  is superseded by FRAME_PLAN's 2026-11-06 sign correction; it is not the baseline gate.
  Read-only accuracy budget passes class 6 (predicted class 4): RSS fp/Fp/Fr/F-alpha/F-beta/size
  **0.86 / 4.07 / 3.89 / 3.45 / 1.49 / 5.33 um**; worst sums
  **1.72 / 9.21 / 6.76 / 7.85 / 3.11 / 9.23 um**. All rows and their declared assumptions are
  in the snapshot. Baseline suite queue and metrics ran through `run-fe.sh`; temporary logs
  are under `claude-hobber-tools/tmp/lm-baseline-*`. No source or geometry changes yet.

  Pre-implementation XC investigation (temporary script `tmp/lm-xc-study.py`, detailed data
  `tmp/lm-xc-investigation.json`): a 16 mm C3D10 mesh with separate interfaces retains the actual
  JFT-40 carrier rib (y 141.3..159.1, z 243..260), which superseded the old fourth y 160..180 rib.
  With the other three ribs 25 deep: independent / tied / finite epoxy / bolt-patch-only front
  stiffness **10.660 / 20.383 / 19.011 / 18.352 N/um**, sole tilt
  **7.976 / 9.898 / 9.641 / 9.473 MN m/rad**. At 35 deep: tied / finite epoxy / bolt-patch-only
  **23.570 / 21.783 / 21.057 N/um**, **10.755 / 10.415 / 10.200 MN m/rad**. This is a prospective
  geometry study, not an adopted design or a completed XC gate. Epoxy E 1718 MPa is from the
  [Henkel EA 9466 TDS](https://datasheets.tdx.henkel.com/LOCTITE-EA-9466-en_GL.pdf) (March 2025);
  nu 0.35 and 0.1 mm film are **assumed**, creep is unqualified. The scale-aware plate approximation
  gives 23.283 / 26.032 / 28.735 / 31.344 N/um with 25 / 30 / 35 / 40 mm bars: 35 still misses
  the 28.896 LM0 gate; the next solid study is 40 mm. The original plate gate remains in force;
  the lower solid-FE datum does not replace it. All construction, slip, stock and travel gates remain open.

  LM0 baseline hobber suite completed: **2,345 assertions passed**. Its frame report gives X/Y/Z moving
  **191.57 / 28.13 / 51.42 kg**; A head **39.16 kg / 49 members**, CoM relative to the output face
  **(-93.7, 84.9, 36.1) mm**. A bearing tilt **35.99 N m gravity + 12.4 N m cutting = 48.39 N m**;
  bearing rating remains **unknown** (pre-existing supplier question). A gravity torque **32.62 N m**
  at zero / **35.10 N m** worst over +/-15 degrees, against eRob **87 rated / 204 peak N m**;
  brake **87 N m assumed**, 2.5x demand. Reported cutting torque about A is zero because the
  declared hob functional point is on that axis; the 100 N still contributes to bearing tilt.
  Scale Abbe offsets X **(y164.5, z129), 209.0 mm**, Y **(x76, z5.5), 76.2 mm**, Z
  **(x215.5, y96), 235.9 mm**; uncompensated errors **5.1 / 1.8 / 5.7 um** at the assumed 5 arcsec.
  Head windows including overtravel: X **49.2..124.0 of 16.5..136.5**, Y **36.5..66.5 of
  16.5..86.5**, Z **21.5..231.5 of 16.5..236.5 mm**. Remaining baseline suites are still pending.

  LM0 hobbing suite: **1,161 assertions passed**; joint-fasteners: pass, bench-mill fit **0 findings**,
  51 joints / 232 screws / 16 dowels. Pre-implementation 40 mm-bar solid study: independent / tied /
  finite epoxy / bolt-patch-only **10.660 / 24.981 / 22.992 / 22.301 N/um**, sole tilt
  **7.978 / 11.179 / 10.794 / 10.575 MN m/rad** (16 mm mesh, same assumptions above). The finite
  bond retains **92.0% / 96.6%** of tied stiffness. This candidate still requires finer FE, slip,
  stock/replication and all travel checks; no rib layout has been adopted.

  LM0 assembly-integrity **passes** (support, interference, sweep and drive fixtures); way-cover
  **passes** (plans, sweep, flange bolts, BOM, drain rule, service and box covers). Neither runner
  prints an assertion count. The complete `frames` command with `crash fasteners sizing swivel
  scales zoffset spindleseat spindledrive procedures roundtrip integrity service sweep` is now
  running; LM0 remains uncommitted until its strict sweep and services pass.

  Refinement checkpoint (40 mm candidate, 12 mm mesh; finite-joint cases still live): independent
  **10.932 N/um / 8.879 MN m/rad**, tied **25.509 N/um / 12.073 MN m/rad**. Compared with
  16 mm, tied front stiffness changes 2.1% and tilt 8.0%; these are intermediate refinement
  results, not a completed gate. Baseline strict sweep is still live; no source changes.

  Fine-mesh finite-epoxy result (40 mm candidate, 12 mm mesh): **23.488 N/um**,
  **11.685 MN m/rad**, retaining **92.1% / 96.8%** of tied stiffness; bolt-only case still running.
  Relative to 16 mm, finite-bond front stiffness changes 2.2% and tilt 8.3%.

  The 12 mm study completed: bolt-patch-only **22.736 N/um / 11.453 MN m/rad**.
  All four joint bounds are now saved in `tmp/lm-xc-investigation.json`; these stiffness
  bounds do not prove the separate every-bolt friction-slip gate. Baseline strict sweep remains live.

  LM0 full-frame intermediate report: **269 members**, assembly procedures validate,
  saved-assembly round trip **passes**. Strict sweep and final service verdicts remain pending.

  2026-11-07 user scope addition accepted for XC: repair the rear-half column sole load path,
  add its before/after contact-and-bolt tilt stiffness and slip gate, and add the global derived-hole
  edge/head-support check. User-observed existing sole is 380 x 100 x 20 under the rear half of
  a 200 mm footprint; verify these datums on the unchanged model before redesign. LM0 strict
  sweep continues unchanged; finish that current step before construction changes.

  Added-XC baseline source inspection: `FoundationLayout.placements` defines column `basePlate`
  as **380 x 100 x 20 mm**, frame `(0,-40,10)` with its underside normal reversed. `FlexsplineHobber`
  fixes `column/basePlate` to `Xbridge` at work x326/z280. This confirms the narrow sole; the
  physical contact and bolt-derived joint must be qualified separately from that fixed mate.
  The Xbridge clamped-face inventory has 18 screws (16 guide-block + 2 nut-block), so it is not
  evidence of a full-footprint column bolt pattern. No geometry has changed; strict sweep remains live.

  **LM0 final gate, 2026-11-07:** full frames command exits successfully, **0 integrity errors
  in 1945 s**, **68 joints / 246 screws / 13 dowelled joints / 26 dowels**; all **13 declared
  service removals clear**. Together with hobber 2345 and hobbing 1161 assertions, joint-fasteners,
  assembly-integrity and way-cover passes above, this completes LM0. Earlier pending statements
  in this entry are chronological checkpoints superseded by this final result. Full frame output
  is preserved in LINEAR_MODULE_BASELINE.json. No construction source has changed.
