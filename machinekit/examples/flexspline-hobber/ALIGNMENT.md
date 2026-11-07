# Hobber alignment plan

How the machine's axes are aligned to each other at assembly, which datum each one references, how it is set, how it is
measured, and which accuracy-budget row it feeds. Status: plan, 2026-11-07. The steps run inside the assembly sequences
(LINEAR_MODULE_PLAN step XA for X, LM4 for Y, the same treatment for Z); the tools are BOM tooling rows (LM6).

## Rule

- **On castings, replicate; on machined plates, grind.** A cast-in insert is placed to about ±0.5–1 mm, which no precision
  interface can use as is. Every precision seat on a casting (rail seats, screw-support seats, the table seat, the column
  sole) is a thin replicated epoxy layer (Moglice / SKC class, process in `machinekit/RAIL_MOUNTING.md`) cast against a
  fixture that references the datum it must relate to. Precision features on machined parts (swivel plate, saddles,
  carriage plate) are ground, from bought ground stock where possible.
- **Location by part type** (from the Structure rule in `machinekit/LINEAR_MODULE_PLAN.md`): rails against a reference edge;
  screw supports fixture-replicated or indicated, then pinned; nut blocks indicated to the screw at both ends of travel,
  then pinned; scale heads shimmed to the vendor's tolerances; structural joints dowelled from the drawing.
- **One master datum per relation, set once, measured, then pinned**, so a service removal returns without re-alignment.
- **Software takes what geometry need not**: encoder zero offsets (A, C), scale error maps and the Abbe tilt table are
  commissioning data, not mechanical adjustments.

## Datums

| Datum | Defined by | Set at step |
|---|---|---|
| Floor | the three leveling feet (machine frame z 0) | A1 |
| P, the X rail plane | the replicated X rail seats, levelled | A2 |
| X line | the X reference rail's edge | A2 |
| C, the work axis | the rotary table, its face replicated parallel to P | A5 |
| Z line | the Z reference rail on the column, set parallel to C | A6 |
| Hob axis (B) | the spindle seat on the swivel plate | A9 |

## Sequence

| Step | What | Method | Measurement | Feeds |
|---|---|---|---|---|
| A1 | Level the bed on its three feet | leveling feet (three points, no rocking), lock nuts | precision level on the X rail seats' area, both directions | gravity terms; repeatability of every later step |
| A2 | X rail seats | replicated on the bed inserts against a straightedge fixture set level; reference rail pushed to its edge, second rail set parallel to it | straightedge and indicator; rail parallelism | `straightnessUm`, `guideAngleArcsec` (X) |
| A3 | X screw supports | seats replicated under the HZMotion mount and BF12 with the alignment arbor referencing the rails and the carriage's nut bore (XA) | arbor runout, parallelism to rails at both ends | drive alignment; no budget row |
| A4 | X carriage and nut | carriage on the blocks; nut block indicated to the screw at both ends of travel, then pinned | indicator on screw, both ends | drive alignment |
| A5 | Table (C) | seat replicated on the bed insert with the table face held parallel to P; table centred on the X line laterally before replication | indicator on the X carriage sweeping the table face while C turns (perpendicularity of C to P, two directions); test mandrel in C read along X (centre on the X line) | C tilt, centre offset; mandrel runout row |
| A6 | Column on the carriage, Z parallel to C | Z rail seats replicated on the column inserts on the bench against the column's reference; column sole replicated on the carriage while the column is held with its Z rails parallel to the C test mandrel (column sole item in XC) | indicator on the Z carriage reading the C mandrel along Z, in two planes | `guideAngleArcsec` (Z), helix (F-beta) |
| A7 | Z screw supports and nut | as A3 and A4, referencing the Z rails | as A3 and A4 | drive alignment |
| A8 | A swivel zero | eRob on its adapter ring; hob axis square to Z at A = 0 measured, encoder zero offset stored | indicator on a ground arbor in the spindle, swept along Z | lead angle setting; A rows |
| A9 | Y and the hob axis | Y riser bars and the spindle seat ground on the swivel plate from one reference edge (LM4); Y parallel to the hob axis by construction | indicator on the arbor along Y | hob shift; Abbe (Y) |
| A10 | Outboard support | P2: indicated coaxial to the spindle axis, then dowelled | coaxiality budget (5 um radial) | runout rows |
| A11 | Scales | housings on their seats, heads shimmed to Yuheng's tolerances; error maps and the Abbe tilt table measured | per-scale mounting tolerances; reference artefact or laser where available | scale and Abbe rows |

## Tools (BOM tooling, LM6)

Leveling-grade precision level, straightedge or granite reference for A2, the drive-seat alignment arbors and height
reference (A3, A7), the replication kit, a test mandrel for C, a ground arbor for the spindle, dial indicators and
magnetic bases, the AF4 long key (P5).

## Open

- Numeric tolerance for each step from `HobberAccuracyBudget` (the budget's assumed rows become the step limits).
- The replication product and its thickness range; the epoxy's creep and stiffness are assumed until one is chosen.
- Whether A6 replicates the column sole in place on the carriage (preferred: one setting against the C mandrel) or on the
  bench against a square; decided with the column sole design in XC.
