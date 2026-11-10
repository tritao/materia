# Hobber alignment plan

How the machine's axes are aligned to each other at assembly, which datum each one references, how it is set, how it is
measured, and which accuracy-budget row it feeds. Status: plan, 2026-11-07. The steps run inside the assembly sequences
(LINEAR_MODULE_PLAN step XA for X, LM4 for Y, the same treatment for Z); the tools are BOM tooling rows (LM6).

## Rule

- **Castings provide mass; ground stock provides small precision interfaces.** Buy ground
  stock, inspect/select it and finish-grind failing small faces. Reserve replication for
  large casting surfaces that need it, under `machinekit/RAIL_MOUNTING.md`; no replicated
  X/Z drive seats or X-carriage block-pad layers remain.
- **Location by joint:** rails at a reference edge; screw supports indicated to the rails
  and actual screw, then match-pinned; nut blocks indicated at both travel ends then
  match-pinned; scales shimmed/adjusted at the head; structural assembly dowels from the drawing.
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
| A3 | X screw supports | two short matched15 mm ground bars; supports indicated to rails with the actual screw and nut at both travel ends, torqued then match-pinned | arbor runout, parallelism to rails at both ends | drive alignment; no budget row |
| A4 | X carriage and nut | inspect the ground20 mm plate, select stock or finish-grind four block lands in one setup; carriage on blocks; nut block indicated to the screw at both ends of travel, then pinned | indicator on screw, both ends | drive alignment |
| A5 | Table (C) | seat replicated on the bed insert with the table face held parallel to P; table centred on the X line laterally before replication | indicator on the X carriage sweeping the table face while C turns (perpendicularity of C to P, two directions); test mandrel in C read along X (centre on the X line) | C tilt, centre offset; mandrel runout row |
| A6 | Column on the carriage, Z parallel to C | Z rail seats replicated on the column inserts on the bench against the column's reference; four matched ground sole pads on the carriage; indicate Z rails parallel to the C test mandrel, then fit assembly pins (qualification OPEN) | indicator on the Z carriage reading the C mandrel along Z, in two planes | `guideAngleArcsec` (Z), helix (F-beta) |
| A7 | Z screw supports and nut | as A3 and A4 with two matched40 mm ground bars, referencing Z rails and actual screw/nut | as A3 and A4 | drive alignment |
| A8 | A swivel zero | eRob directly on the profiled Y-module base; hob axis square to Z at A = 0 measured, encoder zero offset stored | indicator on a ground arbor in the spindle, swept along Z | lead angle setting; A rows |
| A9 | Y and the hob axis | ground module base, derived rail risers and profiled carriage/head interface from reference edges (LM4); indicate hob axis parallel to Y | indicator on the arbor along Y | hob shift; Abbe (Y) |
| A10 | Outboard support | P2: indicated coaxial to the spindle axis, then dowelled | coaxiality budget (5 um radial) | runout rows |
| A11 | Scales | housings on their seats, heads shimmed to Yuheng's tolerances; error maps and the Abbe tilt table measured | per-scale mounting tolerances; reference artefact or laser where available | scale and Abbe rows |

## Tools (BOM tooling, LM6)

Leveling-grade precision level, straightedge or granite reference, test mandrel for C,
ground spindle test arbor, dial indicators/magnetic bases, AF4 long key>=400 mm and the
sequence's checked hand tools. `HobberAssemblyTooling` is the authoritative tooling list;
every X/Y/Z step reference must resolve there. Drive alignment uses the installed screw,
not a separate arbor. Large-casting replication consumables are a separate future process
procurement, not drive-seat tooling.

## Open

- Numeric commissioning tolerances, stock inspection/selection and supplier finish-grinding.
- Large casting replication product/process where needed; no small support-seat layers.
- A6 column squareness and assembly pins; final Y stiffness/accuracy and elastomer/seal qualification.
- Remaining Z ingress and complete Z/wiper service sequence. Wrap-up records these open,
  rather than extending this implementation.
