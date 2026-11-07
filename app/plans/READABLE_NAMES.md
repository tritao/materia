# Readable names: R1–R4

Work only in `materia-hobber-catalog`, branch `readable-names`.
Starting revision: `4a8654c53`. Implement R1, R2, R3, R4 in that order,
one commit each. No push, merge, submodule changes or OCCT rebuild.
Keep the portable scene artifact schema unchanged and row layout/interaction
unchanged. Stop if an artifact schema change becomes necessary or a regression
is caused outside this worktree.

## Decisions

**R1 — Role labels.** Every generated part uses `AssemblyBodies.displayName`
on its last occurrence-id segment. Siblings follow authored `groupPath`, then
`includePath`, then assembly root, independently of kinematics. Normalized
collisions get short numbered suffixes ordered by occurrence ID. Axis words
X/Y/Z stay capitalized, and acronym/word boundaries are split. Editable labels
remain separate from source part numbers and paths. Generated app metadata
supplies detail, search and read-only inspector fields, including after pose
reevaluation and project reload. Explicit user renames remain overrides.
Selection and native scene names consume the same role label; BOM identity
continues to come from components. No old-label fallback or alias is added.

**R2 — Role IDs.** Assign local anchor roles once in InsertAnchorGeometry
(`bolt1`, etc.); foundation and mold builders prefix their insert role.
Preserve source anchor IDs separately for engineering references. Reject every
hexadecimal run of at least eight characters and decimal coordinates in actual
flattened occurrence IDs. AssemblyIntegrity reports the violation and every
constructible/exportable example is audited. Diagnostic pocket hashes and
machined-face names are engineering identities, not scene occurrences.
Gantry cable chains use axis-role member names. The audited deriveFasteners
path adds BOM items and cuts, without creating occurrences.

**R3 — Bounded numbers.** Shared cap: 48 characters for every designation and
BOM number. Keep short readable family/size terms. Replace geometry encodings
with FAMILY-ROLE-eight-hex using exact defining inputs, never rounded display
text. Constructor enforcement and BOM lint must both enforce the cap.
BlockBracket uses sorted binary64 coordinate keys and canonical stored blocks
so declaration order preserves number, values and fingerprint. EnclosurePanel
reuses its existing short BOM formatter. Featured SteppedShaft numbers include
all exact sections, faces, keyways, grooves, threads and shoulder details;
constructor and BOM must share one rule. Canonical stored feature ordering is
required for merging. Short featureless shafts remain readable. PowerSupply
shares its short PSU formatter, hashing exact inputs when it exceeds the cap.
LinearMountCleat must include endBias and exact height: its old rounded-height
number aliases distinct geometry and already breaks the baseline BOM.
Author-declared drawing numbers retain drawing semantics. No compatibility
aliases. Check stability, distinct inputs, quantity and line merging explicitly.

**R4 — Foundation groups (user addition, 2026-11-07).** Casting has its own
group. Cast and post-cure inserts use declared library purpose groups; their
anchor bolts share the same group. Cover guide/bearing, drive, table, column,
enclosure, tray-clamp, lifting and other declared roles. Declare typed installation purpose on cast/post-cure insert placements,
including tray clamps; derive group names from that purpose. Keep catalogue
socket recipe roles and BOM fingerprints unchanged. Do not infer purpose from
hobber IDs. FoundationIntegrity must reject ungrouped members. Reset the
active group after construction so appended members cannot inherit a purpose
silently. Test role coverage, bed/column groups, real anchor colocation,
post-cure members and ungrouped additions. Bed/column subclasses add no members
after foundation construction.

## Initial evidence

All initial baselines were measured before production source edits. Evidence
root: `/media/joao/DATA1/claude-hobber-tools/tmp/rn-baseline/`.
The supplied wrapper filters output; its exit status alone is not proof of a
pass. Inspect summaries and exceptions. Historical B7 counts are not the
current checkout's baseline.

- App tests passed, including CAD plate workflow (reported output-file failure
  did not reproduce).
- ProjectKit passed 405 assertions.
- Hobber passed 2,345 assertions; hobbing passed 1,161.
- BOM-document, BOM-key and procurement finished without exceptions.
- Assembly-integrity and joint-fastener checks passed; bench-mill fit had zero
  findings, 51 joints, 232 screws and 16 dowels.
- Example builds passed, including screwdriver CAD drawing runtime.
- Default-recipe lint: 225 checked, zero invalid, three source-dependent
  unavailable defaults (cast-in-steel-insert, linear-mount-nut-cassette,
  source-casting). New cap violations must not hide as unavailable defaults.
- Smoke: 9/24 passed. Exact messages live in summary.json and shard logs.
- Standalone BOM-example tests failed on the existing cleat-number collision.
- BOM snapshots: 20/23 succeeded. Failures: sheet-router cleat collision,
  machine-tending stale certificate, flexspline-hobber missing structured
  selection requirements on assumed ROBOTDIGG-2049-BT30-D80-6000.

| Smoke shard | Baseline |
| --- | --- |
| 1 | passed |
| 2 | Group path must not be empty |
| 3 | machinekit.transmission.gear-rack machinekit.transmission.gear-rack / designation:9:29-15-105 Envelope: expected one solid, got 3 |
| 4 | Missing component parameter "role" |
| 5 | passed |
| 6 | BOM part number "LINEAR-MOUNT-CLEAT-H29.585" has conflicting metadata or values |
| 7 | passed |
| 8 | passed |
| 9 | passed |
| 10 | passed |
| 11 | passed |
| 12 | passed |
| 13 | Pneumatic gripper and routed hoses do not validate |
| 14 | passed |
| 15 | The tending reach layout was certified for other cell inputs (58c7ffe2139b9fcb6b8accacf5d3096574626ac186d9e3253b890242bcc263bb, now aed7b932354ab54a4d4443f161db0b71825c99f284464ff9a51bfd41ca41e0ce); solve and store it again with `./haxeon/scripts/haxeon run --project=machinekit/examples/machine-tending/haxeon.json -- --store` |
| 16 | The tending reach layout was certified for other cell inputs (58c7ffe2139b9fcb6b8accacf5d3096574626ac186d9e3253b890242bcc263bb, now aed7b932354ab54a4d4443f161db0b71825c99f284464ff9a51bfd41ca41e0ce); solve and store it again with `./haxeon/scripts/haxeon run --project=machinekit/examples/machine-tending/haxeon.json -- --store` |
| 17 | Mill casting Base mass 58.284275965218 kg differs from its checked design (58.6401 kg) |
| 18 | pneumatic service BOM changed after round trip |
| 19 | screwXNut overlaps table at 0,0,-250 by 14.492166911009697 mm³ |
| 20 | single-25: Mate tool/airHoseA-valve (fixed) holds tool/airHoseA on tool/valve, which it does not touch: 0.5 mm apart; Mate tool/airHoseB-valve (fixed) holds tool/airHoseB on tool/valve, which it does not touch: 0.5 mm apart; Nothing holds up 1 member(s) from the ground: tool/airHoseA; Nothing holds up 1 member(s) from the ground: tool/airHoseB |
| 21 | BOM part number "LINEAR-MOUNT-CLEAT-H29.585" has conflicting metadata or values |
| 22 | Group path must not be empty |
| 23 | Barrel material volume: 2733185.60862312 instead of 3330088.212805181 |
| 24 | The tending reach layout was certified for other cell inputs (58c7ffe2139b9fcb6b8accacf5d3096574626ac186d9e3253b890242bcc263bb, now aed7b932354ab54a4d4443f161db0b71825c99f284464ff9a51bfd41ca41e0ce); solve and store it again with `./haxeon/scripts/haxeon run --project=machinekit/examples/machine-tending/haxeon.json -- --store` |

BOM snapshot counts and identities are recorded in
`rn-baseline/bom/baseline-inventory.json` and the raw per-machine JSONs.
R2 must still check all constructible scene assemblies, including strict-BOM
failures; do not narrow coverage to the 20 successful BOM files. Existing
BomExampleTests covers 22 example assemblies plus two CNC-router variants;
machine tending cannot construct. Check the enclosed hobber artifact separately.

## Measurements

Same saved enclosure artifact for before/R1: 403 occurrences, 183 geometry
definitions, 165 distinct scene numbers. Files: `tmp/rn-metrics/before.mtrg`,
`before.json`, `after-r1.json` under the shared temporary root above.

| Stage | Longest label | Average label | Longest occurrence ID | Longest scene number |
| --- | ---: | ---: | ---: | ---: |
| Before | 192 | 53.7221 | 47 | 192 |
| R1 | 48 | 21.9950 | 47 | 192 |
| R2 | 33 | 15.5707 | 38 | 192 |

Before's longest label/number is the coordinate-coded captured roller track.
Longest ID is `machine/column/anchor-297d29ec08a4196e572c-bolt`.
R1's longest remaining label is a formatted hashed anchor; R2 must remove it.
R3 survey: 39 of 165 distinct enclosure designations exceed 48 characters,
including brackets, panels, shafts and a supply. Full survey: rn-designation-survey.json.

## Current state and remaining work

R1 is committed as `61eec37b0`. ProjectKit passed 413 assertions and app naming
passed 24 assertions. A test-only hard-coded schema v1 was corrected to the
codec VERSION constant (v11); no production schema changed. Full R1 regressions
are complete: all focused projects passed (app, hobber 2345,
hobbing 1161, BOM-document/key, procurement, assembly integrity and joint
fasteners). Joint-fastener totals match: 51 joints, 232 screws, 16 dowels,
zero fit findings. Handle 4276 is terminal. All 24 smoke shards match exactly (9 pass, same 15 exceptions).
The standalone BOM-example test reproduces the exact pre-existing cleat
collision. Both regression handles are terminal; no new failures.

R2 is implemented and verified; its separate commit follows. AssemblyIntegrity passes. The initial actual-example audit found
coordinate IDs in rack push screws and enclosure spacers/rollers; those and
hanger sign IDs now use indices. The final audit passed 28 assemblies/4561 occurrences, including RKE/ZPP variants; fresh enclosure metrics
preserve 403 parts/183 definitions. Longest label is now "Tray rear centre
joint seal right" (33), longest path enclosure/trayRearCentreJointSealRight
(38), average 15.5707. Both focused checks are terminal. Full R2 baseline
regressions passed the same focused projects. All24 smoke shards match exactly
(9 pass, same15 exceptions); standalone BOM-example reproduces the same cleat
collision. Focused rack-mount adds64 passing local contact assertions and both
router builds. All R2 handles are terminal; no new regression.
R3–R4 drafts remain unapplied/uncompiled outside source roots in tmp/rn-r3
and rn-r4. Inspect current source before applying; do not
regenerate old drafts over fixes. R2 needs actual all-example/enclosure runtime
proof and anchor regression coverage. R3 needs remaining generators exposed by
constructor enforcement, default-recipe hard failure accounting, exact identity
and merging tests, and BOM baseline updates with reasons. R4 needs compiled
behavioral coverage, including post-cure and non-vacuous anchor assertions.
No step is complete until its required regression comparison and separate
commit are done.

Tools/evidence: rn-regress.sh, rn-compare-smoke.py, rn-r1-results/.
The comparison keeps pending shards pending and flags changed failures for
review. Do not normalize away unexplained geometry or numeric differences.
Detailed preparation history was preserved in tmp/rn-work-log.md.

Before completion, audit every original requirement plus R4 against actual
source, committed history, logs and metrics. Final report includes separate
step explanations/results, complete before/after table, BOM changes and open
decisions. Keep the goal active until all four commits and verification are done.
