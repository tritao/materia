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
hobber IDs. FoundationIntegrity must reject ungrouped members. Use existing `withGroup` scopes for each part addition so appended
members cannot inherit a purpose silently. Test role coverage, bed/column groups, real anchor colocation,
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
| R3 | 33 | 15.5707 | 38 | 45 |

Before's longest label/number is the coordinate-coded captured roller track.
Longest ID is `machine/column/anchor-297d29ec08a4196e572c-bolt`.
R1's longest remaining label is a formatted hashed anchor; R2 must remove it.
R3 survey: 39 of 165 distinct enclosure designations exceed 48 characters,
including brackets, panels, shafts and a supply. Full survey: rn-designation-survey.json.

## Current state and remaining work

R1 is committed as `61eec37b0`; R2 as `5b4976e2e`. Both passed all focused
baseline projects; their 24 smoke shards exactly match the measured baseline
(9 pass, same 15 exceptions), and the standalone BOM-example check reproduces
the initial cleat collision. ProjectKit passed 413; app naming 24. R2 additionally
passed 28 assemblies/4561 occurrence names and 64 local rack contact assertions.

R3 is applied and uncommitted. Its hard constructor/lint cap is 48. Temporary
report-only diagnostic passes were removed before validation. All 225 defaults
and exact-identity/cap-boundary tests pass, with only the same 3 source-dependent
defaults excluded. Actual naming audit passed 28 assemblies/4561 occurrences.
Enclosure export preserves 403 occurrences/183 definitions, with longest number 45.
Distinct scene numbers rise 165→167: two rounded outrigger names previously
covered different exact block inputs. Identical/reordered block definitions
retain one number/values key/fingerprint.

All 23 BOM exports were attempted: 21 succeed (1066 procurement lines), including
newly exportable sheet-router 172. Hobber retains missing structured selection
requirements; tending retains its stale certificate, now with current fingerprint
3314e3d4f2431a95675633dfc45e8b41c1aec07e109e97562e47b87c2d1a5c88.
All 20 previously successful exports preserve line identities/quantities/uses
under canonical block ordering. Lint: 3710 engineering occurrences and 1066
procurement lines, zero invalid; 225 defaults valid, same 3 unavailable.
R3 full focused regressions pass, including hobber 2345/hobbing 1161, app CAD,
BOM-document/key, procurement, integrity and unchanged joint-fastener totals.
The example-build matrix and dedicated mill-frame suite (951) pass. Smoke
improves 9→10 passes: sheet-router shard 6 passes 426 assertions. No previously
passing shard remains failed. Tending shards15/16/24 only change their current
fingerprint; shard 21/standalone now reach the pre-existing hobber selection
failure after the identity collisions are cleared. Other failures match exactly.
HMI child bodies use exact parent recipe keys plus role; their purchased
control number and geometry stay unchanged. The Y bracket test now checks its
recipe/description and unchanged quantity 2 instead of a superseded literal.
All R3 handles are terminal; the separate R3 commit follows verification.

R4 is next. Its library groups use typed placement purposes and existing
`withGroup` scopes, preserving catalogue socket roles and part identity.
Verify all producers/copies, actual artifact group paths, 106 anchor colocation,
post-cure members in the runnable mill-frame fixture, ungrouped rejection and
full regression comparisons before its separate commit. No portable artifact
schema change is needed.

Tools/evidence: rn-regress.sh, rn-compare-smoke.py, rn-r1-results/.
The comparison keeps pending shards pending and flags changed failures for
review. Do not normalize away unexplained geometry or numeric differences.
Detailed preparation history was preserved in tmp/rn-work-log.md.

Before completion, audit every original requirement plus R4 against actual
source, committed history, logs and metrics. Final report includes separate
step explanations/results, complete before/after table, BOM changes and open
decisions. Keep the goal active until all four commits and verification are done.
