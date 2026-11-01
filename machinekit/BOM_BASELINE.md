# BOM baseline

Baseline revision: `d7d644702` (before B1). The full gate ran through
`heavy-run`.

The B0 survey in `BOM_PLAN.md` reports 107 members and 76 BOM lines for the
bench mill. That survey identified recipe keys and raw dimensions in part
numbers, 26 brake lines for one purchase, missing procurement classifications
and fasteners, and included workpiece/tooling. The measurements below cover the
exportable machines.

## Starting gate

The examples and smoke suite built successfully against the prebuilt OCCT
provider. The complete run passed 11 of 24 shards. Results:

| Shard | Result |
| --- | --- |
| 1 | Pass |
| 2 | Fail: `machinekit.transmission.gear-rack machinekit.transmission.gear-rack\|designation:9:29-15-105 Envelope: expected one solid, got 3` |
| 3 | Fail: `machinekit.transmission.gear-rack machinekit.transmission.gear-rack\|designation:9:29-15-105 Envelope: expected one solid, got 3` |
| 4 | Fail: `Missing component parameter "role"` |
| 5 | Pass |
| 6 | Pass: `SheetRouterChecks.run`, 426 assertions. |
| 7 | Pass: `CncRouterChecks.run`. |
| 8 | Pass: `CoreXyPrinterChecks.run`. |
| 9 | Pass: gears. |
| 10 | Pass: `ToolProfileTests.run`. |
| 11 | Pass: `ToolLayoutTests.run`. |
| 12 | Fail: `23HS22-2804S face: 57 != 57.3` |
| 13 | Fail: `Barrel material volume: 2733185.60862312 instead of 3330088.212805181` |
| 14 | Pass: `MillPartTests.run`. |
| 15 | Pass: `ScrewdrivingEquipmentTests.run`. |
| 16 | Fail: ``The tending reach layout was certified for other cell inputs (a0ea47c532d830c9888f4fdd7f6ea0b6144cb71c0c83797fb4c8284131c62de8, now 4b793a015e84eb25ab7624a49f30b28fd06322ddfb97905fd50d40a37c18d1f1); solve and store it again with `./haxeon/scripts/haxeon run --project=machinekit/examples/machine-tending/haxeon.json -- --store` `` |
| 17 | Fail: `screwXNut overlaps table at 0,0,-250 by 14.492166911009694 mm³` |
| 18 | Fail: `single-25: Mate tool/airHoseA-valve (fixed) holds tool/airHoseA on tool/valve, which it does not touch: 0.5 mm apart; Mate tool/airHoseB-valve (fixed) holds tool/airHoseB on tool/valve, which it does not touch: 0.5 mm apart; Nothing holds up 1 member(s) from the ground: tool/airHoseA; Nothing holds up 1 member(s) from the ground: tool/airHoseB` |
| 19 | Fail: `Missing connector "PRELOADED-PINION-MOUNT-ASSUMED-c27eb1e5ed4e/motor"` |
| 20 | Fail: `obsolete free-text material recipe input is rejected` |
| 21 | Fail: `Missing component parameter "role"` |
| 22 | Fail: `Axis assembly has no member "rackFoot0" yet` |
| 23 | Fail: same stale tending reach certificate message as shard 16. |
| 24 | Pass: `FastenerThreadTests.run` and eight other groups. |

The amended B1 gate also passed 11 of 24 shards; comparison found no new
failing shard or exception message. All 22 examples that exported at baseline
exported in the amended shape with the same procurement part numbers. The
bench mill and sheet router also retained their baseline line quantities.

## Initial exported lines

These measurement runs supplied a fixed `2026-10-31T00:00:00Z` timestamp to
the exporter binary for repeatability; it is not their actual generation time.
The public entrypoint supplies the current UTC time.
The initial files predate the explicit use-kind field. A repeat into
`build/bom-baseline-uses/` retained all 22 machines’ line identities,
quantities, use paths and counts.

Counts are per rolled-up line, and categories overlap. `Lint invalid` combines
the character, length, null and long-decimal checks. `Invalid characters/length`
applies the planned character and 40-character rules; `pipe`, `null` and
`long decimal` count those literal patterns (long decimal means four or more fractional digits,
a screening measure rather than proof of improper formatting). `Assumed` is a
case-insensitive text search for generic or assumed in the part number or
description, not an engineering classification. Brake pieces are lines with
the shaft-brake-part recipe. The initial measurement also checked use quantities
and occurrence paths against the assembly tree.
Every initial line lacks a procurement kind and a revision. Presence of specific missing supplies and
workpiece/tooling requires a manual survey and is not inferred from these counts.

| Machine | Lines | Lint invalid | Invalid characters/length | Pipe | Null | Long decimal | Assumed text | Brake pieces |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| bench-mill | 76 | 60 | 60 | 47 | 8 | 1 | 33 | 28 |
| cnc-router | 40 | 24 | 24 | 11 | 0 | 0 | 2 | 0 |
| cobot-arm | 26 | 17 | 16 | 11 | 0 | 8 | 5 | 0 |
| corexy-plotter | 17 | 10 | 10 | 5 | 0 | 0 | 1 | 0 |
| corexy-printer | 55 | 37 | 37 | 20 | 0 | 0 | 6 | 0 |
| enclosed-bench-mill | 114 | 97 | 97 | 84 | 9 | 3 | 36 | 28 |
| flexspline-hobber | 74 | 60 | 60 | 57 | 14 | 19 | 37 | 0 |
| gantry-picker | 62 | 49 | 45 | 12 | 0 | 4 | 4 | 0 |
| gantry-welder | 83 | 52 | 52 | 14 | 0 | 2 | 5 | 0 |
| linear-stage | 11 | 5 | 5 | 4 | 0 | 0 | 0 | 0 |
| mobile-base | 11 | 3 | 3 | 2 | 0 | 1 | 2 | 0 |
| mobile-base-cell | 48 | 26 | 22 | 21 | 0 | 10 | 18 | 0 |
| mobile-welder | 53 | 26 | 26 | 20 | 0 | 6 | 16 | 0 |
| overhead-arm | 76 | 57 | 53 | 30 | 2 | 9 | 18 | 0 |
| picking-station | 7 | 0 | 0 | 0 | 0 | 0 | 0 | 0 |
| press-brake | 37 | 27 | 27 | 18 | 0 | 1 | 13 | 0 |
| robot-arm | 32 | 23 | 19 | 19 | 0 | 9 | 16 | 0 |
| robot-screwdriver | 38 | 24 | 24 | 20 | 0 | 5 | 15 | 0 |
| robot-welder | 43 | 21 | 21 | 19 | 0 | 5 | 14 | 0 |
| sheet-router | 224 | 190 | 190 | 155 | 105 | 32 | 35 | 0 |
| track-arm | 53 | 31 | 31 | 23 | 1 | 5 | 16 | 0 |
| xyz-stage | 41 | 33 | 33 | 11 | 0 | 0 | 2 | 0 |

The bench-mill export contains 28 shaft-brake component lines: 26 sectors,
one hub and one disc. These are one assumed purchase. The B0 description's
26 lines counts the sectors only. B5 records the resulting single purchase line. The lathe directory contains a plan but no machine
implementation. The machine-tending export could not be measured: constructing
`TendingControllers` rejects its stale reach certificate (certified input
`a0ea47c5…`, current input `4b793a01…`). The certificate is owned by a separate
study and was not regenerated.

The bench-mill interference failure involves the X nut and the table's
clearance recess at an extreme pose. Its geometry includes a flanged nut and
the table's machined clearance; the cause has not been isolated well enough
to justify a small geometry change. The BOM export does not run that check.

For the three reference exports inspected by member path, the bench mill and
CNC router each have three lines for stock, end mill and toe clamps. The
robot welder has six workpiece lines under `work/` and one fixture line. These
are candidates for `NotSupplied`; the classification is not present yet.
Those three exports contain no `machinekit.standard.socket-head-cap-screw`
or `machinekit.standard.hex-bolt` recipe lines. This is an observed absence,
not a count of how many fasteners the assemblies need. The picking station
contains one socket-head screw line with quantity 12.

The initial report-only recipe lint constructed 212 built-in defaults, of
which 166 had invalid part numbers. Three further defaults could not be
constructed: `cast-in-steel-insert` needs `role`, `linear-mount-nut-cassette`
needs a mounted-element source identity, and `source-casting` needs `source`.
They remain visible in the report instead of being counted as passing.

The B2 recipe report checked the same 212 constructible defaults and found
zero invalid part numbers. Those three recipes still require explicit inputs;
focused Haxe tests check a representative instance of each.

The B2 report measured all 22 exportable machines. The lint-invalid column
counts procurement lines failing the part-number rules; remaining invalid
lines are non-recipe members. The lint also checked engineering occurrences
and found no invalid recipe-derived numbers.
Across the 22 machines, procurement lines fell from 1,221 to 1,141 and
lint-invalid lines from 872 to 242. These are line counts, not quantities.

| Machine | Initial lines | Initial lint invalid | B2 lines | B2 lint invalid |
| --- | ---: | ---: | ---: | ---: |
| bench-mill | 76 | 60 | 76 | 13 |
| cnc-router | 40 | 24 | 40 | 10 |
| cobot-arm | 26 | 17 | 25 | 6 |
| corexy-plotter | 17 | 10 | 17 | 5 |
| corexy-printer | 55 | 37 | 54 | 14 |
| enclosed-bench-mill | 114 | 97 | 114 | 13 |
| flexspline-hobber | 74 | 60 | 76 | 0 |
| gantry-picker | 62 | 49 | 62 | 33 |
| gantry-welder | 83 | 52 | 83 | 38 |
| linear-stage | 11 | 5 | 11 | 0 |
| mobile-base | 11 | 3 | 11 | 1 |
| mobile-base-cell | 48 | 26 | 48 | 1 |
| mobile-welder | 53 | 26 | 53 | 6 |
| overhead-arm | 76 | 57 | 76 | 23 |
| picking-station | 7 | 0 | 7 | 0 |
| press-brake | 37 | 27 | 37 | 9 |
| robot-arm | 32 | 23 | 32 | 0 |
| robot-screwdriver | 38 | 24 | 38 | 4 |
| robot-welder | 43 | 21 | 43 | 2 |
| sheet-router | 224 | 190 | 144 | 34 |
| track-arm | 53 | 31 | 53 | 8 |
| xyz-stage | 41 | 33 | 41 | 22 |


## B3 bench-mill measurement

Across the 22 exportable machines, B3 produced 1,143 procurement lines;
399 have no fingerprint because their code-only revision inputs remain
undeclared. Recipe-derived procurement lines all have fingerprints.

| Machine | B3 lines | Fingerprint unavailable |
| --- | ---: | ---: |
| bench-mill | 77 | 3 |
| cnc-router | 40 | 23 |
| cobot-arm | 25 | 13 |
| corexy-plotter | 17 | 10 |
| corexy-printer | 54 | 24 |
| enclosed-bench-mill | 115 | 3 |
| flexspline-hobber | 76 | 0 |
| gantry-picker | 62 | 42 |
| gantry-welder | 83 | 63 |
| linear-stage | 11 | 1 |
| mobile-base-cell | 48 | 14 |
| mobile-base | 11 | 6 |
| mobile-welder | 53 | 26 |
| overhead-arm | 76 | 30 |
| picking-station | 7 | 1 |
| press-brake | 37 | 12 |
| robot-arm | 32 | 3 |
| robot-screwdriver | 38 | 10 |
| robot-welder | 43 | 20 |
| sheet-router | 144 | 46 |
| track-arm | 53 | 22 |
| xyz-stage | 41 | 27 |

The bench-mill export has 77 procurement lines, compared with 76 at B2.
The X- and Y-axis motor plates now have separate drawing identities; their
previous dimension-derived number combined them.
Three procurement lines have no fingerprint: the end mill, bearing-block
blank and toe clamp are code-only entries without declared revision inputs.
The remaining 74 have fingerprints. These counts precede purchase-boundary
and supplied-equipment changes in later phases.

## Initial content signals

These are counts of lines, not an assessment of completeness. Fastener lines
count only the five standard screw, bolt, nut and washer recipe types listed
in the initial measurement. The cable, chain, lubrication and enclosure columns
are keyword matches in descriptions or part numbers. The work/tool column
counts lines with selected occurrence-path segments and includes false positives
(such as a robot tool flange). A zero means no matching line, not proof that a
whole category is absent.

| Machine | Fastener recipe lines | Cable/hose text | Chain text | Lubrication text | Enclosure text | Work/tool path candidates |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| bench-mill | 0 | 0 | 0 | 0 | 0 | 3 |
| cnc-router | 0 | 0 | 0 | 0 | 0 | 3 |
| cobot-arm | 2 | 0 | 0 | 0 | 0 | 0 |
| corexy-plotter | 0 | 0 | 0 | 0 | 0 | 1 |
| corexy-printer | 0 | 0 | 0 | 0 | 0 | 1 |
| enclosed-bench-mill | 0 | 1 | 0 | 0 | 22 | 2 |
| flexspline-hobber | 0 | 1 | 0 | 0 | 2 | 1 |
| gantry-picker | 0 | 1 | 0 | 0 | 0 | 7 |
| gantry-welder | 0 | 7 | 0 | 0 | 0 | 11 |
| linear-stage | 1 | 0 | 0 | 0 | 0 | 0 |
| mobile-base | 0 | 0 | 0 | 0 | 0 | 0 |
| mobile-base-cell | 0 | 1 | 0 | 0 | 0 | 8 |
| mobile-welder | 0 | 2 | 0 | 0 | 0 | 11 |
| overhead-arm | 0 | 1 | 0 | 0 | 0 | 8 |
| picking-station | 1 | 0 | 0 | 0 | 0 | 0 |
| press-brake | 0 | 1 | 0 | 0 | 0 | 0 |
| robot-arm | 0 | 1 | 0 | 0 | 0 | 8 |
| robot-screwdriver | 1 | 0 | 0 | 0 | 0 | 10 |
| robot-welder | 0 | 7 | 0 | 0 | 0 | 11 |
| sheet-router | 15 | 6 | 6 | 0 | 0 | 1 |
| track-arm | 0 | 7 | 0 | 0 | 0 | 8 |
| xyz-stage | 0 | 0 | 0 | 0 | 0 | 0 |

## B4 procurement classification

Strict export requires procurement intent and maturity on every engineering line; assumed lines also require structured selection terms. The 22 exportable machines are measured below. The pre-existing machine-tending certificate prevents export.

| Machine | Procurement lines | Make | Buy | Assumed | Fingerprint unavailable |
| --- | ---: | ---: | ---: | ---: | ---: |
| bench-mill | 73 | 24 | 49 | 39 | 0 |
| cnc-router | 37 | 19 | 18 | 8 | 20 |
| cobot-arm | 25 | 15 | 10 | 18 | 13 |
| corexy-plotter | 16 | 7 | 9 | 3 | 9 |
| corexy-printer | 53 | 22 | 31 | 17 | 23 |
| enclosed-bench-mill | 112 | 49 | 63 | 52 | 1 |
| flexspline-hobber | 70 | 36 | 34 | 38 | 0 |
| gantry-picker | 60 | 36 | 24 | 10 | 40 |
| gantry-welder | 75 | 40 | 35 | 17 | 55 |
| linear-stage | 11 | 2 | 9 | 2 | 1 |
| mobile-base | 11 | 3 | 8 | 6 | 6 |
| mobile-base-cell | 42 | 19 | 23 | 19 | 8 |
| mobile-welder | 45 | 18 | 27 | 24 | 18 |
| overhead-arm | 74 | 42 | 32 | 20 | 28 |
| picking-station | 7 | 3 | 4 | 3 | 1 |
| press-brake | 32 | 11 | 21 | 26 | 12 |
| robot-arm | 30 | 16 | 14 | 13 | 1 |
| robot-screwdriver | 35 | 18 | 17 | 18 | 7 |
| robot-welder | 35 | 15 | 20 | 18 | 12 |
| track-arm | 49 | 22 | 27 | 21 | 18 |
| xyz-stage | 41 | 24 | 17 | 7 | 27 |
| sheet-router | 143 | 75 | 68 | 43 | 45 |

Across these machines, B4 has 1,076 procurement lines: 516 Make and 560 Buy. Of those lines, 422 are Assumed and 345 lack fingerprints. Workpieces, tools and clamps remain in the engineering tree as NotSupplied. The full gate passed 11/24 shards; comparison found no new failing shard or exception message.

## B5 purchase-unit measurement

The 22 exportable machines contain 1,004 procurement lines; 350 are Assumed and 335 lack fingerprints. The engineering tree retains each included piece with its local purchase owner. The bench mill has one brake line and 27 included brake pieces, down from 28 procurement lines for its brake in B4. The full gate passed 11/24 shards, with no new failing shard or exception message.

| Machine | Procurement lines | Assumed | Fingerprint unavailable |
| --- | ---: | ---: | ---: |
| bench-mill | 46 | 12 | 0 |
| cnc-router | 37 | 8 | 20 |
| cobot-arm | 22 | 15 | 13 |
| corexy-plotter | 16 | 3 | 9 |
| corexy-printer | 53 | 17 | 23 |
| enclosed-bench-mill | 83 | 23 | 1 |
| flexspline-hobber | 70 | 38 | 0 |
| gantry-picker | 60 | 10 | 40 |
| gantry-welder | 73 | 15 | 53 |
| linear-stage | 11 | 2 | 1 |
| mobile-base | 11 | 6 | 6 |
| mobile-base-cell | 42 | 19 | 8 |
| mobile-welder | 43 | 22 | 16 |
| overhead-arm | 74 | 20 | 28 |
| picking-station | 7 | 3 | 1 |
| press-brake | 31 | 25 | 12 |
| robot-arm | 30 | 13 | 1 |
| robot-screwdriver | 33 | 16 | 5 |
| robot-welder | 33 | 16 | 10 |
| sheet-router | 141 | 41 | 45 |
| track-arm | 47 | 19 | 16 |
| xyz-stage | 41 | 7 | 27 |

## B6 declared content measurement

The 22 exportable machines contain 1,022 procurement lines; 362 are Assumed and 343 lack fingerprints. The bench mill has 55 lines, including 48 M4x16 rail screws, eight M5x12 and four M6x12 motor screws. Its cable and lubrication declarations are provisional as described in the plan. The existing tending certificate failure now reports a different current input fingerprint because the motor-plate geometry changed. The full gate passed 11/24 shards with no new failing shard. Shards 16 and 23 differ from baseline only in that disclosed fingerprint; the certificate was not regenerated.

| Machine | Procurement lines | Assumed | Fingerprint unavailable |
| --- | ---: | ---: | ---: |
| bench-mill | 55 | 18 | 4 |
| cnc-router | 37 | 8 | 20 |
| cobot-arm | 22 | 15 | 13 |
| corexy-plotter | 16 | 3 | 9 |
| corexy-printer | 53 | 17 | 23 |
| enclosed-bench-mill | 92 | 29 | 5 |
| flexspline-hobber | 70 | 38 | 0 |
| gantry-picker | 60 | 10 | 40 |
| gantry-welder | 73 | 15 | 53 |
| linear-stage | 11 | 2 | 1 |
| mobile-base | 11 | 6 | 6 |
| mobile-base-cell | 42 | 19 | 8 |
| mobile-welder | 43 | 22 | 16 |
| overhead-arm | 74 | 20 | 28 |
| picking-station | 7 | 3 | 1 |
| press-brake | 31 | 25 | 12 |
| robot-arm | 30 | 13 | 1 |
| robot-screwdriver | 33 | 16 | 5 |
| robot-welder | 33 | 16 | 10 |
| sheet-router | 141 | 41 | 45 |
| track-arm | 47 | 19 | 16 |
| xyz-stage | 41 | 7 | 27 |

## B7 rollout measurement

The 22 exportable machines contain 1,058 procurement lines, including 380
Assumed lines. None of the procurement lines or 3,798 engineering
occurrences lacks a fingerprint. Part-number lint found no invalid engineering
occurrences or procurement lines; 212
constructible default recipes passed lint and three recipes have no valid
default construction. The reference bench mill has 55 procurement lines.
The machine-tending export still stops at its stale reach certificate.
The full B7 gate passed 11/24 shards and matched all B6 failure messages.
Compared with the untouched baseline, only the accepted tending input
fingerprints in shards 16 and 23 differ.
Assumed selection terms describe known inputs; unrated hoses, terminations,
and other unmodeled details still require engineering selection.

| Machine | Procurement lines | Assumed | Fingerprint unavailable |
| --- | ---: | ---: | ---: |
| bench-mill | 55 | 18 | 0 |
| cnc-router | 38 | 8 | 0 |
| cobot-arm | 25 | 18 | 0 |
| corexy-plotter | 16 | 3 | 0 |
| corexy-printer | 53 | 17 | 0 |
| enclosed-bench-mill | 92 | 29 | 0 |
| flexspline-hobber | 70 | 38 | 0 |
| gantry-picker | 74 | 10 | 0 |
| gantry-welder | 84 | 20 | 0 |
| linear-stage | 11 | 2 | 0 |
| mobile-base | 11 | 6 | 0 |
| mobile-base-cell | 42 | 19 | 0 |
| mobile-welder | 43 | 22 | 0 |
| overhead-arm | 76 | 20 | 0 |
| picking-station | 7 | 3 | 0 |
| press-brake | 31 | 25 | 0 |
| robot-arm | 30 | 13 | 0 |
| robot-screwdriver | 33 | 16 | 0 |
| robot-welder | 33 | 21 | 0 |
| sheet-router | 142 | 41 | 0 |
| track-arm | 47 | 24 | 0 |
| xyz-stage | 45 | 7 | 0 |
