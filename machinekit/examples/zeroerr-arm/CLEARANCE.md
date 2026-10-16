# Straight-arm interference review

The current revision reduces service-cover interference and adds explicit
**simulation planning restrictions**. It does not recover UR/FAIRINO-equivalent
folded travel or qualify physical joint stops.

## What changed

Both ABS-assumption service collars change from OD84 x L30 mm to **OD83 x
L24.5 mm**, with a 2 mm inset from the tube shoulder and R0.5 outer edges.
The main bore remains 80 mm, giving a nominal 1.5 mm shell wall. The lips remain
outside the screw-head envelope; the smallest nominal axial lip/head gap is
0.25 mm. Fits, retention and manufacturing tolerances remain unqualified.

The tube, both structural adapters, all fasteners and every supplier actuator
retain exactly the same STEP geometry records. Joint/module placements and
the 425/401 mm axis spacing are unchanged. Independent STEP checks verify all
31 derived solids, the structural union, 41.998 mm tested cable passage and
zero material overlap between either cover and the other derived parts.

## Collision map and limits

`clearance-review-config.json` defines eleven conditional slices: three elbow
slices with nearby shoulder angles, the original folded witness, and seven
wrist slices with different J5 pitches. Wrist slices include the twelve distal
screws and locating pin. The map uses enclosing visual hulls, a **0.5 mm
distance margin**, 10-degree coarse sampling and transition refinement to
less than 0.1 degree. It is separate from exact CAD overlap observations.

The modified cover recovers approximately **0.2–1.2 degrees** at affected hull
boundaries. The nominal ready-pitch J4 slice changes from about
**−167.81/+160.23 degrees to −168.67/+160.47 degrees**. Elbow boundaries do not
change: those collisions involve the purchased actuator envelopes.

`straight-planning-window.json` restricts exported prototype scenes to:

| Joint | Simulation planning interval |
| --- | --- |
| J3 | −150 to +153 degrees |
| J4 | −154 to +154 degrees |

These are the intersection of the observed central slice intervals, rounded
inward with a 2-degree reserve. They are additional restrictions, **not a
globally collision-free box**. Other joint combinations, tools and fixtures
still require pose and continuous-motion collision checks. Source CAD
diagnostics deliberately remain able to test outside these restrictions.

The raw 60-pose review remains **55 clear / five colliding**, with one failed
Boolean query in a colliding fold. At J4 −170 degrees, cover overlap falls
from 24.19 to 7.76 mm³; at +170 degrees, from 205.30 to 148.11 mm³. These poses
remain excluded. The positive endpoint also contacts the tube/adapter; removing
cover material cannot resolve that structural contact.

**All 54 observed configurations inside the planning window are clear**, and
all five collision witnesses lie outside it. Six observed configurations are
excluded in total. All 35 unique synthetic-cycle configurations remain clear;
the separate continuous quintic hull check also passes. This does not prove
unobserved configurations clear.

## Efficient exact regression

`review-cover-regression.py` verifies identical assembly trees, exported axes,
tool frames and all 49 unchanged vendor/prepared files. For STEP files, the
schema, header and DATA records must be identical after removing only
`FILE_NAME` metadata. Different geometry, partitions, repair files, placement,
probe sets, incomplete observations or stale hashes reject reuse.

After that proof, prior non-cover pair observations remain applicable at the
same poses. Every pair involving either new cover is freshly checked at all
60 required configurations. The collector retains all original non-cover
collisions and failed queries. Four unit tests exercise timestamp acceptance,
geometry/schema rejection and incomplete-file rejection.

The baseline is the completed study at `bc3658a91`, retained locally under
`build/straight/clearance-baseline/`: `current.json`, engineering input, complete
motion report/checkpoints, required probes, original project and provenance.
Its immutable prepared directory must also remain available. Missing baseline
evidence fails closed; regenerate that committed baseline before reusing its
observations. Vendor/prepared assets remain outside Git.

After preparing/exporting the current geometry and generating motion probes:

```sh
python3 machinekit/examples/zeroerr-arm/review-cover-regression.py
./machinekit/examples/zeroerr-arm/run-built.sh --straight --motion-screen --cover-regression --pose-file=machinekit/examples/zeroerr-arm/build/straight/probes-0.json --shard=0
./machinekit/examples/zeroerr-arm/run-built.sh --straight --motion-screen --cover-regression --pose-file=machinekit/examples/zeroerr-arm/build/straight/probes-1.json --shard=1 --shards=2
python3 machinekit/examples/zeroerr-arm/review-cover-regression.py --collect
python3 -m unittest discover -s machinekit/examples/zeroerr-arm -p test_cover_regression.py
```

For a geometry change, export an unrestricted, passive review artifact with
`--straight --engineering --export --geometry-review-export`. Run the reference
example with `--straight-boundaries --review-label=baseline` and then
`--straight-boundaries --review-label=revised`. Run
`review-clearance-boundaries.py`, review its planning-window proposal, copy it
to `straight-planning-window.json`, and use `--straight --apply-planning-window`
to produce the restricted application project without remeshing. The normal
`--straight --export` path also applies the current restriction and performs
full solid/FK checks. Changed geometry requires a fresh restriction review.

Main outputs in `build/straight/` are `clearance-boundaries.png`, the two
`clearance-boundaries-*.json` reports, `clearance-improvement.json`,
`cover-geometry-equivalence.json`, `motion-report.json` and
`planning-window-review.json`.

Further deep-fold travel requires joint-offset/module-packaging work while
checking loads and cabling. Cosmetic housing changes alone cannot separate
intersecting actuator bodies. Shoulder sizing remains a separate release gate.
