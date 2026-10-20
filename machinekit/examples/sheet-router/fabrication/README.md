# SR6 prepared frame and DIY fabrication packet

**Draft, not released for fabrication.** The frozen frame layout and sections are
retained, including sixteen braces and the Y1575 extra leg pair. The governing
fabrication and alignment procedure is [DIY_ROUTE.md](DIY_ROUTE.md). No large
machine tool is required for the complete base.

## Prepared geometry and seams

`SheetRouterFramePreparation.hx` partitions the forty actual fixed-frame tube
blanks into continuous stringers and coped/intersection-trimmed members. It
exports each resulting solid to STEP, checks for overlapping/disconnected pieces,
and extracts seams from the prepared B-rep contacts. Both beds have 68 pieces:

| Bed Y | Joined contacts | Native seam segments | Geometry findings |
| --- | ---: | ---: | ---: |
| 2600 | 172 | 944 | 0 |
| 3150 | 170 | 904 | 0 |

These replace the initial square-ended packet's eighty unresolved weld
annotations and zero seams. Native edge counts are not weld instructions.
`weld-stitches.csv` selects nominal 5 mm legs and starts with 40 mm opposed
stitches, increasing extent only where the preliminary capacity screen requires
it. Flush Butt edges are excluded rather than assigned imaginary fillets.

**Weld qualification remains open.** The conservative screen uses 5 kN force and
arbitrary-direction family moments of 0.1 kNm (brace), 0.25 kNm (table/rack) and
0.75 kNm (post/frame), with an assumed 80 MPa throat-stress limit. These are
engineering screening allocations, not extracted joint loads or a qualified
WPS. 38 short-bed and 36 long-bed short/collinear seam groups do not pass this
allocation, even at their full available length. `weld-detail.json` identifies every failed group and retains
null stress where an arbitrary moment is unresisted. Resolve with verified
joint-load directions and/or a detailed connection before shop release; do not
increase bead size or silently waive a failed group.

The prepared steel mass is approximately 1022.3 kg (short) and 1095.6 kg (long),
using 7850 kg/m³. Removing intersecting interior metal reduces the original
fixed-frame union mass by approximately 69.1/73.2 kg. The frozen beam-centreline
model does not represent this local connection preparation or certify its weld
stiffness. The prior full structural rerun remains evidence for the frozen
layout, not a strength qualification of these draft connections.

## DIY fabrication and metrology

Follow the full sequence in [DIY_ROUTE.md](DIY_ROUTE.md): full tack in jig,
balanced short stitches, thermal relief (VSR fallback), rest and repeat survey,
epoxy-levelled or adjustable rail seats, rails, measured squaring, then SR6-C.
The procedure includes furnace envelope/payload, 2–6 mm proposed epoxy layers,
steel rail-bolt threads, alignment instruments and tolerances. Resin application
approval and witness qualification remain required. The beam/feet go separately
to an ordinary shop; its actual travel and indexed datum transfer must be agreed.

Two bolted/dowelled foot models were checked at both beds' modal and static-X/Y
governing poses. The improved machined-face candidate still gives only
1.909 N/µm worst long-bed Y, below 2 N/µm; its first gantry mode is 25.395 Hz
versus 26.986 Hz welded. Retain welded feet. This finding applies to these
assumed joints; it does not rule out every possible bolted connection.

## Service access

The service table keeps its original plenum envelope, plugs the original lower
ports and moves each lower port 120 mm in Y to clear the supporting tubes.
Six valves are outside the side frame on removable floor stands. The middle
valves shift 300 mm in Y to avoid the extra post pair. The packet records six
suction routes, six feed routes, a rear crossover and the pump feed. Routes use
60/90 mm outside envelopes, R75/R100 bends and 5 mm clearance allowance.

Native checks test routes and 450 mm valve-withdrawal envelopes against the
frame, brackets, service equipment and sampled moving gantry/chain geometry on
both beds (185 checks each after the final stand revision). Six top inspection-lance paths give plenum access after removing the
spoilboard, avoiding a requirement to crawl through the braces. These sampled
clearances do not certify a purchased hose's actual bend radius or installation
clamps. The front sheet-loading mouth remains part of the assembly sweep.

## Reproduction and release

Run the FEM executable with `--export-only --fabrication`; the default output is
`build/sheet-router-work/sr6/native`. Then:

```sh
python3 machinekit/examples/sheet-router/fabrication/render.py \
  --native build/sheet-router-work/sr6/native \
  --packets build/sheet-router-work/sr6/model/packets \
  --output machinekit/examples/sheet-router/fabrication
python3 machinekit/examples/sheet-router/fabrication/detail.py \
  --native build/sheet-router-work/sr6/native \
  --output machinekit/examples/sheet-router/fabrication
```

Per-bed STEP geometry is retained as build evidence; committed CSV, SVG and JSON
views carry source hashes. Cut lengths are projected prepared-stock extents,
without kerf or labelled shop trim allowance; STEP defines each cope. Confirm
actual stock availability before ordering. No obsolete upper-frame tie-ins are
added a second time.

`shop-release.json` is fail-closed. Geometry, service, weld-load/WPS review,
material/relief procedure, resin application approval, shop capability and signed
packet review must pass before drawings are released. Actual relief/rest, rail
seat/alignment and SR6-C modal measurements are later build witnesses. Never
replace those measurements with simulated sign-offs.

Condensed check results and source identities: [validation.json](validation.json).
Full native STEP, solver and run-log evidence is retained in
`build/sheet-router-evidence/sr6-diy.tar.gz` (extracts as `sr6-diy/`).
