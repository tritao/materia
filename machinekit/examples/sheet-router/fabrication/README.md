# SR6 prepared frame and DIY fabrication packet

**Draft, not released for fabrication.** The frozen frame layout and sections are
retained, including sixteen braces and the Y1575 extra leg pair. The governing
fabrication and alignment procedure is [DIY_ROUTE.md](DIY_ROUTE.md). No large
machine tool is required for the complete base.

**2026-10-22 qualification extension:** the populated-ATC, full-history J4
brace study finds a baseline dynamic range of **13.03 MPa** at the long-bed
lower-stringer/middle-leg joint (W066, `lower1-p0` / `post13-p0`, h=50 mm),
above the unchanged **12.669 MPa** cut-off. The historical reaction qualification
below covers its recorded load histories; it does not qualify this expanded
planner envelope. Keep fabrication release gated while the joint rack-support
rerun and planner-history fatigue qualification are resolved. No reduced-brace
geometry is adopted. See the [brace study](../fem/BRACE_STUDY.md).

## Prepared geometry and seams

`SheetRouterFramePreparation.hx` partitions the forty actual fixed-frame tube
blanks into continuous stringers and coped/intersection-trimmed members. It
exports each resulting solid to STEP, checks for overlapping/disconnected pieces,
and extracts seams from the prepared B-rep contacts. Both beds have 68 pieces:

| Bed Y | Joined contacts | Native seam segments | Geometry findings |
| --- | ---: | ---: | ---: |
| 2600 | 158 | 920 | 0 |
| 3150 | 160 | 880 | 0 |

These replace the initial square-ended packet's eighty unresolved weld
annotations and zero seams. Native edge counts are not weld instructions.
`weld-stitches.csv` records the physical selected runs and their actual leg
sizes/end requirements. Flush Butt edges are excluded rather than assigned
imaginary fillets. Deliberate 1 mm cope clearances remove grazing contacts
between crossmembers/plan braces and posts, and end braces and upper rails.

**Qualification uses extracted joint reactions.** The original arbitrary-moment
screen's 38/36 open groups are mapped by stock names in each per-bed
`weld-qualification.json`. That allocation remains historical extent-selection
evidence in `weld-detail.json`; it does not decide strength after actual
reactions are available. The separate prepared-stock model checks every
physical piece-pair weld, all nine full-Z static poses per bed, and a modal
dynamic envelope at the governing pose. It credits 130 base weld groups and
the two beam-foot groups on each bed.

Use **100 million full-load reversals**, 3 m/s² plus the cutting allowance,
FAT36/FAT80 detail categories and the conservative **12.669 MPa** FAT36 cut-off.
The per-group CSV records leg, throat, range, limit and pass/fail. Required
corrections are full post/stringer runs, 6 mm middle post/stringer fillets
(tabbed full-size ends on the short bed), and two 92 mm, 8 mm-leg tabbed fillets
per beam foot. The gantry CSV uses local coordinates from the beam's left end,
beam Y centreline and lower foot datum. Follow
[WELD_QUALIFICATION.md](WELD_QUALIFICATION.md) for the model, sources, effective
lengths, actual load cases and limitations. No full-penetration or higher
fatigue-category upgrade is assumed.

The prepared steel mass is approximately 1022.3 kg (short) and 1095.6 kg (long),
using 7850 kg/m³. Removing intersecting interior metal reduces the original
fixed-frame union mass by approximately 69.1/73.2 kg. The frozen beam-centreline
model does not represent this local connection preparation. The reaction model
replaces those merged frame joints with separate prepared stocks and spatial
weld springs; the prior full structural rerun remains evidence for the frozen
layout, not the source of the new local weld reactions.

## DIY fabrication and metrology

Follow the full sequence in [DIY_ROUTE.md](DIY_ROUTE.md): full tack in jig,
balanced short stitches, thermal relief (VSR fallback), rest and repeat survey,
epoxy-levelled or adjustable rail seats, rails, measured squaring, then SR6-C.
The procedure includes furnace envelope/payload, 2–6 mm proposed epoxy layers,
steel rail-bolt threads, alignment instruments and tolerances. Resin application
approval and witness qualification remain required. The complete relieved
**2550.5 × 364 × 230 mm, 88.28 kg** welded beam-and-feet unit goes separately
to an ordinary shop. Its actual travel/support and indexed datum transfer must
meet the named A/B face tolerances in the checklist.

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
The earlier DIY draft remains in `/media/joao/DEV/materia-fem-runs/sheet-router-evidence/sr6-diy.tar.gz`.
The reaction-qualified native STEP, solver and run-log evidence is retained in
`/media/joao/DEV/materia-fem-runs/sheet-router-evidence/sr6-qualified.tar.gz` (extracts as `sr6-qualified/`).
