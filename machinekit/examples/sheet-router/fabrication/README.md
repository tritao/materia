# SR6 frame fabrication and alignment — initial shop packet

This packet starts SR6 for the raised Z/table/rack datum and the fourth leg pair
at Y=1575. It does not add deep side girders. Structural freeze status and the
full rerun evidence are recorded in `../fem/README.md`. These are fabrication
planning documents; joint preparation, weld access, shop tolerances, the welding
procedure and anchor selection must be detailed before drawings are released.

## Layout, cut list and weld seams

`SheetRouterFabrication.hx` reads the actual `SheetRouter.base`: member endpoints,
section designations, `FrameAssembly.cutLength()` and grouped cut list; every
authored weld pair and leg size; geometry-derived seam candidates and diagnostics;
eight foot datums and both Y rail-pad datums. Run the existing FEM entry with
`--export-only --fabrication` and `SHEET_ROUTER_FABRICATION_OUTPUT` pointing under
root `build/`. The JSON output is the dimensional authority for this draft.

The per-bed CSV and SVG here combine that output with the frozen FEM manifest's four upper-frame tubes (40 fixed-frame stock members total). Coordinates are mm
in machine XYZ; SVGs are centreline projections, not weld-detail drawings. Stock
lengths are the current CAD square/mitre definitions, with no kerf, end allowance
or tube-intersection cope added. The twelve upper-frame corner/post tie-ins are listed separately for joint
detailing; they are not present in the base weld declarations. Resolve every overlapping tube intersection into
an end/cope detail before issuing the cut list to the saw. Check available stock
lengths and add labelled machining/trim allowances rather than changing datums.

The joint schedule carries the authored 5 or 6 mm fillet legs. Geometry-derived
seams are candidates, not an automatic instruction to weld every accessible edge.
Review seam findings, tube wall thickness, access and start/stop positions; define
weld extents, returns and inspection in the shop WPS. Do not silently treat an
unresolved seam as welded. Rail-pad and beam-foot weld distortion is a machining
allowance issue as well as an alignment issue.

The first native extraction has **36 base stock members and 80 weld declarations
per bed, but zero valid seam edges and 80 `weld.no-seam` findings**. The current
intersecting stock envelopes need real end/cope preparation before the seam finder
can identify shop weld paths. This is the first open SR6 detailing item, retained
in each `native.json`; the empty seam-candidate CSV is not an approved weld plan.
Structural freeze fixes the overall layout/sections, not these unfinished joint
preparations or a fabrication release.

Regenerate the views with:

```sh
python3 machinekit/examples/sheet-router/fabrication/render.py \
  --native build/sheet-router-freeze/fabrication \
  --packets build/sheet-router-freeze/packets \
  --output machinekit/examples/sheet-router/fabrication
```

## Jig layout and weld sequence

1. Mark a longitudinal reference line and perpendicular front datum on a rigid,
   surveyed jig. Set three primary height stops first, then shim the remaining
   stops without bending the frame into contact. Locate the eight foot/post
   stations from the per-bed jig table, including the separate Y1575 pair.
2. Set lower side members, end planes and all posts. Clamp with space for weld
   access; check side spacing, diagonals and post heights. Tack symmetrically.
3. Locate table side stringers, seven table crossmembers and the rear rack
   crossmember. Check the table/rack heights against the raised datum before
   tacking. Fit the sixteen braces; retain the lowered Z430 front brace endpoints
   and the full-width front sheet-loading mouth.
4. Tack opposing sides and bays in rotation, working from the middle toward the
   ends. Recheck diagonal and height surveys after each tack group. Trial-fit the
   rail pads, cable-chain supports and vacuum/service routes before closing access.
5. Complete short, balanced weld runs on opposing joints, alternating sides and
   bays. Use the qualified shop WPS for heat input, interpass temperature and
   weld lengths. Avoid completing one side or one long rail-pad seam first. Keep
   a numbered joint/run record against the generated weld schedule.
6. Unclamp after cooling, survey free-state distortion, and stress-relieve the
   frame and welded beam/feet by a process agreed with the fabricator and material
   supplier. Leave finish-machining allowance; do not finish rail faces beforehand.
7. Machine the frame rail-pad seats from one common setup/datum. Welded beam feet
   require stress relief followed by **single-setup machining of the Y-carriage
   seating faces and X-rail faces**. Record their relative squareness and parallelism.
8. Inspect dimensions and welds, deburr/clean, protect machined datums, then coat
   non-datum surfaces. Re-survey after coating and handling. Seal tube ends while
   retaining any fabrication vent/drain provisions required by the shop process.

## Rail pads and dual-Y alignment

1. Install the anchored frame on the qualified slab/anchor arrangement. Adjust
   all eight supports to load-sharing contact; level without twisting the frame.
   Repeat the datum survey after anchor tightening. The FEM assumption was
   50,000 N/mm in each translation at every support, not an anchor specification.
2. Establish the master Y rail against the machined reference face and a surveyed
   straight reference. Tighten incrementally, recording straightness and height.
3. Fit the second Y rail using the gantry/carriage spacing and measured parallelism,
   traversing the full stroke after each tightening group. Verify block preload
   and free motion; do not pull a skewed rail into alignment using the motors.
4. Fit the stress-relieved, jointly machined gantry beam/feet. Verify the X rail
   datum is square to the Y travel and both Y carriage seats are coplanar. This
   machining establishes mechanical dual-Y squareness.
5. Set both Y rack lines using a tooth joining gauge across rack joints. Set mesh,
   preload hinges and reducer backlash without forcing rail alignment. Record
   rack pitch continuity and pinion engagement over travel.
6. Establish independent left/right Y home references against the mechanically
   squared gantry. Set home offsets from the measured square, then repeat homing
   and a full-travel diagonal check. Electronic squaring must reproduce the
   mechanical datum rather than conceal beam/foot machining error.
7. Align X rails, Z rails and spindle tram; verify the 149 mm ram gap, 367.75 mm
   beam-to-longest-tool lever, full Z coverage, rack/tool-change reach and front
   sheet-loading sweep. Record straightness, parallelism, diagonals, tram and
   repeatability with instruments and temperature. Final numerical alignment
   tolerances need a complete machining/servo/cutting accuracy allocation.

The built-up aluminium Z uses bolted/dowelled ribs, not welded 6082-T6. Machine
the rail face after assembly. Vacuum zone valves, pump plumbing, chains and hoses
still need physical service/removal access checks before shop release.

## SR6-C commissioning before cutting acceptance

At the modeled governing poses for both beds, excite X/Y translation and gantry
roll with an instrumented tap/impact or sine sweep. Measure relative tool/bed
response as well as frame/gantry response, identify the first participating modes
and damping, repeat to establish uncertainty, and retain raw data and fit settings.
Replace the assumed 1% damping and modeled frequencies in the dynamic calculation,
tune ZVD from the measurements, and re-check ≤0.05 mm contour deviation and
≤0.05 mm rapid residual at the real planner limits. Required MotionKit OM-R1
jerk-limited execution and optional shaping must then be checked at the drives.
This measured acceptance, alignment and cutting qualification remain SR13/SR16
release gates; the structural model does not close them.
