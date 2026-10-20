# SR6 DIY fabrication route

User-approved route, 2026-10-20. The base is not sent to a large machining centre.
The governing sequence is:

**full tack/jig → balanced welding → relief → rest/survey → epoxy or adjustable
rail seats → rails → measured parallelism/squaring → SR6-C tap test.**

The frozen centreline layout, tube sections, sixteen braces and eight leg stations
remain. The prepared-piece STEP files and seam schedule define the tube ends;
centreline intersections alone are not instructions to pass one tube through another.

## Jig and low-distortion weld plan

Locate all frame stops from the per-bed jig table. Establish three primary height
stops, shim the others to contact without bending the frame, and verify width,
post heights and diagonals. Complete **every tack in the jig before any finish
welding**. Trial-fit pads, service pipes and chain supports at the tack stage.

Use nominal 5 mm fillet legs. Use intermittent, opposed stitches with the lengths
and positions in the checked seam schedule; do not increase leg size to make up
for a missing contact or inaccessible seam. Start with 40 mm stitches where the
available edge permits. A joint needing more capacity gets a documented length
or joint-detail check, not an oversized bead. Every separately coped brace/post
piece must have a connected welded load path.

Number the joints and alternate left/right, front/back and opposing faces,
working from the centre towards the ends. Tack every piece first, then weld one
stitch on the opposing joint before returning. Skip adjacent joints while they
cool. Record interpass temperatures against the agreed WPS; do not force cooling
with water. Light controlled peening between suitable passes is included only
where the WPS permits it; do not hammer thin root edges or use peening to conceal
distortion. Survey diagonals and rail-seat reference points after each sequence
group and again unclamped/cold.

## Relief, furnace booking and rest

The preferred route is commercial thermal relief in a car-bottom furnace.
Quote the **bare prepared frame**, without epoxy, rails, seals, levelling feet,
valves or electrical equipment. Long-bed nominal stock envelope is about
**2550.5 × 4621.25 × 700 mm** (X/Y/Z); short-bed Y is 4071.25 mm. The native
prepared-volume report gives approximately 1022/1096 kg for the short/long
frame. Request at least a 2.8 × 4.9 × 1.0 m
usable envelope and at least 1.5 tonne payload including the jig; the heat treater
must confirm actual fixture clearance and payload before release.

The user's planning cycle is approximately 550–650 °C, about 1 hour per 25 mm
wall, followed by furnace cooling. Tube walls are 4/6 mm; thickness alone gives
about 10–15 minutes, **not a sufficient definition of the entire furnace cycle**.
The contractor sets a qualified cycle for the actual steel grade, with measured
heat-through, soak and controlled furnace cooling; book at least a one-hour
soak after the coldest thermocouple reaches the agreed temperature unless their
qualified procedure specifies otherwise. Retain material identification,
thermocouple locations, temperature/time chart, fixture and cold survey.

Fallback: contractor or DIY vibratory stress relief. Support the frame without
forcing it straight; clamp a variable-speed unbalanced motor driven by a VFD to
a stiff node. Use an accelerometer and a controlled frequency sweep to identify
frame resonances, then dwell 10–30 minutes at or just below each selected
resonance. Record support points, motor position, imbalance, frequency, measured
acceleration, dwell time and temperature. Repeat the sweep after each dwell;
stop for loose joints or excessive response. This is a dimensional-stability
route, not a claim of metallurgical equivalence to thermal relief.

For either route, survey before and after, rest at stable room temperature for
at least 48 hours, then repeat the survey after unloading/re-supporting. Before
pouring, require repeated height/diagonal surveys to agree within 0.02 mm at the
rail-pad reference points; otherwise repeat stabilisation or use adjustable
mounts and keep release blocked. Record temperature and instrument uncertainty.

## Epoxy rail seats: baseline

Preserve the existing **12 mm nominal rail-seat height** and the frozen rail/tool
datums. Replace the solid 12 mm pad envelope with an 8 mm nominal steel backing
strip and a **4 mm nominal epoxy layer**. Frame distortion may vary the pour
between **2 and 6 mm**; values outside that range require correction of the
backing strip or a revised pour specification, not a change in rail elevation.
Each continuous channel is 60 mm clear width, centred on the Y rail line, over
the existing rail-pad length. Leave at least 15 mm dam/meniscus allowance outside
the rail's bearing width and provide overflow witnesses. Use temporary cross
channels/witness cups to establish the common level, then remove them after cure.

Use a machine-grade, low-shrinkage, self-levelling epoxy whose supplier approves
this layer range, sustained bolt/contact pressure, working temperature and cure
conditions. [DIAMANT EP-Flow](https://diamant-polymer.de/en/product-overview/ep-flow/)
is a sourcing candidate explicitly intended for self-levelling mechanical
engineering supports; its product page does **not** establish our 2–6 mm layer
qualification. Obtain that TDS/application confirmation and pour a full-length
witness or representative coupon before release. Do not substitute decorative
casting resin. [DWH 310 FL](https://diamant-polymer.de/en/shop/dwh/310-fl/)
is a machine-interface replication candidate; its moulded-surface accuracy is
not proof that an unsupported free surface meets the rail datum target.

Prepare/degrease/roughen the backing as the selected TDS requires, dam and leak-test,
level the supported frame, and pour both uninterrupted rail channels in a common
thermal window. Observe batch ratio, pot life, maximum batch exotherm, cure and
post-cure requirements. Do not bolt rails down during an incomplete cure or use
bolt preload to pull a wavy rail straight.

Use rail-manufacturer bolt pitch. Prefer drilling/tapping the 8 mm steel backing
before welding, with sealed removable plugs during the pour; after full cure,
clear-drill the epoxy with a rail template and reopen the steel threads. The
alternative is guided drilling/tapping through the cured layer into steel. The
epoxy does not provide the rail bolt threads. Verify thread engagement, bolt
length and pull-out capacity on the actual backing; use steel threaded inserts
or local backing nuts if the selected rail requires more engagement.

Acceptance targets are **0.03 mm straightness per full rail**, **0.03 mm rail
parallelism over travel**, **0.03 mm common-plane error** and **0.02 mm local
seat flatness over 300 mm**. These are engineering assembly allocations, not
claims of resin accuracy or a finished-part tolerance. Survey on a 250 mm grid
with a calibrated 0.02 mm/m precision level (including reversal), a reference
straightedge/laser or tensioned-wire reference, and 0.001–0.005 mm indicator.
Remove dam-edge menisci outside the bearing region. Retain both raw and reduced
surveys at a recorded stable temperature. Correct any failed datum before fitting
rails; gravity levelling does not establish horizontal straightness.

## Adjustable alternative

Use rigid steel pads with opposing jack screws at ≤250 mm stations and solid
shims after adjustment. Set height/twist with the precision level; establish
straightness with the straightedge/laser reference and dial indicator. Set the
second rail from the first with a gauge bridge. Lock screws and support each pad
on solid shims, optionally epoxy-backed; re-survey after locking. No rail span
may be left hanging from jack screws or pulled into contact by mounting bolts.
The same dimensional acceptance targets apply.

## Beam/feet and ordinary machine-shop work

The beam blank is about **2550.5 mm long**, so a nominal 2 m shop bed must not be
assumed to face all datums in one uninterrupted travel. Ask the shop to confirm
supported overhang and actual X-rail seat span, or use overlapping indexed setups
with a common machined datum and a bridge/indicator transfer check. Record rail
face coplanarity and both foot-face relationships; target ≤0.02 mm over the
active rail seats and foot reference transfer. Do not machine the complete base.

Bolted/dowelled feet were screened at both beds' static-X, static-Y and modal
governing poses. The original lumped joint gives long-bed Y 1.632 N/µm versus
2.044 welded. A four-quadrant machined-face model (1.5 MN/mm normal stiffness per
quadrant, 0.2 MN/mm transverse per quadrant; locating dowels; closed preloaded
contact assumed) improves that to **1.909 N/µm**, still below the 2 N/µm gate.
At the modal governing pose, it gives **25.395 Hz versus 26.986 Hz welded**.
Those spring values are model assumptions, not supplier qualification.

Therefore **retain welded feet** in the frozen structure. Bolted+dowelled would
be preferred for DIY re-squaring if it passed, but neither screened joint meets
the present static margin. Stress-relieve the separate beam/feet weldment, then
face its X-rail seats and Y-carriage seating faces at the ordinary shop, with the
indexed datum-transfer checks above. Do not silently waive SR-D3 or redesign
local plates to force adoption of the alternative.

## Rails, measured diagonals and dual-Y squaring

1. Set the anchored frame on all eight supports without twist. Survey again after
   anchor tightening; adjust supports before rail alignment.
2. Establish the first Y rail against the horizontal reference. Tighten in stages
   and survey full length. Set the second using a bridge and indicator, then
   tighten and repeat the full-travel parallelism/height checks.
3. Fit the gantry with drives disengaged. Measure crossed diagonals between the
   same four calibrated rail/gantry reference points. A tape is for coarse setup;
   final setup uses a suitable long-range comparator/laser or calibrated diagonal
   bar. Target **≤0.05 mm diagonal difference**, and **≤0.03 mm transverse square
   error over 1600 mm**, verified by a reversal square/indicator or laser survey.
4. Adjust pad/rail lateral screws and foot/gantry seating shims as needed. Recheck
   rail straightness and parallelism after every squaring correction. Squareness
   comes from these measurements and adjustments, not the as-welded frame.
5. Engage racks using a tooth joining gauge and set pinion preload/backlash.
   Establish separate left/right home references against the measured square.
   Repeat homing at least ten times, target ≤0.02 mm skew spread; record diagonal
   and full-travel checks after homing. Electronic offsets reproduce the measured
   mechanical square rather than loading the rails into a forced skew.
6. Align X/Z, tram the spindle, repeat guide travel/tool-rack/loading checks, then
   commission SR6-C measured modes/damping and shaper tuning before cutting.

## Release gate

`shop-release.json` is fail-closed: valid prepared seams and connected pieces,
weld-size/sequence checks, service clearance and maintenance access, metrology
procedure, selected resin/relief qualification, and signed shop review must all
be satisfied before the packet is labelled released. Generated STEP/CSV files
are draft until then. Physical before/after relief, epoxy and alignment surveys
are retained as build witnesses and are required before hardware/cutting acceptance.
