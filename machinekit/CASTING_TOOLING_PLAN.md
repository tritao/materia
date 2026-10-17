# Casting tooling plan

**Goal.** From a cast part designed in materia, generate everything needed to make it: the mould,
the insert fixtures, the cut and print files, the pour quantities, and a step-by-step build, pour
and demould sequence. Mould design becomes a generated output, not a drawing job.

1. A casting is designed as a part. It has a body (walls, ribs, pockets), cast-in inserts with
   tolerances and finishes, a reference face and a pour face. Design rules for the casting
   material are checked as it is designed.
2. The mould is derived from it:
   - the cavity is the envelope minus the casting;
   - it is split into **panels** (flat walls), **cores** (pockets and channels) and **printed
     pieces** (shapes panels and cores cannot make);
   - **insert fixtures** hold every insert at its datum during the pour.
3. The mould is a materia assembly. Interference, demouldability and removal-order checks run on
   it like on any machine.
4. Outputs go to machines materia already models:
   - panels and jig plates as sheet cut plans and CNC router jobs;
   - cores as hot-wire foam-cutter jobs;
   - printed pieces as printer jobs;
   - plus a bill of materials and the epoxy/aggregate quantities.
5. A generated document walks through building the mould, placing inserts, pouring, vibrating,
   curing, demoulding and finishing (replication or grinding of precision faces).

**First customer:** the flexspline hobber's bed and column
(`machinekit/examples/flexspline-hobber/PLAN.md`, HB5 and Phase D). **First process:** mineral
casting (epoxy granite) in disposable panel-and-foam moulds. **Later:** sand-cast iron and
aluminium, and reusable metal moulds.

The work spans kits:
- `machinekit/haxe/src/machinekit/casting/`: the casting design model, rules, cavity and
  decomposition, the mould assembly, fixtures, the pour plan;
- `cadkit`: the solid operations it needs (envelope, cavity, split by planes, wall-thickness
  probing);
- `manufacturingkit` (`SheetCutPlan`), `camkit` (router profile and drill jobs, `Slicer`): the
  outputs;
- the foam-cutter work (`FoamJob`): core outputs, once it is on main;
- `app`: the mould view (exploded, removal sequence) and the instruction document.

## What exists

Surveyed 2026-10-17 on branch `flexspline-hobber` (a71f86f61) and local main 73b480f4c.

- **Hobber foundations (HB5, branch only).** `machinekit.hobbing` has:
  - `MineralCasting`: a thick-walled, cross-ribbed body with insert recesses, mass from CAD volume
    × density, BOM by casting weight;
  - `CastInsert`: a steel insert with role, position tolerance and finish, where finish is only
    "ground after casting";
  - `MineralFoundation` / `FoundationLayout`: a hard-coded insert layout per bed or column;
  - `FoundationIntegrity`: insert position and parallelism, and that precision mounts land on
    inserts.

  It states plainly that enclosed cavities "require process/core design later". This is the seed
  to generalise: it is hobber-specific, and its layout is fixed in code.
- **Materials.** `MaterialLibrary` has `cast-iron`, and on the branch a `mineral-cast` entry with
  assumed modulus, damping and thermal fields (`MaterialDef` gained optional fields).
- **Solids.** CadKit `Shape.cut/common/fuse`, `PlanarRegion` 2D booleans and offsets,
  `cad_shape_distance`, sweeps and lofts. `Solids.cut` is used by `MineralCasting`.
- **Checks.** `AssemblyIntegrity` covers support, interference, sampled joint sweeps (with
  `sweepStates`), seating and drive consistency.
- **Outputs.**
  - `manufacturingkit` `SheetCutPlan` (stock spec, kerf, placements, cuts, retained regions) with
    validation and export;
  - `camkit` `CamJob` (profile, pocket, drill), `CamSheetProfiles` (sheet blanks to profile
    inputs), `Slicer` and `PrinterProfile`;
  - the CNC router example cuts sheet parts.
- **Foam cutting.** `FoamJob`, wire kinematics and kerf are on the `foam-cutter` branch, not on
  main.
- **Assembly planning.** The cabinet-assembly work sequences operations with fixtures and
  checks. It is on its own branch, and the user decides when it lands.

## What is missing

- A general casting model: process, material rules, insert finishes other than grinding, pour
  and reference faces.
- Cavity derivation; decomposition into panels, cores and printed pieces; panel joints (screws,
  dowels, sealing fillets).
- Demouldability (a removal direction and order) and insert fixtures.
- Pour planning: volume, mix, venting, exotherm limits, compaction and cure.
- Replication setups for precision faces.
- The instruction document, and a mould view in the app.
- Structural or modal analysis of the casting. That stays a separate capability; see Later.

## Decisions

**CT-D1. Precision comes from inserts and fixtures, never from the mould.**
- Mould pieces are held to about ±0.5 mm (assumed, a router-cut panel class).
- Every precision surface is an insert, finished one of three ways:
  - **AsCast**: located by the fixture, finish as poured;
  - **Replicated**: left low and brought to a reference with a thin epoxy replication layer;
  - **GroundAfter**: oversize, ground after curing.
- The fixture, not the mould, sets insert position. Its accuracy budget is stated per insert.
- **Rejected:** precision moulds for prototypes. Too costly, and they do not remove the need for
  finishing.

**CT-D2. A casting is a design object, independent of any one machine.**
- `CastingDesign` (in `machinekit.casting`) holds:
  - the body solid;
  - insert child parts (role, datum frame, tolerance, finish class, anchoring);
  - the process (`MineralCast` first);
  - the material;
  - the reference face (down in the mould) and the pour face (open, up).
- The hobber's `MineralCasting` / `FoundationLayout` become one use of it. The layout becomes
  data, and inserts become placed parts, not a hard-coded table.
- **Rejected:** keeping casting knowledge inside `machinekit.hobbing`. Every machine frame would
  copy it.

**CT-D3. Process rules are data per process, and checks name the feature.**
- Mineral cast (all assumed until checked):
  - minimum wall about 30 mm;
  - maximum solid section about 150 mm (cure exotherm);
  - inside radii at least 5 mm;
  - insert embedment and anchoring (fins, undercuts or studs);
  - minimum edge distance from inserts to walls;
  - shrinkage about 0.02 % (negligible);
  - no draft needed for disposable panel moulds;
  - density 2400 kg/m³.
- Sand-cast metals later add draft, shrinkage (about 1–2 %), machining allowance, gating and
  risers.
- Wall thickness is probed with `cad_shape_distance` or ray sampling. A violation names the body
  feature and the measured value.

**CT-D4. The mould is derived, never drawn.**
- The cavity is the casting's envelope box, plus a wall margin, minus the casting.
- Decomposition, in order:
  1. outer planar faces of the cavity become **panels**: sheet parts with thickness, edge joints
     and fastener patterns;
  2. internal pockets and channels that panels cannot release become **cores**: foam blocks,
     ruled where possible so the hot-wire cutter can make them; otherwise router-cut foam slabs
     glued in layers;
  3. whatever remains becomes **printed pieces**, segmented to the printer's build volume.
- A cavity region no rule can assign is a diagnostic naming the region. The generator never
  silently fills it.
- **Rejected:** one monolithic printed or milled mould. It is slow and costly at bed size
  (around 650 × 550 mm), and panels are faster and reusable as templates.

**CT-D5. The mould is an assembly, so existing checks apply.**
- `MouldAssembly` (a MachineAssembly) holds panels, cores, printed pieces, fixtures, fasteners,
  dowels and the cured casting as a fixed part.
- Interference and support come from `AssemblyIntegrity`. The BOM covers sheet stock, foam,
  filament, screws, dowels, release agent and sealant.
- Assembly sequencing can later be handed to the assembly-planning work. Until then the generator
  emits its own ordered steps (CT8).

**CT-D6. Demouldability is a removal order with collision-free sampled sweeps.**
- Each mould piece gets a removal direction (panels along their normal; cores along their free
  axis, or "lost": dissolved or broken out).
- An order exists in which each piece clears the cured casting and the pieces still in place over
  its sampled sweep (the `AssemblyIntegrity` sweep machinery).
- Fixtures are removed before the panels they bolt to.
- Failure names the piece and the casting feature that traps it.

**CT-D7. Insert fixtures are generated from insert datums.**
- Per insert: a jig plate (or bar) with locating pins or bolts into the insert's threaded holes,
  bolted to a rigid fixture frame spanning the mould.
- The fixture holds the insert at its datum within the stated budget: insert tolerance minus
  replication or grinding stock.
- Checks:
  - each insert is fully constrained (six degrees of freedom);
  - fixtures do not cross the pour face where the pour must flow;
  - every fixture comes off after curing.
- Jig plates are sheet or router parts, so they share the panel outputs.

**CT-D8. Pour planning is computed from the cavity.**
- Volume and mass. Mix by weight from a stated recipe (epoxy binder about 10–14 %, graded
  aggregate; assumed).
- Batch count from the mixer size, and the pour sequence.
- Pour and vent points: every enclosed upward pocket's highest point must vent, and trapped-air
  regions are a diagnostic.
- Compaction (vibration time) and cure (time and temperature), as process data.
- Exotherm check: no section above the process limit (CT-D3).

**CT-D9. Outputs reuse existing kits; nothing new is invented for cutting or printing.**
- Panels and jig plates become a `SheetCutPlan` with kerf and margins, then `CamSheetProfiles`
  and router `CamJob`s (profiles, holes for screws and dowels).
- Cores become `FoamJob`s once the foam-cutter work is on main. Before that: DXF profiles plus
  slab-stack fallbacks cut by the router.
- Printed pieces become STL plus `Slicer` jobs with a `PrinterProfile`.
- Every output keeps provenance back to the mould piece and casting feature.

**CT-D10. The instruction document is generated and is the acceptance artefact.**
- Steps, in order: cut, print and foam; assemble the mould; seal and release-coat; mount the
  fixture frame and inserts; check insert positions; mix and pour in batches; vibrate; cure;
  remove fixtures; demould in the CT-D6 order; replicate or grind precision faces; inspect.
- Each step references its parts, quantities, tools and checks. Rendered as an HTML/Markdown
  document from the model, with images from the app later.

**CT-D11. Generic names in code;** vendors appear only as sources in docs.

## Reference numbers (assumed until checked against a datasheet or a measurement)

| Quantity | Value | Source |
|---|---|---|
| Mineral cast density | ≈ 2400 kg/m³ | literature range 2300–2500, assumed |
| Binder fraction | ≈ 10–14 % by weight | literature range, assumed |
| Minimum wall | ≈ 30 mm | assumed (HB5 uses 25–30 mm) |
| Maximum solid section (exotherm) | ≈ 150 mm | assumed, check with the resin datasheet |
| Linear shrinkage | ≈ 0.02 % | assumed |
| Mould panel tolerance | ±0.5 mm | assumed, router-cut panel class |
| Panel stock | 18 mm coated MDF or birch plywood | assumed |
| Replication layer | 0.5–2 mm | assumed, check with the replication resin datasheet |
| Hobber bed envelope | about 650 × 550 × 120 mm | HB5 `FoundationLayout` |

## Test castings

- **Ribbed block:** a box with four open ribbed pockets, two rail-seat inserts and four threaded
  bushes. Panel-only mould, no cores.
- **Cable-channel block:** the ribbed block plus an internal L-shaped channel. Needs a core and a
  vent.
- **Trapped feature:** an inward overhang no panel can release. Must be rejected, naming the
  overhang (or solved by a lost core when allowed).
- **Hobber bed and column:** from HB5, ported to `CastingDesign`.

## Steps

### Phase A: design model, cavity, decomposition (can start on the hobber branch now)

**CT0. Baseline and test castings.**
- Record the gate on the starting commit. Build the test castings as `CastingDesign`s.
- Tests: casting solids are valid; masses match volume × density.

**CT1. Casting design model and rules (CT-D2, CT-D3).**
- `CastingDesign`, `CastingInsert` (finish classes AsCast, Replicated, GroundAfter; anchoring;
  datum; tolerance), `CastingProcess` with `MineralCastRules`.
- Port the hobber's `MineralCasting` and `CastInsert` onto it. Make the layout data and delete
  the hard-coded table.
- Rule checks: minimum wall, maximum section, inside radii, insert edge distance and embedment.
- Tests:
  - each rule fires on a crafted violation, naming the feature and value;
  - the hobber bed passes;
  - `FoundationIntegrity` checks still pass after the port.

**CT2. Cavity and decomposition (CT-D4).**
- Envelope plus margin, minus the casting, gives the cavity. Planar outer faces become panel
  candidates. Classify the rest as cores (with their free axis, or lost) or printed pieces.
- Panel layout: thickness, edge joints (butt with screws, rebates as an option), screw and dowel
  patterns.
- Tests:
  - the ribbed block gives exactly the 5 panels around an open top, plus the pocket-forming
    pieces;
  - the cable-channel block adds one core;
  - the trapped feature is reported;
  - the union of pieces equals the cavity to 1e-6 of its volume.

**CT3. The mould assembly (CT-D5).**
- `MouldAssembly` with panels, cores, printed pieces, fasteners, dowels, the casting fixed, and
  sealing fillets.
- `AssemblyIntegrity` interference and support. BOM.
- Tests: no interference except the declared casting contact; the BOM lists stock, foam, screws
  and dowels with quantities.

**CT4. Demouldability and removal order (CT-D6).**
- Removal directions and an order search, with sampled sweeps against the casting and the
  remaining pieces.
- Tests:
  - all test castings except the trapped one demould, with a valid order;
  - the trapped one names the piece and the feature;
  - reversing a dependent pair is rejected.

### Phase B: fixtures, outputs, pour plan, document

**CT5. Insert fixtures (CT-D7).**
- A fixture frame across the mould, with a jig plate per insert, generated from the datums.
- Constraint check, pour-path clearance, removal before panels.
- Tests:
  - every hobber-bed insert is held within its budget (by the fixture's stated tolerance chain);
  - a missing locator is reported as under-constrained;
  - a fixture crossing the pour opening is reported.

**CT6. Manufacturing outputs (CT-D9).**
- `SheetCutPlan` for panels and jig plates, then router `CamJob`s.
- Cores: DXF and slab stacks now; `FoamJob`s once foam cutting is on main (`blocked:` until
  then).
- Printed pieces: STL plus `Slicer`.
- Tests:
  - the cut plan validates;
  - router jobs reproduce the panel outlines and holes within tolerance in the router simulation
    (stock comparison);
  - printed pieces fit the build volume;
  - provenance links back to the pieces.

**CT7. Pour plan and replication (CT-D8, CT-D1).**
- Volume, mass, mix, batches, pour and vent points, trapped-air diagnostics, exotherm,
  compaction and cure.
- For Replicated inserts: the dam, reference bar or plate, layer thickness and resin quantity.
- Tests:
  - volume matches the CAD casting volume to 0.1 %;
  - the cable-channel block's vent is required;
  - a deliberately thick section fails the exotherm check.

**CT8. Instruction document (CT-D10).**
- Generated, ordered steps from CT3–CT7, with quantities and checks, as HTML/Markdown.
- Tests: the hobber bed's document lists every piece, insert, batch and removal step exactly once,
  in an order consistent with CT4 and CT5.

**CT9. App view.**
- Exploded mould, removal-sequence playback, pour/vent markers, rule diagnostics on the casting.
- Tests: an app check loads the hobber bed's mould project and plays the removal sequence without
  collisions.

**CT10. Acceptance.**
- Hobber bed and column: mould, fixtures, cut plans and jobs, pour plan and document generated end
  to end from the casting.
- The trapped-feature casting is refused with the right feature named.
- Quantities match a hand calculation (volume, batches, sheet area) within 2 %.
- A short report of every assumed number.

### Phase C: later

- Sand casting for iron and aluminium: draft, shrinkage, machining allowance, gating and risers,
  core prints. Reusable pattern and core-box output.
- Reusable metal moulds for series production.
- Robot-built moulds through assembly planning, once it is on main.
- Cost model: material, machine time, labour.
- Structural and modal analysis of castings with an external FEA solver (for example CalculiX):
  mesh, materials, supports, joints, mode shapes in the app. First use: hobber bed and column
  modes against the hob's ≈ 50 Hz revolution and ≈ 600 Hz gash frequencies (hobber Phase D).

## Order

```
CT0 ── CT1 ── CT2 ──┬── CT3 ── CT4 ──┬── CT8 ── CT9 ── CT10
                    ├── CT5 ─────────┤
                    ├── CT6 ─────────┤
                    └── CT7 ─────────┘
```

- CT5, CT6 and CT7 depend on CT2 and can run in parallel. CT8 needs CT3–CT7.
- CT6's foam-cutter output is gated on the foam-cutter work reaching main. The DXF and slab
  fallbacks are not gated.
- The hobber plan needs nothing from this plan until its Phase D. CT1's port of the hobber
  foundations must keep `FoundationIntegrity` passing.

## Working rules

- Work on the `flexspline-hobber` branch while the hobber is its only customer. Split it out if
  another machine starts using it.
- Dependencies count only once they are on local `main`. A blocked step is marked
  `blocked: <what>`.
- Commit per step, e.g. `Casting tooling: cavity and decomposition (CT2)`, with Progress updated.
- Diagnose before working around. Numbers not from a datasheet or a measurement are marked
  assumed.
- No legacy compatibility. Heavy suites go through the heavy-run wrapper.

## Progress

| Step | State | Commits |
|---|---|---|
| CT0 | planned | |
| CT1 | planned | |
| CT2 | planned | |
| CT3 | planned | |
| CT4 | planned | |
| CT5 | planned | |
| CT6 | planned | |
| CT7 | planned | |
| CT8 | planned | |
| CT9 | planned | |
| CT10 | planned | |
| Phase C | later | |
