# ManufacturingKit

ManufacturingKit contains Materia's sheet-stock planning and local cut-execution
records in `materia.sheet`, and reusable manufacturing-tooling input contracts in
`materia.tooling`. It depends on ProjectKit for shared materials, frames and units;
it has no MachineKit, CadKit or native CAD dependency. The `.sheet.json` format
and companion-file convention remain unchanged.

Run the sheet-domain tests with:

```sh
./haxeon/scripts/haxeon run --project=manufacturingkit/tests/haxeon.json
```

The picking-station command-line demo exercises stock registration, planning,
exports, completed cuts, reopening, and remnant reuse:

```sh
./haxeon/scripts/haxeon run --project=machinekit/examples/picking-station/haxeon.json -- \
  demo /tmp/picking-station.sheet.json /tmp/picking-station-exports
```

Use working copies of the project and its `.sheet.json` companion for real
operations. The checked-in picking-station files are shareable examples; the
demo creates a separate destination and refuses to overwrite it.

## Casting tooling inputs

The first stage of [the tooling implementation plan](TOOLING_PLAN.md) provides
detached casting snapshots, explicit casting setups, stock/process recipes,
typed review diagnostics and a strict versioned JSON codec. The second stage
adds the panel contracts and joint generators described below. Mold generation
and the full fabrication/CAM package remain subsequent stages.

- `CastingToolingInput` describes an additive union of convex profile prisms,
  box-shaped design cuts and separately classified steel inserts/recesses.
- Coordinates are right-handed and explicitly in millimetres. Each prism
  extrudes a counterclockwise local XY profile along +Z; each box is centered
  in local X/Y with its bottom at Z = 0. Cut extensions outside the casting
  remain design tools and must be clipped by the later planner.
- `CastingSetup.sourceToMold` maps source coordinates into a mold frame with
  +Z upward, gravity toward -Z and the open fill side toward +Z. Every ordinary
  void requires a core strategy. Reusable formers require a unit withdrawal
  direction in mold coordinates. Insert recesses are reserved for fixtures.
- `ToolingRecipe` records measured panel stock, EPS supplier properties and
  process references. Unknown EPS properties are explicit nulls. Material
  specification strings do not register new materials or establish qualification.
  Panel stock records allowed blank rotations (0/90 degrees) and an optional
  width-alignment restriction for the later sheet-planning bridge.
- `ToolingCodec.encode/decode` round-trip a `ToolingDocument` containing input,
  setup and recipe. They reject unknown/missing fields, unsupported units or
  versions, bad profiles/frames, duplicate IDs and invalid insert references.
  Named collections and object fields are canonicalized for stable fingerprints;
  profile vertices retain geometric order. UTF-8 document size is limited to
  2 MiB, named collections to 512 records and convex profiles to 64 vertices.
  Codec version 4 requires explicit hardware choices, fit allowances, core wire axis,
  fill height and freeboard. Closed vocabularies are typed; earlier versions are
  rejected with no compatibility reader.
  An insert's original design label is preserved separately as `sourceRole`.
- Bind `setup.sourceFingerprint` using `ToolingCodec.inputFingerprint(input)`.
  Source changes invalidate that binding even if the revision was not updated.
  `recipeFingerprint` and `documentFingerprint` identify other manufacturing-input
  changes. Evidence references alone do not make a process pass.
- `ToolingReview.inspect` reviews valid input contracts and reports outstanding
  geometry, process, load and stock checks. It does not claim fabrication readiness.

MachineKit's `machinekit.tooling.FoundationToolingAdapter.snapshot(foundation,
id, revision)` converts existing `MineralFoundation` definitions through their
public APIs. It preserves chamfers, flared feet, feature IDs, insert bore/finish/
tolerance and local mounting frames without retaining source components.
ManufacturingKit's records and codecs remain usable independently of that adapter.

Run the pure tooling checks and the adapter's CAD equivalence checks:

```sh
./haxeon/scripts/haxeon run --project=manufacturingkit/tests/tooling/haxeon.json
./haxeon/scripts/haxeon run --project=machinekit/tests/tooling/haxeon.json
```

## Exact panel tooling

`materia.tooling.panel` is independent of casting, CAD and CAM:

- `PanelPlan` contains stock, a label zone/side, contact face and operations.
  Its single `OutsideProfile` operation owns the outline; there is no competing
  polygon-only representation. Profiles support exact directed lines and arcs.
- `PanelOperation.kind` explicitly selects outside profile, inside profile,
  pocket, open-sided rebate or drill. Each operation has an enum machining side,
  setup ID, finished contour, depth, through/blind flag and tool diameter.
  `reliefToolDiameter`, when present, must match that tool and the relief arcs.
  The later `camkit.CamToolingPanels` bridge must use the declared tool, apply
  compensation in CamKit and sample arcs only at a declared CAM chord tolerance.
- `PanelContours.operation` is the shared exact-contour boundary for CAD,
  drawings and future CAM. It returns detached line/arc records. ManufacturingKit
  does not produce SVG/DXF text or apply cutter compensation.
- `PanelJoints` generates both members and mating datums together: asymmetric
  tab/slot pairs with integrated circular root reliefs and optional rounded tips,
  rebated corners with a retained shelf, and a round-primary/relieved-secondary
  pin pattern plus an independent clearance bolt. Joint clearances are total
  opening-minus-member allowances, not per-side offsets.
  `PinBoltLayout` supplies panel size and all three feature positions; the
  secondary slot follows the actual primary-to-secondary baseline.
- `PanelCoupon.make` produces a shared plug and a strip of slots for selected
  clearances. Stock thickness includes any existing facing. Select actual fit
  by cutting the coupon with the intended sheet and tool.
- `PanelCodec` validates and round-trips the exact local part. It checks closed,
  simple CCW line/arc contours, stock bounds, setup faces, cut depths, contact
  skins, conservative edge/web ligaments and label zones. These are geometric
  checks, not material-strength qualification. A panel record is bounded to
  256 KiB, 128 operations and 512 segments per contour.
- `PanelAssemblyPlan` checks references and mating datums, then topologically
  orders blocking relations supplied by the 3D checker. It does not derive
  blocking from 2D outlines or endpoint poses.

MachineKit realizes these records through `ToolingPanel` and `PanelCad`, draws
them through `PanelDrawing` -> `cadkit.drawing.Drawing2D` -> existing `Svg`/`Dxf`
writers, and computes removal through `SweptRemoval`. The sweep unions the start,
end and boundary-face prisms; cylindrical faces are split at tangent generators
to avoid folded prisms. Plane/cylinder boundaries and straight translations are
supported; other surface types fail explicitly. Travel exceeds the full set's
bounding diagonal. A declared volumetric contact tolerance defaults to
`1e-5 mm³`; cycles report blocked member IDs. Assembly order reverses removal
order for these straight translations.

`PanelKit` combines panel components, mating connectors and geometric blocking
checks. It assumes loose closure hardware has been removed before panel removal;
it does not simulate thread release or rotating clamps. Panel and nominal plain
pin recipes are registered in the default MachineKit registry.

The [generic joinery example](../machinekit/examples/panel-tooling/README.md)
generates the coupon, exact drawings and assembled/exploded preview scenes.


### Single-pocket box mold

`materia.tooling.casting.BoxMoldPlanner.plan(document)` builds the explicit
orthogonal single-pocket template from a validated `ToolingDocument`: five
panels, EPS/covering geometry, a ruled block cutting definition, an insert
fixture and classified contact cuts. `BoxMoldCodec` version 4 stores the strict
source document and resolved records and verifies regeneration on load. It
rejects unknown versions and hand-edited generated records. Dimensions remain
millimetres and profiles remain finished geometry, without cutter compensation.

Panel codec version 3 requires `contactOpenings` in typed records and JSON.
Use an empty list for an intact skin. Naming an opening permits CAD to
represent it but does not qualify it: the consuming mold must supply a matching
`PenetrationPlan`. The box planner checks references, insert seal margins and cutting definitions;
MachineKit checks contact coverage, source reconstruction and
continuous swept blocking graphs. ManufacturingKit emits no CAD or drawing text.

See [box-mold example](../machinekit/examples/box-mold/README.md). The initial
supported template has one rectangular envelope, one bed-accessible or fill-accessible EPS pocket
and one bed-facing insert with recipe-selected bore, thread, screw and clearance
diameters. The example selects M6; the library does not infer a thread from a hole. Reusable
formers and nonorthogonal setups are rejected. Physical qualification remains
pending, including a broad-area core restraint, accurate independent insert
datums, anchorage, finishing stock, material/process evidence and handling.


Core supports are reusable neutral records in `casting.CoreSupport` and generated
by `CoreSupports`. Pass optional `CoreSupportOptions` to `BoxMoldPlanner.plan`;
defaults use `Auto`, one-third measured stock for embed depth, recipe joint
clearance and the declared recipe adhesive-line thickness. The options codec validates closed enum modes,
finite dimensions and unknown fields; mold codec version 4 persists them and
rejects earlier versions. Auto chooses a bed socket when the cavity opens onto the bed,
otherwise a bridge when it opens onto the fill face. Explicit incompatible modes
fail without changing the source casting.

A bed socket gets a classified `core-sealed` contact cut, inset laminated EPS
foot and separate ruled cutting definition. A bridge gets four exact-operation
panels, round/slot locating interfaces and separate external fixing hardware.
The permanently bonded bridge/carrier is one removal group. Records include glue
thickness, bond area, embed depth and pad closure requirements. CAD realizes these
records and includes their full sweeps in both sequences. Geometric support does
not imply adhesive, material, clamp or uplift qualification.


### Multiple-core casting mold review

`casting.CastingMoldPlanner.plan(document, options)` extends the explicit box
workflow to multiple EPS cores, overlapping reusable formers and inserts at
different heights. Both the single-pocket and multi-core planners use
`MoldPanels.box` for the five-panel shell. ManufacturingKit remains ProjectKit-only.

`BoxVolumes` resolves exact orthogonal unions/differences and separates connected
formers into physical members. `CastingMoldPlan` retains every source feature's
ownership, laminated EPS geometry, shared bridge/carrier allocation, independent
datum volumes, named opening seals and design-change requests. Supported source
geometry is one rectangular envelope with axis-aligned cuts and downward-facing
insert mounting faces. EPS pockets must open at the fill face; rigid details use
explicit X-side or Z withdrawal directions. Unsupported layouts fail.

`CastingMoldOptions` includes typed precision routes, replication evidence fields (implemented fields only),
grouted-joint inputs, `CompactionSpec`, per-insert `InsertAnchorage`, and handling
requirements. Missing evidence remains pending. Nonempty anchorage geometry is
rejected until a corresponding source-adapter revision can describe it; it is
never silently added to the casting. Precision allowances never move source
inserts. `CastingMoldCodec` version 3 stores and verifies complete regeneration;
`MoldVolumeCodec` version 3 stores finite boxes and exact operation-derived sealing
profiles for rebuildable CAD. A side former fills its router reliefs with the same
profile that cuts the panel window; those exterior sealing sections preserve
source geometry and allow straight withdrawal.

MachineKit realizes and exports the neutral records through `CastingMoldCad`,
`ToolingVolume` and `CastingMoldExport`. Exact bounds only prune impossible
intersections; all accepted removal pairs still use boundary-face swept solids
and boolean intersections. Installation and demolding use separate geometric
blocking graphs, including the loaded and released bridge groups.

See [mill base tooling](../machinekit/examples/bench-mill/TOOLING_README.md) for
commands, generated artifacts, scope and unresolved physical checks. This phase
emits review geometry and drawings, not CAM or fabrication approval.


Review hardening: `PanelJoint` records and validates tab/slot, seat and pin mating
sizes in addition to frames. Clearance must cover measured stock variation plus
declared lining and machining allowances. All internal profiles/pockets/rebates
check cutter radii and slot width, and rebates participate in ligament checks.
`ToolingIds` owns the common ID vocabulary and bounded derived names.

Mold plans carry `ToolingValidation` records and separate `FabricationReadiness`,
never infer readiness from codec or CAD success. Missing physical qualification
remains pending. Insert fixtures declare support datums and a precision route;
penetrations declare a sealing face, seal material and bore plug separately from
release treatment. Core cutting records declare entry/exit faces and a selectable
X/Y/Z wire direction. The current box templates support full-height fill with
zero freeboard and reject other fill setups explicitly. Bridge pads/rails stay
outside the open fill footprint.


Phase 4 review decisions are explicit: feature draft and priority, a declared
inserts/EPS/datums/priority/ID overlap rule, unsupported-feature records, source
finish validation, source-to-service orientation and typed lifting qualification.
Zero-draft rigid box details request a split/draft/sacrificial alternative; the
planner never fabricates positive draft from an unchanged source box. Missing
machining or replication allowance produces a source-interface request. Dam and
pour-group fields are removed until their geometry and checks are implemented.

Nominal datum supports have pin/slot receivers, tool-only threads and sealed bed
penetrations. They remain bed-referenced; insert XY clamping, accuracy and access
need qualification. Receiver cuts store explicit `PanelOperation` records, sharing
the contour codec/CAD path with panel machining. Pocket-floor drills name their
access pocket, while unrelated cuts still undergo exact line/arc ligament checks.
Steel bores, masks and fixture pins participate in their actual removal sequence.
Casting mass/CoM includes bored steel and recipe mix density; flip/reorientation
is derived and appears after withdrawal, with qualification status.

Run the full mill acceptance suite without generating all exports:

```sh
./haxeon/scripts/haxeon run --project=machinekit/tests/tooling/haxeon.mill.json
```
