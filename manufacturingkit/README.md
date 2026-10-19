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
  Codec version 2 uses enum abstracts for units, insert roles/finish and stock
  coating/direction. Version 1 is rejected; there is no compatibility reader.
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
