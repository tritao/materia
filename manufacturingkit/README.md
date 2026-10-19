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
typed review diagnostics and a strict versioned JSON codec. Panel generation,
joinery, removal-path validation and fabrication exports are subsequent stages.

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
