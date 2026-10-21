# Materia project formats

ProjectKit contains portable data types and codecs shared by Materia's kits. It
has no native code or dependencies on other kits. The public namespaces are
`materia.project` for scene artifacts, materials, and appearances;
`materia.assembly` for assembly definitions, records, frames, and codecs; and
`materia.units` for length units and SI conversion. Keep project evaluation and
other engineering runtime behavior in separate packages so ProjectKit remains a portable
base layer. Presentation evaluation is pure display derivation and has no native dependencies.

`materia.project.SceneArtifact` is the shared writer and reader for generated
viewport geometry. A project generator writes the artifact to a file; Materia
loads and validates that file after the generator exits. The current `MTRG`
version 42 contains a metres-per-coordinate scale and independent mesh parts.
Each part has a stable ID, display name, color, vertex and normal streams,
triangle indices, CAD face ranges, and persistent edge identities. It can also
carry rigid instance poses, named connector frames, and fixed, revolute,
continuous, or prismatic joints. Readers reject unsupported versions, duplicate
IDs, invalid units, malformed buffers, and out-of-range indices. Earlier
versions are rejected. The current format can also carry a versioned assembly definition
and a separate kinematic state alongside the pose record. The current format adds
validated moving-belt routes and independent-axis playback metadata, including
an optional streamed virtual-device deployment.

The artifact is derived output. The project's source code and manifest remain
authoritative. CadKit authors CAD occurrences and mates; ProjectKit stores their
portable assembly record; Materia evaluates tree-joint coordinates into preview
poses. Generated project documents can persist an `AssemblyStateRecord` override
separately from the source assembly definition.

## Kinematic definitions

`AssemblyDefinition` is the versioned kinematic model alongside the legacy
`AssemblyRecord`. It stores reusable component definitions with definition
connectors, separate occurrences, typed fixed/revolute/continuous/prismatic
joints, explicit unit axes, optional limits, and a `tree` or `closure` role.
`AssemblyDefinitionCodec` validates and encodes this schema independently from
the binary scene artifact version. `AssemblyStateRecord` stores joint
coordinates and root placements separately from the definition. Scene artifact
version 42 can carry both payloads; geometry part IDs identify component
definitions so repeated occurrences can share one geometry payload.

Assembly schema version 11 includes a reusable `assemblies` table. An occurrence selects a
subassembly by setting `assembly` and `definition` to its table ID. Each entry
can expose connectors from its members, including connectors passed through a
deeper subassembly. `AssemblyDefinitionFlattener` prefixes nested IDs with the
occurrence path and resolves exposed connectors before solving or simulation.
Saved root poses may name a subassembly occurrence; flattening carries those
poses to its leaf roots. Definitions and states use generated JSON wire codecs.
The older human-readable JSON form, keyed by field name with a `schemaVersion`,
is no longer read; decoding it fails with an error that says so.

The legacy assembly record codec remains because scene artifacts still carry a
legacy `assembly` record beside the definition, and the editor's tree view is
built from it.

Run the direct ProjectKit suite with
`./haxeon/scripts/haxeon run --project=projectkit/tests/haxeon.json` from the
repository root. It exercises units, assembly codecs and frames, scene artifact
versions, and materials without loading a downstream kit or any native library.
Keep it passing when changing the portable assembly or scene records consumed by
other kits.

For read-only validation, `AssemblyDefinitionFlattener.flattenView` borrows an
already-flat definition and expands nested definitions. `flatten` continues to
return an independent editable copy. Neither validation path mutates its source.

`SceneArtifact.decode` returns independent mesh buffers. `decodeView` validates the
same format while borrowing mesh streams from its input; those views retain the
input storage, which callers must keep immutable. The project loader uses this
path for its immutable artifact snapshot. Scalar and text reads avoid temporary
byte buffers in both paths. See [decoding benchmarks](tests/decoding/README.md) for
native allocation measurements and the Chrome runner.

Scene artifact schema 42 includes optional project runtime settings: a selected job,
dynamic occurrence IDs and joint motion tracks. Native publishers resolve these
from source manifests so downloaded artifacts open independently of source files.
Only the current schema is accepted. Project settings
validate occurrence/joint references, duplicate IDs and bounded motion keyframes.

Machine motion uses typed wire records for axis and digital programs and per-hull
contact settings. Hydraulic actuator data uses wire field 28; motor brakes retain
field 27. These records are part of assembly schema 10 and scene schema 42.
The optional press controller record names its owned ram, depth and backgauge axes,
measured top and finger switches, and program steps in millimetres. Position tolerances,
request deadlines and simulated unloading limits are explicit saved values; the loader
validates references, controller ownership and ordered clamp/depth travel before execution.

## Assembly presentation

`materia.presentation.AssemblyPresentation` schema 1 stores views, typed occurrence
selectors, section planes, staged display offsets and assembly, disassembly or service
procedures separately from engineering definitions and state. Scene artifact version 42
carries an optional validated `presentation` payload beside `assemblyDefinition`.
Earlier artifact versions are rejected and must be regenerated.

`PresentationCodec` validates references against the flattened assembly. Unknown
selectors, duplicate view or step IDs, invalid stages, non-finite values and checked
motions without fingerprints fail at the boundary. Authored procedures can only carry
illustrative movement evidence. Generated and authored entries use the same codec.

`PresentationEvaluator.evaluate` takes solved poses and a session cursor and returns
independent display poses, visibility, world section planes, highlights, joint overrides,
a camera specification and an evidence badge. It never edits its inputs. Callers apply
joint overrides to an isolated preview solve; they must not write them to engineering
state. The current geometry fingerprint is supplied by the geometry owner. Changing it
marks cached checks Stale, including failed checks. Simulation playback suppresses
explode offsets and procedure motions. `reset` reconstructs the default cursor.

The cursor's step is the number of completed steps. The active movement interpolates
along its recorded path using `progress` in 0..1; completed removals are hidden.
`Disassembled` starts with the procedure's affected parts removed. Every evaluation
folds from the beginning, so Previous and Restart need no undo history. Nested scope
frames are recovered from solved internal roots; callers may supply explicit scope
poses alongside leaf poses. `Procedures.reverse` emits an independent explicit reverse
procedure and requires initial values for every joint it restores.

The direct suite includes presentation codec and artifact round trips, strict validation,
staged and nested composition under parent rotation, section and visibility rules,
procedure determinism, reset, engineering input isolation and evidence invalidation.
Viewer integration and producer migrations are tracked separately in
[the assembly presentation plan](../app/plans/ASSEMBLY_PRESENTATION.md).
