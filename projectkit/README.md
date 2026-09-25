# Materia project artifacts

`materia.project.SceneArtifact` is the shared writer and reader for generated
viewport geometry. A project generator writes the artifact to a file; Materia
loads and validates that file after the generator exits. The current `MTRG`
version 6 contains a metres-per-coordinate scale and independent mesh parts.
Each part has a stable ID, display name, color, vertex and normal streams,
triangle indices, CAD face ranges, and persistent edge identities. It can also
carry legacy rigid instance poses, named connector frames, and fixed, revolute,
continuous, or prismatic joints. Readers reject unsupported versions, duplicate
IDs, invalid units, malformed buffers, and out-of-range indices. Versions 2–4
and 5 remain readable. Version 6 can also carry a versioned assembly definition
and a separate kinematic state while retaining the legacy pose record for older
scene consumers.

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
version 6 can carry both payloads; geometry part IDs identify component
definitions so repeated occurrences can share one geometry payload.

The legacy codec remains available for existing scene artifacts. Its joint
records did not store tree-versus-closure roles or explicit axes; the
`fromLegacy()` adapter uses a deterministic spanning tree and the historical
local-Y axis convention when converting those records.
