# Materia project artifacts

`materia.project.SceneArtifact` is the shared writer and reader for generated
viewport geometry. A project generator writes the artifact to a file; Materia
loads and validates that file after the generator exits. The current `MTRG`
version 2 contains a metres-per-coordinate scale and independent mesh parts.
Each part has a stable ID, display name, color, vertex and normal streams,
triangle indices, and CAD face ranges. Readers reject unsupported versions,
duplicate IDs, invalid units, malformed buffers, and out-of-range indices.

The artifact is derived output. The project's source code and manifest remain
authoritative. Assembly transforms, joints, materials, and simulation state
require new versioned fields before this format can represent them.
