# Materia project artifacts

`materia.project.SceneArtifact` is the shared writer and reader for generated
viewport geometry. A project generator writes the artifact to a file; Materia
loads and validates that file after the generator exits. The current `MTRG`
version 3 contains a metres-per-coordinate scale and independent mesh parts.
Each part has a stable ID, display name, color, vertex and normal streams,
triangle indices, and CAD face ranges. Readers reject unsupported versions,
duplicate IDs, invalid units, malformed buffers, and out-of-range indices.
Version 3 can also carry rigid instance poses, named connector frames, and
fixed, revolute, or prismatic joints. Version 2 geometry remains readable.

The artifact is derived output. The project's source code and manifest remain
authoritative. CadKit authors CAD occurrences and mates; ProjectKit stores their
portable assembly record; Materia applies the poses to preview geometry.
Joint motion and simulation state are separate from this initial pose record.
