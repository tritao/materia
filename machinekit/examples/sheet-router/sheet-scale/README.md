# SR9 sheet stock and machining artifacts

Run from the repository root:

```sh
machinekit/examples/sheet-router/sheet-scale/check.sh
```

The script builds StockKit twice with the same compiler: compact uniform tiles and
`SK_UNIFORM_TILES=OFF` as the dense reference. No OCCT rebuild is required. The host
checks reuse the existing generated FFI runtime; the preview test explicitly loads
the newly built core and checks its initial storage, so it cannot silently use an
older dense library. Heavy work uses the shared heavy-run gate.

ProjectKit passes 388 assertions, the snapshot worker passes eight, and the
artifact-to-CNC loader passes five. The full host app compiles (2947 sources).
Browser runtime and whole-editor memory were not exercised. Machine-readable
results and tested source hashes are in [validation.json](validation.json).

## Storage evidence

Both beds use 1600 mm wide, 18 mm thick mesh stock at **0.5 mm**, all three dexel
grids, four cutting threads, initial and finished snapshots, volume accounting,
and a 2×2-tile contour. The cabinet fixture has two columns of three/four blanks,
0.5 mm onion skin and four blind holes per blank. It exercises 48/64 moves; it is
a backend workload, not an approved cutting program or SR11 tab-strength check.

| Bed Y | Initial stock bytes | After job bytes | Peak process RSS |
|---|---:|---:|---:|
| 2600 mm | 31,442,608 | 150,129,280 | 184,385,536 |
| 3150 mm | 37,492,720 | 175,620,880 | 213,581,824 |

Both peaks are below the assumed **300,000,000-byte stock-backend budget**. RSS
includes both retained snapshots and scratch in this test process. It does not
measure the editor, retained full-sheet render meshes or GPU buffers. Whole-editor
memory qualification remains separate; no whole-sheet renderer claim follows from
these numbers.

The small box/mesh fixture hashes include all public ray data (including normals
and source IDs), removed-volume results and contact data. Compact and dense builds,
one and four threads, give respectively `d59364abe695549b` (box) and
`dda59198b1ab2b1b` (mesh). Each restores an initial snapshot after cutting. Three
existing native suites also pass in both storage modes.

Equal rays share one exact interval list plus an optional occupancy mask; the first
edit materializes a private dense tile. Changed tiles are packed during cutting,
preventing abandoned interval slots from growing across the cabinet job. Compression
compares floating-point bit patterns, including surface normals. Mesh-import hit
scratch is bounded to one tile-row strip rather than the complete sheet.

## Program dependency and coordinate contract

`SceneArtifactMachining` accepts exactly one of `program` (legacy inline text) and
`programFile` (portable relative path). An external file requires lowercase
`programSha256`. Programs are read separately, with a 64 MB bound before allocation
in the app file reader and a hash check before use. The 20,000,000-byte test program
loads without increasing the artifact's bounded JSON section. Missing, changed or
ambiguous dependencies and `..` paths fail explicitly.

For source projects, paths are relative to the source project manifest's directory;
the file must already exist there and should be declared among generator cache
inputs when it influences generated geometry. For a prebuilt `.mtrg`, paths are
relative to that artifact. Browser files must be present in its virtual filesystem;
there is no implicit network fetch. `MateriaProjectRunner.exportArtifactBundle`
returns `preview.mtrg` and the verified sidecar at its declared relative path. The
single-file export refuses external dependencies rather than publishing a broken
artifact. Existing inline projects retain their single-file export.

`workOffset` remains G54. `workOffsets` adds distinct G55–G59 entries; each offset is
three finite machine-coordinate values in metres. `stockSpacing` is a positive
finite pitch in metres (default 0.0005). Named `targets` declare a geometry `part`,
its `stock` occurrence, SI `position` and a Z `angle` in radians. Mesh vertices are
scaled from artifact units, rotated, then translated into that stock's local frame.
Target IDs and meshes remain distinct in the loaded job. Deviation preview uses
one union of the targets on the selected blank, avoiding a full sheet lattice per
finished part. This is geometric comparison, not per-part retention or vacuum
qualification; SR8's freed-component checks and SR11 operation policy apply there.

The existing snapshot preview worker contours tile chunks on its own thread. The
new eight-assertion fixture compares every mesh stream with synchronous contours
before cutting, after a local cut and after snapshot restoration. Restoring X/Y
rays conservatively invalidates intersecting rows/columns, so the worker may rebuild
more chunks than the direct cut, while still avoiding a full rebuild in the fixture.
Host and browser preparation share the target-body filter.

Retained run packet: `build/sheet-router-evidence/sr9-sheet-scale.tar.gz`
(SHA-256 `cee05548db25268f7015b04b43f1ece7a6915f6d9d6ef18bcfd490b99e1ec29d`). It contains regular files only and a verified member
manifest. Re-running the checks updates working logs under `build/sheet-router-work/sr9`.
