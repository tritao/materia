# Modeling with CadKit

`MountingPlate.hx` builds an 80 × 50 × 6 plate from a sketch, subtracts four
radius-3 holes on a 60 × 30 grid, extrudes it, selects the four outer vertical
edges, and applies radius-2 fillets. Its `build()` method returns an owning
`Part`. Its `main()` writes `mounting-plate.step` in the working directory.

The example is compiled and exercised by `./scripts/test-haxeon`. To run it
standalone after building CadKit:

```sh
repo_root="$PWD"
haxeon_root="$repo_root/../haxeon"
"$haxeon_root/.tools/haxe/haxe" --cwd "$haxeon_root" -cp src \
  --run compiler.tools.HaxeonCompiler --target=hl \
  "--output=$repo_root/build/haxeon/mounting-plate.hl" --entry=MountingPlate \
  "--root=$repo_root/haxe/src" "--root=$repo_root/examples/modeling" \
  "--ffi-interface=$repo_root/haxe/abi/CadKit.hxi" \
  "--ffi-projection=$repo_root/haxe/abi/CadKit.hxmap" \
  "$repo_root/examples/modeling/MountingPlate.hx"
LD_LIBRARY_PATH="$repo_root/build/debug/core:$repo_root/build/debug/lin64/gcc/libd:$haxeon_root/out:$haxeon_root/.tools/hashlink" \
  "$haxeon_root/.tools/hashlink/hl" "$repo_root/build/haxeon/mounting-plate.hl"
```

See [the modeling API](../../haxe/MODELING.md) for ownership, selections,
workplanes, additional operations, and document integration.

## Procedural excavator geometry

`ProceduralExcavator.hx` is the code-authored geometry example for the
FreeCAD `AssemblyExample`-style excavator. It generates 13 separate OCCT
components: the base and base pin, boom, stick, bucket, two bucket links, and
the inner and outer members of three hydraulic cylinders. The plates use
polygonal CAD profiles with bored pivots; the bucket includes a cutting lip and
teeth; cylinder bodies are hollow and their pin eyes are bored. No source
geometry is imported. `ProceduralExcavatorAssembly.hx` places the parts using
named pin frames, revolute mates, prismatic cylinder slides, and checked
linkage closures.

Running the example prints JSON with solid, face, edge, volume, surface-area,
and bounding-box metrics for every component. It also writes each generated
B-rep as a STEP file for external inspection; those files are diagnostic
outputs, not model inputs. To capture the matching inventory from FreeCAD, open
`AssemblyExample.FCStd` and run `assembly_example_baseline.py` in its Python
console. Compare the per-part metrics and fixed-view silhouettes when refining
geometry. The source solids retain their CAD coordinates. The preview artifact
records instance poses separately, so placement is not baked into the B-reps.

The refined geometry pass produces one solid per occurrence. The boom, stick,
and bucket profiles now use dimensions scaled from the captured reference
bounds, with the bucket built from paired cheeks, a cross barrel, a shell web,
a cutting edge, and five teeth. This face/edge comparison tracks the remaining
topology differences:

| Component | Generated faces/edges | FreeCAD reference |
| --- | ---: | ---: |
| Base | 63 / 137 | 44 / 122 |
| Base pin | 3 / 3 | 3 / 3 |
| Boom | 29 / 81 | 31 / 87 |
| Stick | 28 / 62 | 24 / 66 |
| Bucket | 92 / 272 | 97 / 247 |
| Bucket link 1 | 22 / 51 | 22 / 54 |
| Bucket link 2 | 18 / 45 | 23 / 57 |
| Boom cylinder outer/inner | 13 / 34; 8 / 13 | 8 / 12; 6 / 9 |
| Stick cylinder outer/inner | 10 / 20; 13 / 22 | 8 / 12; 6 / 9 |
| Bucket cylinder outer/inner | 11 / 22; 13 / 22 | 10 / 18; 6 / 9 |

Across all 13 components, generated volume is 2,030,373 mm³ versus 2,066,341
mm³ in FreeCAD (1.7% lower). Face and edge totals are 323/784 versus 288/705;
the remaining topology gap is concentrated in the base and hydraulic details.
The boom is 1,170,459 mm³ versus 1,204,575 mm³, the stick is 331,043 mm³ versus
333,942 mm³, and the bucket is 59,986 mm³ versus 58,312 mm³. Their sorted
bounding dimensions are 61.0 × 206.7 × 436.6, 32.0 × 119.8 × 379.6, and 101.5
× 103.4 × 118.0 mm; the references are 60.9 × 203.7 × 436.3, 31.8 × 121.2 ×
379.8, and 101.5 × 103.4 × 117.6 mm. These figures predate the two additional
stick linkage bores; the geometry inventory needs a refreshed comparison.

Run it after building CadKit and Haxeon:

```sh
repo_root="$PWD"
haxeon_root="$repo_root/../haxeon"
"$haxeon_root/.tools/haxe/haxe" --cwd "$haxeon_root" -cp src \
  --run compiler.tools.HaxeonCompiler --target=hl \
  "--output=$repo_root/build/haxeon/procedural-excavator.hl" \
  --entry=ProceduralExcavator \
  "--root=$repo_root/haxe/src" \
  "--root=$repo_root/examples/modeling" \
  "--ffi-interface=$repo_root/haxe/abi/CadKit.hxi" \
  "--ffi-projection=$repo_root/haxe/abi/CadKit.hxmap" \
  "$repo_root/examples/modeling/ProceduralExcavator.hx"
LD_LIBRARY_PATH="$repo_root/build/debug/core:$repo_root/build/debug/lin64/gcc/libd:$haxeon_root/out:$haxeon_root/.tools/hashlink" \
  "$haxeon_root/.tools/hashlink/hl" \
  "$repo_root/build/haxeon/procedural-excavator.hl"
```

## Editable mounting plate

`EditableMountingPlate.hx` builds the same design with the explicit recording
builder. Its seven named dimensions and selected output survive JSON reload.
Its `resize(width, depth, spacingX, spacingY)` groups and validates parameter
edits and recomputes the corner fillets. Width/depth, hole radius, spacing,
extrusion height, and fillet radius are exposed through its feature parameters.
The example conservatively keeps holes clear of the rounded boundary.

Use the standalone command above with entry/source `EditableMountingPlate`
and output `editable-mounting-plate.hl`. Running it writes
`editable-mounting-plate.json`, `editable-mounting-plate.step`, and
`editable-mounting-plate-reloaded.step`. It changes dimensions and hole spacing,
checks the undo/redo path, reloads the saved document, and exports it again.
Both examples are exercised by `./scripts/test-haxeon`, including shared named
dimensions, direct bound-feature edits, failed-builder cleanup, and persistence.

## Open the generated assembly in Materia

`materia.project.json` points to the Haxeon package and names a viewport
entrypoint. Materia compiles that entrypoint as a separate program, which builds
the B-rep components with CadKit and writes a versioned scene artifact. The
artifact declares metres per coordinate and stable IDs for all 13 components.
Materia loads them as posed CAD preview objects, retaining the assembly's named
connectors and joints for the hierarchy and inspector. STEP is never an input.
The compatibility `AssemblyRecord` still carries those preview poses.

CadKit also exposes a versioned definition/state path. `AssemblyModel.definition()`
converts an existing assembly to component definitions, occurrences, explicit
tree/closure roles, axes, limits, and default joint coordinates;
`AssemblyModel.initialState()` creates a separate runtime state. A state can
change a tree coordinate with `setJoint()`, recompute placements with
`forwardKinematics()`, and read a derived pose with `worldPose()`. Closure edges
are not used for FK; `closureResiduals()` reports their current geometric error
until the joint-coordinate loop solver is added. `AssemblyBuilder` can author
repeated occurrences that reference the same component definition directly.

The generated artifact writes both the new definition/state payload and the
compatibility record. Existing scene artifact readers remain supported, and the
current editor continues to use the compatibility record until its hierarchy
and preview path switches to occurrence instancing.

The viewport entrypoint opts into a generated artifact cache. Materia reuses the
artifact when its Haxeon source graph, compiler sources, and native runtime build
have not changed. Scripts that read additional data files must list those files
in the entrypoint's `cache.inputs` array, relative to `materia.project.json`.
Entrypoints without a `cache` declaration execute on every open. The cache is
stored under `$XDG_CACHE_HOME/materia/generated-artifacts`, or `~/.cache/materia`
when `XDG_CACHE_HOME` is unset. Only deterministic entrypoints should opt in;
omit `cache` when output depends on the clock, environment, or undeclared inputs.

Saving this view as a Materia scene keeps a relative reference to the project
manifest, plus authored transforms, appearance, removals, linked copies, and
ordinary scene objects. It does not embed the generated mesh snapshots.
Opening the saved scene runs the manifest again and applies those edits to the
new geometry, so the manifest and its Haxeon sources must remain available.

Build the app once, then launch the built app with:

```sh
./haxeon/scripts/haxeon build --project=app/haxeon.json
./app/run-built.sh --project=./cadkit/examples/modeling/materia.project.json
```

`run-built.sh` accepts the normal Materia arguments and skips rebuilding the
app. Use the build command again after changing app or native sources.

The save/reopen integration check uses the app's native build:

```sh
./haxeon/scripts/haxeon run --project=app/haxeon.project-source.json \
  --output="$PWD/app/build/host/project-source.hl"
```

The same `materia.project.json` structure can later select other entrypoint
kinds, such as a robot simulation, while each runtime continues to use its own
Haxeon package manifest.
