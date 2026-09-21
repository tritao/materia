# SceneKit

SceneKit provides the retained scene database, render planning and GPU
execution, spatial queries and picking, and interaction state for the Materia
workspace. It consumes sibling NativeKit for platform and GPU services and
sibling Haxeon for binding validation.

The source is split into three dependency-ordered libraries:

- `scene`: retained data, transactions, snapshots, and stable identities;
- `scene_render`: render plans, spatial indexing, picking, and GPU execution;
- `scene_interaction`: selection, hover, and interaction presentation.

Configure, build, and test from this directory:

```sh
cmake -S . -B build -GNinja -DCMAKE_BUILD_TYPE=Debug
cmake --build build
ctest --test-dir build -L scene --output-on-failure
```

Override sibling checkout locations with `NKSCENE_NATIVEKIT_DIR` at CMake
configure time, or `NATIVEKIT_DIR` and `HAXEON_DIR` when running binding tools.
The Haxeon end-to-end smoke test uses the SceneKit build directory:

```sh
NATIVEKIT_BUILD="$PWD/build" scene_render/tools/test-haxeon.sh
```
