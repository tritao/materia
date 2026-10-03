# CadKit

Headless CAD core with a C ABI and OCCT underneath. NativeKit and Haxeon are
integration layers above `cadkit-core`; they are not core dependencies.

## Build the first smoke test

```sh
cmake -S . -B build/debug -DCMAKE_BUILD_TYPE=Debug
cmake --build build/debug --parallel
ctest --test-dir build/debug --output-on-failure
```

The first configure builds the pinned vendored OCCT source with the foundation
and modeling modules needed by the initial primitive, bounds, transform, and
tessellation/topology/boolean API. OCCT is currently pinned to stable release
`V8_0_1`.

## Haxeon integration

`cadkit-core` remains headless and depends only on OCCT and the standard
library. Haxeon consumes its annotated C ABI from above; it is not a core
dependency. The default build produces a shared `libcadkit-core` for
Haxeon's dynamic FFI loader. See [`haxe/README.md`](haxe/README.md) for HXI
generation and the runtime smoke test.

## Building OCCT once

Compiling OCCT takes most of an hour, so a build never starts it by accident. CadKit uses a prebuilt install of the
pinned source, found through `CADKIT_OCCT_DIR` or in the shared cache at
`<checkout>/../materia-cache/occt-<revision>-release` (the revision is the last commit that touched
`third_party/occt`; `MATERIA_CACHE_DIR` moves the cache). Create it once with:

```sh
cadkit/scripts/build-occt-prefix.sh
```

Every checkout and worktree next to that cache then finds it automatically. If it is missing, configuring fails with
that instruction. To compile OCCT as part of one build instead, pass `-DCADKIT_BUILD_OCCT_FROM_SOURCE=ON` (or set the
environment variable of the same name). A changed OCCT pin changes the revision, so the old prefix is not reused.
