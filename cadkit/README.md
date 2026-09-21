# CadKit

Headless CAD core with a C ABI and OCCT underneath. NativeKit and Haxeon are
integration layers above `cadkit-core`; they are not core dependencies.

## Build the first smoke test

```sh
cmake -S . -B build/debug -DCMAKE_BUILD_TYPE=Debug
cmake --build build/debug --parallel
ctest --test-dir build/debug --output-on-failure
```

The first configure builds the pinned OCCT submodule with the foundation and
modeling modules needed by the initial primitive, bounds, transform, and
tessellation/topology/boolean API. OCCT is currently pinned to stable release
`V8_0_1`.

## Haxeon integration

`cadkit-core` remains headless and depends only on OCCT and the standard
library. Haxeon consumes its annotated C ABI from above; it is not a core
dependency. The default build produces a shared `libcadkit-core` for
Haxeon's dynamic FFI loader. See [`haxe/README.md`](haxe/README.md) for HXI
generation and the runtime smoke test.
