# Project loading integration checks

Build with `./haxeon/scripts/haxeon build --project=app/tests/project-loading/haxeon.json --compiler-only`.
Run `build/host/main.hl` with the same HashLink runtime and native-library paths as
`app/run-built.sh`, and set `MATERIA_INSTALL_ROOT` to this checkout.

The fixture exercises actual compiled generators, declared deterministic output,
editable input, source/input invalidation, corruption recovery and cancellation.
It also injects a memory preparation store to check reuse, build invalidation and
unavailable storage. Standalone artifacts open without running generators, saved
documents retain their artifact reference, and failed replacement preserves the
current document. Native geometry handles are fresh on every load.

Pass `--gantry` to open the picker, yaw picker and welder twice, compare their
assembly/physics/mission data, and check that repeat loading skips preparation.

For full browser coverage, build `app/web/build.sh`, then run:

```sh
app/web/test.sh --artifact /path/to/gantry.mtrg
```

Use `MATERIA_WEB_TARGET=wasm32` when building to test the linear backend; the
default is wasm-gc. Chrome checks 127 objects for the measured Gantry fixture,
render completion, warm preparation reuse, persistence after reload, checksum
repair after corrupting OPFS data, and malformed replacement preserving the scene.
It also cancels a cold preparation through the actual Cancel button, checks that
no scene/cache was published, and checks that frames advance during preparation.
The script accepts other nonempty artifact fixtures too.

The shared stdlib/backend checks are
`haxeon/tests/integration/test-json-parser.sh`,
`haxeon/tests/integration/test-wasm-runtime-boundaries.sh`, and
`haxeon/tests/integration/test-compiler-server.sh`.

On 2026-10-07, the 17,782,918-byte, 127-occurrence Gantry artifact reached its first
CPU-rendered frame in about 0.80 s on native with preparation and 0.24 s from cache.
Chrome measured 5.6–7.5 s initially and 3.0–3.5 s cached on wasm-gc; the wasm32 run
measured 4.9 s initially and 2.7–4.2 s cached, including reload. These runs overlapped
other compilation work and do not establish a backend ranking. These measurements preceded shared compiled assemblies. The shared Wasm `StringBuf` now copies
UTF-8 chunks once; replacing its repeated growing-prefix joins reduced the earlier
wasm-gc cached open from about 7.4 s. JSON Unicode/large-document checks cover the change.

These artifact measurements exclude source compilation. A source or toolchain change
can still require a native generator rebuild. Browser loading takes a prebuilt `.mtrg`;
first-time preparation now runs in an isolated dedicated Web Worker.

After sharing `CompiledAssembly` on 2026-10-07, the same artifact measured 0.61–0.69 s
cached on wasm-gc and 0.58–0.68 s cached on wasm32, including reload. Assembly setup
and installation each fell to tens of milliseconds, without JSON round-trip copies
or recompilation of topology during installation. Native measured 2 ms setup and
6 ms installation; total cached open was 0.29 s in that run. First browser opens
still include physics preparation (3.87 s wasm-gc and 2.39 s wasm32 in these runs).
The timing variation includes concurrent work; use the same artifact and runtime
for controlled comparisons.

`ProjectLoadingTests --ownership` checks model identity, independent joint values,
root input/record isolation and independent evaluated poses. `--gantry` also asserts
that the document installs the exact compiled model produced by its loader.

The preparation guest shares `ProjectScenePreparation`, `PortableCadGeometry` and
`PreparedSceneCodec` with native loading. `app/web/build.sh` compiles its separate
entry and checks its imports with `app/web/tools/check-preparation-imports.js`; adding a
native library, callback or filesystem runtime dependency fails that check.
Worker results and cache blobs use the same bounded checksum-validated format.

The browser cancellation test holds the real worker in synchronous work for five
seconds at its preparation boundary, then clicks the editor's Cancel button. This
makes dispatch timing deterministic without a delay or test hook in production.
Cached opens use `tryLoadCached`, which cannot compute missing preparation; cache
misses and corruption go to the worker. Native tests also validate that this fast
path writes nothing on a miss and that malformed worker data cannot be published.

After moving cold preparation into the worker, the same Gantry fixture measured
2.90 s cold and 0.54–0.70 s cached on wasm-gc. The wasm32 run measured 3.23 s cold
and 0.52–0.53 s cached. These are CPU first-render timings from separate runs,
not a controlled backend comparison. The browser checks assert that frames
continue advancing during preparation; the main benefit is UI responsiveness.

`app/web/test.sh --examples` drives the shared Start page catalog and its download
stage. It checks all 25 entries without eager asset requests, cancellation, failed
request retry, warm and persisted reuse with downloads blocked, a selected welding
job, worker animation assets restored into the native filesystem, and a compiled
setup example requiring no downloads. Use both browser guest targets. The native
publisher is `app/web/build-examples.sh`; its content-addressed assets include the
runtime settings normally resolved from project source manifests.

`app/web/test.sh --missions` checks actual mission execution: the published six-axis
arm must resume after pausing and complete a pick/place cycle without missing native planning functions
or project UI extension errors. The native fixture also verifies that binary
artifacts are not read as JSON project UI manifests.
