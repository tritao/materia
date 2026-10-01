# Browser build

The reference editor also runs in the browser as two WebAssembly modules that
share one linear memory:

- **Guest**, `materia_guest.wasm`: the editor itself, compiled by Haxeon from
  `app.MainWeb` (see [Guest target](#guest-target)).
- **Host**, `materia_web.{js,wasm}`: an Emscripten executable linking NativeKit,
  UIKit, SceneKit, SimKit with MuJoCo, RobotKit's runtime and, when given OCCT,
  CadKit. It provides WebGL2, input, fonts and the C functions the guest
  imports.

`materia.js` loads both, checks that they agree on the
[memory contract](../../nativekit/docs/wasm-host-memory.md), and drives
`app.MainWeb.frame` from `requestAnimationFrame`.

## Build and run

```sh
./nativekit/tools/setup-web.sh          # once: installs Emscripten
./app/web/build.sh                      # about 2 minutes from clean
python3 -m http.server --directory app/build/web/site 8080
```

### CadKit and OCCT

CadKit links when `MATERIA_WEB_OCCT_DIR` names an Emscripten install of the
pinned OCCT source (`cadkit/third_party/occt`); without it CadKit stays
unavailable. OCCT takes long to build, so build it once into a shared prefix,
with the toolkits CadKit uses, static, and with `-fwasm-exceptions` like the
rest of the host. `-UOCC_CONVERT_SIGNALS` drops OCCT's conversion of signals to
exceptions, which the browser has no signals for: it puts a `setjmp` inside
every guarded `try`, and LLVM emits invalid Wasm for that mix under Wasm
exceptions (V8: "br_table: label arity inconsistent").

```sh
source nativekit/.tools/emsdk/emsdk_env.sh
emcmake cmake -S cadkit/third_party/occt -B <build> -G Ninja \
  -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=<prefix> \
  "-DCMAKE_CXX_FLAGS=-fwasm-exceptions -UOCC_CONVERT_SIGNALS" \
  "-DCMAKE_C_FLAGS=-fwasm-exceptions -UOCC_CONVERT_SIGNALS" \
  -DBUILD_LIBRARY_TYPE=Static -DUSE_FREETYPE=OFF -DUSE_OPENGL=OFF -DUSE_GLES2=OFF \
  -DUSE_XLIB=OFF -DUSE_TK=OFF -DUSE_TCL=OFF -DUSE_TBB=OFF -DBUILD_DOC_Overview=OFF \
  -DBUILD_RESOURCES=OFF -DBUILD_MODULE_ApplicationFramework=OFF \
  -DBUILD_MODULE_DataExchange=OFF -DBUILD_MODULE_Draw=OFF \
  -DBUILD_MODULE_Visualization=OFF -DBUILD_MODULE_FoundationClasses=OFF \
  -DBUILD_MODULE_ModelingData=OFF -DBUILD_MODULE_ModelingAlgorithms=OFF \
  "-DBUILD_ADDITIONAL_TOOLKITS=TKBO;TKDE;TKDESTEP;TKFillet;TKOffset;TKMesh;TKPrim;TKTopAlgo;TKXSBase"
ninja -C <build> -j8 && cmake --install <build>
MATERIA_WEB_OCCT_DIR=<prefix> ./app/web/build.sh
```

The toolkit list is the one `cadkit/CMakeLists.txt` builds for the desktop.

`app/web/test.sh` opens the page in headless Chrome and checks that the editor
starts and draws. `--click X,Y` clicks after startup and `--screenshot PATH`
saves the result.

`app/web/test.sh --tour` walks through the edits people make most (an empty
scene, adding rectangles from the Add menu, undo and redo, typing a position
into the inspector, delete, the 3D view) and checks the editor's state after
each step. Controls are found by style key or accessibility label through
`window.materia.inspect()`, which prints `app.MainWeb.report` as a
`materia-report` console line, so the tour survives layout changes. Run it
against both guest targets:

```sh
./app/web/build.sh && ./app/web/test.sh --tour
MATERIA_WEB_TARGET=wasm32 ./app/web/build.sh && ./app/web/test.sh --tour
```

## Guest target

`build.sh` compiles the guest with Haxeon's `wasm-gc` backend, which keeps Haxe
values as Wasm GC objects collected by the browser. It needs Wasm GC: Chrome
119, Firefox 120 or Safari 18.2 and later. `MATERIA_WEB_TARGET=wasm32` builds
the linear-memory backend instead, which keeps Haxe values in the shared memory
with Haxeon's own collector and runs where Wasm GC does not.

Both use the same host and the same C interfaces. Measured on 2026-10-01 in
headless Chrome with software WebGL, a frame while hovering the 3D view took
3.1 ms on wasm-gc and 25 ms on wasm32; download size (about 5.5 MB gzipped)
and startup (under half a second to the first frame) were about the same.

`build.sh` does four things:

1. Generates wasm32 FFI interfaces for every kit into `app/build/web/hxi`, by
   running each kit's own `check-hxi.sh` with the wasm32 targets. They describe
   the host's C ABI, which both guest targets call.
2. Compiles the guest with arguments derived from `app/haxeon.json`
   (`tools/guest-arguments.py`). `haxeon build` cannot target wasm32 yet,
   because native providers only build for the host and Android.
3. Links the host and exports every C function the guest imports from it.
4. Runs `tools/check-imports.js`, which lists every guest import whose Wasm
   type differs from the host function it binds to.

## What works and what does not yet

The editor shell, docking, inspector, console and the 3D viewport all work,
and so does editing primitive objects. The following do not work yet:

- **Kits with no browser build:** AnimKit, StockKit, and CadKit unless built
  with OCCT (see [CadKit and OCCT](#cadkit-and-occt)). The page gives their
  imports stubs that throw `haxeon.wasm.HostError`, which the editor catches
  and logs; `window.materia.unavailable` lists them.
- **Opening projects:** this compiles and runs child processes, which the browser
  cannot start.
- **Files:** they live in an in-memory filesystem for the session.
- **Threads:** project loads and workspace saves run on the UI thread, and the
  simulation steps on it each frame. RobotKit's self-driven runtimes and
  recordings, and its serial and simulated-board devices, are not built.
