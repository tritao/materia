# Browser build

The reference editor also runs in the browser as two WebAssembly modules that
share one linear memory:

- **Guest**, `materia_guest.wasm`: the editor itself, compiled by Haxeon from
  `app.MainWeb` (see [Guest target](#guest-target)).
- **Host**, `materia_web.{js,wasm}`: an Emscripten executable linking NativeKit,
  UIKit and SceneKit. It provides WebGL2, input, fonts and the C functions the
  guest imports.

`materia.js` loads both, checks that they agree on the
[memory contract](../../nativekit/docs/wasm-host-memory.md), and drives
`app.MainWeb.frame` from `requestAnimationFrame`.

## Build and run

```sh
./nativekit/tools/setup-web.sh          # once: installs Emscripten
./app/web/build.sh                      # about 2 minutes from clean
python3 -m http.server --directory app/build/web/site 8080
```

`app/web/test.sh` opens the page in headless Chrome and checks that the editor
starts and draws. `--click X,Y` clicks after startup and `--screenshot PATH`
saves the result.

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

- **Kits with no browser build:** CadKit (OCCT), SimKit and MuJoCo, RobotKit's
  runtime, AnimKit and StockKit. The page gives their imports stubs that throw,
  so features that call them fail when used, and the page's `window.materia.unavailable`
  lists them.
- **Opening projects:** this compiles and runs child processes, which the browser
  cannot start.
- **Files:** they live in an in-memory filesystem for the session.
- **Threads:** project loads and workspace saves run on the UI thread.
