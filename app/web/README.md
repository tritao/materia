# Browser build

The reference editor also runs in the browser as two WebAssembly modules that
share one linear memory:

- **Guest**, `materia_guest.wasm`: the editor itself, compiled by Haxeon's
  `wasm32` backend from `app.MainWeb`.
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

`build.sh` does four things:

1. Generates wasm32 FFI interfaces for every kit into `app/build/web/hxi`, by
   running each kit's own `check-hxi.sh` with the wasm32 targets.
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
