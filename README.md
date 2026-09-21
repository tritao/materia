# Materia

Materia is an experimental native application stack for building interactive
3D, CAD, simulation, and editor software. It combines a Haxeon-managed
application layer with NativeKit platform services and a retained UIKit UI.
SceneKit, SimKit, and CadKit provide the domain layers above that foundation.

The repository is an active integration workspace. The reference application
under `app/` is the smallest complete desktop host and is the best place to
start.

## Repository layout

| Directory | Purpose |
| --- | --- |
| [`app/`](app/) | Reference editor shell and Component Lab host |
| [`haxeon/`](haxeon/) | Haxe-compatible compiler, HashLink runtime, and project CLI |
| [`nativekit/`](nativekit/) | Windows, surfaces, input, GPU, and native runtime services |
| [`uikit/`](uikit/) | Retained UI layout, text, rendering, styles, and Haxe widgets |
| [`scenekit/`](scenekit/) | Retained scene data, rendering, picking, and interaction |
| [`simkit/`](simkit/) | Deterministic simulation orchestration and physics integration |
| [`sensorkit/`](sensorkit/) | Backend-independent sensor scheduling and measurement models |
| [`robotkit/`](robotkit/) | Complete robotics layer: runtime, protocol, world orchestration, and hosts |
| [`cadkit/`](cadkit/) | Headless CAD core and C ABI above Open CASCADE |

The native components are kept as sibling projects so they can be built and
tested independently. Their focused instructions live in each package's
README.

## Quick start

Materia uses Git submodules for the compiler, runtime, and native dependencies.
Clone them recursively:

```sh
git clone --recurse-submodules https://github.com/tritao/materia.git
cd materia
```

Install the pinned Haxeon development tools and check the host environment:

```sh
./haxeon/scripts/bootstrap-tools.sh
./haxeon/scripts/haxeon doctor
```

The reference app requires Git, a C/C++ toolchain, CMake, and the native
dependencies used by NativeKit UI. Build it with:

```sh
./haxeon/scripts/haxeon build --project app/haxeon.json
```

Build output is written below `app/build/` and is intentionally ignored by
Git.

## Run the reference editor

```sh
./haxeon/scripts/haxeon run --project app/haxeon.json
```

The host supports a headless workspace snapshot and deterministic diagnostics:

```sh
./haxeon/scripts/haxeon run --project app/haxeon.json -- --snapshot
./haxeon/scripts/haxeon run --project app/haxeon.json -- \
  --capture-dir=build/captures/default --frames=3
```

Open the reusable UIKit Component Lab with `--lab`, or capture one story:

```sh
./haxeon/scripts/haxeon run --project app/haxeon.json -- --lab
./haxeon/scripts/haxeon run --project app/haxeon.json -- \
  --story=text-field/editing --capture-dir=build/captures/text-field --frames=3
```

See [`app/README.md`](app/README.md) for the editor model, capture artifacts,
and host integration details.

## Development

Use the package README files for focused native build and test commands:

- [NativeKit](nativekit/README.md) and [UIKit](uikit/README.md) for platform and UI work;
- [SceneKit](scenekit/README.md) and [SimKit](simkit/README.md) for scene and simulation work;
- [CadKit](cadkit/README.md) for the headless CAD core;
- [Haxeon](haxeon/README.md) for compiler, runtime, and project-tool development.

When changing a submodule, publish its commit before updating the corresponding
submodule pointer in Materia so fresh recursive clones remain buildable.
