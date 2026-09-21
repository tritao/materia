# SimKit

SimKit provides deterministic simulation orchestration and optional physics
backends for the Materia workspace. It consumes sibling SceneKit for stable
scene identities and synchronization, and sibling NativeKit through SceneKit.

The source is split into:

- `sim_core`: clocks, worlds, bodies, joints, snapshots, host integration, and
  scene synchronization;
- `sim_mujoco`: the MuJoCo physics backend.

Configure, build, and test from this directory:

```sh
git submodule update --init simkit/vendor/mujoco
cmake -S . -B build -GNinja -DCMAKE_BUILD_TYPE=Debug
cmake --build build
ctest --test-dir build -L sim --output-on-failure
```

The sibling checkout locations can be overridden with
`NKSIM_SCENEKIT_DIR` and `NKSCENE_NATIVEKIT_DIR`. Binding tools accept
`SCENEKIT_DIR`, `NATIVEKIT_DIR`, and `HAXEON_DIR`.
