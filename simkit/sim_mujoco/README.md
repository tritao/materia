# NativeKit MuJoCo adapter

`NativeKit::sim_mujoco` is the first engine adapter for `NativeKit::sim_core`.
It is disabled by default and uses the vendored MuJoCo `main` submodule when
enabled:

```text
NK_BUILD_SIM_MUJOCO=ON
```

Initialize the dependency in a fresh checkout with:

```sh
git submodule update --init vendor/mujoco
```

Set `NK_SIM_MUJOCO_USE_VENDORED=OFF` to consume an installed MuJoCo CMake
package instead. `NK_MUJOCO_TARGET` can select a non-default package target.

The adapter exposes only NativeKit simulation handles. MuJoCo's `mjSpec`,
`mjModel`, `mjData`, and numeric IDs remain implementation details of this
module. Structural body and joint changes rebuild the internal model; ordinary
ticks use the compiled model/data pair.

The current adapter supports free bodies plus fixed, revolute, and prismatic
NativeKit joints.

A kinematic body is pinned like static geometry, without mass or degrees of
freedom, and SimKit places it at its prescribed pose before each step; SimKit
adds its twist to the velocities reported for the links articulated beneath
it. Contacts see no velocity for it, so a moving kinematic platform does not
drag resting bodies along by friction. A body without explicit inertial
properties has its centre of mass at its origin.

Position, velocity, and effort targets are batched through
the internal backend seam and applied through private MuJoCo actuators and
`mjData.ctrl`. MuJoCo actuator objects and types are intentionally not part of
the NativeKit API.

Haxeon consumers use `bindings/nativekit-sim-mujoco.hxi` and the thin
`nativekit.sim.MujocoSimWorld` façade. It creates a MuJoCo-backed world while
returning the engine-neutral `Shape`, `Body`, `Joint`, `StepResult`, and
`SimSnapshot` types from `sim_core`; joint targets are staged and sent in one
native batch at the next step. Binding checks are available through
`tools/check-hxi.sh`, `tools/check-haxeon.sh`, and
`tools/check-haxeon-runtime.sh`.

With examples enabled, the headless falling-body/hinge and stacking demos are
built as `nativekit_sim_mujoco_falling_hinge` and
`nativekit_sim_mujoco_stacked_boxes`. With tests enabled,
`nativekit_sim_mujoco_benchmark` measures fixed-step throughput for a 64-body
stack.
