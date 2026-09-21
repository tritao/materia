# NativeKit simulation core

`NativeKit::sim_core` is the deliberately small, headless simulation boundary.
It is enabled with `NK_BUILD_SIM_CORE=ON` and depends only on
`NativeKit::scene`, which in turn depends on `NativeKit::nativekit`.

The world borrows its `nkscene_scene`; it never destroys or owns that scene.
The world owns bodies, joints, shapes, the deterministic simulation clock, and
the selected physics backend. Mutable world operations are single-owner-thread
operations. Callers on other threads should enqueue application commands and
apply them from the simulation owner.

`nativekit_sim_host.h` provides the optional `nksim_host` owner-thread loop.
Creating a host does not start a thread or change the world. `nksim_host_start`
transfers world ownership to the host thread; `nksim_host_stop` joins it and
transfers ownership back to the caller. While a host is running, callers must
not invoke mutable `nksim_world_*`, body, joint, or shape operations directly.
Submit force and joint-target batches through the host instead.

The host supports three scheduling modes:

* `NKSIM_HOST_MODE_EXTERNAL` waits for `nksim_host_step`, which returns the
  step's owned scene change set.
* `NKSIM_HOST_MODE_REALTIME` schedules fixed ticks against monotonic wall time.
* `NKSIM_HOST_MODE_UNBOUNDED` runs fixed ticks as quickly as the owner thread
  can complete them.

Every host publishes an immutable latest `nksim_snapshot`. Snapshot reads do
not touch the mutable world and are safe after the host has handed them to a
consumer. `pause` stops automatic scheduling but does not prevent an explicit
host step.

Each `nksim_world_step()` performs one fixed tick. The built-in backend is an
internal gravity-only, no-collision backend used to test the headless boundary.
It integrates dynamic bodies, accepts kinematic scene poses, and leaves static
bodies at their scene-initialized pose. After backend state is read, dynamic
body poses are committed to the scene in one transaction and the owned
`nkscene_change_set` is returned in the step result.

Physics backends are an internal C++ contract. They are deliberately not part
of the public C ABI yet; `NativeKit::sim_mujoco` is the first engine adapter
planned for this boundary. MuJoCo types and model/data pointers must not cross
the Sim API.

Haxeon consumers can use `bindings/nativekit-sim.hxi`, the typed wrappers in
`bindings/haxe/nativekit/sim/`, and `bindings/nativekit-sim.hxmap`.
