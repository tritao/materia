# SensorKit

SensorKit turns immutable world truth into imperfect sensor observations.
It is intentionally a sibling of SceneKit and SimKit: SimKit supplies physical
state, SceneKit supplies geometry and rendering, and SensorKit owns sampling,
measurement models, and delivery semantics.

The initial implementation is `sensor_core`. It has no dependency on a physics
engine, SceneKit, ROS2, SDF, or a transport. It currently provides:

- immutable-value truth and measurement types for an IMU;
- deterministic periodic and manual scheduling;
- separate capture and delivery timestamps;
- latency, jitter, integration windows, and dropouts;
- Gaussian noise, bias random walks, quantization, and clipping;
- an insertion-ordered sensor manager.

The current follow-up library is:

```text
sensor_core    backend-independent scheduling, models, and data types
sensor_sim     SimKit truth adapters
sensor_render  SceneKit GPU rendering and RGBA8 camera capture
```

`sensor_sim::ImuTruthAdapter` consumes an immutable `nksim_snapshot` and
derives linear acceleration from consecutive body velocities. It never reads
the mutable simulation world. The first sample after construction or reset has
zero estimated acceleration; this is deliberate and makes seek/replay
boundaries explicit.

`sensor_render::SceneCameraAdapter` compiles a camera view against an immutable
SceneKit snapshot, renders it into an off-screen RGBA8 target, and packages the
readback as a `CameraFrame`. The adapter borrows the GPU renderer and keeps
publishing, ROS2, recording, and application callbacks outside the sensor
model.

The first camera slice intentionally covers perspective RGB capture only.
Depth, segmentation, distortion, exposure, and camera noise can reuse the
same off-screen render path without changing the core sensor boundary.

The core API follows the invariant:

```text
truth snapshot -> sensor model -> sensor sample
```

For a stationary IMU, for example, a world-frame kinematic acceleration of
zero and gravity of `(0, 0, -9.81)` produce a measured specific force of
`(0, 0, 9.81)` before noise and post-processing.

## Build

```sh
cmake -S sensorkit -B /tmp/materia-sensorkit-build -DNK_BUILD_TESTS=ON
cmake --build /tmp/materia-sensorkit-build
ctest --test-dir /tmp/materia-sensorkit-build --output-on-failure
```
