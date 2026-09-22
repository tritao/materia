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
- an insertion-ordered runtime registry for scheduled sensors and producers.

The current follow-up library is:

```text
sensor_core    backend-independent scheduling, models, and data types
sensor_io      Haxeon-compatible MessagePack packets, fanout, and buffer replay
sensor_sim     SimKit truth adapters
sensor_render  SceneKit GPU rendering and RGBA8 camera capture
```

`sensor_io` is the external packet slice. It reuses Haxeon's
versioned `HMPK` envelope and encodes image-like sensor data as one
integer-keyed MessagePack map whose payload is a packed binary field. Camera,
depth, and segmentation share this `PackedFrame` wire shape; `messageType` and
`pixelFormat` select the interpretation of the data. The canonical definitions
and field tables live in [`schema/sensor_wire.nkw`](schema/sensor_wire.nkw) and
the generated [wire reference](sensor_io/generated/sensor_wire.md). The C++
decoder provides a generic `PackedFrameView` whose data span points directly
into the encoded message, an owning `PackedFrame`, and typed camera, depth, and
segmentation wrappers. Generated Haxeon records are checked in under
`sensor_io/haxe/materia/sensor/wire/`.

The initial packed formats are RGBA8 camera data, little-endian R32F depth,
little-endian R32U or U64 segmentation labels. Payloads are tightly packed
with no row padding; the stride is still carried explicitly so a future
version can add padded or tiled representations without changing the framing.

IMU samples use the same HMPK envelope and shared header field IDs. Their
binary field contains 24 little-endian binary64 values: angular velocity,
linear acceleration, and both row-major 3x3 covariance matrices. The C++ API
provides a zero-copy `ImuSampleView`, an owning `ImuSample` decoder, and a
variant adapter for measurements returned by `SensorRuntime`. The matching
Haxeon record is `sensor_io/haxe/materia/sensor/wire/ImuSampleMessage.hx`.

LiDAR scans use the same envelope and carry dimensions plus one packed 12-byte
record per ray: little-endian float32 range, little-endian float32 intensity,
one hit flag, and three reserved bytes. Records are ordered vertical-major,
then horizontal. The first transport version intentionally omits backend hit
points and normals; the C++ API provides zero-copy `LidarScanView`, owning
decode, and runtime variant dispatch. The matching Haxeon record is
`sensor_io/haxe/materia/sensor/wire/LidarScanMessage.hx`.

The `nksensor::stream` API is compiled into `sensor_io` alongside the codec.
`make_sensor_packet` encodes one core measurement and stores its bytes in
shared immutable storage. `PacketFanout` delivers that same packet to live
transports, recorders, UI callbacks, or tests. `PacketBuffer` is a small
deterministic in-memory recorder/replay source that preserves packet order and
exact HMPK bytes; it intentionally does not choose a file format or transport
implementation.

`sensor_sim::ImuTruthAdapter` consumes an immutable `nksim_snapshot` and
derives linear acceleration from consecutive body velocities. It never reads
the mutable simulation world. The first sample after construction or reset has
zero estimated acceleration; this is deliberate and makes seek/replay
boundaries explicit.

`sensor_sim::SceneLidarAdapter` builds a read-only spatial index from one
SceneKit snapshot, transforms each configured ray by the sensor pose, and
batch-raycasts the rays before handing the results to `LidarSensor`. LiDAR range
is distance along the unit ray; a ray at 45 degrees can therefore have a
different local-X projection. Dropped ticks short-circuit before scene queries.

`sensor_render::SceneCameraAdapter` compiles a camera view against an immutable
SceneKit snapshot, renders it into an off-screen RGBA8 target, and packages the
readback as a `CameraFrame`. The adapter borrows the GPU renderer and keeps
publishing, ROS2, recording, and application callbacks outside the sensor
model.

Camera post-processing is a second off-screen pass. On the GLCore and GLES3
backends, SensorKit keeps the rendered image on the GPU while applying exposure,
gain, seeded Gaussian noise, quantization, radial distortion, and pixel
dropout. Other backends use the same deterministic CPU reference model. The
raw scene pass and post-processing pass use separate images so a pass never
samples from the image it is currently rendering to.

The processing contract is intentionally small and fixed: each output pixel
is inverse-mapped through radial distortion, independently dropped if selected,
then processed in this order: exposure and gain, additive Gaussian noise,
quantization, and final clamping. RGB values are normalized to `[0, 1]`; alpha
is preserved unless a pixel is dropped, in which case the result is opaque
black. Both implementations use nearest-neighbour lookup for distortion so
the CPU fallback and GPU path have the same sampling semantics.

The first camera slice intentionally covers perspective RGB capture only.
Perspective metric depth capture is now also available through
`SceneCameraAdapter::capture_depth`; it converts the renderer's nonlinear
depth-buffer value into camera-forward distance in metres. Segmentation,
distortion, exposure, and camera noise can reuse the same off-screen render
path without changing the core sensor boundary. The CPU reference path is
documented alongside the GPU shader: it copies the source image before
inverse-mapping distorted output pixels, then applies the same effect order.

The core API follows the invariant:

```text
truth snapshot -> sensor model -> sensor sample
```

`SensorRuntime` is the thin orchestration layer above the individual models.
Its single registry owns registration and scheduling, orders due ticks by
capture time, skips dropped ticks, and dispatches backend-provided producers
into a typed measurement batch. Producers can close over an immutable SimKit or
SceneKit snapshot, while runtime remains independent of those backends and of
publishing or transport.

Camera, depth, and segmentation sensors share `PinholeConfig` through each
model's `projection` field; RGB-specific clear color and post-processing and
segmentation's background label remain on their respective configs.

For a stationary IMU, for example, a world-frame kinematic acceleration of
zero and gravity of `(0, 0, -9.81)` produce a measured specific force of
`(0, 0, 9.81)` before noise and post-processing.

## Build

```sh
cmake -S sensorkit -B /tmp/materia-sensorkit-build -DNK_BUILD_TESTS=ON
cmake --build /tmp/materia-sensorkit-build
ctest --test-dir /tmp/materia-sensorkit-build --output-on-failure
```

Regenerate the checked-in protocol artifacts with
`cmake --build /tmp/materia-sensorkit-build --target sensor_wire_generate`.
CTest checks that generation is current and exercises the schema validator.
When the sibling Haxeon checkout and local toolchain are available, the
`sensor_wire_haxe_vectors` target compiles the generated Haxe records and checks
them against the same HMPK vectors used by the C++ tests.
