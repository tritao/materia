# Perception and its streaming foundations: plan

Status: plan, not started. Branch `perception`, worktree
`/home/joao/dev/materia-perception`. Audience: the agent implementing it.

Perception output (detections, obstacles, later maps) has to be produced off the
control path. It must reach local behaviors and remote world hosts, and be
recorded and replayed deterministically. RobotKit's streaming, recording and
robot boundary can't do that safely yet. This plan fixes those foundations
first, then builds perception on top, then connects the two end to end.

| Phase | What | Depends on |
|---|---|---|
| 1 | Outbound traffic control in `robotd` (priority, drop policy) | none |
| 2 | Subscriptions, capabilities, RKF1 schema lock | 1 |
| 3 | Recording v6: MessagePack channels, no built-in kinds, LZ4 | 2 |
| 4 | In-process perception: inference module, types, detector, config | none |
| 5 | Robot event stream, then observations end to end (wire, events, recording, replay) | 1–4 |
| 6 | Clock mapping data model (no estimation) | none |
| 7 | Spike: sensorkit as the simulated camera source | 4 |

Each phase is committed separately, with tests passing at every commit. Stop
after each phase and summarize what changed, what was decided and any deviation
from this plan, so it can be reviewed before the next phase starts. Phase 3
builds on phase 2's RKF1 lock. Phases 4 and 6 don't depend on 1–3 and may be
done in any order relative to them.

## Decisions already made (do not re-litigate)

- **Placement is deployment data.** Each perception pipeline has two independent
  fields in `deployment.json`:
  - `host` says who executes it: `robotd`, later `perceptiond` or `worldd`.
  - `consumers` says who receives its observations: `local`, `worldd`.

  There is no `auto` value. Simulation and replay are input sources, not hosts.
- **Pipelines are code.** Pre-processing, model binding and post-processing are
  code. Config selects a pipeline by name and gives the model path, digest and
  tuning values.
- **Observations are their own type.** They are never stored in `RobotSnapshot`.
  They reach consumers through the robot event stream (phase 5).
- **Inference never runs on the owner thread or the `robotd` main loop.**
  Overload drops the oldest work (latest-frame-wins), and the submitter never
  blocks.
- **One ONNX Runtime.** Use the pinned prebuilt release via
  `robotkit/policy/cmake/OnnxRuntime.cmake`. Don't build it from source or add a
  second copy.
- **Transport.**
  - TCP (RKF1) stays for control and streams. Don't adopt ENet.
  - Priority comes from a scheduler in `robotd` (phase 1), not from a new
    protocol.
  - A second bulk connection, UDP, QUIC and HTTP are all out of scope.
- **Recording format.**
  - Recordings stay MCAP.
  - Payloads become MessagePack using the same haxeon `@:wire` classes as RKF1.
    JSON payloads and base64 images go away.
  - Every recorded type is a registered channel. There are no built-in kinds.
  - There is no backward compatibility with v5 recordings: drop the v5 reader
    and writer paths.
  - Replay order stays file order plus ordinal, never source clocks.
- **Clocks are never synchronized.** Source and received times stay separate,
  and a cross-clock mapping always carries an error bound.
- **Minimal protocol.** RKF1 stays point-to-point. Subscriptions are a filter,
  not a general pub/sub system. The robot event stream is narrow, not a message
  bus.

## Verified facts about the current code

Line numbers are approximate. Re-read before editing.

- **`robotd`.**
  - `RobotServer.run` (`robotd/src/RobotServer.hx` ~117) is a single-threaded
    Haxe loop. Each pass it runs `publishSnapshot`, which also calls
    `applyBehavior` for local behaviors, then pumps transport events and checks
    the control lease.
  - The native `RobotRuntime` has its own owner thread.
- **The transport queues sends.** `NativeTransport.send` →
  `nk_transport_send` appends to one FIFO per connection. A writer thread drains
  it, and the queue is capped at 4 MiB (set in `transport/NativeTransport.hx`
  ~27).
  - A full queue returns `NK_ERROR_QUEUE_FULL`
    (`nativekit/src/transport/transport.cpp` ~1791-1830).
  - The public API (`nativekit/include/nativekit_transport.h`) can't report how
    many bytes are queued, and the FIFO has no priority.
- **A full queue closes the connection.** `RobotServer.sendTo` (~676) closes the
  connection on any send failure.
  - For the controller: `closeClient` → `releaseRemoteOwner` (~550) →
    `runtime.submitStop64(..., true)`, which issues an emergency stop.
- **Images are re-sent on every snapshot.** `sendStateTo` (~624) sends
  `RobotState` and then every sensor frame, including full camera images, to the
  controller and every observer on every new snapshot sequence, whether or not
  the sensor produced a new frame.
  - One 640×480 rgb8 image is about 0.9 MB, so a few queued images can
    emergency-stop the robot.
  - Faults go only to the controller.
- **`Hello` and `Welcome`.** `Hello` (`protocol/Hello.hx`) has `@:id` 1-4:
  protocolVersion, clientName, schemaFingerprint, requestedRole. `Welcome` has
  `@:id` 1-7 (session, robot, lease). RKF1 has no schema lock. RKD6 does
  (`schema/device_wire6.wire.idl`, `.lock.json`, `wire6.json`,
  `python -m tools.wire check`).
- **The `Robot` interface** (`haxe/robotkit/world/Robot.hx`) is snapshot-only:
  `snapshot()`, `sensors()`, `fault()`, `submit()`, `stop()`,
  `setChangeListener()`, and so on.
  - `WorldBehaviorRunner` (40 lines) deduplicates snapshots and runs behaviors.
  - Implementers include `SimulatedRobot`, `RemoteRobot`, `ReplayRobot` and
    `RecordingRobot`. Find them all with grep.
- **Recording.**
  - Event kinds are a closed enum: `world/RobotRecordingEvent.hx` has 7 kinds.
    Both `RobotRecordingCodec.VERSION` and `RobotRecordingEntry.VERSION` are 5.
  - The native writer (`runtime/src/recording.cpp`) hard-codes the kinds:
    `Names[]`, `SchemaV1-5`, `validKind`, loops over kinds 1..7, and
    `enqueue` requires version 5. It has one channel per kind (`robotkit/<name>`,
    schema `robotkit.<name>.v5`) and a bounded byte queue that counts drops.
  - The C ABI is in `runtime/include/robotkit_runtime.h` (~114-134). The Haxe
    bindings in `runtime/bindings/robotkit-runtime.hxi` are regenerated by
    `runtime/tools/check-hxi.sh`.
  - Readers and writers: `McapRobotRecording.hx`, `McapRecordingReader.hx` and
    `ReplayRobot.hx`, which builds its timeline around lines 79-105.
- **Camera frames.**
  - `world/SensorFrame.hx` has string ids, Int64 sequence, source/received ns
    timestamps and clock ids, and an optional `CameraImage`.
  - `world/CameraImage.hx` supports `rgb8`, `depth32f` and `jpeg`.
  - On the wire they travel as `CameraFrame` (type 17), with pixels as an
    attachment.
- **Existing perception package** (`haxe/robotkit/perception/`):
  - `Perception` is synchronous: `observe(Array<SensorFrame>):PerceptionSnapshot`.
  - `Detection` is planar: a `Pose2` in a frame plus timestamps. It has no pixel
    box and no model provenance.
  - `FrameAwarePerception` maps values into the localization frame.
- **`worldd`** is a headless `RobotWorld` plus `SimulationHarness`
  (`haxe/robotkit/worldd/WorldHost.hx`, 55 lines). It has no server. Remote
  robots are `RemoteRobot` clients of a `robotd`.
- **Deployment config.** `haxe/robotkit/deployment/SerialDeployment.hx` accepts
  `schemaVersion` 3 and 4 and silently ignores unknown top-level keys. The device
  fingerprint covers only the layout bytes and the RKD6 lock. Fixtures are in
  `tests/fixtures/device-deployment/`.
- **The `policy` module is the template for a native module.** Its layout:
  - `robotkit/policy/{include,src,bindings,tests,tools/check-hxi.sh}`.
  - `option(RK_BUILD_POLICY)` in `robotkit/CMakeLists.txt`.
  - FFI entries in `robotkit/haxeon.json`.
  - Packaging in `robotd/native/CMakeLists.txt`.

  Its ABI (float32, static shapes, 1024 values at most) can't run image models.
  Leave it unchanged.
- **Sensorkit** is standalone C++. It uses double seconds, uint64 ids and RGBA8.
  - Simulated cameras: `SceneCameraAdapter` in `sensor_render`, which uses
    SceneKit's GPU renderer.
  - Its wire format `PackedFrameMessage` (`schema/sensor_wire.wire.idl`)
    already has generated Haxe records in `sensor_io/haxe/materia/sensor/wire/`.
- **Time and frame conventions** are in `robotkit/ARCHITECTURE.md`:
  - Time (~82-115): keep source and received separate, clock ids on everything.
    A missing receive time is 0 and is never copied from the source time.
  - Frames (~71-80): right-handed, +Z up, +X forward, `A_T_B`.
  - Deployment boundary (~708).
  - The existing MCU clock-sync bound is `clock_sync_bound_ns` in
    `deployment.json`.

## Phase 1: outbound traffic control in robotd

Goal:
- Bulk traffic can never delay or close essential traffic.
- Every frame is sent under a message-family drop policy.
- A sensor frame is sent once, not once per snapshot.

1. **Message families and policies.** Put the classification in one place:
   - *essential*: `Welcome`, `RobotState`, `Fault`, control replies and acks.
     Never dropped. If an essential send fails, the connection closes as it does
     today.
   - *sensor* (numeric `SensorFrame`) and *camera* (`CameraFrame`): latest-wins,
     keyed by `sensorId`.
   - Later families (phase 5 observations) register their own policy in the
     same table.
2. **Nativekit queue query.** Add a function to nativekit that reports a
   connection's queued send bytes, for example
   `nk_transport_get_send_queue(handle, &queued_bytes, &capacity)`.
   - Follow the header's existing conventions, then regenerate the Haxe
     bindings the way nativekit does it.
   - Include a native test.
3. **Per-connection scheduler in `robotd`.**
   - Essential frames go straight to the transport.
   - Bulk frames go into per-connection, per-key latest-wins slots held in
     `robotd`. They are flushed only while queued bytes are below a budget, which
     keeps headroom for essential frames. Start with half the capacity and make
     it configurable.
   - Replacing an unsent slot counts a drop.
   - Flushing happens in the main loop after essential sends, round-robin across
     keys so one camera can't starve another.
   - The loop only enqueues, since the transport's writer thread already does
     the I/O. So "writes off the owner loop" is already true. Don't add threads.
4. **Send each frame once.** Track the last sent sequence per connection and per
   `sensorId`, and put a frame in a slot only when its sequence advances. Reset
   the tracking when a session changes.
5. **Drop counters.** Keep them per connection and per family, readable by
   tests, and log them at a bounded rate.
6. **Tests** (the existing integration harness in `tests/integration/`,
   `tests/world-tcp.sh`, and `RobotHost --camera-fixture` with a large image):
   - A controller that stops reading while large camera frames flow keeps
     control, and no emergency stop happens.
   - Faults and state still arrive once reading resumes.
   - Each sensor sequence is sent at most once per connection.
   - Round-robin fairness across two cameras.
7. **Docs.** Update the "Deployment boundary" section of ARCHITECTURE.md.

## Phase 2: subscriptions, capabilities, RKF1 schema lock

Goal: clients choose what they receive and at what rate, the server says what it
offers, and published RKF1 message shapes can't change silently.

1. **`Hello` subscriptions.** Add `@:id(5) subscriptions`, a list of
   `{family:String, maxRateHz:Float}`, where 0 means unlimited.
   - A missing or empty field means legacy behavior: every family at the
     snapshot rate. Old clients keep working, and phase 1 already makes that
     safe.
   - Check how haxeon wire decodes a missing `@:id` field before relying on it.
   - Update `RobotClient` to send subscriptions. Controllers created by current
     callers should subscribe to essential and numeric sensor families only,
     plus camera where the caller asks for it.
2. **`Welcome` capabilities.** Add `@:id(8) capabilities`, the families this
   server can emit. Clients must not assume a family that isn't listed.
3. **Enforcement.**
   - The scheduler skips families a connection didn't subscribe to.
   - It applies `maxRateHz` per key, using the robotd monotonic clock.
   - Essential families are always delivered whatever the subscription says.
4. **RKF1 schema lock.** Produce a lock file covering every `@:wire` class used
   by RKF1 (`protocol/*.hx`): the class name and each field's id, name and type,
   plus the `RobotMessageType` values. Add a check that fails when a published
   field is changed or removed. Additions are allowed only with new ids.
   - First check whether the existing `tools/wire` tooling (used for RKD6 and
     sensorkit) or haxeon's wire support can describe these classes. Prefer
     extending that tooling to writing a new one.
   - Register the check as a test.
5. **Tests.**
   - A legacy `Hello` receives everything.
   - A subscribed client receives only its families.
   - Rate limiting works.
   - Capabilities are reported.
   - Changing a field id fails the lock check.

## Phase 3: recording v6 (MessagePack channels, no built-in kinds, LZ4)

Goal: every recorded type is an ordinary registered channel whose payload is
the MessagePack encoding of a `@:wire` class.
- The wire and the recording share one schema definition, protected by the
  RKF1 lock from phase 2.
- Blobs are raw bytes.
- Chunks can be compressed.
- There is no v5 compatibility.

1. **Recorded types become `@:wire` classes.** Each of today's seven kinds
   (command, snapshot, sensor, fault, world, world_event, process_event) gets a
   `@:wire` class.
   - Reuse existing RKF1 classes where one already matches (for example
     `RobotStateMsg`, `SensorFrameMsg`, `CameraFrame`).
   - Add recording-specific classes only where none exists.
   - Camera pixels are a `Bytes` field, not base64, not an attachment reference.
   - Add every new class to the RKF1 lock (phase 2), so wire and recording
     compatibility are checked by the same tool.
2. **Native writer (`runtime/src/recording.cpp`, `robotkit_runtime.h`).**
   - Remove the kind enum (`RK_RECORDING_*` kinds), `Names[]`, `SchemaV1-5`,
     `validKind`, the per-kind loops and the version-5 checks.
   - Add
     `rk_recording_writer_register_channel(writer, topic, schema_name,
     schema_encoding, schema_data, schema_data_len, message_encoding,
     &channel_id)`.
   - Add `rk_recording_writer_enqueue(writer, channel_id, ordinal,
     recording_timestamp_ns, payload, payload_len)`. The writer treats the
     payload as opaque bytes.
   - Channel conventions:
     - topic `robotkit/<name>`;
     - message encoding `msgpack`;
     - schema encoding `robotkit-wire`;
     - schema data is that class's entry from the RKF1 lock (name, field ids,
       names and types), so a recording describes its own payloads.
   - Keep the file-level `robotkit.schema_version` metadata, bumped to 6.
   - Before relying on custom encoding strings, confirm in the pinned MCAP
     source that MCAP accepts them.
   - Keep log time as the wall-clock recording timestamp and publish time as
     the ordinal, as today.
   - Keep the bounded byte queue, the drop accounting and the terminal-status
     record.
   - Regenerate `runtime/bindings/robotkit-runtime.hxi`.
3. **Compression.**
   - `recording.cpp` currently compiles compression out
     (`MCAP_COMPRESSION_NO_LZ4`, `MCAP_COMPRESSION_NO_ZSTD`).
   - Add LZ4 as a pinned dependency, following the style of the MCAP
     `FetchContent` in `robotkit/CMakeLists.txt`: prefer a small source build
     of lz4, with an override for a system copy.
   - Expose the chunk compression choice (`none`, `lz4`) in the writer options,
     with `lz4` as the default. zstd is out of scope.
   - Measure write CPU and file size on a camera-heavy recording and report the
     numbers in the phase summary.
4. **Haxe side.**
   - A `RecordingChannel<T>` interface: channel name, the `@:wire` class, and
     encode/decode through haxeon's MessagePack.
   - A `RecordingChannels` registry that registers the seven core channels at
     startup the same way any other channel registers. No special cases.
   - `RobotRecordingEvent` becomes a generic `(robotId, channel, payload)`
     record, or keeps typed cases built on the registry. Choose one and record
     the reason; the goal is that adding a channel touches only the new channel
     and its registration.
   - Update `McapRobotRecording`, `McapRecordingReader`, `RobotRecording`,
     `RecordingRobot` and `ReplayRobot`.
   - The reader rejects files whose schema version isn't 6, with a clear
     message.
   - Unknown channels are skipped and counted, or cause an error in strict mode.
5. **Remove v5.**
   - Delete the JSON codec paths (`RobotRecordingCodec` JSON encoding,
     base64 image handling) and any v5 fixtures.
   - Update docs and tests that reference v5.
   - Keep ordinal-based replay order unchanged.
6. **Tools** (put them under `robotkit/tools/`, and check whether a recording
   tools location already exists first):
   - `dump`: prints any v6 recording as JSON lines, using the embedded schemas.
     This is the debugging path, replacing human-readable payloads.
   - `foxglove-export`: writes a copy of a recording that Foxglove Studio can
     open. Camera frames go to Foxglove's well-known raw image schema, and
     other channels become JSON. It can be Python using the `mcap` package that
     `tests/mcap-independent.sh` already uses. Detections are added in phase 5.
7. **Tests.**
   - A round trip for every core channel, including a camera frame with a large
     image, both uncompressed and with LZ4.
   - Registering and round-tripping a test channel without touching any core
     file.
   - Skipping unknown channels, and rejecting non-v6 files.
   - Counting queue-full drops.
   - `tests/mcap-independent.sh` updated to decode MessagePack payloads with
     Python and check them against the embedded schemas.
   - `dump` output on a fixture recording.
8. **Docs.** Rewrite the recording part of ARCHITECTURE.md (around the
   "Persistent recording is an adapter below the world boundary" paragraphs)
   for v6.

## Phase 4: in-process perception

Goal: camera `SensorFrame`s become timestamped, provenance-carrying image
detections through ONNX Runtime, off the owner thread, configured from
`deployment.json`.

### 4a. Native inference module (`robotkit/inference/`)

Copy the `policy` layout: `RK_BUILD_INFERENCE`, `RobotKit::inference`, linked to
`onnxruntime::onnxruntime` through `OnnxRuntime.cmake`. The Haxe package is
`robotkit.inference`.

C ABI (`robotkit_inference.h`), with an opaque `rk_inference_session` and
`struct_size` fields:

- **Creation.** `create(path, options)` takes the intra-op thread count and uses
  the CPU provider only. A dynamic batch dimension becomes 1, and a dynamic H/W
  is an error unless the options supply it.
- **Tensor queries.** Report input and output names, element types and shapes.
  Support float32, plus uint8 input only if ORT supports it without extra code.
- **Synchronous run.** `run` uses caller-owned buffers, checks sizes, and
  returns `RK_ERROR_*` codes consistent with `policy`.
- **Model digest.** A helper computes the model file's SHA-256.
- **Async path.** One worker thread with a latest-wins input slot and a result
  slot, modelled on `rk_recording_writer_*`. `submit` never waits, and
  `poll_result` returns the newest result and the number of inputs dropped since
  the last poll.
- **Native image pre-processing.** An async submit variant takes a raw `rgb8`
  image (width, height, stride, bytes) plus pre-processing options: target
  size, letterbox padding value, scale/mean/std normalization, and NCHW or NHWC
  layout.
  - The worker does letterbox-resize (bilinear), normalization and layout
    conversion before running the model. None of this per-pixel work happens in
    Haxe or on the `robotd` main loop.
  - The result carries the letterbox transform (scale and padding) so boxes can
    be mapped back to source pixels.
  - Don't add OpenCV. This is a small, self-contained C++ routine.

Before relying on ORT behavior (thread safety of `Run`, dynamic shapes, uint8),
read the pinned release's headers under `ROBOTKIT_ONNXRUNTIME_DIR`.

Tests:
- shapes and a known output;
- the error paths;
- `submit` returns promptly during a slow run;
- drop counting;
- pre-processing: letterbox geometry for wide, tall and exact-fit images;
  normalization values; NCHW and NHWC layout; a stride larger than the row
  width.

Generate a tiny YOLO-style fixture model, with a known box and score for a known
input, using a script next to
`robotkit/tools/humanoid/policies/make_test_fixtures.py`. Commit both the script
and the `.onnx`.

### 4b. Observation types (`haxe/robotkit/perception/`)

Leave `Detection` unchanged, since a pixel box has no pose until it's lifted.
Add:

- **`ImageDetection`** (immutable): label, score in [0, 1], and an axis-aligned
  box in source-image pixels.
- **`ImageDetectionObservation`** (immutable):
  - **Provenance:** `producerId` (for example `robotd/front_objects`),
    `pipelineId`, `sensorId`, `modelId`, `modelDigest`.
  - **Frame:** `sourceFrameId`. Boxes are in that image's pixels.
  - **Time:** the input's `sequence`, `sourceTimestampNs`,
    `receivedTimestampNs`, `sourceClockId` and `receivedClockId`, plus
    `completedTimestampNs` and its clock id. Never substitute one timestamp for
    another.
  - **Payload:** `detections`, and `droppedFrames` since this producer's
    previous observation.
- **`PerceptionPipeline`**, an interface (haxeon has no class-to-anonymous
  structural subtyping).
- **`PerceptionHost`**: `submit(frame:SensorFrame)` routes by `sensorId` and
  never blocks, and `poll():Array<ImageDetectionObservation>` returns finished
  observations.

Validate constructor arguments like the existing perception types do.

### 4c. Detector pipeline (`ObjectDetectorPipeline`)

- Accept `rgb8` only, and reject other encodings clearly.
- Submit the raw `rgb8` image to the async session with pre-processing options
  (the fixture is NCHW float32). Letterboxing and normalization happen natively
  (4a).
- Decode boxes, apply the threshold and NMS in Haxe, and map boxes back to
  source pixels using the letterbox transform returned with the result.
- Tests: letterbox round-trip, NMS, threshold, an empty result, and rejection of
  non-`rgb8` input.

### 4d. Deployment config

Add an optional, strict `perception` array to `SerialDeployment` (unknown keys
inside the section are rejected):

```json
"perception": [
  {
    "id": "front_objects",
    "input": "front_camera",
    "pipeline": "object_detector",
    "model": "models/detector.onnx",
    "modelSha256": "…",
    "host": "robotd",
    "consumers": ["local", "worldd"],
    "options": { "scoreThreshold": 0.4, "maxRateHz": 15 }
  }
]
```

- **Schema version.** Bump `schemaVersion` to 5 and keep accepting 3 and 4.
  Without the bump, an older binary would silently ignore the section.
- **References.** `input` must be a camera sensor in the model, and its frame
  must exist. `pipeline` must be in a code registry. `id`s must be unique. The
  model digest is checked at load.
- **Host and consumers.** Reject `host: worldd` combined with a `local`
  consumer. The error must explain the network round trip.
- **Fingerprint.** Keep the section out of the device fingerprint.
- **Fixtures.** Put valid and invalid fixtures in
  `tests/fixtures/device-deployment/`.

### 4e. Replay parity

Feed the same recorded `SensorFrame`s to `PerceptionHost` twice: once through
`ReplayRobot` and once directly. The observations must be identical except for
`completedTimestampNs`.

## Phase 5: robot event stream and observations end to end

Goal: an observation produced in `robotd` reaches local behaviors, a remote
`RobotWorld` and a recording, and replays in the same order.

1. **Robot event stream.** Add a narrow, bounded, typed stream to the `Robot`
   boundary:
   - `RobotEvent` enum, starting with `Observation(ImageDetectionObservation)`
     and `Overflow(count)`. Faults and world events are future cases; don't add
     them now.
   - Each event carries a per-robot ordinal that increases monotonically. It is
     assigned where the event enters the robot boundary and restored from the
     recording during replay, so order is deterministic and never depends on
     arrival time.
   - `events(afterOrdinal:Int64, max:Int):Array<RobotEvent>` reads from a
     bounded ring. A consumer that falls behind gets one `Overflow` marker.
   - Decide between two designs and record the reason:
     - add `events` to `Robot` itself and implement it in every implementer;
     - add a separate `RobotEventSource` interface.

     Prefer the first unless haxeon makes it impractical.
   - Deliver events to behaviors: extend `WorldBehaviorRunner` and
     `WorldBehaviorContext` so a behavior sees the events that arrived since its
     last update, alongside the snapshot.
2. **Wire message.** Add `ImageDetectionObservationMsg` (`@:wire`,
   `RobotMessageType` 19) in the observation family, with a latest-wins policy
   per producer (phase 1).
   - It must be subscribable (phase 2) and advertised in `Welcome`.
   - Add it to the RKF1 lock.
   - `RobotClient` gets a listener, and `RemoteRobot` feeds the event stream.
3. **Hosting in `robotd`.**
   - Parse the deployment `perception` section in `RobotHost`, and build a
     `PerceptionHost` for pipelines with `host: robotd`.
   - The main loop submits new camera frames and polls observations. Neither
     call blocks.
   - A `local` consumer delivers observations into the local robot's event
     stream. A `worldd` consumer emits them on the wire.
   - Pipelines with `host: worldd` run in a `PerceptionHost` inside
     `WorldHost`, fed from `RemoteRobot` camera frames. The world side must
     subscribe to those frames.
4. **Recording.** Register a `perception.image_detections` channel (phase 3),
   reusing `ImageDetectionObservationMsg`. `RecordingRobot` records events, and
   `ReplayRobot` replays them with their original ordinals. Extend
   `foxglove-export` to write detections as Foxglove image annotations.
5. **Tests.**
   - End to end over TCP: a camera fixture, then `robotd` perception with the
     fixture model, then a `RemoteRobot` in a `WorldHost`, then a behavior that
     sees the observation.
   - Record and replay yield the same event sequence.
   - A stalled observer doesn't affect the controller.
   - An `Overflow` marker is emitted when the ring is exceeded.

## Phase 6: clock mapping data model

Goal: a single, explicit representation for relating two clocks, ready for
fusion and deadlines. It has no estimator.

1. **`ClockMapping` (immutable).** `fromClockId`, `toClockId`, `offsetNs`,
   `skewPpb`, `errorBoundNs`, `validFromNs` in the `to` clock, and `source`,
   which describes how the mapping was obtained.
2. **`ClockMappings` registry.** `map(timestampNs, from, to)` returns a
   `MappedTimestamp {valueNs, errorBoundNs}`, or null when no mapping (or chain
   of mappings) exists.
   - Error bounds add up along a chain.
   - The identity mapping has a bound of 0.
   - A mapping is never guessed.
3. **Align with the MCU link.** Read how `clock_sync_bound_ns` and the RKD6 clock
   sync are computed today. Express that link as a `ClockMapping` if it fits,
   and match its naming and units.
4. **Nothing uses it yet.** In particular, absolute command deadlines stay
   rejected. Document it in ARCHITECTURE.md's time section as the only
   sanctioned way to compare timestamps across clocks.
5. **Tests.** Identity, a chain, a missing path, bound accumulation, and skew
   applied over an interval.

## Phase 7: spike on sensorkit as the simulated camera source

This is a spike. The deliverable is a decision note and a prototype, not a
production path.

1. **Adapter.** Decode sensorkit's `PackedFrameMessage` (camera) using its
   generated Haxe records into a RobotKit `SensorFrame` and `CameraImage`. This
   is the one conversion point:
   - capture time → `sourceTimestampNs` on a `sensorkit.sim` clock id;
   - delivery time → received time only on the simulation clock, never mixed
     with robotd's monotonic clock;
   - uint64 sensor ids → string ids through the robot model's sensor list;
   - RGBA8 → either strip alpha to `rgb8` or add an `rgba8` encoding to
     `CameraImage`. Measure the cost and choose.
2. **Prototype.** Render a trivial scene with `SceneCameraAdapter`, convert the
   frame, and run it through the phase 4 detector pipeline in a test or tool.
3. **Measure.** Build cost (the SceneKit and GPU dependencies), whether it runs
   headless in CI, and the frame time.
4. **Decision note.** Write `robotkit/plans/SENSORKIT_CAMERA.md` with a go or
   no-go on making sensorkit RobotKit's sensor simulator, the constraints found,
   and what a production version would need.

## Out of scope (add to `robotkit/TODO.md`, not the root `TODO.md`)

- A second TCP connection for bulk, and UDP, QUIC or HTTP (model distribution
  over `GET /models/<sha256>`, health, recording download).
- Clock offset estimation between processes.
- World fusion, tracking and shared maps.
- `perceptiond` as a separate process.
- GPU execution providers and other inference backends.
- OpenCV. Reconsider it when a real need appears: camera capture (V4L2),
  calibration and undistortion, or heavier image processing. Use the
  already-vendored `stb_image` for JPEG decode.
- `jpeg` and `depth32f` inference inputs.
- Lifting `ImageDetection` into planar `Detection` (depth or intrinsics) through
  a `Perception` adapter for `FrameAwarePerception`.
- Segmentation, pose and occupancy observations.
- Fault and world-event cases in the robot event stream.
- A production sensorkit integration (it depends on the phase 7 decision).

## Working rules

- **Stay in this worktree.** The main checkout at `/home/joao/dev/materia` is
  shared by other sessions, so never switch branches or commit there.
- **Submodules.**
  - These are clones of the main checkout's submodules at the pinned commits:
    - `haxeon` (with `.tools/` and `out/haxeon_runtime.hdll` copied in);
    - `nativekit`;
    - `simkit/vendor/mujoco`.
  - Other submodules (uikit, motionkit and animkit vendors) aren't initialized.
    If a build needs one, clone it the same way from
    `/home/joao/dev/materia/<path>` and check out the pinned sha.
    `git submodule update` may fail, because pins can be local unpushed commits.
- **Changing a submodule.** Phase 1 changes nativekit, and phase 7 may touch
  sensorkit. Commit inside the worktree's submodule clone on a branch named
  `perception`, then bump the pin in the superproject. Report the submodule
  commits in the phase summary, so they can be brought into the main checkout's
  submodule repos at merge time.
- **OCCT and CadKit.** Don't rebuild OCCT or CadKit. If a build needs them,
  point it at the existing builds
  (`CADKIT_BUILD_ROOT=/home/joao/dev/materia/cadkit/build/debug`,
  `CADKIT_OCCT_DIR`).
- **haxeon gaps.** If haxeon lacks a feature or stdlib member, fix haxeon
  instead of working around it in kit code. Known quirks: no class-to-anonymous
  structural subtyping (use an interface), a reduced `Math`, and no
  static-function imports.
- **Diagnose first.** When a compiler or tool error is unclear, read its source
  and confirm the cause before working around it.
- **Verify library behavior.** Check third-party behavior (ONNX Runtime,
  MCAP, haxeon wire) in the pinned source or headers before relying on it.
- **Docs.** Keep `robotkit/ARCHITECTURE.md` current in each phase that changes
  a boundary: deployment boundary, recording, time, and a new "Perception"
  section.

## Done when

- **Phase 1.** Under heavy camera traffic, a stalled controller keeps control
  with no emergency stop. Frames are sent once per sensor sequence. Drops are
  counted. The nativekit queue query is tested.
- **Phase 2.** Legacy and subscribing clients both work. Rate limits and
  capabilities are tested. The RKF1 lock check runs in the test suite.
- **Phase 3.**
  - Every recorded type is a MessagePack channel, with no kind enum or v5 code
    left.
  - A new channel needs no core edits.
  - LZ4 round-trips, with size and CPU numbers reported.
  - `dump` and `foxglove-export` work on a fixture.
  - The independent Python check decodes the payloads.
- **Phase 4.** `RK_BUILD_INFERENCE=ON` builds on linux-x64, and ctest and the
  `.hxi` check pass. The detector, host, deployment and replay parity tests
  pass.
- **Phase 5.** The end-to-end TCP test passes. Record and replay reproduce the
  event order. Overflow is signalled.
- **Phase 6.** The mapping tests pass and ARCHITECTURE.md documents the rule.
- **Phase 7.** The decision note exists, with measurements.
- **Every phase.** `robotkit/tests/run-all.sh` passes, and `robotkit/TODO.md`
  lists the out-of-scope items.
