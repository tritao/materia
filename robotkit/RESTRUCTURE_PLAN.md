# RobotKit restructuring plan

Status: R0–R6 done and merged into local main (2026-10-04); R7 (review fixes) planned.
R0–R6 ran in `robotkit-restructure`, based on local main
`712019dbc` after transmissions X9e, then synchronized with `575c06898`
(the concurrent exosuit-followon merge). Each phase is validated, committed and merged
to local main before the next phase begins. Mechanical moves remain separate commits.

The review's verdict, which this plan accepts: the core execution design is right. The problem is
scope and taxonomy. RobotKit has become "everything robotics": `robotkit/haxeon.json` makes
KinematicsKit, SimKit and VisionKit dependencies of all of it, `robotkit.world` holds miscellaneous
public types, two buffered-motion abstractions coexist, interpretation profiles sit on the
mechanical model, and process/work semantics live beside generic robot concepts. This plan fixes
boundaries and removes duplicates. It does not change the execution architecture.

## Invariants (do not change)

- `Robot` is the one boundary shared by simulated, remote, physical and replayed robots.
- `RobotModel → RobotRuntimeCompiler → RobotRuntimeBlueprint → RobotRuntime`: validate and compile
  before anything runs.
- Immutable snapshots; stable semantic ids, never backend indices; explicit clock domains.
- `RobotWorld` (logical robots) stays separate from `Simulation` (the physical universe). Do not
  merge them.
- One shared simulation universe; `robotd` owns one deployed robot and never becomes a fleet
  manager; AutomationKit stays above RobotKit.
- MotionKit produces plans; RobotKit executes them.

## Rules for every phase

- **No backward compatibility** (repo policy): one schema version per saved format, anything else
  rejected with one generic message. Bump versions when a format changes, regenerate fixtures and
  examples with the current code, and delete old code paths instead of keeping shims or
  deprecated aliases.
- **Moves and renames are mechanical commits** with no behaviour change, separate from commits that
  change behaviour, so they are easy to review and to merge other branches across.
- **Gate:** each phase's final commit passes the full suite (`tools/robotkit-restructure-suite.sh`, the maintained successor to `x7-suite.sh`), plus the
  RobotKit native tests, robotd, the device compiler tests, the humanoid tools, the worker demo and
  the robot welder checks.
- **Coordinate:** phases R3–R5 touch files that other live branches edit (robot welder, humanoid,
  mobile base, perception). Do them at a quiet point, just after a sync of local main, and record
  in this file which branches must rebase.

## R0 — Before hardware ships (cheap now, expensive later)

Completed on 2026-10-04. Generation-6 naming is frozen independently of wire
revision 12; MotionKit and RobotKit share the planner-free `trajectory_core`;
RobotRuntime takes an endpoint, with serial construction in its factory.

The maintained phase gate passed against synchronized main `575c06898`: full
workspace tests (including 4,939 RobotWorld and 9,865 MotionKit assertions), all
52 compile targets, all RobotKit suites (16 native tests, device host/MCU,
recording/wire checks, CAD bridge, MuJoCo and managed TCP scenarios), robotd,
humanoid tools, welder and completed worker demo. Both C interfaces passed the
four-platform ABI audit; all 8 MotionKit Release and 3 CAD native tests passed.
Generated runtime fixtures match the checked-in curves exactly. Source moves,
protocol documentation, endpoint changes and validation fixes are separate
commits. R0 is merged to local main before R1 begins.

- **Device protocol naming.** The sync marker is `RKD6` and the wire version is 12, which invites
  "why is RKD6 version 12?". `DEVICE_PROTOCOL.md` says no hardware has shipped and in-place changes
  are allowed until then. Pick one scheme and apply it everywhere (marker, docs, code identifiers
  such as `rkd6_endpoint`, `robotkit_device_compiler6`):
  - either a neutral marker (`RKD\0`) plus a wire revision; or
  - an explicit "generation 6, wire revision 12" in the name and docs.

  Then freeze the scheme in `DEVICE_PROTOCOL.md` for the first hardware release.
- **`trajectory_core`.** `robotkit_runtime` links `motionkit_core`, which bundles ruckig, TOPP-RA,
  Descartes, OPW and Eigen. The runtime uses only `mk_trajectory_*` and `mk_validate`
  (`trajectory.cpp`, `validation.cpp`).
  - Split those into a small native library (name it for what it is, e.g. `trajectory_core`) that
    both MotionKit and RobotKit link.
  - `robotkit_runtime`, robotd and device builds must no longer link the planners.
  - Check the dependency graph reads MotionKit → trajectory_core ← RobotKit runtime.
- **Endpoint-neutral `RobotRuntime`.** `RobotRuntime.createSerial` and `identifySerial` move behind
  a `RuntimeEndpoint` abstraction (start, submit, observe, stop), with in-memory, serial, simulation
  and virtual-device endpoints constructed by factories. The native runtime already separates
  these internally; make the Haxe API match. Adding CAN or EtherCAT later must not add another
  `RobotRuntime.create…`.

## R1 — One buffered execution abstraction, structured capabilities

Completed on 2026-10-04, after R0's main merge `7db70f2cf`. The public command
and runtime APIs now use execution plans exclusively; polynomial payloads remain
as plan segments. Capabilities carry accepted modes, execution limits/features,
timing and stream kinds, and shared validation guarantees. Adapter policies can
reduce endpoint support and are enforced when accepting plans. Recording is
version 7 and the remote envelope/handshake is version 2; previous versions are
rejected. Recording fixtures and raw TCP clients are regenerated, with a
reproducible fixture generator.

All phase gate checks passed: full workspace (4,946 RobotWorld and 9,865 MotionKit
assertions), all 52 compile targets, all RobotKit suites (16 native tests, device
host/MCU, schemas, CAD bridge, MuJoCo and every managed TCP mode), robotd,
humanoid, welder, completed worker demo, 8 MotionKit Release and 3 CAD tests,
four-platform C ABI audits and exact runtime-fixture comparison. The full runner
received SIGTERM while launching the welder after its preceding suites passed;
the unchanged welder, worker and remaining native checks passed in separate runs.
Mechanical guarantee-type movement remains a separate commit. R1 is merged to
local main before R2 begins.

- **Remove `TrajectoryChunk`.** `RobotCommand` keeps three kinds:
  - immediate control: `JointTargets`;
  - planned control: `ExecutionPlan` (`ExecutionPlanSubmission`);
  - lifecycle: `Hold`, `Resume`, `Abort`.

  Users today: `RobotCommand`, `TrajectoryChunk`, `TrajectorySegment`/`SegmentArrays` where only
  chunks use them, `RobotRecordingCodec`, `RobotRecording`, `RuntimeRobotAdapter`, `RemoteRobot`,
  `RobotRuntime`, and tests in RobotKit (`RobotWorldTests`) and MotionKit (`SessionTests`,
  `StreamTests`). Migrate them to execution plans. Bump the recording and remote protocol versions.
- **Structured `RobotCapabilities`.** Replace the six `supports…` booleans (12 files use them) with:
  - `controlModes` (position, velocity, effort, servo);
  - `execution` (plans, maximum polynomial degree, maximum joints, timed events, replacement
    boundaries, hold/resume);
  - `timing` (deadline support, clock mapping);
  - `streams` (camera, depth, lidar, …; filled by R6).

  Capabilities carry parameters and guarantees, not just presence. Where a guarantee can be
  checked, use MotionKit's `ValidationReport` vocabulary (`Proven`, `Sampled`, `Unchecked`,
  `Failed`).

## R2 — `RobotModel` is mechanical truth; interpretation is a profile

Completed on 2026-10-04 after R1's main merge `bce647683`. Mechanical role
configuration types moved in a separate commit. RobotModel schema 9 contains
mechanical truth only; RobotProfile schema 1 owns composable mobile and fork
roles. The compiler requires both records and validates stable IDs. CAD,
authoring/undo, manipulation, factories, robotd and deployment consumers use
that pair. SerialDeployment is schema 6; saved authoring records explicitly
contain canonical model and profile records. Earlier formats are rejected.

All required checks passed: workspace (4,950 RobotWorld and 9,865 MotionKit
assertions), all 52 compile targets, all RobotKit suites and managed TCP modes,
16 native tests, device host/MCU, robotd, humanoid, welder, completed worker demo,
8 MotionKit Release and 3 CAD tests, four-platform ABI audits and exact planner
fixture comparison. The additional complete app suite passed, including worker
and gallery documents. The updated MJCF importer compiled and matched the
complete current JSON fixture semantically. The consumers runner initially
found old saved worker records; after conversion its worker demo passed in a
separate run. R2 is merged to local main before R3 begins.

- `RobotModel` keeps what physically exists: links, joints (including `floatingBase`, which changes
  the rigid-body topology), actuators, transmissions, couplings, elastic networks, sensors,
  frames and collision.
- Move `mobileBase` (`RobotMobileConfiguration`) and `forkMechanism` (`RobotForkConfiguration`) to a
  typed `RobotProfile`, e.g. `DifferentialBase(left, right, …)`, `ForkMechanism(…)`,
  `Manipulator(…)`, held beside the model and compiled and validated against its stable ids.
  - Users to migrate (about 20 files): `app/src`, ProjectKit, the CadKit bridge, robotd,
    RobotKit `manipulation`/`mobile`/`runtime`, the mobile-base example and tests.
  - The blueprint compiler takes model + profile.
  - Saved robot models bump their schema; profiles get their own record.
- Rule from here on: a new robot kind (gripper, tool changer, conveyor, welder, legged base) is a
  profile, never a new `RobotModel` field.

## R3 — Process and work semantics out of RobotKit

Completed on 2026-10-04. ProcessKit now owns work geometry, process paths,
finishing/scanning/welding/earthwork skills, simulated weld tools and arc
models. Surface registration, work-patch planning and process MotionKit
lowering moved with their domain dependencies. CAD face and wall work-surface
bridges moved too, preventing a CAD bridge / MotionKit / ProcessKit cycle.
RobotKit retains generic robot, skill and tool mechanisms and the frozen
external sensor frame validation contract. Mechanical moves are separate
from boundary fixes and documentation; no old import aliases remain.

All required gate stages passed: workspace (4,950 RobotWorld and 9,865 MotionKit
assertions), all 52 compile targets, all RobotKit suites and managed TCP modes,
16 native tests, device host/MCU, robotd, humanoid, welder, completed worker demo,
8 MotionKit Release and 3 CAD tests, four-platform ABI audits and exact planner
fixture comparison. R3 is merged to local main before R4 begins.

Started from the quiet point immediately after R2 main merge `8be94384e`.
Live branches with committed changes to moved modules require rebase/import
updates: drywall-scoped-d8 (1 modules), gantries (2 modules), machine-tending (13 modules), mobile-welder (1 modules), motion-loose-ends (1 modules).
Other worktrees and their uncommitted work remain untouched.

- Move `robotkit.work` (16 files: `BucketSweep`, `DigCyclePlanner`, `EarthworkRegion`, `HeightMap`,
  `CoverageMap`, `RasterToolpathGenerator`, `WorkSurface`, …) and `robotkit.process` (`Toolpath`,
  `ToolpathPoint`) into ProcessKit (or a `workkit` beside it, if earthwork turns out to be its own
  domain).
- Move the process-specific skills as well: `Paint`, `Sand`, `FinishSurface`, `ScanSurface`,
  `RegisterSurface`, `SurfacePlanRunner`, `ToolpathPlanRunner`, `WeldPlan`, `WeldRunner`,
  `WeldSeam`, `DigTrench`, `GradeRegion`, `DumpAt`.
- Simulated process tools (`SimulatedWelder`, the arc model) move with them, behind RobotKit's
  generic tool interfaces.
- RobotKit keeps the generic mechanisms: `Tool`, `Manipulator`, `Gripper`, `Skill`, the skill
  lifecycle, execution and process events, and generic skills (`GoTo`, `FollowPath`, `Dock`,
  `Charge`, `PickPallet`, `PlacePallet`, `HandlePart`).
- ProcessKit already holds the welder's runtime pieces (`WeldingPlanRunner`, `ProcessDevice`, …),
  so this finishes an extraction that is underway.

## R4 — Split `robotkit.world`

Completed on 2026-10-04. The world namespace now contains only RobotWorld,
WorldSnapshot, RobotWorldEvent and RobotWorldSubscription. Robot contracts,
capabilities and observations live in core; execution plans and process events
in execution; recorder/replay types in recording; adapters in their endpoint
namespaces; CameraImage in streams. Imports and recording generators were
updated repository-wide, without re-export aliases. The move is a separate
mechanical commit.

All required checks passed: workspace suites (4,950 RobotWorld and 9,865 MotionKit
assertions), all 52 compile targets, all RobotKit suites and managed TCP modes,
16 native tests, device host/MCU, robotd, humanoid, welder, completed worker demo,
8 MotionKit Release and 3 CAD tests, four-platform ABI audits and exact planner
fixture comparison. The initial workspace run exposed an existing race in the
planner shutdown test: an unbounded worker could finish before its registration
was counted. Both test workers now wait on bounded lookahead; the complete
MotionKit suite passed separately after that deterministic test fix. Every other
workspace suite passed in the initial run. R4 is merged before R5 begins.

Started from the quiet point immediately after R3 main merge `d1a42c128`.
Live branches with committed changes to affected files require rebase/import
updates: drywall-scoped-d8 (12 files), gantries (10 files), machine-tending (21 files), machinekit-restructure (1 files), mobile-welder (2 files), motion-loose-ends (10 files), sketching (1 files), uikit-extract (1 files), x7-transmissions (8 files).
Other worktrees and their uncommitted work remain untouched.

`robotkit.world` has 54 files and really means "miscellaneous public types". In one mechanical
commit, move them to:

- `robotkit.core`: `Robot`, `RobotId`, `RobotDescription`, `RobotCapabilities`, `RobotSnapshot`,
  `RobotStatus`, `RobotHealthSummary`, `RobotFault`, `RobotCommand`, `JointTarget`,
  `JointTargetMode`, `StopMode`, `CoupledJoint`, `SensorFrame`, `RobotSensorFrames`,
  `ImmutableFloatArray`, `ImmutableSensorArray`, `RobotThreadToken`, `RobotEvent`, `RobotEventRing`.
- `robotkit.execution`: `ExecutionPlanSubmission`, `TrajectorySegment`, `SegmentArrays`, and the timed
  process events (`ProcessTimedEvent`, `ProcessEventValue`, `ProcessEventCodec`,
  `ProcessHoldPolicy`, `ProcessChannelDeclaration`, `FiredProcessEvent`).
- `robotkit.world`: `RobotWorld`, `WorldSnapshot`, `RobotWorldEvent`, `RobotWorldSubscription`.
- `robotkit.recording`: `RecordingRobot`, `ReplayRobot`, `RobotRecording*`, `McapRecording*`,
  `McapRobotRecording`, `RecordingChannel(s)`, `RecordingSchemas`, `CoreRecordingChannels`,
  `PerceptionRecordingChannel`.
- Adapters stay with their adapter package (R5): `RemoteRobot` (remote), `SerialRobot` (serial),
  `SimulatedRobot` (simulation), `RuntimeRobotAdapter` (runtime).
- `CameraImage` goes to perception or the streams package (R6).

Update every import across the repository in the same commit. No re-export shims.

## R5 — Package boundaries

Completed on 2026-10-04. Eight owning packages now have independent physical
source roots: core, sim, remote, serial, recording, autonomy, inference and
policy. The convenience package excludes ONNX addons. Core's exact dependency
closure is core + NativeKit + TrajectoryKit. Only simulation directly imports
SimKit; only autonomy directly declares VisionKit/KinematicsKit. Pure spatial
values and immutable image/detection stream records remain in core. Serial
adapters accept model/profile data without deployment/vision interpretation;
SerialDeployment.openRobot owns that interpretation. Optional inference
implementations register their factories explicitly in ONNX consumers.

The dependency audit passed. An independent core consumer compiled 194 sources
and created a runtime with simulation, vision, inference and policy disabled;
its runtime's only project-library dependency is trajectory_core. The full
52-target compile sweep and the additional core consumer's compile check passed
(53 executable targets total). All workspace suites passed (4,950 RobotWorld,
9,865 MotionKit assertions), including the core consumer run separately before
adding it to the workspace manifest. All RobotKit/TCP suites, 16 native tests,
device host/MCU, robotd, humanoid, welder, completed worker demo, 8 MotionKit
Release and 3 CAD tests, four-platform ABI audits and exact fixtures passed.
A native schema check initially retained the old source path; it was corrected
and the complete RobotKit stage passed again. Editing the gate script while
its workspace stage was running caused a shell read-offset error after all
workspace tests had reported OK. The saved script passes bash syntax checks;
all remaining stages passed separately from that version. R5 is merged before
R6 begins.

Started from the quiet point immediately after R4 main merge `183d90140`.
Live branches with committed changes to affected files require rebase/path
updates: drywall-scoped-d8 (8 files), machine-tending (17 files), mobile-welder (2 files), motion-loose-ends (1 files), sketching (1 files), uikit-extract (1 files), publish-facility (1 files), x7-transmissions (7 files).
Other worktrees and their uncommitted work remain untouched.

The goal is the dependency direction: someone who wants only the robot model and runtime must not
need the simulation or vision stack.

- `robotkit-core` (model, core, execution, blueprint compiler, clocks, safety contracts) depends on
  NativeKit and `trajectory_core` only.
- `robotkit-sim` holds the simulation adapter: `robotkit.runtime.Simulation`, `SimulationSpace`,
  `SimulationHarness`, `SimulationClosure`, the presentation snapshots, `DifferentialDrivePlant`,
  `HolonomicDrivePlant`, `SimulatedRobot`, `SimulatedSuctionTool`. It is the only part that depends
  on SimKit.
- Remote (client and protocol), serial/RKD, and recording become their own adapter packages.
- `robotkit-autonomy` (`mobile`, `localization`, `perception`, `navigation`, `manipulation`) is the
  only part that depends on VisionKit (today: `perception`, `deployment`) and KinematicsKit (today:
  `manipulation`, `mobile`, `kinematics`, `work`).
- Inference and policy (ONNX) become optional, as `TODO.md` already asks.
- Keep the number of packages small: create one only where a dependency must stop. Each one costs
  haxeon project setup and test wiring.

## R6 — Snapshots vs sensor streams, robotd authentication

Completed on 2026-10-04. RobotSnapshot now keeps coherent joint/runtime/safety/
execution state, small numeric measurements, and immutable stream sequence
headers. Camera, depth, point-cloud, LiDAR, and inference samples use timestamped
SensorStreams with cancellable subscriptions and retained latest samples.
Robot.sensors() is removed. Runtime, remote, serial, recording, replay, perception,
world behaviors, and application presentation use the new boundary. WorldSnapshot
copies all execution fields and stream headers. Recording retains stream events
separately without inserting sensor delivery into the control replay cadence.

RKF1 is version 3 and recording schema is version 8. Wire locks, generated
recording schemas, fixtures, raw clients, and consumers use only the current
formats. RemoteRobot authenticates separate control and bulk TCP connections;
stalled camera/inference queues cannot occupy its control socket.

Every robotd TCP host now requires a strict authorization file before opening
its listener. Hello authenticates an explicit identity/token against configured
SHA-256 digests; Welcome carries the authenticated identity and its independent
observe/command/deployment grants. Neither anonymous clients nor loopback clients
receive a bypass. Commands still require the authenticated control lease.
Deployment requests require the deployment grant, a current session and a fresh
sequence, an idle controller, and an operator-configured name. The complete
deployment is validated before the old host closes and restarts. Clients cannot
submit deployment paths. Editor and worldd clients require explicit credentials;
the CLI reads ROBOTKIT_IDENTITY and ROBOTKIT_TOKEN.

Local main advanced with the whole-weldment W4 work during this phase. The final
gate runs on the combined tree after syncing main `1245b8c99` in `a6f6addeb`.
Superseded preliminary gates were stopped and rerun after that sync. Other
worktrees and their uncommitted changes remain untouched.

The saved workspace gate completed successfully in 894 seconds.

Final validation: all 53 executable compile targets; all 22 workspace projects
(including 4,972 RobotWorld and 9,865 MotionKit assertions and the independent
core-only consumer); all RobotKit suites and authenticated TCP modes; 16 native
RobotKit tests; Rust device host/MCU checks; authenticated RKD6 virtual-device
TCP integration; the complete application suite; robotd in-memory, humanoid,
whole-weldment robot welder, and completed worker demo; 8 MotionKit Release and
3 CAD tests; four-platform ABI audits and exact trajectory fixtures. Robotd
without --auth was also verified to stop before opening its listener.

- **Snapshot vs streams.** Today there are three ways to say two things: `robot.snapshot()`,
  `robot.sensors()` (which returns the previous snapshot's frames, per `TODO.md`), and sensors
  inside `RobotSnapshot`.
  - `RobotSnapshot` becomes the small, coherent control observation: joints, runtime, safety,
    execution progress, and the latest sequence numbers of each sensor stream.
  - Sensor data moves to timestamped streams with subscription: camera, depth, point cloud,
    lidar, inference outputs. Small encoder and IMU values may stay on the snapshot.
  - Remove `Robot.sensors()`.
  - Over the network, streams use the second bulk-data connection that `TODO.md` already
    proposes.
- **robotd authentication.** robotd currently "does not authenticate clients". Add authenticated
  identities and authorization (who may observe, who may command, who may change deployment) to the
  robotd session model before any physical deployment beyond a bench on an isolated network.

## R7 — Review fixes for R0–R6

Status: planned (2026-10-04), from a read-only review of R0–R6 on local main `9162a8c83`. The
overall shape is right:
- core's dependencies are exactly NativeKit and TrajectoryKit;
- the R3–R5 mechanical commits change only packages and imports;
- no re-export shims or aliases were left;
- robotd authentication is mandatory, with no default credential.

The review also found two compile breaks, fixed in the merge `564237cd6`:
- MotionKit's `PlanCheckTests` used `robotkit.world.RobotRecording`;
- VisionKit's RobotKit test imported `ImageDetectionLifter` from `robotkit.streams`.

VisionKit's RobotKit test now runs in the gate's consumer stage. The rest is below, most urgent
first. The gate is `tools/robotkit-restructure-suite.sh` (all stages).

### R7a — robotd session security

- **One client must not be able to block control.**
  - `RobotServer` gives the controller slot to the first accepted socket before any Hello
    (`RobotServer.hx` around line 301), and `sendFault(…, fatal = true)` sends the fault but
    never closes the socket. A client with no token or a wrong token holds the slot indefinitely,
    and every real controller is demoted to observer.
  - Assign the slot only after a successful Hello.
  - Close the connection after any fatal fault.
  - Add a Hello deadline, and cap unauthenticated connections and failed attempts per peer.
  - Bound frame buffering before authentication (frames can declare up to 16 MB × 65 parts).
  - Test: a silent client, then a wrong-token client, then a valid controller gets control.
- **State the threat model.** The token is a static bearer secret sent in cleartext, with no
  challenge, nonce or expiry, and frames after Hello have no integrity. The README must say that
  robotd requires an isolated robot network or a tunnel (WireGuard/SSH) until the transport is
  authenticated. Plan TLS-PSK or HMAC challenge-response with per-session keys as the next step,
  before any deployment off an isolated network.
- **Auth file:**
  - refuse to start if it is group- or world-writable or not owned by the robotd user;
  - document random tokens of at least 128 bits;
  - salt the digests or use a KDF.

  The file also maps deployment names to paths, so it is a trust root.
- **Deployment changes:** validate a new deployment fully (open the serial device, compile the
  blueprint) before disposing the running host, and keep the old host on failure. Test a
  successful change, not only denials.
- **Remove the `legacy` subscription mode** (`OutboundScheduler`): empty subscriptions mean
  essential-only. Apply the "no bulk data on the control connection" rule to the effective
  families, not the requested ones. Update the raw test clients
  (`tests/integration/authentication.py`) that send empty subscriptions.
- Remove `RobotServer.handleLegacyJointTarget` if nothing current sends that message.
- Make the unreachable 426 "unsupported protocol" fault reachable (version check before decode),
  or remove it.
- `RemoteRobot`'s bulk stream connection: reconnect after a drop, and report a stream fault
  instead of going silently stale.

### R7b — Finish "execution plans only" and capabilities

- **Remove the native raw-segment path:** `RK_COMMAND_TRAJECTORY_SEGMENTS`,
  `supports_trajectory_queue` and `rk_robot_runtime_submit_segments` (`robotkit_runtime.h`), with
  their implementation in `runtime_c_api.cpp`, `runtime.cpp` and `validation.cpp`, and the
  `tests/c_api.c` cases. Bump `RK_API_VERSION`. Execution plans are then the only buffered input
  at every layer.
- **Delete `RecordingSchemas.legacyCommand()`,** its acceptance in `McapRecordingReader`, and its
  generator in `tools/recording/generate_schemas.py`. Bump the recording version for X9e's
  `controlAcceleration` (field 17), and bump the plan-submission version for the same field.
  The no-compatibility policy applies.
- **Capabilities that tell the truth.** Read hold/resume, timed events, replacement, timing,
  control modes and polynomial/segment limits from the endpoint, not from Haxe constants
  (`RobotRuntime.hx` around lines 110–122). A serial/RKD endpoint must not advertise the
  simulation's contract.
  - Enforce the limits where plans enter: `RemoteRobot.submit` and robotd, not only
    `RuntimeRobotAdapter`.
  - `MotionSystem`/`PlanExecutor` reject a robot without `holdResume` at construction, instead of
    `hold()` throwing after session state has changed.

### R7c — Profiles as typed roles

- **A missing profile is an error, not an empty one.** `KinematicGroup` defaults to
  `new RobotProfile()`, so `MissionPlayer`'s `Manipulator` (built without a profile) silently stops
  moving a mobile base. Pass the robot's profile everywhere a `Manipulator` or `KinematicGroup` is
  built.
- Give worldd's `WorldHost.addSimulatedRobot` a profile parameter.
- Turn `RobotProfile`'s two nullable fields into typed roles as R2 planned (`DifferentialBase`,
  `HolonomicBase`, `ForkMechanism`, `Manipulator`, …), so a new robot kind is a new role, not a new
  field and schema bump.
- `RobotProfileCodec` errors name the field and the path (e.g. inside `deployment.json`).

### R7d — Package leaks

- **MotionKit must not need ONNX.** `motionkit/robot/haxeon.json` depends on `robotkit-policy`
  for one enum (`CommandRejection`, used by `ServoSession`). Move that enum to core, or to a small
  shared package, and drop the dependency.
- **Simulation must not need autonomy.** `robotkit/sim` depends on `robotkit-autonomy` for
  `MobileBase`/`HolonomicDrive` (used by the drive plants), which pulls in KinematicsKit
  (`UnicycleEnvelope`) and VisionKit. Move the drive kinematics the plants need into core (or
  sim), so sim depends on core and SimKit only.
- **worldd must not need ONNX.** Move `WorldHost` out of the inference package into its own
  worldd package; inference stays optional.
- Remove `robotkit-autonomy`'s unused dependency on `robotkit-remote`. Declare inference's
  VisionKit dependency explicitly.
- `tools/check-robotkit-packages.py` checks every package's dependency closure, not only the
  umbrella. Assert that core, sim and motionkit-robot never reach ONNX, KinematicsKit or
  VisionKit. Packages split across two source roots (`robotkit.mobile`, `runtime`, `skill`,
  `safety`, `localization`) need the standalone compile of each package in the gate.

### R7e — Process leftovers and the endpoint API

- **Move the process tools:** `Sprayer`, `Sander`, `SurfaceTool` and their `Simulated*` versions
  go to ProcessKit. `ToolRuntime`/`ChannelToolAdapter` get a generic tool registration instead of
  hard-wired process tools.
- **Remove the copied weld validator.** Delete `RobotRuntime`'s hand-copied `tool_weld` sensor
  validator (magic indices). Sensor-kind validation is registered by the package that owns the
  kind (ProcessKit's `WeldSensor`).
- Move the process tests (weld, work, excavator, wall finishing, terrain, construction skills) from
  `robotkit/tests` to ProcessKit, so RobotKit's tests don't depend on ProcessKit.
- **`RuntimeEndpoint` as planned:**
  - a small start/submit/observe/stop surface, not the 13-method native ABI;
  - not exposed as a public field of `RobotRuntime`;
  - closing it goes through the runtime and raises `RobotRuntimeError`.
- **The blueprint once.** `RobotRuntime.create(blueprint, endpoint)` takes the blueprint twice
  without checking. The endpoint carries the blueprint it was built from, and the runtime uses
  that one. Make simulation and virtual-device endpoints real factories, and replace
  `Simulation`'s unchecked `cast runtime.endpoint:NativeRuntimeEndpoint` with a typed path.
- **TrajectoryKit's own names:**
  - C ABI `tc_*` (or similar), not `mk_*`/`MK_API_VERSION`;
  - its own C++ namespace, not `namespace motionkit`;
  - Haxe types (`ExecutionPlan`, `Trajectory`, `ValidationReport`) in the TrajectoryKit package, not
    `motionkit.trajectory`;
  - its tests in `trajectorykit/native/tests`, not compiled from MotionKit's sources (they now run
    twice).
- **One trajectory registry per process.** Check that MotionKit and RobotKit load a single
  `trajectory_core` on every platform (handles are only valid in one registry). Make the static
  and shared build modes agree.

### R7f — Protocol and documentation

- **`DEVICE_PROTOCOL.md` must state one rule before hardware ships.** Peers require the exact wire
  revision, and a post-release change is a new revision that every board must be reflashed for (or
  a new generation). Remove "negotiate a new version", which doesn't exist (`rkd6_endpoint.cpp`
  and `frame6.rs` check equality). Rename "Protocol version 8–12" in the body to "wire revision".
- **Stale paths:**
  - `robotkit/ARCHITECTURE.md` (`robotkit.process`, `robotkit.work`,
    `runtime.SimulatedWelder`);
  - `motionkit/plans/*`, `CONSTRUCTION_ROADMAP.md`, `FOLLOWUP_PLAN.md`,
    `kinematicskit/plans/KINEMATICS.md` (`robotkit/haxe/robotkit/...`);
  - `machinekit/examples/mobile-base/PLAN.md` (`RobotModel.mobileBase`);
  - the robotd README ("validates the selected record").
- Bump the `materia.scene` envelope version, since its robot records changed in R2.

Each part is committed with the full gate passing, and this file records what moved and any number
that changed.

## Later (only when needed)

- **One controller lifecycle** (`start(context)`, `update(context)`, `stop(reason)`), with `Skill`
  (task semantics and result), robotd behaviours and policy control as different uses of it,
  without merging their meanings. Do it when a third way of hosting a control loop appears, not
  before.

## End state

```
                         AutomationKit
                 missions / fleet / traffic
                              │
                    ProcessKit / domain skills
                              │
                       RobotKit autonomy
     navigation / perception / localization / manipulation / mobile
                              │
                         RobotKit core
       Robot / Model + Profile / Snapshot / Command / Execution
                     clocks / safety contracts
              ┌───────────────┼───────────────┬──────────────┐
           remote         simulation       serial/RKD      recording
              │               │               │
           robotd           SimKit        RKD device
              │
         RobotRuntime ── trajectory_core ── MotionKit (produces ExecutionPlans)
```
