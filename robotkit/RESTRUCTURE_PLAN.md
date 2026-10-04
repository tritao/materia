# RobotKit restructuring plan

Status: in progress (2026-10-04), in `robotkit-restructure`, based on local main
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
