# Automatic tool change

One arm exchanges end effectors at a dock in the middle of a mission: for
example a gripper and a screwdriver, a welding torch and a probe, or two
machine-tending grippers. This started as step SD15 of the robot screwdriver
(`machinekit/examples/robot-screwdriver/PLAN.md`). It moved here because the
same machinery serves machine tending, the welder and any multi-tool cell.

What a tool change has to be:
- **Physically real.** The tool is a CAD subassembly with its own joints (jaws,
  feed slide, spindle) and its own mass and collision geometry. It is attached
  to the flange or parked in its dock. It is not a static configuration picked
  at build time, and not a kinematic object trailing the flange by one tick.
- **Measured.** The cell knows which tool is mounted, and whether it is locked,
  only from fresh, clocked lock and identity feedback, never from what was
  commanded.
- **Owned.** A tool's process outputs (spindle, vacuum, torch) belong to one
  process lease. An exchange cannot start while the outgoing tool is still
  live.
- **Recoverable.** Every interruption leaves a named physical state: which
  tool is where, and locked or not. Resume continues from it rather than
  replaying.

## What already exists (SD15 drafts, 2026-10-13/14)

Compiled and tested in the RobotKit tool-capability suite (660 assertions) and
CadBridge (194):
- `ToolRuntimeSelection`:
  - routes outputs and sensor frames to the selected end effector;
  - an exclusive process lease;
  - outputs initialised safe on selection;
  - a lease cannot be released with unsafe outputs;
  - selection requires the shared lock and fresh feedback for the requested
    identity.
- `ChangerLockObservation`: measured mounted-tool identity, lock state, source
  clock and acquisition time.
- `ToolChangeTransaction`:
  - the outgoing tool is made safe and deselected before unlock;
  - it then advances through measured unlocked, empty, attached and locked
    observations;
  - lock commands can be retried;
  - stale or backward feedback is rejected, keeping the retryable phase.
- The CadBridge shared-lock factory for an `EndEffectorSet`, with a measured
  short-to-long exchange that changes the selected TCP.
- `AssemblySimulationBridge.subtree` / `toFloatingRobotModel`: split a CAD tool
  subtree off the robot, rebase it at its root and compile it as a floating
  articulated runtime that keeps its internal joints. Not compiled yet.

Uncompiled native drafts (RobotKit/SimKit, both backends):
- A removable fixed joint from one robot runtime's link to a second runtime's
  floating root. It captures the measured relative pose (no snap) and rejects
  duplicates and cycles.
- A fixed follower from any session body, such as a static dock, to a floating
  root.
- Regressions for attach, follow, internal joint actuation and release. In the
  last run, the RobotKit MuJoCo test failed at static-dock reattachment.

Mission data, SceneArtifact v45 at the time:
- A `toolChanger` record: master and dock connectors, removable CAD
  occurrences, work channels, unique nonzero identity codes (inputs least
  significant bit first; zero means empty), lock command, measured lock and
  identity inputs, initial tool, and maximum feedback age.
- A `changeTool` step.
- Validation across handling steps.

## Steps

**TC0. Land the native joints.** Build and run the robot-to-robot and
dock-to-robot followers on SimKit and MuJoCo:
- Fix the static-dock reattachment failure, diagnosing it first. Done
  2026-10-14 during SD12, which uses the fixed object follower. There were
  three causes:
  - The MuJoCo backend rebased only a moved root's rest frame when a joint
    attached it, so a floating tool's own links no longer met their joints at
    rest. The whole subtree now moves rigidly.
  - The tests stepped manually after `start()`, but realtime refuses manual
    steps.
  - The tests also released after manual steps without stopping the host.

  RobotKit SimKit and MuJoCo native suites pass.
- Rerun the generated FFI audit.
- Checks:
  - no-snap attach;
  - following under arm motion;
  - the tool's internal joints still actuate;
  - its mass loads the arm;
  - release keeps the pose;
  - topology changes only at a stopped or post-tick boundary.

**TC1. Tools as their own runtimes.** `AssemblyRobot` compiles the arm without
the changer tools and compiles each tool subtree as a floating articulated
runtime. The tool's own sensors and joints (jaws, feed, spindle, force/torque)
bind to its runtime. At startup the initial tool is attached at the flange and
the others are held in their docks.
- Checks: the gripper and screwdriver behave as today when mounted, and a docked
  tool stays put.

**TC2. Lock and identity I/O.**
- The simulated changer reports lock state and identity code from the physical
  joint state: locked means the follower joint exists and the lock output is
  on.
- It publishes those as clocked cell inputs, and a hardware cell reads the same
  inputs.
- `ToolChangeTransaction` consumes only these.
- Checks: stale, missing, wrong-identity and conflicting feedback each stop
  the exchange with a named reason.

**TC3. Dock motion.**
- Dock approach, insert, retract and fetch paths are planned from the CAD dock
  and master connectors, with the tool's collision geometry following whichever
  body carries it.
- The exchange runs only at a mission boundary where no process lease is held.
- Checks: collision-free dock motion with each tool mounted, with none, and
  with the dock occupied.

**TC4. Live TCP, collision and capabilities.**
- On a measured lock the mission switches the active TCP and the arm's
  collision description, and moves the tool's capabilities (screwdriver,
  gripper) to the selected runtime.
- Planning in progress is invalidated rather than continued with the old tool.
- Checks: a move planned for the screwdriver's TCP is rejected after a change
  to the gripper.

**TC5. Mission step and recovery.**
- `changeTool` runs the transaction end to end.
- Interruptions are named:
  - mid-unlock;
  - the tool left in the dock but not released;
  - nothing mounted;
  - the wrong tool mounted.
- Resume continues from the measured state.
- Checks: each interruption, then a resume, then a complete exchange on both
  backends.

**TC6. Cells.** One-arm variants:
- the screwdriver cell (SD15a with gripper and screwdriver);
- machine tending with two grippers.

Show equivalent mission behaviour on both backends and scripted cell I/O.

Order: TC0 → TC1 → TC2 → TC3 and TC4 (independent) → TC5 → TC6.
