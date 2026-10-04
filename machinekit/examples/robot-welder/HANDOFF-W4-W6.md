# Robot welder: finish W4, W5 and W6

A self-contained brief for continuing the robot welder. `PLAN.md` next to this file is the design record. Read it in full
first, especially "Keeping phase 2 open", the W0–W3 notes and "Hardening after W3". This file says what is left and how
to work.

## Where things stand

- **Worktree:** `/home/joao/dev/materia-worktrees/mobile-welder`, branch `mobile-welder`. Local `main` is at the merge
  `2b9574aa5`, which brings in W0–W3, the hardening pass, and two W4 commits:
  - `16673a215`: a robot tool owns its channels' stop policy (`ToolChannels`, `WeldChannels`, `SuctionChannels`,
    `blueprint.addTool`);
  - `d7d3d0f14`: swept clearance of an arm and its tool against the cell's hulls (`ArmClearance`, `ClearanceTests`).
- **The merge has not been built or run.** It resolved seven conflicts, all additions from both sides. The one
  behavioural change is that `MissionPlayer.toolArm` now uses main's `robot.toolFrames` for both suction and torch.
- **This branch is synced with main** (2026-10-04). Local `main` `712019dbc` (transmissions X7–X10, the `WeldBeads`
  compile fix, and `robotkit/RESTRUCTURE_PLAN.md`) is merged in as `bb044fa83`. It has not been built or run since.
- **The draft is restored on top of that merge**, uncommitted (26 files). Two things were changed while restoring it:
  - **Its `RobotArm.hx` change was dropped** in favour of main's version. The draft had added a required
    `ArmTool.ready()` against the old spec table, which the ground rules below forbid.
  - **`ArmWeldingTool.ready()` is therefore unwired.** The welding tool's ready pose still has to be given, from
    `WeldingCell` or the tool's constructor.

  The draft's other changes to files main also changed (`ProjectSourceTests.hx`, `AssemblyPreview.hx`,
  `WeldingPlanRunner.hx`) applied without conflicts. They have not been compiled yet.
- **RobotKit is about to be restructured; read `robotkit/RESTRUCTURE_PLAN.md`.** Two phases affect this work:
  - **R3 moves process semantics out of RobotKit into ProcessKit:** `WeldPlan`, `WeldRunner`, `WeldSeam`,
    `SimulatedWelder` and the arc model, behind RobotKit's generic tool interfaces. Put new welding code in ProcessKit
    from the start (`processkit`, or `machinekit.welding` for CAD-side pieces), not in `robotkit.skill` or
    `robotkit.tool`. Don't do R3's moves yourself unless the user asks; they are scheduled at a quiet point.
  - **R0 renames the device protocol** (the `RKD6` marker against wire version 12) and puts `RobotRuntime` behind a
    `RuntimeEndpoint`. W6 builds on both, so check R0's state before starting W6. If R0 hasn't landed, use the
    protocol's current names, keep W6's protocol-facing code in one place, and say so in the report.
- **Another session builds on this code.** A Codex session in the `machine-tending` worktree (MT1–MT5) uses `ArmTool`,
  `ArmClearance` and the mission machinery from this branch. Its gripper tool implements `ArmTool`.
  - It is also reworking `RobotArm.hx` for drive geometry (`GearedArmJoint`, gearbox size, driver and power-supply
    members); that branch was at MT2, `73a14c58e`, on 2026-10-03.
  - **Keep the welder's change to `RobotArm.hx` as small as possible, ideally none.** Give the welding tool's ready pose
    from `WeldingCell` or the tool's constructor, not by editing the arm's spec table, so the next sync does not collide
    with theirs.
- **Uncommitted W4 work in progress** sits in the worktree from a session that stopped midway. Its state is unknown, and
  it may not compile:
  - `processkit/WeldCorner.hx`: the corner-turn rule;
  - `processkit/WeldPathPlanner.hx`: roll and approach chosen by reach and clearance;
  - `machinekit/welding/WeldingMission.hx`: the mission generated from the weldment;
  - `machinekit/welding/WeldingTorchNeck.hx` and `WeldingTorchNozzle.hx`: the torch split into collision parts;
  - `robotkit/tool/ConvexDistance.hx`, pulled out of `ConvexSolid`;
  - tests in `processkit/tests/src/WeldPlanningTests.hx` and `robotkit/tests/src/tests/ClearanceTests.hx`;
  - edits to `WeldingPlanRunner`, `WeldPlan`, `RobotWelderPreview`, `WeldingWorkpiece`, `ArmWeldingTool`,
    `AssemblyPreview`, `ComponentCapability` (`ArcTorch` gained `stickoutMm`), `EndEffectorControls`, `MissionPlayer`
    and `ProjectSourceTests`, plus a new `materia.seam.project.json`.

  Treat it as a draft: keep what is sound, finish it, and commit it in logical pieces.

## Ground rules

- **Work only in this worktree.** Never modify `/home/joao/dev/materia` (the main checkout is shared by several sessions)
  or any other worktree. Reading them is fine. Don't push. Don't create branches. Don't use bare `git stash`; the stash is
  shared with other sessions.
- **Never rebuild OCCT.** Always export `CADKIT_OCCT_DIR=/home/joao/dev/materia-cache/occt-f1dc4efb-release` (the shared
  prebuilt install). Use the CadKit build at `/home/joao/dev/materia-worktrees/topological-naming/cadkit/build/debug`,
  read-only.
- **Disk is tight.** Check `df -h /` before a native build. Don't populate submodules you don't need, and don't create
  clones or worktrees.
- **Keep validation fast** (the user's rule):
  - after each change, run only the focused unit suites it touches;
  - run the app simulation check once per milestone, with `PROJECT_SOURCE_ONLY=welder`;
  - add `arm` and `mobile` only when `MissionPlayer`, `ApplicationSimulation` or other shared app code changed;
  - run MachineKit smoke once per milestone, and only if MachineKit changed;
  - run RobotKit world tests and the MotionKit suite only when you touched those kits;
  - never rerun suites you didn't affect, and run heavy suites one at a time (parallel heavy runs get killed with exit
    137).
- **No legacy compatibility** (the user's rule). Old saved data is not migrated or quietly reinterpreted. Each saved
  format has one current schema version, and anything else is rejected with one generic message. When a format changes,
  bump its version, delete the old handling, and regenerate examples and fixtures instead of keeping legacy ones.
- **Don't break the other session's API.** Don't add required methods to `ArmTool`: machine-tending implements it, and
  Haxe interfaces have no default methods. Put optional tool data in the constructor of `RobotArm` or of the tool instead.
  Keep `ArmClearance` and the `MissionPlayer` step machinery source-compatible, or call out every change in your report.
- **Model cleanly.** Derive things from the CAD (seams, frames, wire, stickout, grounding, names). Put each policy in the
  layer that owns it. Respect `robotkit/ARCHITECTURE.md`: Cartesian, tool and work concepts stay above `RobotRuntime` and
  the native runtime. Match the surrounding code's style and comment density.
- **Commits:** logical commits on `mobile-welder`, in the repo's message style. Don't commit build directories or
  temporary hacks.
- **Syncing to main.** At the end of each milestone (W4, W5, W6), once its checks pass, sync it onto local `main`; the user
  wants finished work on main with no topic branches.
  1. If `main` is an ancestor of `HEAD`, fast-forward it with an old-value guard:
     `git update-ref refs/heads/main <new-full-sha> <old-full-sha>`. Full SHAs are required.
  2. If `main` moved, merge `main` into `mobile-welder` first, then fast-forward.
  3. Never touch the checked-out branch of the shared checkout.

## Coordination with the RobotKit restructuring

Another session is restructuring RobotKit at the same time, in the `robotkit-restructure` worktree, following
`robotkit/RESTRUCTURE_PLAN.md`. On 2026-10-04 it was in R0, splitting trajectory storage and validation into
TrajectoryKit. The two sessions take turns on main at agreed sync points.

**Order of work:**

| Stage | Welder session | Restructure session |
| --- | --- | --- |
| Now | W4 | R0, R1, R2 (little overlap with W4) |
| Sync point A | W4 lands on main, then **pause** (read and plan only) | Merges main, then does R3 and R4 at a quiet point |
| Sync point B | Merges main, fixes imports, then does W5 | Continues R5, R6 |
| W6 | Starts only once R0's protocol rename and `RuntimeEndpoint` are on main | — |

**Why this order:**
- **R3 moves the files W4 edits** (`WeldPlan`, `WeldRunner`, `WeldSeam` and `SimulatedWelder`, into ProcessKit).
- **R4 rewrites imports everywhere,** including ProcessKit's welder code, since `FiredProcessEvent` and other
  `robotkit.world` types move.
- **Editing a file while another branch moves it is what makes merges painful,** so the welder session is quiet while
  R3 and R4 run.
- **W5's weave lives in `motionkit/path`,** which R0 is changing (`PathTimeLaw`, `NativeJointPath`). Build it on R0's
  layout, after sync point B.

**Rules:**
- **Ownership.**
  - This session owns `machinekit/haxe/src/machinekit/welding`, the welding parts of ProcessKit, the `robot-welder`
    example and the app's welder pieces (`WeldBeads`, the weld parts of `MissionPlayer` and `SimulatedTools`).
  - The restructure session owns RobotKit's structure, TrajectoryKit and the device protocol's naming.
  - Don't edit the other session's files, apart from fixing the compile errors its moves cause after a merge.
- **New welding code goes in ProcessKit** (or `machinekit.welding` for the CAD side), never in `robotkit.skill` or
  `robotkit.tool`, so R3 has nothing more to move.
- **Before starting any milestone, merge main.** At that point:
  - if main contains R3 or R4 (welder files moved out of `robotkit.skill` and `robotkit.tool`, or `robotkit.world`
    split up), fix imports and paths first, with nothing else in the same commit;
  - if the restructure session has announced that R3 or R4 is under way but it hasn't reached main yet, stop and
    report rather than continuing to edit the welder files it moves.
- **Sync often.** Fast-forward main as soon as a milestone's checks pass, so the restructure session's next merge
  picks it up. Report the sync in your milestone report: the main SHA, and that W4/W5/W6 has landed.
- **The machine-tending session** (`machine-tending` worktree) also builds on `ArmTool`, `ArmClearance` and the mission
  code. Keep those source-compatible, as the ground rules say.

## Test recipes

Run all of these from the worktree root, with these variables set:

```
OCCT=/home/joao/dev/materia-cache/occt-f1dc4efb-release
B=/home/joao/dev/materia-worktrees/topological-naming/cadkit/build/debug
WT=/home/joao/dev/materia-worktrees/mobile-welder
export CADKIT_OCCT_DIR=$OCCT
```

- **Kit suites:**

  ```
  LD_LIBRARY_PATH=$B/core:$OCCT/lib:$WT/haxeon/out haxeon/scripts/haxeon run --project <dir>/haxeon.json
  ```

  `<dir>` is one of:
  - `robotkit/tests/tool-capabilities` (weld, process and tool tests);
  - `processkit/tests`;
  - `projectkit/tests`;
  - `robotkit/cadbridge/tests`;
  - `machinekit/tests` (the MachineKit smoke, about 3 min);
  - `machinekit/examples/robot-welder`;
  - `robotkit/tests` (world tests);
  - `motionkit/tests`.
- **App simulation checks (project-source suite).**
  - Compile, about 1 min. `--output` must be absolute, or the binary silently lands elsewhere:

    ```
    haxeon/scripts/haxeon build --project app/haxeon.project-source.json --compiler-only --output=$WT/app/build/host/project-source.hl
    ```

    If the native host libraries are missing, run the same command without `--compiler-only` once (about 8 min).
  - Run:

    ```
    HAXEON_HOME=$WT/haxeon PROJECT_SOURCE_ONLY=welder \
    LD_LIBRARY_PATH=$WT/haxeon/out:$WT/haxeon/.tools/hashlink:$WT/app/build/host/native/app:$WT/app/build/host/native/kinematicskit-native:$WT/app/build/host/native/stockkit:$OCCT/lib \
    haxeon/.tools/hashlink/hl app/build/host/project-source.hl
    ```

    `PROJECT_SOURCE_ONLY` is one of `welder`, `arm`, `mobile`, `router` or `belts`.
- **Runtime trouble.** If a program fails with "Failed to load function haxeon_runtime@...", rebuild the runtime with
  `haxeon/scripts/build-native.sh`.

## haxeon (this repo's own Haxe compiler): quirks and one known bug

- Use `interface` for shared behaviour; anonymous typedefs are only for pure data (there is no class-to-typedef
  structural subtyping).
- `Math` is reduced: there is no `atan` or `asin` (use `atan2`). Grep `haxeon/stdlib/Math.hx` before using any other
  `Math` function.
- A field initialised with an enum constructor may need a type annotation (error E1002).
- Statics initialise in declaration order.
- `<` doesn't work on strings; use `Reflect.compare`.
- Narrow a `Null<Int>` before doing arithmetic on it.
- There are no methods on an `enum abstract`.
- **Known compiler bug, being fixed separately; do not edit haxeon.** A closure that calls a method on a captured local
  that was reassigned after its declaration sees the local's initial value. For example:

  ```
  var t = T.identity();
  if (c) t = x;
  function f() return t.compose(y);   // uses identity()
  ```

  Field reads are fine. Avoid the pattern by computing into a local that is never reassigned, or by using a helper
  function. Suspect this bug whenever a closure shows stale values.
- When a compile error is unclear, read the haxeon compiler source to find the root cause before working around it.

## W4: weld the whole weldment (finish)

Goal: the cell's default mission welds every seam of the weldment. That is 10 seams: two 180 mm plate fillets, and two
tube posts with four 40 mm sides each, welded as chains. Every approach, weld path and retract must be checked clear,
and nothing in the mission may be hard-coded.

0. **Check the starting point.** Main is already merged.
   1. If `main` moved again, bring it in the same way: set the draft aside with
      `git stash push -u -m mobile-welder-w4-draft-<date>`, record its SHA from `git stash list --format='%H %gs'`,
      merge `main`, restore the draft with `git stash apply <sha>`, then drop the stash entry, finding its current index
      by its tag first.
   2. Wire the welding tool's ready pose without touching `RobotArm.hx` or adding a method to `ArmTool` (see the ground
      rules). Then remove `ArmWeldingTool.ready()`.
   3. Delete the pre-path weld decoding in `SceneArtifact.hx`: the "A weld written before paths existed" branch in the
      weld step's decoder. A step without `path` is rejected.
   4. Compile everything the merge and the draft touch: each affected kit suite once, then the app `--compiler-only`
      build. Fix the draft until it compiles.
   5. Run `PROJECT_SOURCE_ONLY=welder` and `arm` once. This merge also checks main's X8 arm drives against the welder.
1. **Corner-turn length is derived, not fixed** (`WeldCorner`). Derive it from the reorientation angle and the wrist's
   angular speed and acceleration limits at the weld's travel speed, so the tool tip keeps travel speed while turning.
   - Bound it, at most 45% of either segment, and document the rule.
   - Unit tests: a sharper corner gets a longer turn; a straight join gets none; the limits are respected.
2. **Clearance chooses roll and approach** (`WeldPathPlanner` over `ArmClearance`).
   - For each segment, among the rolls the runner already searches (`withRolls`), pick one that is both reachable and
     clear.
   - Choose the approach and retract direction (along the wire, or along the face bisector) and their height so that
     they clear.
   - Check the swept motion: sample the planned motion finely, check the torch neck, nozzle and arm links against the
     work, fixtures, table and equipment hulls with a margin, and exclude only the wire-tip zone at the seam.
   - A violation fails planning with a reason naming the part and the pose.
   - Unit tests: a deliberately colliding approach is rejected and an alternative is found; a case with no clear roll
     reports clearly.
3. **Mission generated from the weldment** (`WeldingMission`).
   - One `weld` step per seam or chain, built from `Weldment.find` and `WeldSeams.chains`, so adding a member adds its
     welds.
   - Derive from the CAD the reference member (`frame`), the metal carrier and the stickout (`ArcTorch.stickoutMm`).
     Remove the hard-coded `"work/weldMetal"`, `"work/basePlate"` and `WeldingTorch.STICKOUT` uses in the generator and
     preview.
   - **Order:** nearest-neighbour by air-move distance plus a reorientation cost. Choose each seam's direction to suit
     the order. Document it.
   - **An unweldable declared joint fails the generator check** (the MachineKit smoke): the CAD is ours to fix. It is
     never skipped silently.
4. **The cell's default mission welds the whole weldment.** Keep the single-seam project (`materia.seam.project.json`)
   for the existing focused tests.
5. **App test** (`PROJECT_SOURCE_ONLY=welder`), on MuJoCo, plus the test backend if it's cheap:
   - the whole weldment welds;
   - every bead is within 2 mm of its seam's length;
   - every leg is within 0.5 mm of 5 mm;
   - no clearance violation;
   - the total cycle time is reported.

   The existing checks must still pass: single seam, arc loss and recovery, weld held in the air, displaced workpiece,
   post chain and crater dropout.
6. **PLAN.md:** W4 notes (generation, ordering, the clearance design with its margins, sampling and cost, the corner
   rule, stop-policy ownership), then the Progress row. Sync to main.

## W5: weaving and multi-pass

Goal: welds bigger than one pass lays, and a weave for wider beads and gap bridging, with the bead model and the checks
still meaningful.

1. **Weave as a MotionKit path modifier**, in `motionkit/haxe/motionkit/path`.
   - A `PosePath` that offsets the base path laterally across the seam frame as a function of distance along the seam.
     Patterns: sine, triangle and zigzag. Parameters: amplitude, frequency per mm or per second, and a dwell at each edge.
   - It must provide exact poses and derivatives (`PoseDerivatives`), so the time law and plan checks work unchanged.
   - **Travel speed means progress along the seam,** not tool-tip speed: the time law runs on seam distance, and the tool
     tip moves faster.
   - Process events stay at seam distances.
   - The clearance check and the motion check run on the woven path.
   - Unit tests: offsets and derivatives match numeric differences; events land at the right seam distance; zero
     amplitude equals the base path.
2. **Weave in the recipe and the bead.**
   - `WeldingRecipe` chooses a weave for legs above a threshold; document the rule, for example a weave from about 6 mm.
     Amplitude is tied to the leg, and travel speed is recomputed from the deposited area.
   - `WeldBead` stations project the tool tip onto the seam, so a woven pass deposits into the right stations. The leg
     comes from the area, as before.
   - Test: a woven 7 mm fillet measures within 0.5 mm.
3. **Multi-pass.**
   - The recipe splits a leg beyond a single-pass maximum (about 8 mm) into root, fill and cap passes, with their own
     process values and torch offsets in the seam frame (toward each face, and lifted by the metal already deposited).
   - The scene file's `weld` step gains `passes`: one path and process per pass, or a pass list over one path. Choose
     cleanly and validate. Under the no-legacy rule, bump the scene artifact version, reject older ones, and regenerate
     the examples.
   - **Earlier beads count as grounded work for later passes,** so arc length and touch see the deposited metal: grounded
     work gains the bead's runtime geometry, or a convex approximation per station.
   - Interpass: an optional dwell (a stand-in for cooling).
   - Test: a 10 mm fillet in three passes. The final leg is within 0.5 mm, each pass strikes on the earlier metal, and
     there are no clearance violations.
4. **Restart hump** (from the W3 review). A restart's overlap currently doubles the metal over about 10 mm (a 9.4 mm leg
   at the hump). Reduce it, for example by ramping wire speed up over the overlap or shortening it, and test that the leg
   at a restart stays within 1 mm of the target.
5. **PLAN.md:** W5 notes and the Progress row. Sync to main.

## W6: real welder interface

Goal: the same weld missions drive a real welder through the robot's device link, with the arc going off on every stop
and fault, behind the existing device-neutral boundary (`WelderOutputs`, `WelderFeedback`, `WelderProcessDevice` in
ProcessKit). Hardware isn't available yet, so everything is proven against simulated devices.

1. **One channel contract.** Write down the welder's channels and their meaning in one place: arc on, wire speed in
   m/min, voltage setpoint in V, optional job number, and the six-value `tool_weld` sensor (arc established, current,
   voltage, touch, fault code, power). Reference it from both the simulated welder and the device layers.
2. **Over RKD6.** Carry the welder's channels and its sensor over the device protocol (`robotkit/device_protocol`), the
   way other process channels and tool sensors already travel.
   - Use the channel stop policy that RKD6 v12 already carries (`channel_stop_policy`), so the device itself takes arc and
     wire off on a commanded stop, an emergency stop, a lost link and a fault.
   - Teach `device_virtual` (Rust) a simulated welder behind those channels (a simple arc model is enough), and test from
     the host: arc on, arc established, a stop takes the arc off on the device, and a link loss takes it off.
3. **Retrofit I/O profile.** A device-side mapping from the welder channels to the retrofit board's I/O:
   - the trigger is a digital output (an optoMOS relay);
   - wire speed and voltage are 0–10 V analog outputs, with configurable scaling;
   - current comes from a Hall-effect sensor on an analog input;
   - arc voltage comes from an isolated analog input;
   - touch comes from a digital input on a separate sensing circuit.

   Arc established is derived from current above a threshold. A fault comes from no arc within a timeout, or from a
   short.

   Put the mapping where board configuration lives (see `robotkit/device_protocol/boards`). If the nucleo-g474re board
   has the pins, add the profile there, checked with `cargo check --offline` only; otherwise define the profile and test
   it in `device_virtual`.
4. **Robot power source over Modbus TCP.** A host-side `WelderOutputs`/`WelderFeedback` implementation over a register
   map. Define the map as data so a real vendor's map (Miller Auto Deltaweld, Fronius, Megmeet) can be dropped in. Test
   against an in-process fake Modbus server: setpoints are written, the arc-established and fault bits are read, and the
   connection dropping makes the device unsafe and faults the process run.
5. **The same mission everywhere.** One weld mission runs unchanged against:
   - the simulated welder;
   - the RKD6 virtual device;
   - the Modbus fake.

   Every stop, abort, fault and link loss ends with the arc off.
6. **PLAN.md:** W6 notes and the Progress row. Sync to main.

## Report at the end of each milestone

- the commits, with SHA and subject;
- what was built, and the decisions taken;
- the suites run, with pass or fail and their key numbers (bead legs and lengths, cycle time);
- what is left open;
- any problems hit.
