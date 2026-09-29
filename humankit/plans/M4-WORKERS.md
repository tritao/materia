# HumanKit M4: workers that do jobs

Implementation plan, written to be picked up by an agent with no prior
context. Read all of it before starting; the "Environment" section has traps
that cost hours if ignored.

## Goal

Higher-level code can give a simulated human a job — "walk to rack A, pick
the part in slot B3, carry it to the assembly table, place it, press the start
button" — and the job runs physically: the part really leaves the rack, moves
with the hand, and lands on the table, while robots see the worker's body as
an obstacle and safety logic can read where the worker is.

**Acceptance demo (end of M4e):** in one MuJoCo session, a worker moves a
part from a rack slot to a table while a robot arm cycles nearby. A headless
test asserts the part's final pose, zone occupancy transitions, and the
worker/robot separation stream; the app runs the same scenario from the
command line and screenshots are captured.

## Where things stand (main, as of `ada9a92b`)

- **HumanKit** (`humankit/haxe/humankit`): `HumanCharacter` (skinned glTF
  character, `ClipPlayer`, `reach(limb, target, weight, ?pole)` /
  `release(limb)` via AnimKit two-bone IK, `attach(prop, bone)` visual props),
  `HumanWalker` (`follow(route, speed, loop)`, gait-matched clip rate, speed
  ramp, braking, bounded turning, `rootTransform()`), `HumanReachTask`
  (hard-coded walk → reach ramp → hold → release state machine),
  `HumanBodyProxy` (15 capsules), `HumanDescription`, `HumanPose`,
  `HumanGrip`, `HumanLimb` (`ArmL`, `ArmR`, `LegL`, `LegR`).
  **Reach targets are in the character's model space**, not world space; the
  character root is placed from `walker.rootTransform()` (+X forward).
- **humankit-sim** (`humankit/sim`): `HumanActor(session, proxy, pose, root)`
  and `pushPose(time, pose, root)` — the capsules join a SimKit session as one
  kinematic actor (timed keyframes). No Haxe tests exist for it yet.
- **humankit-facility** (`humankit/facility`): `FacilityWalk.routeFromPath`,
  `routeFromFacilityRoute`.
- **AutomationKit** (`automationkit/haxe/materia/automation/facility`):
  `Facility` with `Zone` (id, frame, `robotkit.mobile.Footprint` boundary), `Station` (id,
  zone, frame, `Pose2`), `Rack extends Station` with slot **IDs only (no slot
  positions)**, `FacilityRouter.route(fromStation, toStation)`.
- **SimKit session** (`simkit/sim_core/include/nativekit_sim_session.h`,
  `src/session.cpp`): objects are created/destroyed **only while stopped**,
  their motion type is fixed for life; `drive_object` moves a KINEMATIC
  object while running. Nothing can pick up or carry an object. RobotKit's
  grippers (`robotkit/haxe/robotkit/tool/SimulatedGripper.hx`) only report a
  grasp state; they never move the object either.
- **MuJoCo backend** (`simkit/sim_mujoco/src/mujoco_backend.cpp`): a
  DYNAMIC root and a *childless* KINEMATIC root both get a MuJoCo free joint;
  the kinematic one gets nominal mass `kKinematicBodyMass`, gravity
  compensation, and is re-placed on its trajectory before every substep
  (`kinematic_drives()` / `place_kinematic_drives()` in `step()`). A
  kinematic root that carries other bodies stays welded (see
  `robotkit/ARCHITECTURE.md`, "MuJoCo kinematic-root velocity leak"). Topology
  changes rebuild the MuJoCo model (`rebuild()`).
- **AnimKit** only exposes two-bone IK (`ak_instance_set_ik`). No aim IK.
- **Test character** `animkit/assets/quaternius/worker.glb`, clips:
  `Walk`, `Idle`, `Interact`, `Run`, `Wave`, … (no carry/pick clips — carry is
  IK over the walk). Tests load it as in `humankit/tests/src/HumanKitTests.hx`.
- **App**: `app/src/editor/CharacterPreview.hx` is the only consumer; it
  implements `app/src/SessionParticipant.hx` (`join(session)` while stopped,
  `leave()`) and is registered via `ApplicationSimulation.addParticipant`.
  Robots come from `SensorConfiguration.robotModels()` and are added in
  `ApplicationSimulation` with `addRobotAtPose`.

## Environment (read first)

- **Never work in `/home/joao/dev/materia` directly or switch its branch** —
  other sessions share it. Make a worktree (script in the appendix):
  `make-worktree.sh humankit-m4` → `/home/joao/dev/materia-worktrees/humankit-m4`
  on new branch `humankit-m4` from `main`.
- If a haxeon-compiled program fails with a missing native function (e.g.
  `__reflect_delete_field`), the copied `haxeon/out/haxeon_runtime.hdll` is
  stale: run `fill-nested.sh haxeon /home/joao/dev/materia/haxeon` then
  `haxeon/scripts/build-native.sh` in the worktree.
- **Never build OCCT.** Before any build that pulls in CadKit (app,
  cadbridge suites) `export CADKIT_OCCT_DIR=/home/joao/dev/materia-cache/occt-f1dc4efb-release`.
  A build dir configured without it must be deleted first.
- Use ccache: `export CCACHE_BASEDIR=/home/joao/dev CMAKE_C_COMPILER_LAUNCHER=ccache CMAKE_CXX_COMPILER_LAUNCHER=ccache`.
- Disk space on `/` has run out before; check `df -h /` if builds fail
  oddly, and keep native build dirs out of `/tmp` if space is low.
- **haxeon** is this repo's own Haxe compiler. When it lacks a feature or
  mis-compiles something, fix it in haxeon (commit in the worktree's
  `haxeon` clone, with a test in `tests/compiler` or `tests/programs`,
  registered in `tests/driver/TestCatalog.hx`; run `scripts/format.sh` then
  `scripts/test.sh`) instead of working around it. Known quirks: no member
  access on `Null<T>` fields (bind to a local first); secondary module types
  need their own file; no class-to-anonymous structural subtyping (use an
  interface).
- Diagnose root causes before workarounds; remove debug code before
  committing. Check vendored sources (`simkit/vendor/mujoco`) before claiming
  how MuJoCo behaves.
- Don't use `git stash`; don't `git add -A` (build dirs are not all ignored).
- Commit per milestone; end messages with
  `Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>` (or your model's line).

### Build and test commands (run from the worktree root)

Native SimKit (C API, both backends):

```
cmake -S simkit -B <build>/sim -DNK_BUILD_TESTS=ON -DNKSIM_BUILD_MUJOCO=ON -DMUJOCO_BUILD_TESTS=OFF -DCMAKE_BUILD_TYPE=Release
cmake --build <build>/sim -j 6 && LIBGL_ALWAYS_SOFTWARE=1 ctest --test-dir <build>/sim -j 4
```

Native RobotKit (needed whenever SimKit changes):

```
cmake -S robotkit/robotd/native -B <build>/rk -DRK_BUILD_TESTS=ON -DROBOTD_BUILD_TESTS=ON -DNKSIM_BUILD_MUJOCO=ON -DMUJOCO_BUILD_TESTS=OFF -DCMAKE_BUILD_TYPE=Release
cmake --build <build>/rk -j 6 && ctest --test-dir <build>/rk -j 4
```

Regenerate Haxe FFI bindings after changing a C header:
`bash simkit/sim_core/tools/check-hxi.sh` (and `simkit/sim_mujoco/tools/check-hxi.sh`,
`robotkit/runtime/tools/check-simkit-hxi.sh` if their headers change). Never
hand-edit `.hxi` files.

Haxe suites: `./haxeon/scripts/haxeon run --project <dir>/haxeon.json` for
`humankit/tests`, `animkit/tests`, `automationkit/tests`, `robotkit/tests`,
`robotkit/tests/mujoco`, `app/tests`; `haxeon build --project app/haxeon.json`
for the app. The cadbridge suites (`robotkit/cadbridge/tests`,
`robotkit/cadbridge/tests/mujoco`) need `LD_LIBRARY_PATH` pointing at a
directory containing **only** a symlink to your app build's
`app/build/host/native/app/libcadkit-core.so` (pointing it at the whole app
native dir loads a stale MuJoCo and segfaults).

App screenshots must be headless, never on the real display:
`LIBGL_ALWAYS_SOFTWARE=1 xvfb-run -a -s "-screen 0 1600x1000x24" ./app/run-built.sh --reset-workspace --capture-dir <dir> ...`

## Milestones

Do them in order; each ends with all affected suites green and a commit.

### M4a — SimKit: objects that can be held and released (≈ largest risk)

Shared by humans and robot grippers.

**API** (`nativekit_sim_session.h`, implemented in `session.cpp`):

```c
/* Attaches a DYNAMIC object to a carrier while stopped or running: from the
 * next tick it follows the carrier body's frame at `offset` (carrier-local
 * pose) and is not moved by contacts. carrier is any session body: an actor
 * part (nksim_session_get_actor_body) or a robot link body. */
NKSIM_API nksim_result NKSIM_CALL nksim_session_hold_object(
    nksim_session session, nksim_object object, nksim_body carrier,
    const nksim_pose *offset);
/* Releases a held object: it is DYNAMIC again from the next tick, starting
 * with the carrier point's velocity at release. */
NKSIM_API nksim_result NKSIM_CALL nksim_session_release_object(
    nksim_session session, nksim_object object);
/* Reports whether an object is held and by which carrier (0 when free). */
NKSIM_API nksim_result NKSIM_CALL nksim_session_get_object_carrier(
    nksim_session session, nksim_object object, nksim_body *out_carrier);
```

Rules: only DYNAMIC objects can be held; holding an already-held object
re-targets it; releasing a free object is `NKSIM_ERROR_INVALID_STATE`;
destroying a held object or its carrier's actor releases it; **reset
restores every object to free, at its reset pose**; the carrier's pose used
each tick is the carrier's pose *for that tick* (actors: the interpolated
keyframe pose the session already computes; robot links: their latest state).

**Design** (confirm with a spike first, then implement):

1. Add a backend hook for switching a body between DYNAMIC and KINEMATIC
   without changing topology, e.g. `PhysicsBackend::body_set_motion_type(body,
   motion_type, mass)` (default: `NKSIM_ERROR_UNSUPPORTED`), and plumb a
   world-level internal function the session calls. Held = KINEMATIC driven
   every tick to `carrier_pose * offset` using the existing kinematic drive
   path (`drive_parts` in `session.cpp`, `World::refresh_kinematic_bodies`),
   so the backend sees the true velocity over the tick.
2. **MuJoCo**: a childless root already has a free joint whether DYNAMIC or
   KINEMATIC, so the switch is a runtime model edit, not a rebuild: set
   `model->body_mass`/inertia (nominal kinematic values vs. the object's
   real ones), `model->body_gravcomp` (1 while held), and include/exclude it
   in `kinematic_drives()` (make that function consult the body's *current*
   motion type, not only `desc`). After editing masses, check in the vendored
   source (`engine_setconst.c`) whether `mj_setConst` must be re-run for
   `dof_invweight0`/`body_invweight0`; if so, measure its cost. The
   non-DYNAMIC pair collision excludes (`add_self_collision_excludes`) are
   built at model build time: a held object must stop colliding with STATIC
   and KINEMATIC bodies while held **or** keep colliding — decide which, and
   document it. Recommended: keep generating contacts against DYNAMIC bodies
   only while held (it can push other parts), and add/remove the exclusion
   with a rebuild only if MuJoCo gives no runtime switch (check
   `mjModel::body_contype`/`geom_contype`/`geom_conaffinity`, which are
   runtime-editable per geom).
3. **Deterministic backend** (`simkit/sim_core/src/physics_backend.cpp`):
   implement the same switch; it already treats non-DYNAMIC bodies as pinned.
4. Session bookkeeping: store `carrier`, `offset`, and the original motion
   type in `Object`; include held objects in the per-tick drive step before
   `nksim_host_step`; clear holds in `reset()`.
5. Regenerate `nativekit-sim.hxi`; add Haxe wrappers in
   `simkit/sim_core/bindings/haxe/nativekit/sim/SimSession.hx`
   (`holdObject(object, carrier, ?offset)`, `releaseObject(object)`,
   `objectCarrier(object)`).

**Tests** (`simkit/sim_core/tests/session.cpp` and
`simkit/sim_mujoco/tests/session.cpp`, both backends):
- A box held by a kinematic actor part moving along +X tracks
  `carrier * offset` to 1e-9 on every tick (deterministic) / 1e-6 (MuJoCo)
  and its reported velocity matches the carrier's.
- Released while moving: it keeps the carrier velocity, then falls and comes
  to rest on the floor.
- Held box pushes a free DYNAMIC box it sweeps through (MuJoCo).
- Reset returns it free, at rest, at its reset pose; hold/release error
  cases; destroying the carrier's actor releases it.
- robotkit native tests still pass (they share SimKit).

### M4b — HumanKit: the action queue (animation only, no physics)

New files in `humankit/haxe/humankit/` (one type per file):

- `HumanAction` interface: `start(worker:HumanBody):Void`,
  `advance(seconds:Float):Void`, `isDone():Bool`, `failure():Null<String>`.
  `HumanBody` is a small class bundling `HumanCharacter` + `HumanWalker` and
  the world↔model transforms (`toModel(worldPoint)`, derived from
  `walker.rootTransform()`), so actions take **world-space** targets.
- `HumanJob`: an ordered queue; `add(action)` returns the job for chaining;
  `advance(seconds)` runs the current action, starts the next when done,
  stops at the first failure (`failure()` reports which action and why);
  `isDone()`, `cancel()` (releases limbs, stops walking).
- Actions:
  - `WalkTo(route | point, speed)` — a single point walks straight there.
  - `ApproachFor(target, limb)` — computes where to stand and which way to
    face to reach a world point with the limb (stand-off along the target's
    horizontal direction by a reach distance derived from the
    `HumanDescription` arm lengths), then walks there and turns to face it.
    Fails with a clear reason if the target is above the reachable height or
    below waist height (crouching is out of scope).
  - `Reach(limb, target, ramp)` / `ReleaseLimb(limb, ramp)` — ramped IK
    weight, as `HumanReachTask` does today.
  - `Pick(target, hands)` and `Place(target, hands)` — reach, hold the IK
    (a `grip` flag the physics layer in M4c observes), then keep the hands at
    a carry pose.
  - `Carry(posture)` — keeps one or both hands at a carry pose relative to
    the pelvis/chest while later `WalkTo`s run (IK weight 1 over the walk
    clip). Two-hand carry places the hands either side of the object.
  - `Press(point)` — reach, short hold, release.
  - `Wait(seconds)`, `PlayClip(name, seconds)`.
- Re-express `HumanReachTask` as `WalkTo` + `Reach` + `Wait` + `ReleaseLimb`
  internally, keeping its public API and existing tests passing.

**Tests** (`humankit/tests/src/HumanKitTests.hx`, add sections; use
`worker.glb`): the job runs actions in order and reports completion; hand
landmark (`HumanPose.bonePosition(HandR)`) within 2 cm of a pick target at
full weight; approach stands within reach and faces the target; an
unreachable target fails `ApproachFor` with the reason; during `Carry` +
`WalkTo`, the carrying hand stays within 3 cm of its carry pose relative to
the chest while the feet keep the gait (pelvis advances at walker speed);
`cancel()` leaves no limb under IK.

### M4c — humankit-sim: `HumanWorker`

New `humankit/sim/haxe/humankit/sim/HumanWorker.hx`: owns a `HumanBody`,
`HumanActor`, and the current `HumanJob`; `new(session, character, proxy,
startPose)`; `run(job)`; `advance()` on simulation time (follow
`CharacterPreview.advance`'s lead-time pattern: advance animation to
`session.simulationTime() + LEAD_TICKS * fixedTimestep()` and push actor
keyframes for that time).

- **Physics pick/place**: when a `Pick` reaches full weight, the worker calls
  `holdObject(object, handCapsuleBody, offset)` with the offset that keeps
  the object where it is relative to the hand capsule; `Place` releases at
  the target. Map hand → the proxy's `hand` capsule for that side (see the
  `add(...)` calls in `HumanBodyProxy.standard`; the hand capsule only exists
  when the rig has a knuckle joint, so fall back to that side's `forearm`
  capsule). The carrier body is `nksim_session_get_actor_body(actor, part)`
  with the capsule's index; `SimActor` has no Haxe accessor for it yet — add
  `partBody(index)`. Jobs reference objects as
  `SimObject`s plus a world grasp point.
- **Safety signals**, published each tick through a small listener API
  (`onTick(worker, signals)`), later consumable by mixed-scene safety:
  - `zones`: IDs of facility zones whose `Footprint` contains any capsule's
    floor projection (pass zones in as polygons to keep humankit-sim free of
    AutomationKit; humankit-facility adapts).
  - `separation`: minimum distance from any worker capsule to each robot's
    link bodies — take link poses from the session snapshot and treat links
    as their collision bounds; exact capsule-vs-shape distance is a later
    refinement. Document the approximation.
- Add a Haxe test project `humankit/sim/tests` (entry `HumanSimTests`, native
  dependency set like `robotkit/tests/mujoco` so MuJoCo is available) with:
  part picked from a shelf box and placed on a table box ends at rest within
  1 cm of the place target; a part released mid-walk falls; zone occupancy
  flips entering/leaving a zone; separation to a robot link that moves
  toward the worker decreases monotonically and matches a hand calculation;
  `HumanActor` contact with a free box (the missing HumanActor test).

### M4d — Facility targets

- AutomationKit: give `Rack` slot poses — `slots:Array<RackSlot>` with `id`
  and a pose relative to the rack (`x, y, z` + yaw); keep `slots()` /
  `hasSlot()`; add `slotPose(id)` in the facility frame. Update any facility
  builders/demos (`app/src/editor/FacilityRouteDemo.hx`) and
  `automationkit/tests`. Check whether facilities are serialized anywhere
  (search for codecs of `Facility`/`Station`) and extend them if so.
- humankit-facility: `FacilityTargets` resolving IDs to world targets:
  `stationStandPose(stationId)`, `rackSlotPoint(rackId, slotId)`,
  `route(fromStationId, toStationId)` via `FacilityRouter`; and a job builder
  `FacilityJobs.fetch(facility, rackId, slotId).deliver(stationId, placePoint)`
  producing a `HumanJob`.
- Tests in `humankit/tests`: slot poses resolve through rack pose; the fetch
  job's routes follow the facility lanes and end within reach of the slot.

### M4e — App and the acceptance demo

- `SensorConfiguration`: `humans()` alongside `robotModels()` (id, character
  asset path, start pose, optional job name); `ApplicationSimulation` adds a
  `HumanWorker` participant per entry when a session is built.
- Command line for the demo (mirroring the `--character-*` flags in
  `app/src/Main.hx`): e.g. `--worker-demo=rack-to-table` builds the demo
  facility (rack with a part, table, a robot arm cycling) and runs it.
- A headless test (in `humankit/sim/tests` or `app/tests`) of the full demo
  asserting the acceptance criteria; screenshots at start / carrying /
  placed, captured headless.
- Optional in this milestone: switch RobotKit's simulated gripper to real
  holds (`SimulatedGripper` closing on a detected object calls
  `holdObject` with the tool-flange link body; opening releases).

## Out of scope for M4 (next candidates)

Editor placement and project-document persistence of workers; obstacle
avoidance and yielding between workers/robots; look-at (expose ozz
`IKAimJob` through AnimKit); crouching/kneeling for low targets; bone-length
fitting to `HumanDescription`; data-driven job descriptions; exact
capsule-vs-shape separation.

## Appendix: worktree scripts

`make-worktree.sh <name>` — new worktree + branch from main with submodules
cloned at their pinned commits and haxeon's toolchain copied:

```bash
#!/usr/bin/env bash
set -euo pipefail
name=$1
main=/home/joao/dev/materia
wt=/home/joao/dev/materia-worktrees/$name
git -C $main worktree add -q -b "$name" "$wt" main
cd "$wt"
for path in $(git config -f .gitmodules --get-regexp path | awk '{print $2}'); do
  sha=$(git ls-tree HEAD "$path" | awk '{print $3}')
  rm -rf "$path"
  git clone -q --no-checkout "$main/$path" "$path"
  git -C "$path" checkout -q --detach "$sha"
done
mkdir -p haxeon/out
cp -a $main/haxeon/.tools haxeon/
cp -a $main/haxeon/out/haxeon_runtime.hdll haxeon/out/
echo "ready $wt ($(git log --oneline -1))"
```

If a pinned submodule commit is missing from the main checkout's clone,
clone that submodule from another worktree that has it instead.

`fill-nested.sh <repo-dir> <source-dir...>` — clones missing nested
submodules (recursively) from the first source that has the pinned commit;
needed before `haxeon/scripts/build-native.sh`:

```bash
#!/usr/bin/env bash
set -euo pipefail
repo=$1; shift
[ -f "$repo/.gitmodules" ] || exit 0
for path in $(git -C "$repo" config -f .gitmodules --get-regexp path | awk '{print $2}'); do
  sha=$(git -C "$repo" ls-tree HEAD "$path" | awk '{print $3}')
  [ -n "$sha" ] || continue
  if [ ! -e "$repo/$path/.git" ]; then
    found=""
    for source in "$@"; do
      if git -C "$source/$path" cat-file -e "$sha^{commit}" 2>/dev/null; then found="$source/$path"; break; fi
    done
    [ -n "$found" ] || { echo "MISSING $repo/$path ($sha)"; continue; }
    rm -rf "$repo/$path"
    git clone -q --no-checkout "$found" "$repo/$path"
    git -C "$repo/$path" checkout -q --detach "$sha"
    echo "filled $repo/$path"
  fi
  "$0" "$repo/$path" $(for source in "$@"; do echo "$source/$path"; done)
done
```
