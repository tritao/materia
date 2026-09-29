# HumanKit M5: workers in documents, jobs as data

Implementation plan for an agent with no prior context. Read all of it, and
read the **Environment** section and appendix of `humankit/plans/M4-WORKERS.md`
first: its rules (worktrees, no OCCT builds, fixing haxeon gaps in haxeon,
build/test commands, worktree scripts) all apply here. Lessons added since
M4 are under "Environment additions" below.

## Goal

A user can put a worker into a scene in the editor, give it a job made of
steps that name other scene objects ("walk to the rack, pick `part-1`, place
it onto `table`, press `start-button`"), save the document, reopen it, and
press Play: the worker does the job physically next to the document's robots,
and the safety signals (zone occupancy, separation from each robot) appear in
the telemetry panel. No Haxe code is needed for any of it.

**Acceptance (end of M5d):** the rack-to-table demo exists only as a saved
example document. The app test loads it, runs it, and asserts the part rests
within 2 cm horizontally / 1 cm vertically of where the job says, flat and
still, with the zone and separation signals present. The hard-coded demo code
(`WorkerDemoJob`, `enableWorkerDemo`'s scene building, `HumanConfiguration`)
is gone.

## Where things stand (main at `c6ed8fd3`)

- **HumanKit actions** (`humankit/haxe/humankit`): `HumanJob` (ordered
  actions, `abort`, `cancel`), `WalkTo` (`new(point)` / `WalkTo.along(path)`),
  `ApproachFor(target, arm)` (stands so the shoulder faces a world point at a
  comfortable, height-aware reach), `Pick(target, hands)` (palm centre to a
  grasp point; fails if > 2 cm off), `Carry`, `Place(target, hands)` (puts the
  held object's point on target, settles, opens), `Press`, `Wait`,
  `PlayClip`, `Reach`, `ReleaseLimb`. `HumanBody` holds the character, walker
  and IK state (`gripPoint`, `solveReach`, `setHeldPoint`).
- **humankit-sim** (`humankit/sim`): `HumanWorker(session, character, proxy,
  startPose)`; `bindPick(pick, simObject, graspPoint, hand)`,
  `bindPlace(...)`, `run(job)`, `advance()` before each session step;
  `addZone(HumanZone(id, polygon))`, `addRobotLinkPose(id, poseFn, radius)`,
  `onTick(worker, HumanWorkerSignals{zones, separation})`. It pins parts while
  the hand reaches onto / withdraws from them and levels parts while placing.
- **Conventions M4 settled** (keep them): a Pick's grasp point is **1 cm above
  the centre of the part's top face**; a Place target is **where the part's
  centre rests** (support top + half the part's height); a delivery ends by
  walking back off the support.
- **humankit-facility**: `FacilityJobs.fetch(facility, rack, slot).deliver(
  station, point)` builds a job from AutomationKit IDs. Facilities are **not**
  in documents (only hard-coded `app/src/editor/FacilityRouteDemo.hx`), so M5
  targets scene objects, not facility IDs.
- **App**:
  - Scene objects are `SceneObjectData` records (`app/src/SceneObjectData.hx`)
    encoded by `app/src/SceneCodec.hx` (`encodeObject`; the decoder whitelists
    kinds around line 306). Kinds plug in through
    `app/src/editor/ObjectKindProvider.hx` and `ObjectKindRegistry.hx`;
    `app/src/editor/StockSimulationKind.hx` is a complete external kind to
    copy (add menu, default record, inspector properties with dropdowns via
    `PropertyOption`, commands).
  - Humans today: `app/src/HumanConfiguration.hx` records inside the sensor
    configuration (`SensorConfiguration.humans()` / `addHuman`, saved under
    `"humans"`), consumed by `ApplicationSimulation.rebuild` (search
    `configuration.humans()`), which builds a `HumanWorker` per entry and
    wires the demo through `app/src/editor/WorkerDemoJob.hx`.
  - `Main.enableWorkerDemo` builds the demo scene (floor, rack, table, part, a
    one-joint robot arm) and `--worker-demo=rack-to-table` runs it;
    `app/tests/src/tests/WorkerDemoTests.hx` tests it.
  - Robot separation bounds: `app/src/editor/RobotLinkBounds.hx`.

## Design decisions (already made; don't relitigate)

1. **A worker is a scene object** of a new kind `human-worker`, not a sensor
   configuration entry: that gives hierarchy, selection, gizmo, inspector,
   undo and save for free. The M4 sensor `"humans"` records are migrated on
   load and then removed.
2. **Job targets are scene object IDs** (or explicit points). Facility
   authoring in documents is a later milestone; `humankit-facility` stays as
   a code-level builder.
3. **No path planning in M5.** `walkTo` goes straight, or through explicit
   `via` points the user lists. Obstacle avoidance is a later milestone.
4. **The job format is versioned JSON owned by HumanKit** (pure Haxe, no app
   or simulation dependency), so tools and tests can build and validate jobs
   without the editor.

## Job format (version 1)

Stored in the worker's scene object as `worker.job`:

```json
{
  "version": 1,
  "loop": false,
  "steps": [
    {"action": "walkTo", "target": {"object": "rack"}, "via": [[1.0, 0.5]]},
    {"action": "pick", "object": "part-1", "hand": "right"},
    {"action": "walkTo", "target": {"point": [3.0, 2.2]}},
    {"action": "place", "onto": "table", "offset": [0.0, 0.0]},
    {"action": "press", "target": {"object": "start-button", "anchor": "top"}},
    {"action": "wait", "seconds": 1.0},
    {"action": "playClip", "clip": "Wave", "seconds": 1.5}
  ]
}
```

- `hand`: `right` (default), `left`, or `both` (two-hand pick/place, using the
  existing spread in `Pick`/`Place`).
- `walkTo.target.object` walks to the object's nearest side, stopping 0.5 m
  from its footprint and facing it; `target.point` is a floor point.
- `pick` implies `ApproachFor` + `Pick` at the grasp-point convention; then a
  carry.
- `place` implies `ApproachFor` + `Place` at the object's resting point on
  `onto`'s top face (centre plus `offset`, in `onto`'s local x/y), then a step
  back. The held object is known from the preceding `pick`.
- `press` anchors: `top` (top-face centre), `front` (centre of the face
  toward the worker), `center`.
- `loop: true` repeats the steps (a part must be back where the first pick
  finds it; the loop is for demos and throughput studies, and stops on the
  first failure).

## Milestones

Do them in order; each ends with every affected suite green and a commit.

### M5a — HumanKit: job spec, codec, validation, builder (pure)

New files in `humankit/haxe/humankit/` (one type per file; see the M4 plan's
haxeon notes):

- `HumanJobSpec` (+ step types): `parse(json:String)` / `toJson()` with
  strict validation and clear messages (unknown action, missing field,
  non-finite number, bad hand, `place` without a preceding `pick` of the same
  object, a `pick` while that hand already holds something). Unknown future
  fields are an error, a newer `version` is an error naming the version.
- `HumanJobTargets` interface the builder resolves against:
  `box(objectId):Null<HumanTargetBox>` with centre, half-extents and yaw in
  world space (an oriented box is enough for all anchors above).
- `HumanJobBuilder.build(spec, targets, body):{job:HumanJob,
  holds:Array<{action:HumanAction, objectId:String, grasp:Array<Float>,
  hand:HumanLimb}>}` producing the M4 action sequence with the M4
  conventions, so the simulation layer can bind pick/place actions to
  physical objects without re-deriving points.
- `HumanJobSpec.check(spec, targets, body):Array<String>` for edit-time
  warnings: unknown object IDs, a target above reach or below the waist (use
  the same `COMFORT` reach as `ApproachFor`), a `playClip` clip the character
  lacks.

Tests (`humankit/tests`): JSON round trip; every validation error; the
builder's grasp and place points for a rotated `onto` box with an offset;
`walkTo` object stop distance and facing; `check` warnings; a built job run in
pure animation reaches its points (palm within 2 cm).

### M5b — humankit-sim: run a spec

- `HumanWorker.runSpec(spec, targets, objectsById:Map<String, SimObject>)`:
  builds with `HumanJobBuilder`, binds the holds, runs the job. A spec naming
  an object that is not a dynamic `SimObject` fails the job with a message.
- `loop` support: when the job completes and `spec.loop`, rebuild and rerun.
- Rewrite `humankit/sim/tests` pick-and-place scene to use a spec (keep every
  M4 assertion: placement, level, at rest, no jump, zones, separation).
- Add a `both`-hands pick/place test with a wider part.

### M5c — App: the `human-worker` object kind

- `SceneObjectData`: add `@:optional var worker:WorkerObjectData` with
  `asset:String`, `job:String` (the JSON), `zones:Array<String>` (IDs of
  scene objects whose footprints are safety zones). Encode/decode it in
  `SceneCodec` (add the kind to the decoder whitelist), with validation of
  the job via `HumanJobSpec.parse` on load (a bad job loads, but the object
  reports the error; don't make a document unloadable because of a job).
- `HumanWorkerKind implements ObjectKindProvider` registered in
  `ObjectKindRegistry`: section "People", label "Worker", command
  `scene.create-worker`; default record uses
  `animkit/assets/quaternius/worker.glb`, standing at the scene origin. Its
  width/height/depth come from the character's rest bounds (selection box and
  picking).
- Presentation: in edit mode the worker shows its character in the idle pose
  at the object's pose (reuse `HumanCharacter`; see
  `app/src/editor/CharacterPreview.hx` for loading and scene attachment). The
  gizmo moves it on the floor and rotates it about Z only (clamp z to 0 and
  rotation to yaw on edit).
- Inspector (`properties` / `commands`): asset path; zones (multi-select of
  scene object IDs); the job as a list of step groups, each with an action
  dropdown and the fields that action needs (object fields are dropdowns of
  scene object IDs, numbers are numeric fields); commands to add a step,
  remove, and move up/down; the job's parse error or `check` warnings shown
  as read-only text. Edits go through the existing scene object command path
  so undo/redo works — test it.
- Hierarchy shows workers like other objects.

Tests (`app/tests`): codec round trip of a worker with a job; a malformed job
loads with an error reported; creating a worker through the command; editing
a step and undoing it; the inspector lists a dropdown of object IDs.

### M5d — App: simulation, telemetry, example document, migration

- `ApplicationSimulation.rebuild`: build a `HumanWorker` for each
  `human-worker` object (replacing `configuration.humans()`), resolve targets
  from the scene's objects (an adapter implementing `HumanJobTargets` from
  `SceneObjectData` boxes) and `objectsById` from the simulation's created
  objects. Register every simulated robot's links automatically with
  `RobotLinkBounds`, and each listed zone object's footprint as a
  `HumanZone`. Keep the M4 rule that a failed rebuild leaks no characters.
- Telemetry: show per worker the current step, failure (if any), occupied
  zones, and minimum separation per robot (`app/src/editor/TelemetryPanel.hx`).
- Example document `app/examples/worker-rack-to-table.materia` (a new
  folder; documents save as `.materia` through
  `app/src/ProjectDocumentSession.hx`): floor, rack, table, part, the
  one-joint robot arm, and a worker whose job is pick `part` → place onto
  `table`, with zones `rack` and `table`. Produce it by building the scene in
  code once and saving it through the normal save path, so it is exactly what
  the app writes; commit the file, not the generator.
- `--worker-demo=rack-to-table` now loads that document and starts the
  simulation. Delete `WorkerDemoJob`, the scene building in
  `enableWorkerDemo`, and `HumanConfiguration`.
- Migration: a document whose sensor records contain `"humans"` (M4 format)
  loads each as a `human-worker` object at that pose with an empty job and a
  note in its job error text naming the old job name; saving writes the new
  form only. Test it with a fixture document in the old format.
- Rewrite `WorkerDemoTests` around the example document with the acceptance
  assertions above (reuse its settle loop, tolerance checks, zone and
  separation checks, and the no-leak-on-failed-rebuild check).

### M5e — Documentation and screenshots

- `humankit/README.md`: the job format and builder; `humankit/sim/README.md`:
  `runSpec`; `app/README.md`: adding a worker and editing its job.
- Headless screenshots of the example (editor with the worker selected and
  its job in the inspector; carrying; placed) under `xvfb-run` (see the M4
  plan) into a build directory, not the repo.

## Out of scope for M5

Facility (stations/racks/lanes) authoring in documents; path planning and
obstacle avoidance; crouching; look-at; bone-length fitting; exact
capsule-to-shape separation; the safety consumer that slows robots (next
milestone candidate).

## Environment additions since M4

- A fresh worktree's `haxeon` clone lacks its nested submodules
  (`vendor/hashlink`, …): run `fill-nested.sh haxeon
  /home/joao/dev/materia/haxeon` (M4 appendix) before `haxeon/scripts/test.sh`
  or `build-native.sh`.
- Native test executables keep `assert` active in Release (`-UNDEBUG`), so
  tests really run their setup. Don't put large structs on test stacks:
  `rk_simulation_robot_desc` is 1.7 MB, and CMake's default Release `-O3`
  inlines whole test files into `main`; allocate such structs on the heap.
- When checking ctest output, look for `***Exception` (crashes) as well as
  `***Failed`.
- Don't run two heavy builds (the app, haxeon's `scripts/test.sh`) at once: a
  run was OOM-killed (exit 137). Run suites sequentially.
- haxeon lowers enum abstracts to their underlying type; a bare enum
  abstract value resolves from the current package first. If a switch on an
  enum abstract still reports an ambiguous value, fix haxeon (see
  `tests/compiler/SamePackageEnumAbstractValueMain.hx`), don't qualify around
  it.

## Verification before reporting

Run, on the final head, sequentially: `humankit/tests`, `humankit/sim/tests`,
`animkit/tests`, `automationkit/tests`, `robotkit/tests`,
`robotkit/tests/mujoco`, `app/tests`, a build of `app`, and native SimKit and
RobotKit in a fresh default-Release configure (commands in the M4 plan).
Report per-milestone commits and results, the example's placement numbers,
and stop — don't merge to main.
