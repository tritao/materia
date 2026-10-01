# Reference editor shell

Project-owned inspector controls and selection handlers use the [project UI extension protocol](docs/project-ui-extension.md).

This is the app-side integration example for the shared Haxeon UI widgets and
SceneKit. `src/Main.hx` composes:

- a persisted `DockWorkspaceModel` with Hierarchy, Perspective, Inspector,
  Console, and Telemetry panels;
- `TreeView`, the SceneKit perspective viewport, `PropertyInspector`, and `PlotView` panel content;
- one `CommandRegistry` shared by the toolbar, context menu, and command
  palette;
- right-click viewport context actions plus keyboard shortcuts for save, frame,
  palette, and workspace reset;
- a file-backed docking snapshot under `build/reference-editor-workspace.json`
  (override it with `REFERENCE_EDITOR_WORKSPACE`).

The editor starts with an empty scene in a perspective view. Use `--demo` to
load the two starter boxes, the BIM example, and sample telemetry.
Click an object in the viewport or hierarchy to select it; the yellow outline
tracks selection. Edit Name, Position X/Y (metres), Width, Height, Colour
(`#RRGGBB`), or Visible in the inspector.
Hidden objects remain selectable in the hierarchy. Click empty viewport space
or the Scene root to clear object selection.

## Workers in scene documents

Use **Add → People → Worker** to place a worker in the scene. Select it to
edit its asset, floor position, yaw, job, and safety zones in the Inspector.
**Add step**, **Remove step**, and **Move step up/down** edit the ordered job;
each step exposes its action and relevant object, hand, point, or duration
fields. Object fields list IDs already in the scene. Picked parts need a
dynamic collision body. Assign scene boxes as zones to show occupancy in
Telemetry. Press Play to run the worker beside the document's robots;
Telemetry shows its current step, failure, zones, and minimum robot
separation. The job and zones are saved in the `.materia` document.

Open [`examples/worker-rack-to-table.materia`](examples/worker-rack-to-table.materia)
for a complete floor, rack, table, part, robot, and worker. From a built app,
`./app/run-built.sh --worker-demo=rack-to-table` opens this document and starts
the simulation with the arm joint cycling. The demo opens an untitled copy, so
Save asks for a new filename. Older documents with sensor `humans` load those people as
workers with empty jobs; the Inspector shows the former job name so it can be
authored as steps before saving in the new format.

The example's arm cycle is a document `robotMotions` track. Version 1 tracks
name a robot and joint, use increasing `{time, position}` keys starting at zero,
and interpolate linearly in simulation time. A non-looping track holds its last
position; a looping track must end where it starts.

A project can ship such tracks itself: its manifest's `robotMotions` names a JSON file with a `tracks`
list (keyed by assembly joint id, positions relative to the generated initial pose) and an optional
`grips` list of `{time, link, action}` vacuum commands, `grip` or `release`, which repeat with the
looping motion. The manifest's `dynamicParts` lists parts to simulate as free dynamic bodies rather
than bolting them to the assembly. A `grip` holds the free object touching the named link, and a
`release` lets it go.

- The hierarchy's Add menu groups primitive, CAD, feature, and import commands.
  Search filters objects by name or type and includes matching CAD features.
  Duplicate (`Ctrl+D`) and Delete sit beside Add; double-click a tree row to frame
  it. F2 or the object context menu opens Rename; the context menu also offers
  Duplicate, Delete, and Frame. Edits support undo/redo and mark the document as unsaved.
- Create and duplicate select the new object; duplicates retain appearance and
  visibility with a small XY offset. Delete selects a neighbour or the Scene root.
  Undo restores the previous objects, order, and selection; redo retains object IDs.
- Left-drag an object to move it in XY. The initial grab point is preserved, movement
  previews live, and release creates one undo step. Escape cancels the active drag.
- Saving commits an active drag before writing. New, Open, and Close cancel it before
  continuing through the normal unsaved-change confirmation.
- Viewport controls expose Frame, Grid, Snap, and 0.1, 0.2, and 0.5 m spacing.
  The perspective view also exposes Reset and lighting presets.
- Hold Shift while dragging to lock movement to the first dominant axis. Arrow
  keys nudge by 0.1 m; Shift+Arrow nudges by 1 m. Each nudge is independently undoable.
- Positive finite dimension edits rebuild geometry and picking bounds together;
  framing, duplication, undo/redo, and scene persistence use the edited size and colour.
- The Perspective tab renders extruded SceneKit geometry through the GPU.
  Visibility, object colours, depth, and the yellow selection highlight stay
  synchronized with the inspector.
- Left-drag empty space orbits, middle-drag pans, and the wheel zooms. Frame selected
  fits the full 3D bounds and Reset view restores the default camera.
- Perspective clicks use a camera ray against the boxes' 3D bounds. Left-dragging a
  selected box moves it on its current Z plane with snapping and one undo step.
- Undo (`Ctrl+Z`) and Redo (`Ctrl+Shift+Z`) are available in the toolbar and palette.
- The toolbar has two tiers: file, history, document name, and More above; the mode switcher,
  transport, and Frame below. The lower tier is tinted while a simulation is active.
  The transport group drives the shared simulation through `sim.*` commands
  (`editor/SimulationCommands`): Play/Pause (`F5`), Step (`F10`), Reset (`Shift+F5`), and
  Stop, which discards the running simulation and returns to the editor pose. Labels show when the
  toolbar has room, every button has a hover explanation with its shortcut, and a state chip
  (Design, Paused, or Running) sits beside them. Play and Step
  build or rebuild pending configuration first; the Sensors panel buttons call the same commands.
- A Start tab opens beside the 3D view on a plain launch, with new-file shortcuts, recent files,
  and the bundled examples listed in `editor/ExampleCatalog`. `--snapshot --example=ID[,ID...]`
  opens the same examples headlessly, in order, for checks (`--example-settle=SECONDS` lets a
  running simulation step between them). Project examples build on a worker thread: the Start page shows a spinner, the current
  phase and elapsed time with a Cancel button, the rest of the editor stays responsive, and later
  opens reuse the cached build. "Show this page at startup" and the recent-file list are
  stored in `preferences.json` beside the workspace layout.
  Start is a transient panel: it opens at launch (when enabled) or from **Show Start page**, and is never
  saved in a mode's layout or the workspace file, so switching modes or restarting does not bring it back.
- The Console is a read-only text area over a 1000-line log: select with the mouse, copy with
  `Ctrl+C`, scroll with the wheel, or use **Copy all** (`console.copy-all`). It follows new output
  until you scroll away from the end.
- Design (`Ctrl+1`) and Simulate (`Ctrl+2`) modes switch dock layout only; the document,
  selection, and undo history are shared. Each mode remembers its layout for the session,
  Reset workspace restores the active mode's default, and only the Design layout is saved to
  disk. Play or Step enters Simulate, and Design returns to the mode it came from.
- New (`Ctrl+N`) starts a fresh scene; in demo mode it restores the starter objects.
- Open (`Ctrl+O`), Save (`Ctrl+S`), and Save As (`Ctrl+Shift+S`) use native file
  dialogs. The toolbar shows the current filename and `*` for unsaved changes.
- New, Open, and window close ask Save / Discard / Cancel when edits are unsaved.
  Cancelling a chooser or a failed save keeps the current document open.
- Docking preferences remain separate and save automatically. The command
  palette also exposes Save workspace.

`EditorScene` provides the scene API and UIKit `EditorDocument` history.
`editor/SceneModel` owns authored records and structural changes;
`editor/SceneReconciler` stages atomic updates across those records, CAD sessions,
and SceneKit nodes. `editor/ScenePresentation` owns SceneKit state, materials,
face hover, and publication.
`editor/ScenePropertyProvider` builds inspector descriptors, with kind-specific
fields contributed by `editor/ObjectKindRegistry`. `editor/SelectionModel` owns
object, feature, face, and edge selection with a separate revision.
`editor/ObjectKindRegistry` supplies creation defaults, CAD sessions, kind-specific
properties, menu entries, and hover support.

A **Stock simulation** object (Add ▸ Machining) cuts a machining program from a
block of stock with StockKit and shows the result in place of a box. The app has
no CAM workspace yet, so the program is a built-in CamKit demo sized to the
block: an island pocket and four drilled holes, written to G-code and compiled
back with CncKit so every surface maps to a real G-code line. Its inspector has
a timeline (moves cut), colouring by operation or by deviation from the
finished part (leftover yellow, gouge red), rapids through stock, shank
contact, the deepest gouge, and the operation and G-code line of the last
surface clicked in the viewport. The timeline and colouring are view state and
do not enter undo history. `StockSimulationSession` holds the simulation;
`editor/StockSimulationKind` supplies its properties. `EditorSceneTree` caches child lists
by content revision. Inspector bindings retain object identity so undo works
after changing selection. `ProjectDocumentSession` owns the document path and
atomic file publication; `SceneDocumentController` coordinates file commands
and unsaved-change prompts. Opening a script-owned or generated project file
asks before running its registered code. Stale generated-project edits appear
in the console with a discard command. Script override refreshes reconcile
records into the retained scene, preserving viewport and panel state.
`EditorPerspectiveViewport` composites SceneKit's
renderer directly into the GPU surface and passes single-edit change sets to
its incremental render path. `editor/TelemetryPanel` owns the retained plot.
`editor/HierarchyPanel`, `editor/InspectorPanel`, and `editor/SensorPanel`
build the dock content; `editor/SceneObjectCommands`,
`editor/SceneViewCommands`, and `editor/EditorDocumentCommands` register the
corresponding actions. `Main` composes these modules and the desktop host.
The orbit camera lives in `editorkit`; `app/PerspectiveCamera` retains the
existing import names for callers.

The Sensors workspace tab edits RobotKit sensor definitions without exposing
runtime or hardware handles. It supports adding/removing LiDAR and IMU sensors,
selection, identity, update rate, LiDAR ray/range and angular coverage settings,
deterministic noise, link selection, and explicit shared or independent frame
mounts. Sensor changes
participate in document undo/redo and are saved atomically with the scene.
Apply/Rebuild validates and constructs a replacement RobotKit/SimKit runtime
before retiring the current one; a failed rebuild leaves the running
configuration intact. Reset restores the applied physics state without copying
unapplied editor values into it. Applying settings to physical hardware is
never automatic.

Sensor documents retain an independent model for every configured logical
robot. One application-owned simulation compiles all of those models, imports
collision-enabled scene objects as bodies, and attaches `SimulatedRobot`
adapters to the shared `RobotWorld`. Attached robots not owned by that
simulation are treated as remote and read-only. Rebuild replaces all simulated
robots and environment bodies together; pending edits never mutate active
physics, and any validation or construction failure preserves the active world.
Each robot record includes its stable identity, model name, links, joints,
limits, optional actuator, frames, sensors, and explicit world pose. Pending
state covers robot physics edits and collision-environment changes. Names,
colour, visibility, and selection do not require a physics rebuild.

Generated projects with an assembly add its occurrence links to the same
simulation, even when the document has no configured sensor robot. Link mass
and inertia come from each part's generated volume and material density. A
rebuild starts from the saved assembly joint coordinates; simulated part poses
appear in the viewport after stepping or while running, and Stop or Reset
returns to the editor pose. Joint editing in the Inspector is disabled while
simulation is active. Joint couplings use MuJoCo equality constraints and
deterministic enforcement. MuJoCo also compiles loop closures as equality
constraints; the Test backend rejects them with a Simulation panel diagnostic.

Generated parts default to collision enabled. Assembly-owned parts attach one
bounded convex hull to each link. MuJoCo uses the hull for contact; the Test
backend uses an oriented box around the hull, including its offset from the
link origin. A flat or degenerate part gets a box with at least 0.5 mm
thickness and a warning in the Simulation panel. Loose generated parts still
use scene boxes. A single hull fills bores and U-shaped openings; multiple
hulls per link, possibly generated with CoACD, are a separate future decision.
Turning collision off on one part removes its link contact on Rebuild.
Collision and mass overrides remain sparse project field edits; the saved
file schema is unchanged.

Scene rectangles persist an extrusion depth plus independent collision-enabled,
dynamic-body, and mass settings. Visibility affects rendering only. Application
runs default to the bundled MuJoCo backend, while the sensor panel can select
the Test backend used by automated checks; changing backend remains pending until
Rebuild. The perspective viewport overlays read-only runtime robot poses, sensor mounts,
and LiDAR rays. New/Open stops and detaches the previous document's simulated
robots but leaves independently attached remote robots untouched.

## Script-owned setups

Materia setup scripts are ordinary compiled Haxe classes registered in
`SetupScriptRegistry`; they reuse `RobotModel`, `SensorConfiguration`, and the
existing simulation APIs. They are not interpreted and are **not sandboxed**:
registered code runs in the Materia process with the process's normal file,
network, and environment permissions. Evaluation returns configuration data
only—robot topology and poses, stable sensor/frame IDs and mounts, environment
objects, timestep, noise values, and backend. Runtime handles and running
behaviors remain application-owned; behavior code continues to issue commands
through the existing world while simulation is active.
Scripts remain responsible for cleaning up any temporary resources they create
during evaluation. Materia owns and disposes only the configuration placed in
the output object, including partially produced output when evaluation fails.

Open the bundled two-robot example with:

```sh
../haxeon/scripts/haxeon run --project haxeon.json -- \
  --setup-script=materia.examples.two-robot
```

The Sensors panel identifies each selected value as `script` or `override`, reloads the registered
script transactionally, and optionally records stable-ID typed overrides. Overrides cover sensor
acquisition and mounts, robot poses, environment physics/geometry, timestep, and backend. Script
fields are otherwise read-only. Reload or validation failure leaves the active
simulation untouched; successful evaluation becomes pending configuration and
only Apply/Rebuild restarts physics. Undo/redo affects overrides, never script
source. Script-owned scene files persist the script reference, configuration
version, override-contract version, override mode, and overrides—not a competing copy of the evaluated
robots or environment. Renamed or removed IDs are reported as stale overrides.
They can be removed explicitly, while all override changes and reversions remain undoable.
Older numeric rate overrides migrate to the current typed contract when opened;
unsupported future contracts are rejected before replacing the document.
New files also store a versioned package ID, package version, SHA-256 source digest,
and SHA-256 digest of the canonical evaluated configuration. Opening a file with
different compiled script content rejects the replacement and leaves the current
project open. Legacy files acquire these identities when saved. After editing the
bundled example source, regenerate its committed source manifest with
`python3 app/tools/check-script-identity.py --write`; the scripted test command
checks that the manifest matches the source.

The same registry and validation/rebuild path has a headless entry point:

```sh
./haxeon/scripts/haxeon run --project app/scripted/haxeon.json -- \
  materia.examples.two-robot
```

CI should use the authoritative wrapper. Its project places native output in
`app/build/scripted`, the shared ignored application build area, so jobs can cache that
directory and reuse the native dependency build:

```sh
app/scripted/test.sh
```

Scene files are UTF-8 JSON with `format: "materia.scene"`, `version: 1`, and an
`objects` array plus an optional `sensors` robot configuration. Each rectangle stores its stable string `id`, `label`, `type`,
`x/y/z`, `width/height/depth`, collision and dynamics settings, `red/green/blue`,
and `visible` fields. Selection, camera,
undo history, native handles, and docking preferences are not serialized.
The loader validates field types, unique IDs, finite coordinates, positive
dimensions, and color ranges before replacing the scene. Unsupported versions
and malformed files leave the current scene and history intact. Files are
written atomically, and history retains the saved state as an undo boundary.
Older files without physics fields remain compatible through validated defaults.

Run the scene editing, document, and CAD plate regression checks from the repository root:

```sh
./haxeon/scripts/haxeon run --project app/tests/haxeon.json
```

The scripted regression opens and saves the two-robot setup, checks typed overrides,
steps MuJoCo, and replays both robots' sensor frames from MCAP. To also validate
that recording with an independent Python MCAP reader, run:

```sh
app/tests/scripted-mcap-independent.sh
```

The scripted inspector layout and override → Apply → Run → Pause → Reset interaction
are checked headlessly by `app/tests/scripted-inspector/haxeon.json`. To capture the
1320×900 desktop view on an isolated display, run:

```sh
app/tests/scripted-inspector-visual.sh
```

`Main` is a complete desktop host. It initializes NativeKit, creates a resizable
window and GPU surface, routes typed native input into the shared `UiContext`,
and renders from the surface's request-driven frame callback. The main thread
drains the event queue and waits between frames, while resize, scale, surface
loss, shutdown, and callback failures are handled explicitly. Embedders can
still create `ReferenceEditorApp` directly and call `view()`/`submit(frame)`
from another host.

Run the editor with:

```sh
../haxeon/scripts/haxeon run --project haxeon.json
```

Use `--snapshot` after `--` for the headless workspace JSON path, or
`--reset-workspace` to discard the persisted panel arrangement before startup.
Add `--project=/absolute/path/to/materia.project.json --simulate` to a snapshot
run to load a generated assembly, rebuild and start MuJoCo, and print its
running state and generated-part count alongside the workspace snapshot.
Use `--perspective` to activate the GPU viewport tab at launch, including for
deterministic frame captures.

Workspace layout changes are saved by a background worker after 150 ms without
another change. It keeps the latest snapshot when changes arrive quickly and
flushes pending changes before the editor closes. Explicit Save waits for its
workspace snapshot to reach storage. The worker writes a temporary file and
renames it over the workspace file so an interrupted write cannot leave a
partial layout.

Perspective diagnostics report render dimensions, GPU composition mode, CPU
transfer bytes, and render latency in `app-state.json`. SceneKit renders into a
shared GPU image that UIKit composites directly, without an RGBA readback. The
interactive camera uses left-drag orbit, middle-drag pan, and wheel zoom.
Frame selected will fit the selected rectangle (or the whole scene when selection
is empty), while Reset view restores the documented default camera.

The editor has explicit design and simulation presentation modes. Apply/Rebuild
enters simulation mode; both running and paused simulations render and pick the
latest physics poses in perspective. Geometry editing is disabled until
the Design action detaches the simulation. Reset restores the applied poses,
while Save always writes the unchanged design poses.

To connect the editor to a running headless robot process, pass its TCP
endpoint. The editor keeps its own NativeKit event loop and uses RobotKit's
typed client over loopback:

```sh
../haxeon/scripts/haxeon run --project haxeon.json -- \
  --robot=127.0.0.1:17890
```

For a deterministic diagnostic run, render a bounded number of frames and
capture the visual, structural, application, event, and performance state:

```sh
../haxeon/scripts/haxeon run --project haxeon.json -- \
  --capture-dir=build/captures/default --frames=3
```

The 3D viewport smooths geometry edges with multisampling, four samples per
pixel by default or as many as the GPU supports. Choose Off, 2x or 4x in the
viewport options menu; the choice is saved in `preferences.json`. For a run that
should not touch the saved choice, such as a capture, pass `--msaa=SAMPLES`
(`--msaa=1` turns it off). The `sampleCount` in `app-state.json` is the count the
last render used.

The capture directory contains `frame.png`, `ui-tree.txt`, `layout.json`,
`app-state.json`, `frame-metrics.json`, and `events.jsonl`. Paths are resolved
from the app project directory. Native PNG capture currently uses ImageMagick's
`import`; if it is unavailable or cannot access the desktop session, the other
artifacts are still written along with `screenshot-error.txt`.

`frame-timeline.jsonl` records the duration and UI submit/render timing for every
captured frame.

The built-in worker demo uses one MuJoCo session for a rack, a dynamic part,
a walking worker, an assembly table, and a cycling robot arm. For a headless
result after ten simulated seconds, run from the repository root:

```sh
./app/run-built.sh --snapshot --worker-demo=rack-to-table --worker-demo-step=1000
```

The output reports the job, part position, current zone, and robot separation.
Omit `--snapshot` to view it in the editor. The headless capture command is in
`humankit/sim/tests/README.md`; screenshots are generated under that test's
ignored build directory.

To judge a worker's pose by eye instead of by its numbers, capture frames of the rack-to-table
demo at chosen moments (headless, software GL) and lay them out on a contact sheet:

```sh
python3 app/tools/capture-worker-frames.py --seconds 1.3,2,5 --crop 1130,380,1850,680 --out /tmp/worker
```

Moments are wall-clock seconds after launch, frames are 2560x1600, and `--crop` takes a pixel box so the
worker fills each tile. Build the app first. The contact sheet needs Pillow.

For a bounded HashLink CPU/allocation/GC capture with process RSS sampled at
100 ms intervals, run from the repository root:

```sh
python3 app/tools/profile-editor.py --frames 240
python3 app/tools/profile-editor.py --no-profile --frames 240
python3 app/tools/profile-editor.py --no-profile --idle-seconds 10
python3 app/tools/profile-editor.py --no-profile --scenario tab-inspector --cycles 20
```

The output directory contains `editor.hlpc`, `editor.perfetto.json`,
`memory.jsonl`, `frame-timeline.jsonl`, `profiler.log`, and `launch.log`.
Open `editor.hlpc` with `haxeon/.tools/hashlink/hlprof-live report` or view the
Perfetto file as a timeline. Profiling adds frame overhead, so compare timings
across profiled runs rather than treating them as normal frame rates. Pass editor
options after `--`, for example `-- --perspective`.

The idle mode stops its isolated editor process after the requested interval;
it reports RSS and CPU changes over the final five seconds. Captured frames also
record view, tree/style, native layout, reconciliation, custom paint, and native
render durations, along with per-frame style resolutions, cache hits and misses,
and changed-node counts.

The timed desktop capture keeps normal event-driven frame scheduling. The
`tab-inspector` scenario runs without a display server: it submits the real
editor UI, clicks Hierarchy and Sensors through `UiContext`, edits the inspector
field, and checks the committed name `Profile box`. It records each action and
submit in `actions.jsonl` and `frame-timeline.jsonl`. These headless timings
include UI tree construction and native layout, but no window or GPU rendering.

The command prints per-action frame and tree/style medians, node counts, and
style-cache misses.

UIKit's reusable Component Lab can be opened through the same desktop host:

```sh
../haxeon/scripts/haxeon run --project haxeon.json -- --lab
../haxeon/scripts/haxeon run --project haxeon.json -- \
  --story=text-field/editing --capture-dir=build/captures/text-field --frames=3
```

Applications can supply their own `ComponentStoryRegistry`; the built-in lab
catalog is used when no registry is provided.

The project metadata lives in `haxeon.json`; the UI showcase compiler command
is the reference for supplying the shared UI and FFI source roots when building
this app against the repository checkout.

## Judging a job by eye

`python3 app/tools/worker-event-sheet.py --out /tmp/events --crop 700,300,1700,1000` runs the rack-to-table job headless with a trace
(`--snapshot --worker-demo=rack-to-table --worker-demo-step=N --worker-demo-trace`), reads when each action began, and captures a frame a moment
after each with its name on the tile, in a contact sheet. `--asset animkit/assets/quaternius-ual/ual-work.glb --surface 0.6` makes a temporary variant
with the library character and lower tops (the example is put back afterwards). `capture-worker-frames.py` takes frames at moments you choose.

## The worker gallery

`./app/run-built.sh --worker-demo=gallery` (or *Worker gallery* on the Start page) opens `app/examples/worker-gallery.materia`: six workers side by side, 6 m apart, each
with a rack, a table and a part, running a short document job in the one simulation. The lanes are the cases the worker is built for: the bundled
worker at table height (the rack-to-table demo as it was); the library character bending over 0.8 m deep tops; crouching to a bench at 0.6 m; kneeling to
a shelf at 0.3 m; using the left hand and turning right round to a table behind it; and using both hands for a long part and then pressing a panel. The file is written
by `app/tools/make-worker-gallery.py` (`--check` says whether it is up to date); `--snapshot --worker-demo=gallery --worker-demo-step=N` runs it headless and prints each
worker's result and the tick it finished at, and `WorkerGalleryTests` holds all six to finishing with the part on its table.
