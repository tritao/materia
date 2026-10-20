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
tracks selection. A completed viewport click selects the hit part directly; repeated
clicks keep that part selected. Select an assembly in the tree to select its members.
Picking in 3D also highlights the corresponding Hierarchy row,
opens its ancestor groups and scrolls it into view without moving keyboard focus.
An active tree search is preserved, with the selected row included so it remains
reachable. Redraws preserve scroll position and manual branch expansion.
Generated assemblies are organized by their authored subassemblies. Each part
occurrence appears once under its owner; rigid bodies and moving joints do not
reparent parts. Assemblies show their part count and have an inspector with
Frame, Hide/Show, and Isolate actions. A part row acts on that occurrence only.
Double-click an assembly or part to frame it. Joint controls remain in **Machine**.
Older exports without ownership metadata appear under their root assembly until
regenerated. Saved visibility from the former body/Parts hierarchy is preserved
as individual part visibility; **Show all** clears it.

Edit Name, Position X/Y (metres), Width, Height, Colour
(`#RRGGBB`), or Visible in the inspector.
Use the eye at the right of a Hierarchy row, or press **Space** while that tree
row has keyboard focus, to show or hide it. Hidden rows stay selectable and use
muted labels; an open eye on a muted row means its own visibility is enabled but
an object or parent mask is hiding it. A dash in an open group eye indicates
that some descendants are hidden. Group changes include filtered-out and
collapsed descendants, preserving each child's own Visible setting and nested
group state. The search field and other text inputs keep their normal Space behavior.

The viewport toolbar's **Visibility** button opens the same hierarchy as a
checkbox popup. It stays open while toggling components and has its own expansion
and scroll state, independent of the scene tree's search and selection. Mixed
checkboxes indicate partially visible groups. Expand an assembly to control its
children; showing a hidden parent preserves child settings. **Show all** and
**Isolate selected** are also available in the popup. Escape or an outside click
closes it. Visibility controls remain available during simulation.

Right-click an object or group for **Isolate selected** or **Show all**. Isolation
preserves hidden descendants when the selected row is a group; Show all clears
all visibility masks and enables every object's Visible setting. These actions
are undoable and saved with ordinary scenes and generated project overrides.
Visibility affects rendering and picking, independently of collision or simulation.
Click empty viewport space or the Scene root to clear object selection.

The focused visibility regression suite includes routed mouse/keyboard controls,
nested and filtered groups, undo, and scene/generated-project save/open:

```sh
./haxeon/scripts/haxeon run --project=app/tests/haxeon.visibility.json
```

Perspective rendering uses the displayed viewport dimensions for the camera,
overlays and pointer input. The GPU texture is capped at 2048 pixels per axis;
that resolution limit does not change the camera aspect ratio. Window and dock
resizes refresh the image even when the capped texture dimensions stay the same.
The focused resize/input regression covers wide, tall and fractional layouts,
selection, hover, dragging and cursor-anchored zoom:

```sh
./haxeon/scripts/haxeon run --project=app/tests/haxeon.viewport-coordinates.json
```

Studio lighting uses a directional key with restrained fill and a darker lower
hemisphere so recess walls read distinctly. The perspective viewport adds subtle
screen-space ambient occlusion within 25 mm and broadens highlights with a
studio roughness floor. The Soft preset reduces occlusion and softens highlights
further. These are display settings; authored part materials are unchanged.
Collision and near-distance indicators use an RGB tint below selection and hover.
They retain each surface's material, opacity, alpha mask and textures, including
cosmetic thread meshes. Clearing an indicator restores the original colour.

Occlusion affects indirect light on opaque geometry, preserves transparent
backgrounds, and ignores blended/masked surfaces as occluders. It uses visible
depth, so hidden surfaces and subpixel recesses cannot contribute.

CAD display defaults to **Feature edges**. The viewport's edge-display button cycles
through **All edges**, **Shaded**, and **Feature edges**. Feature display suppresses
an edge only when both sides of its tessellated boundary have matching surface
normals (within one degree); uncertain, open, sharp and non-manifold boundaries
stay visible. All edges restores every original edge ID for inspection. The
setting changes the display only, including newly published CAD geometry, and
does not edit the document, CAD shape or simulation geometry.

The focused regression covers smooth splits, sharp/open boundaries, a cylinder's
periodic seam and end rims, restored selection IDs and scene revision isolation:

```sh
./haxeon/scripts/haxeon run --project=app/tests/haxeon.feature-edges.json
```

## Machine travel controls

Opening an assembly with movable joints opens the **Machine** tab beside Inspector.
Use a bounded axis slider or enter a position and press **Set** or Enter. **Min**
and **Max** move to the authored travel stops; **Initial** restores that joint's
initial coordinate. Distances display in millimetres and angles in degrees,
independently of the assembly's source units. Rotary joints have a dial and an
endless horizontal scrubber linked to numeric entry. The dial chooses the nearest
orientation on click and accumulates turns while dragged through zero. Drag the
scrubber horizontally to rotate by any number of turns; hold Shift on either
control for one-tenth sensitivity. Continuous rotary controls display orientation
from 0° up to 360° (360° wraps to 0°), while joint coordinates retain accumulated
turns. Numeric entry normalizes angles such as 450° to 90° and chooses the nearest
equivalent orientation. Bounded rotary joints retain their authored travel limits. Press Enter
or Apply for an exact angle, and Reset to restore the initial angle. Edits update the complete assembly and support Undo/Redo. Slider drags
preview live and record one undo step on mouse release; cancellation restores
the starting position without adding history. Coordinate edits reuse the compiled
assembly topology, while keeping each preview configuration independent.

**Inspect** beside an axis frames its moving part and opens a section through it.
The section cuts away the near half of the view to expose internal movement.
**Flip side** reverses the cut; **Close section** restores the complete model.
Inspection affects the view and picking only; it does not alter geometry, joint
limits, visibility settings or Undo history.

Coupled screw/pulley joints and closure-dependent joints follow their driving axes.
Manual editing is disabled while simulation or a script owns the machine. These
are design-position controls, not live machine commands or a collision interlock.

```sh
./haxeon/scripts/haxeon run --project=app/tests/haxeon.machine-control.json
```

## Mission monitor

In Simulate, projects with an authored mission open the Mission sidebar tab. It shows
startup, step progress, loop count, completion, and failures, including progress while
paused. Select a step to inspect its target part and connector. The list follows the
current step until you scroll away; **Follow current step** resumes following. Use the
shared Play/Pause, Step, Reset, and Stop controls to run the simulation. Mission steps
are read-only here.

## Project configurations and views

Project variants select complete generated scenes independently of runtime missions.
Use **Configuration**, **Study**, and **Demo** in the viewport card or **Project**
panel. These selectors are separate from the Mission panel and never create a
mission or playback status for a plain machine. Open the source `.project.json`;
a prebuilt `.mtrg` contains only the selected geometry and cannot regenerate another
configuration. Source selection is locked while simulation is running or paused.

```json
"defaultVariant": "small",
"variants": [
  {"id": "small", "label": "Small", "kind": "configuration",
   "summary": "Small machine", "entrypoint": "small"},
  {"id": "compact-vmc", "label": "Compact VMC", "kind": "configuration",
   "summary": "Larger machine study", "entrypoint": "compact-vmc"},
  {"id": "spindle", "label": "Spindle drive", "kind": "study",
   "summary": "Motor study", "entrypoint": "spindle-drive"}
]
```

IDs must be unique, entrypoints must exist, and `defaultVariant` must name a variant.
Kinds are `configuration`, `study`, or `demo`. Only one scene variant is installed
at a time: selecting a study does not imply that it inherits the selected machine
configuration. Empty inactive selectors make that distinction visible.

The project controller builds replacements in the background with Cancel. Failure,
cancellation, document changes and simulation locks preserve the installed scene
and selection. Saved `.materia` documents store `variantId` in project-record version
2; older applications reject that version instead of silently loading the default
geometry. Portable artifact metadata also retains the selected variant.

Existing `jobs`/`defaultJob` manifests, including the robot welder's seam presets,
remain supported and appear as **Scene preset** in the Project UI. Runtime mission
steps stay in Mission. Legacy `jobId` documents retain their format; if their source
has migrated to variants, reopening regenerates the matching variant and the next
save writes version 2. A manifest must not mix `variants` and legacy `jobs` lists.
Projects with neither keep their `defaultEntrypoint`.


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
`release` lets it go. Track positions for a prismatic joint are in metres. A joint's
`overtravel` (assembly joint limits) is how far it can pass its limits before meeting its end stop:
the simulation puts the stops there and faults only beyond them. A joint whose assembly gives none
gets 1 mm or 1 degree, so one parked on its limit does not fault on numerical noise.

A project whose assembly is a machine can give it a CNC job in the manifest's `cnc` block:
`{"program": "<file>.ngc", "workOffset": [x, y, z], "axes": ["x", "y", "z"], "loop": true}`. The
program is LinuxCNC G-code; `axes` names the prismatic joints that are the machine's X, Y and Z
(their assembly coordinates are machine coordinates, and their limits need velocity and
acceleration); `workOffset` is G54 in machine coordinates, in the assembly's unit. The simulation
compiles the program with CncKit and streams it to the machine through MotionKit
(`CncProgramPlayer`); a compile or travel error fails the simulation build.

In the 3D viewport, a completed left click selects the outer group under the pointer.
Repeated clicks at the same point descend one hierarchy level at a time, stopping at the part;
clicking elsewhere or changing the view starts again at the outer group. A drag moves the part
without drilling. The Scene tree expands ancestors and scrolls to the selected row. Space in the
viewport or tree toggles the selection's visibility using the eye button's undoable operation;
a hidden part or group stays selected so a second press shows it again. Holding Space does not
repeat the toggle, and typing in a text field does not change visibility.

The hobber's CNC pane shows cycle metrics and inspection qualification before profile and
lead charts. Charts stack in narrow panes and sit side by side from 760 logical pixels.
Sampling, coverage, assessment warnings, load assumptions and run details expand independently.
The raw-sample table previews the first tooth on both flanks in pages of 24 rows; **Copy all
samples as CSV** includes every inspected tooth, both trace families, positions in mm and raw
and filtered deviations in µm. Opening details does not rerun stock removal or inspection.

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
- Middle-drag or Alt+left-drag orbits; add Shift to pan. The wheel zooms toward the cursor. Orbit starts at the surface under the cursor
  and holds that pivot through the drag without recentering the view. Empty-space
  drags retain the previous pivot. Frame selected
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
  and the bundled examples listed in `editor/ExampleCatalog`. Examples have task category filters
  and search across their titles and variant descriptions. Related welding, cobot and mill examples
  appear under family headings on the same page. Every variant has its own card: click anywhere
  on it to open the example directly. Search shows only matching variants, and card rows adapt to
  the dock pane width. The three most recent existing files stay above the catalogue. `--snapshot --example=ID[,ID...]`
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
  dialogs. The toolbar uses its available width for the full document path (or
  generated source path) and `*` for unsaved changes. Paths that do not fit are
  shortened from the left, preserving the filename; hover or focus reveals the
  complete path in an overlay only while the inline label is truncated.
- New, Open, and window close ask Save / Discard / Cancel when edits are unsaved.
  Cancelling a chooser or a failed save keeps the current document open.
- Docking preferences remain separate and save automatically. The command
  palette also exposes Save workspace.

Project loading runs the build and fingerprint tools as cached HashLink executables.
The generator uses Haxeon's reusable compiler session, and its compiled code is reused
across opens and editable recipe changes. Deterministic entrypoints opt into geometry
caching with `"cache": {"inputs": [...]}`; list every external file that affects their
output. The Gantry picker, yaw picker, and welder enable this cache. Source contents,
manifests, compiler options, declared inputs, and tool/native library metadata invalidate
cached artifacts. Recipe edits always execute the generator with the new document.
Caches live below `${XDG_CACHE_HOME:-$HOME/.cache}/materia`, in `project-tools` and
`generated-artifacts`; stable generated entrypoints live in `project-sources`.
Compiler workers use Haxeon's existing idle timeout and resident limits. Cancelling an
open closes its helper connection, cancels that compiler transaction, and preserves the
previous published bytecode. Unsupported hosts and `HAXEON_COMPILER_SERVER=0` use the
one-shot driver.

Desktop opens emit a `project-open-profile` JSON line after the first successful frame
render. It reports Open-to-frame time, fingerprinting, compilation, generation, artifact
decoding, geometry/physics preparation, scene installation, and first rendering. Physics
preparation is also measured within the geometry phase. The frame endpoint is CPU render
completion, not monitor scan-out. Capture a real desktop open with:
`./app/run-built.sh --project=/absolute/path/materia.project.json --capture-dir=/tmp/materia-open --frames=3`.
The first compiler/tool bootstrap is included when those caches are empty.

Hashing uses Haxeon's shared C SHA-256 kernel on native and both browser Wasm
backends. The standard-Haxe project helpers call the same kernel through
`build.execution.ContentDigest`; cache identity still hashes the complete content.

Artifact decoding borrows mesh streams from the immutable snapshot through
`SceneArtifact.decodeView`. Read-only assembly validation uses `flattenView`,
avoiding JSON round-trip copies of already-flat definitions. Editable callers
retain the independent-copy `decode` and `flatten` APIs. Native and browser decoder
measurements are documented in [the decoding benchmark](../projectkit/tests/decoding/README.md).

Derived preparation is cached separately in `materia/prepared-scenes`. Each `.mtrp`
entry contains packed vertex streams, original bounds, mass properties, and collision
hulls. Bounds and physics values are stored as binary float64, preserving exact values.
Its identity includes the source artifact SHA-256, format/preparation settings, and
the compiled application build identity. The compiler emits the identity beside the
module as `.build-id`; loading never scans application source files.
Native geometry handles and editable state are
created anew for every open. Entries have a checksum, bounded decoding, and atomic
publication; missing, stale, or corrupt entries are regenerated from the artifact.
Preparation caching also applies to editable generators without caching their execution:
new artifact bytes select a different entry. Removing `prepared-scenes` is safe.
Bump `PreparedProjectCache.VERSION` when changing its format or preparation conventions.

Native and browser opening share `ProjectArtifactLoader`: it accepts `.mtrg` bytes
and an optional `PreparedSceneStore`, decodes the artifact, obtains portable prepared
data, and creates fresh runtime geometry. `ProjectScenePreparation` owns the shared
geometry/physics calculations. `ProjectSourceLoader` selects an explicit prebuilt
`.mtrg` source or the native project generator; compilation and manifest policy remain
outside materialization. The document installs a completed result after loading succeeds.
A saved project can reference a prebuilt artifact without executing source code.

Assembly loading publishes a `cadkit.modeling.CompiledAssembly`: immutable topology,
lookup tables and compiled kinematics. The artifact loader transfers its freshly
decoded definition into that model; scene installation and configuration edits reuse
it through `AssemblyState.fromModel`. Each state owns its joint values, root poses
and evaluated pose buffers. Authored topology changes create a new model. Existing
`new AssemblyState(editableDefinition)` callers keep independent snapshot semantics.
The shared definition's nested records and arrays must remain read-only, matching
`kinematicskit.KinematicModel`'s ownership contract.


In the browser, use File > Open, drop a `.mtrg` onto the viewport, or call
`await window.materia.openArtifact(file)` with a File, Blob, ArrayBuffer or Uint8Array.
The imported artifact is retained under `/files/artifacts/<content hash>/<name>`;
prepared data lives under `/cache/materia/prepared-scenes`. The virtual filesystem
mirrors both to the origin private file system (OPFS), so a reload can reuse them.
Unavailable storage still permits loading. Cache entries are disposable; corrupt
entries are rebuilt, and a different application build selects a new cache key.
Native cache policy instead uses the user's filesystem cache directory.

Browser opening consumes prebuilt artifacts; source compilation and recipe
regeneration still require the native toolchain. Artifact metadata is retained,
but manifest-only supplements cannot be inferred from artifact bytes.

Browser artifact opens reuse validated cached data on the editor thread. Cache
misses run artifact decoding and geometry/physics preparation in a reusable
dedicated Web Worker. Its small Wasm guest has private memory and no
native-library, rendering, or storage imports. A versioned job protocol transfers
binary buffers; `PreparedSceneCodec` is shared by worker results and disk caches.
The page owns persistence, and the editor validates completed data before creating
runtime geometry and installing the scene. Cancellation terminates busy work and
preserves the previous scene; the next request recreates the worker. Successful
jobs reuse it. The cache identity includes both editor and preparation builds.
No pthreads or shared Wasm memory are required. Transfers between JavaScript
contexts move ownership, while staging between the editor and worker Wasm memories
still requires copies. This removes the long preparation pause from the UI; final
materialization and installation remain on the editor thread.
Browser profiles end at CPU render completion, as native profiles do.

Check both browser backends by building with `MATERIA_WEB_TARGET=wasm-gc` or
`MATERIA_WEB_TARGET=wasm32`, then running
`app/web/test.sh --artifact /path/to/fixture.mtrg`. This checks actual rendering,
warm-cache reuse, File > Open, OPFS persistence after reload, corrupt-cache repair,
and failed replacement preserving the current scene. Measurements and native fixture
instructions are in [the project loading checks](tests/project-loading/README.md).

The application and robotd CMake aggregates declare their required libraries with
`native.cmake.libraries`; build completion is checked against those real files,
without synthetic `.hdll` markers.

Run the loading/cache integration checks with
`./haxeon/scripts/haxeon run --project=app/tests/project-loading/haxeon.json`.
Add `-- --gantry` to compare first and repeat loads of all three Gantry variants.

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
The orbit camera lives in `scenekit`; `app/PerspectiveCamera` retains the
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
interactive camera uses middle-drag orbit, Shift+middle-drag pan, and cursor-centered wheel zoom.
Orbit chooses the visible surface under the cursor on press, preserving its screen
position and the current zoom. The pivot stays fixed for the drag and its ring is
drawn at that surface point. A drag starting in empty space retains the previous
pivot; framing, fitting, standard views and Reset view restore the camera target as
the fallback pivot.
Alt+left-drag (with Shift for pan) is available without a middle button. Left input selects and edits.
Hold the right mouse button for mouse-look and WASD flight; Q/E move down/up and Shift increases speed.
Release the right button or press Escape to stop flying. Movement also stops when the window loses focus.
Press F to frame the selection and set its center as the orbit pivot; Home or the Fit button fits all visible objects.
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
a shelf at 0.35 m; using the left hand and turning right round to a table behind it; and using both hands for a long part and then pressing a panel. The file is written
by `app/tools/make-worker-gallery.py` (`--check` says whether it is up to date); `--snapshot --worker-demo=gallery --worker-demo-step=N` runs it headless and prints each
worker's result and the tick it finished at, and `WorkerGalleryTests` holds all six to finishing with the part on its table.

Browser Start pages use the same example catalog as desktop. Only a small index
loads at startup; selecting an example acquires hash-verified prebuilt project or
scene files and required animation assets. Downloads support cancellation and retry,
and OPFS retains both source artifacts and prepared scenes for later opens. The
native build publishes the downloadable files; see [browser examples](web/README.md#examples-and-local-storage)
for hosting, bundle reuse and checks.

## Inspection camera

In Simulate mode, the 3D toolbar's **Camera** button opens an independent
inspection view below the right-hand Sensors/CNC tabs. The camera pane and
button are hidden in Design; returning to Simulate restores its layout.
Drag its tab into another dock group; the workspace remembers its placement
for the Simulate session. **Picture in picture** moves that same view
into the main viewport, and **Dock** returns it to a pane. `--inspection-camera`
requests the pane when Simulate mode is entered (with `--simulate`, at launch).

Scene-authored presets choose a target, framing and camera-local hidden parts.
The camera follows target translation while keeping its orientation steady when
the part rotates. Use the mouse wheel over the inspection image or the zoom
buttons. Orbit and pan use the regular 3D viewport gestures; Reset restores the
preset at the target’s current position. Navigation stays independent of the
main view and cannot select or edit parts. Zoom changes the lens rather than
moving into the geometry; wheel
events stay within the inspection view, including at its zoom limits.
Both views show the same live simulation and stock; inspecting does not change
selection, document visibility or history. Scenes without presets can inspect
the selected part.

Generated scenes may supply `project.inspectionCameras`: unique `id`, `label`,
occurrence `anchor`, world-aligned `offset` in metres from its displayed centre,
`span` in metres, `yaw`/`pitch` in radians, and `hiddenPrefixes` of occurrence ids.
These presets are optional and use the existing scene artifact format.


## Unified hobber scene

`machinekit/examples/flexspline-hobber/hobber.scene.json` opens one enclosed
machine with a detailed tool and cup. The viewport's Play scenario controls
select Idle / doors, Cutting (default), or Load / cut / unload. Pause before
changing scenarios; selecting another resets the runtime and stock without
changing the document. Play drives the selected scenario through measured guard
feedback. Full-cycle transfers require open/unlocked doors and standstill; the
actual cut requires closed/locked feedback. Faults never restart a cycle.

The adjacent Visible / Transparent / Hidden controls are presentation only.
Physical bodies, switches and guard-lock logic remain installed, and the
inspection camera keeps its own cutaway visibility.
A CNC job may carry `machining.mission: {id, label}`. This names a finite,
single-program machining task executed by the existing CNC player. Its Mission
view follows the player's real state, including feed hold, failure and completion;
it does not create robot steps or a second executor. Named machining missions
require `loop: false` (or omitted). Existing unlabelled CNC demos remain compatible.
`machining.kinematicPreview: true` explicitly opts a packaging model into ideal
motion with authored positive axis velocity and acceleration limits. Travel and
stock/tool geometry checks still apply; motor-load plan checks and drive-error
allowances do not. The Mission view displays this mode. Omitting the flag retains
the existing drive-qualified CNC behavior.
