# Reference editor shell

This is the app-side integration example for the shared Haxeon UI widgets and
SceneKit. `src/Main.hx` composes:

- a persisted `DockWorkspaceModel` with Hierarchy, XY Viewport, Perspective, Inspector,
  Console, and Telemetry panels;
- `TreeView`, `GpuViewport`, `PropertyInspector`, and `PlotView` panel content;
- one `CommandRegistry` shared by the toolbar, context menu, and command
  palette;
- right-click viewport context actions plus keyboard shortcuts for save, frame,
  palette, and workspace reset;
- a file-backed docking snapshot under `build/reference-editor-workspace.json`
  (override it with `REFERENCE_EDITOR_WORKSPACE`).

The editor starts with two extruded SceneKit boxes in an orthographic XY view.
Click an object in the viewport or hierarchy to select it; the yellow outline
tracks selection. Edit Name, Position X/Y (metres), Width, Height, Colour
(`#RRGGBB`), or Visible in the inspector.
Hidden objects remain selectable in the hierarchy. Click empty viewport space
or the Scene root to clear object selection.

- Add rectangle, Duplicate (`Ctrl+D`), and Delete are available in the hierarchy
  panel, viewport context menu, and command palette. Rename through the inspector's
  Name field. These edits support undo/redo and mark the document as unsaved.
- Create and duplicate select the new object; duplicates retain appearance and
  visibility with a small XY offset. Delete selects a neighbour or the Scene root.
  Undo restores the previous objects, order, and selection; redo retains object IDs.
- Left-drag an object to move it in XY. The initial grab point is preserved, movement
  previews live, and release creates one undo step. Escape cancels the active drag.
- Saving commits an active drag before writing. New, Open, and Close cancel it before
  continuing through the normal unsaved-change confirmation.
- Optional grid snapping is available from the viewport context menu with 0.1,
  0.2, and 0.5 m spacing choices.
- Hold Shift while dragging to lock movement to the first dominant axis. Arrow
  keys nudge by 0.1 m; Shift+Arrow nudges by 1 m. Each nudge is independently undoable.
- Positive finite dimension edits rebuild geometry and picking bounds together;
  framing, duplication, undo/redo, and scene persistence use the edited size and colour.
- The Perspective tab renders the same extruded SceneKit geometry through the GPU.
  Visibility, object colours, depth, and the yellow selection highlight stay
  synchronized with the XY view, simulation, and inspector.
- Left-drag empty space orbits, middle-drag pans, and the wheel zooms. Frame selected
  fits the full 3D bounds and Reset view restores the default camera.
- Perspective clicks use a camera ray against the boxes' 3D bounds. Left-dragging a
  selected box moves it on its current Z plane with the same snapping and undo rules
  as the XY viewport.
- Undo (`Ctrl+Z`) and Redo (`Ctrl+Shift+Z`) are available in the toolbar and palette.
- New (`Ctrl+N`) starts a fresh scene with the two starter objects.
- Open (`Ctrl+O`), Save (`Ctrl+S`), and Save As (`Ctrl+Shift+S`) use native file
  dialogs. The toolbar shows the current filename and `*` for unsaved changes.
- New, Open, and window close ask Save / Discard / Cancel when edits are unsaved.
  Cancelling a chooser or a failed save keeps the current document open.
- Docking preferences remain separate and save automatically. The command
  palette also exposes Save workspace.

`EditorScene` owns the SceneKit scene, published snapshot, spatial index, and
UIKit `EditorDocument` history. `EditorSceneTree` and `EditorSceneViewport`
consume that state. Inspector bindings retain object identity so undo works
after changing selection. `SceneDocumentSession` owns the document path and
atomic file publication; `SceneDocumentController` coordinates file commands
and unsaved-change prompts. The XY viewport provides a canvas projection of the
boxes, while `EditorPerspectiveViewport` composites SceneKit's renderer directly
into the Perspective tab's GPU surface.

The Sensors workspace tab edits RobotKit sensor definitions without exposing
runtime or hardware handles. It supports adding/removing LiDAR and IMU sensors,
selection, identity, update rate, LiDAR ray/range settings, deterministic noise,
link selection, and explicit shared or independent frame mounts. Sensor changes
participate in document undo/redo and are saved atomically with the scene.
Apply/Rebuild validates and constructs a replacement RobotKit/SimKit runtime
before retiring the current one; a failed rebuild leaves the running
configuration intact. Reset restores the applied physics state without copying
unapplied editor values into it. Applying settings to physical hardware is
never automatic.

Sensor documents retain an independent model for every configured logical
robot. One application-owned simulation compiles all of those models, imports
visible scene rectangles as static obstacles, and attaches `SimulatedRobot`
adapters to the shared `RobotWorld`. Attached robots not owned by that
simulation are treated as remote and read-only. Rebuild replaces all simulated
robots and environment bodies together; pending edits never mutate active
physics, and any validation or construction failure preserves the active world.
Each robot record includes its stable identity, model name, links, joints,
limits, optional actuator, frames, sensors, and explicit world pose. Pending
state covers both robot edits and scene-content or document replacement;
selection alone does not require a physics rebuild.

Scene rectangles persist an extrusion depth plus independent collision-enabled,
dynamic-body, and mass settings. Visibility affects rendering only. Application
runs default to the bundled MuJoCo backend, while the sensor panel can select
the deterministic backend used by tests; changing backend remains pending until
Rebuild. The XY viewport overlays read-only runtime robot poses, sensor mounts,
and LiDAR rays. New/Open stops and detaches the previous document's simulated
robots but leaves independently attached remote robots untouched.

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

Run the scene editing and document regression checks from the repository root:

```sh
./haxeon/scripts/haxeon run --project app/tests/haxeon.json
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
Use `--perspective` to activate the GPU perspective tab at launch, including for
deterministic frame captures.

Perspective diagnostics report render dimensions, GPU composition mode, CPU
transfer bytes, and render latency in `app-state.json`. SceneKit renders into a
shared GPU image that UIKit composites directly, without an RGBA readback. The
interactive camera uses left-drag orbit, middle-drag pan, and wheel zoom.
Frame selected will fit the selected rectangle (or the whole scene when selection
is empty), while Reset view restores the documented default camera.

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

The capture directory contains `frame.png`, `ui-tree.txt`, `layout.json`,
`app-state.json`, `frame-metrics.json`, and `events.jsonl`. Paths are resolved
from the app project directory. Native PNG capture currently uses ImageMagick's
`import`; if it is unavailable or cannot access the desktop session, the other
artifacts are still written along with `screenshot-error.txt`.

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
