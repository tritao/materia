---
title: The workspace
description: The editor window, its toolbar and status bar, the dock layout, and what every panel shows.
status: draft
---

# The workspace

The editor window has a two-row toolbar at the top, a status bar at the bottom,
and between them a dock of panels that you can rearrange.

## The toolbar

**Top row (file).** From left to right: **New**, **Open**, **Save**, **Undo**,
**Redo**, the document path, and **More**.

- The document path shows the open file, or the project manifest for a
  generated scene, with `*` when there are unsaved changes. A path too long for
  the bar is shortened from the left; hover over it to see all of it.
- **More** opens a menu with Save As, Export STEP, Undo, Redo, Frame selected,
  Fit all, Reset perspective view, the three lighting presets, Toggle grid,
  Toggle simulation overlays, Toggle dark theme, Show Start page, Open command
  palette, Editor Settings..., and Reset workspace.

**Bottom row (mode and simulation).** From left to right: the **Design** and
**Simulate** mode buttons, the transport controls (**Play** or **Pause**,
**Step**, **Reset**, **Stop**), a state chip, and **Frame**. Hover over a
transport button to see what it does and its shortcut. The chip reads
**Design**, **Paused** or **Running**, or the mission's state while one runs,
with "rebuild pending" when the scene changed since the simulation was built.
The whole row is tinted while a simulation exists. See
[Simulating](simulate.md).

While a project or a simulation is being prepared, the bottom row is replaced
by a progress line with a spinner, the phase, the elapsed time and **Cancel**.

On a narrow window the buttons lose their labels (below about 940 pixels) and
then Undo and Redo move into **More** (below about 630 pixels).

## The status bar

From left to right:

- **Selection or problem.** The selected object ("Gantry carriage selected"),
  or "Ready". It shows instead, in this order of priority: the number of stale
  project edits, the running mission's step, a simulation error (in red), or
  the project being opened.
- **Mates**, when the open assembly has mates: whether they are satisfied,
  in red when they conflict.
- **Grid and snap**, on wide windows: "Grid 0.2 m · Snap Off".
- **Runtime**: Design, Paused or Running (with "Rebuild pending"), the physics
  backend (MuJoCo or Test backend), and the robot world's state (World
  Offline, Connecting, Ready or Fault).

## Modes

**Design** (Ctrl+1) and **Simulate** (Ctrl+2) are two dock layouts over the
same document. Switching changes which panels are shown and where; the
document, selection and undo history stay the same. Play switches to Simulate
for you, and Stop switches back. See
[The editor and projects](../concepts/platform/editor-and-projects.md#modes-and-play).

**Design layout:** the left column holds Hierarchy, BIM and Sensors as tabs;
the 3D view is in the middle with Inspector and Machine as tabs to its right;
Console and Telemetry are tabs along the bottom.

**Simulate layout:** a wide 3D view with Console and Telemetry below, and one
tab group on the right: Mission (when the project has one), Sensors, CNC,
Machine, Inspector and Hierarchy. The Machine tab is in front when the machine
has movable joints, otherwise Mission, otherwise Sensors.

## Rearranging panels

- Drag a tab onto another panel to dock it there: onto the middle to add it as
  a tab, onto an edge to split, or between tabs to reorder.
- Drag the divider between two panels to resize them.
- When a tab group is too narrow, tabs other than the active one show only
  their icon.
- **Close panel** in the command palette closes the active panel, if it can be
  closed.
- **Reset workspace** (in **More** or the palette) restores the current mode's
  default layout.

Each mode remembers its layout for the rest of the session. Only the Design
layout is saved, to `build/reference-editor-workspace.json`, a moment after
each change and when the editor closes; **Save workspace** in the palette
writes it at once. The Start page and the inspection camera are never saved in
a layout.

## Panels

### Start

The Start page: new file, recent files, and the example catalog. See
[Starting the editor](start-and-projects.md#the-start-page).

### Hierarchy

The scene tree, under the heading **SCENE**.

- **Add** opens a menu of things to add, in sections: Primitive (rectangle),
  CAD (empty CAD part, mounting plate, L bracket), Machining (stock
  simulation), People (worker), Feature (sketches, extrusion, sheet base and
  flange, hole, pocket, vertical fillet) and Import (STEP part). Items that do
  not apply to the current selection are greyed.
- **Duplicate** and **Delete** act on the selected object.
- **Search objects...** filters the tree by name or type, including CAD
  features. The selected row stays visible while filtering.
- Generated machines are grouped by their subassemblies. Each part appears
  once, under the assembly that owns it, and an assembly row shows its part
  count.
- The eye at the right of a row shows or hides it. A hidden row keeps its
  place with a muted label. An open eye on a muted row means the row itself is
  visible but a parent hides it; a dash in a group's eye means some children
  are hidden.
- Click a row to select it; double-click to frame it in the 3D view.
- Right-click a row (or press Menu or Shift+F10) for **Show / hide (Space)**,
  **Isolate selected**, **Show all**, **Rename (F2)**, **Duplicate**,
  **Delete** and **Frame selected**.

With the tree focused: Up and Down move the selection, Left and Right collapse
and expand, Home, End, Page Up and Page Down jump, F2 renames, and Space shows
or hides the selected row.

While a simulation exists the panel says "Simulation active" and editing
commands are disabled; visibility still works.

### BIM

The building model editor: **Spatial** and **Types** trees with the selected
object's or type's properties, and its own **Undo BIM** and **Redo BIM**. It is
empty unless the document has a BIM model (the `--demo` scene has one).

### 3D

The viewport. See [The 3D viewport](viewport.md).

### Inspector

The selected object's properties. With nothing selected it says "Select an
object to edit its properties."

- **Scene objects** have Object (Name), Geometry (Position X and Y, Width,
  Height and Depth in metres, for objects that have them), Rendering (Visible,
  Colour as `#RRGGBB`, Finish, Metallic, Roughness) and Physics (Collision,
  Dynamic body, Mass in kg) sections. Edits are undoable.
- **Assemblies** selected in the tree show their part count and path, with
  **Frame assembly**, **Hide assembly** or **Show assembly**, and **Isolate
  assembly**.
- **CAD parts** add buttons for the feature commands that apply: Create sketch,
  Sketch on face, Extrude, Pocket, Fillet vertical edges, Create sheet base,
  Create edge flange, Export flat DXF and SVG, and the selected face or edge.
  While a sketch is being edited: Add rectangle, Clear sketch, **Apply sketch**
  and Cancel. When a recompute cannot find a face or edge it used, the
  Inspector offers the candidates to choose from. See
  [CAD documents](../concepts/modelling/cad-documents.md).
- **Parts with mates** say whether they are still free to move and offer
  **Make ... joint** when the mates describe one. See
  [Assemblies and mates](../concepts/modelling/assemblies-and-mates.md).
- **Workers** show their job steps with Add step, Remove step, Move step up and
  Move step down.
- **Script-owned objects** show where each value comes from and **Revert object
  overrides**.
- Projects with a UI extension (the picking station) add their own rows and
  buttons here.

### Machine

Design positions for every movable joint, under **MACHINE CONTROL**. It opens
in front automatically when a project with movable joints opens.

- Each linear axis has a slider, a number field in millimetres with **Set**,
  and **Min**, **Max** and **Initial**.
- Each rotary axis has a dial, a horizontal scrubber that turns endlessly, and
  a number field in degrees with **Apply** and **Reset**. Hold Shift while
  dragging for one-tenth sensitivity.
- **Inspect** beside an axis frames its moving part and cuts away the near half
  of the view. **Flip side** reverses the cut; **Close section** removes it.
- **Save current pose as initial** makes the current joint positions the
  machine's starting pose.
- Joints that follow another axis (screws, belts, closed loops) move with it
  and have no control of their own.

Slider drags preview live and become one undo step when you release. These are
design positions, not machine commands. The panel is read-only while a
simulation exists ("Stop simulation to edit these design positions") or while
a setup script owns the machine. Machines with a guarded enclosure also show
door preview controls here, and in Simulate the guard's state. See
[Machine-axis views](../concepts/motion/machine-axes.md).

### Sensors

The robots and sensors in the document, and the simulation controls.

- **ROBOTS** lists the configured robots; click one to select it.
- **SENSORS** lists its sensors. **LiDAR**, **IMU** and **Camera** add one;
  **Remove** removes the selected one. The selected sensor's settings appear
  below.
- **SIMULATION** shows the mode and whether changes are pending, with
  **Apply** (or **Rebuild**), **Undo**, **Redo**, **Run**, **Pause**, **Step**,
  **Reset** and **Stop**, and a choice of **Test backend** or **MuJoCo**.
- For a script-owned setup: **Reload script**, **Enable overrides** or
  **Disable overrides**, **Revert simulation**, **Remove stale**, and per-sensor
  rate and revert buttons. Each value is marked as coming from the script or an
  override.

Remote robots (from `--robot`) are shown read-only.

### Mission

In Simulate, for projects with a mission. See
[Simulating](simulate.md#missions-and-jobs).

### CNC

In Simulate, for projects with a machining or print program. See
[Simulating](simulate.md#cnc-and-print-programs).

### Inspection camera

In Simulate only: a second live view that follows a part. See
[Simulating](simulate.md#the-inspection-camera).

### Console

The editor's log, under **CONSOLE**: every command result, warning and error,
newest at the bottom. It keeps the last 1000 lines and follows new output until
you scroll up. Select text with the mouse and copy it with Ctrl+C, or use
**Copy all**. Stale project edits are listed at the top with **Discard stale
edits**.

### Telemetry

Under **TELEMETRY**: a frame-time plot, the physics step number and simulated
time, and, for each simulated worker, its current step, the zones it is in and
its distance from each robot. The plot has data only in the `--demo` scene.

## See also

- [The editor and projects](../concepts/platform/editor-and-projects.md)
- [NativeKit and UIKit](../concepts/platform/nativekit-and-ui.md): the widgets
  and dock the editor is built from
- [Commands and shortcuts](commands.md)

## Open questions

- **The viewport context menu cannot be opened.** `app/README.md:13-14` lists
  right-click viewport context actions. `Main.hx` still builds a
  `scene-context-menu` (`app/src/Main.hx:883-904`) but nothing sets
  `contextMenuVisible` to true, and the right mouse button now flies the camera
  (`app/src/EditorPerspectiveViewport.hx:1160`).
- **No shortcut for Reset workspace.** `app/README.md:13-14` lists a keyboard
  shortcut for workspace reset; `workspace.reset` is registered without one
  (`haxeon/packages/ui/haxe/haxeon/ui/docking/DockWorkspaceCommands.hx:26`).
- **The Project panel is unreachable.** A `project` panel is registered
  (`app/src/Main.hx:1830`) but is in neither default layout
  (`app/src/EditorWorkspaceLayout.hx`) and nothing opens it; variants are
  reachable only from the card in the 3D view. `app/README.md:152-153` says
  the selectors are in "the viewport card or Project panel".
- **The simulation hint names the wrong button.** The Hierarchy shows
  "Simulation active. Press Design to edit." (`app/src/editor/HierarchyPanel.hx:21`),
  but switching to Design does not end the simulation; editing needs Stop
  (`canEditObjects` in `app/src/Main.hx`).
- **Add menu labels.** `app/README.md:191` says **Add → People → Worker**; the
  menu item is labelled **Add worker** (the command label,
  `app/src/editor/SceneObjectCommands.hx`).
- **Telemetry plot.** The frame-time plot is filled only with generated sample
  data in demo mode (`app/src/editor/TelemetryPanel.hx:26-32`); otherwise it
  stays empty.
