---
title: The 3D viewport
description: Move the camera, select and hover parts, drag objects and joints, pick faces for mates, and change what the 3D view shows.
status: draft
---

# The 3D viewport

The **3D** panel shows the open scene. It has a toolbar along its top, a
view-angle button in the top-left corner of the picture, and, for projects with
variants or a mission, a card below that button.

## Move the camera

| To | Do |
|---|---|
| Orbit | Middle-drag, or Alt+left-drag |
| Pan | Shift+middle-drag, or Shift+Alt+left-drag |
| Zoom | Mouse wheel |
| Fly | Hold the right button and move the mouse to look; W, A, S, D move, E goes up, Q goes down, Shift moves four times faster |
| Stop flying | Release the right button, or press Escape |
| Frame the selection | F, or **Frame** |
| Fit everything visible | Home, or **Fit** |
| Return to the default camera | **Reset** in the viewport toolbar |

Orbiting turns around the surface under the cursor when the drag starts, and a
small ring marks that point; a drag that starts over empty space keeps the
previous centre. The wheel zooms towards the surface under the cursor. Flying
also stops when the window loses focus.

**View angles.** The button in the top-left corner of the picture names the
current view. Click it for **Default perspective**, **Top view**, **Front
view** and **Right view**.

## Select and hover

- **Click** a part to select it. Selection in the 3D view and the Hierarchy are
  the same: the row is highlighted, its groups open and the tree scrolls to it.
- On a CAD part, the click also selects the face or edge under the cursor; the
  Inspector names it ("Selected face 3").
- **Click empty space** to clear the selection.
- **Hover** highlights the part, or the face of a CAD or generated part, under
  the cursor. While a simulation runs, hover and selection follow the simulated
  positions.
- **Space** shows or hides the selection, like the eye in the Hierarchy. A
  hidden part stays selected, so pressing Space again shows it. In the 3D view
  this works only while no simulation exists; the Hierarchy's eyes and the
  **Visibility** popup work at any time.

Hidden parts are not drawn and cannot be clicked. Visibility affects only the
view; collision and simulation ignore it.

## Move things

Moving is possible only while no simulation exists and the document is not
owned by a setup script.

**Scene objects.** Left-drag an object to move it in its horizontal plane. The
point you grabbed stays under the cursor, positions snap to the grid when
**Snap** is on, and releasing makes one undo step. Escape cancels the drag.

With an object selected, the arrow keys nudge it by 0.1 m and Shift+arrow by
1 m; each nudge is its own undo step.

**Joints of a generated machine.** Left-drag a part of an assembly to move it
along its joints. The viewport toolbar says what is being dragged, or why the
part does not follow. For exact positions use the [Machine panel](workspace.md#machine).

**Sketches.** While a sketch is being edited, left-drag on the sketch plane to
draw a rectangle, or drag an existing sketch point to move it; its constraints
are kept. Escape cancels the current drag. See
[CAD documents](../concepts/modelling/cad-documents.md).

## Add a mate between two faces

1. Run one of **Mate faces: Planar**, **Coaxial**, **Parallel** or
   **Perpendicular** from the command palette (Ctrl+K).
2. Click a face of an assembly part. The viewport toolbar prompts for each
   step.
3. Click a face of another part. Flat and round faces can be mated.

Escape cancels the pick. The status bar then reports whether the mates are
satisfied. See [Assemblies and mates](../concepts/modelling/assemblies-and-mates.md).

## The viewport toolbar

From left to right:

- **Frame**, **Fit**, **Reset**: the camera actions above.
- **Camera** (Simulate only): opens the inspection camera. See
  [Simulating](simulate.md#the-inspection-camera).
- **Grid**: show or hide the floor grid.
- **Snap**: snap dragged and placed objects to the grid.
- **The grid spacing** (for example "0.2 m"): a menu with grid spacing 0.1,
  0.2 and 0.5 m; lighting Studio, Soft and Contrast; anti-aliasing Off, 2x and
  4x (options the graphics card cannot do are greyed); **Toggle collisions**;
  and **Cosmetic Threads**.
- **Visibility**: a checkbox tree of every component. It stays open while you
  tick boxes, has its own expansion and scrolling, and offers **Show all** and
  **Isolate selected**. A mixed box means a partly visible group. Escape or a
  click outside closes it. It works during simulation too.
- **The CAD display button**: cycles **Feature edges**, **All edges** and
  **Shaded**. Feature edges hides edges between smoothly joined faces. This
  changes the display only.
- **A status message**: what a drag or mate pick is doing, or else a summary of
  collisions.

Machines with an enclosure scenario, such as the flexspline hobber, add a row
below the toolbar with **Play scenario** (Idle / doors, Cutting, Load / cut /
unload) and **Enclosure** (Visible, Transparent, Hidden).

## Collisions and section views

**Collisions.** While **Toggle collisions** is on (the default), objects that
intersect are tinted red and objects closer than the near distance (10 mm by
default) amber, with a line between their closest points. The toolbar
summarises what it found. The tint never changes the parts' materials. See
[The collision model](../concepts/modelling/collision.md) and
[Settings](settings.md).

**Section views.** **Inspect** beside an axis in the Machine panel frames its
moving part and cuts away the near half of the model. **Flip side** reverses
the cut and **Close section** removes it. A section changes only the view and
picking.

## Simulation overlays

While a simulation runs, the viewport draws LiDAR rays, and a mission's route,
costmap, sensed obstacles and odometry. **Toggle
simulation overlays** (in **More**) hides them.

## See also

- [SceneKit rendering](../concepts/platform/rendering.md)
- [Frames and transforms](../concepts/foundations/frames.md)
- [The workspace](workspace.md)
- [Commands and shortcuts](commands.md)

## Open questions

- **Click selection.** `app/README.md:232-234` says a click selects the outer
  group and repeated clicks descend one level at a time.
  `app/README.md:20-22` says a click selects the hit part directly, which is
  what the code does (`EditorScene.selectHit`, `app/src/EditorScene.hx:2027`).
- **Shift axis lock.** `app/README.md:261-262` says holding Shift while
  dragging locks movement to the first dominant axis. The object drag has no
  axis lock (`app/src/PerspectiveSceneDrag.hx`).
- **Mates only from the palette.** The mate commands are listed in the
  viewport context menu (`app/src/Main.hx:883-904`), which cannot be opened, so
  the command palette is the only way to start a mate pick.
