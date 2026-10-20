---
title: Simulating
description: Run a machine in simulation with Play, Pause, Step, Reset and Stop; follow missions and CNC jobs; choose project variants; record a session.
status: draft
---

# Simulating

Pressing **Play** turns the open scene into a running simulation: the assembly
becomes a robot model, the parts become physics bodies, and whatever the
project carries (a mission, a CNC or print program, joint tracks) starts
driving it. What gets built, and at which fidelity, is explained in
[Simulation worlds](../concepts/execution/simulation-worlds.md) and
[Fidelity levels](../concepts/execution/fidelity.md).

## Design and Simulate

The editor has two modes, **Design** (Ctrl+1) and **Simulate** (Ctrl+2). They
are layouts, not states: you can look at the Simulate layout without a
simulation, and switching back to Design does not stop one. See
[The workspace](workspace.md#modes).

A simulation exists from the first Play or Step until Stop. While it exists:

- the lower toolbar row is tinted and the 3D view shows the simulated
  positions;
- adding, deleting, moving and renaming objects, Machine panel positions and
  variant selection are disabled;
- visibility, the camera and every view option still work;
- saving writes the design state, never the simulated poses.

## The transport controls

The transport buttons are in the middle of the lower toolbar row. The Sensors
panel has the same controls.

| Button | Shortcut | What it does |
|---|---|---|
| **Play** | F5 | Builds the simulation if needed, runs it, and switches to Simulate. |
| **Pause** | F5 | Freezes time and keeps the simulation. Shown in place of Play while running. |
| **Step** | F10 | Builds the simulation if needed and advances it by one step. Disabled while running. |
| **Reset** | Shift+F5 | Returns the simulation to its starting state and stays in Simulate. |
| **Stop** | none | Discards the simulation, returns the scene to its design pose, and switches back to the mode you were in before Play. |

**Building.** The first Play or Step builds the simulation in the background.
The transport area shows a spinner, the current phase and **Cancel**; the rest
of the editor stays responsive. If the build fails, the reason appears in the
Console and in red on the status bar, and nothing is replaced.

**Stopping** also takes a moment: the transport area shows "Stopping
simulation…" until it is done.

**Rebuild pending.** If the configuration changes while a simulation exists
(for example a sensor setting), the state chip and status bar add "rebuild
pending". Play and Step rebuild first; **Apply** or **Rebuild** in the Sensors
panel rebuilds without running. A failed rebuild keeps the running simulation.

Two more commands are in the command palette only:

- **Simulation: Emergency Stop** asks every simulated robot to stop.
- **Simulation: Reset Fault and Retry Mission** recovers a failed mission, when
  the failure allows it.

### Physics backend

Simulations use MuJoCo by default. The Sensors panel switches between
**MuJoCo** and **Test backend** (a simpler deterministic backend used by
automated checks); the change applies on the next rebuild. The status bar names
the active backend.

## What the status bar and chip show

The **state chip** beside the transport buttons reads **Design**, **Paused**
or **Running**. While a mission runs or has failed it shows the mission's state
instead, such as "Step 3 / 12 · pick", "Homing X", "Mission complete" or
"Mission failed", with a red dot for a failure.

The **status bar** shows, on the left, the same mission state in more detail
(for example "Homing X · finding switch", or the failure message) or a
simulation error in red. On the right it shows Design, Paused or Running, the
backend, and the robot world's state. See
[The workspace](workspace.md#the-status-bar).

## Missions and jobs

A project can carry a **mission**: an ordered list of steps for its robots,
such as picks, places and moves. Projects with a mission show a card in the 3D
view with the mission's status and **Show steps**, and Simulate opens the
**Mission** panel.

The Mission panel shows:

- the status: startup (homing), the current step and loop, completion or the
  failure;
- one button per robot program when the cell has several controllers; pick one
  to follow it;
- the list of steps, each marked Current, Done or Failed. Click a step to
  select its target part and see its connector. The list follows the current
  step until you scroll; **Follow current step** resumes following.

Steps are read-only here; they come from the project's code. A mission failure
is also written to the Console.

A CNC job can be named as a machining mission. The Mission panel then shows its
label and state with **Open CNC program**. A job marked as a kinematic preview
says "Kinematic preview · motor loads are not qualified." See
[Execution plans](../concepts/motion/execution-plans.md) and
[Process vs. robot](../concepts/process/process-and-robots.md).

## CNC and print programs

Projects with a machining or print program show it in the **CNC** panel, under
**CNC PROGRAM** or **PRINT PROGRAM**. Before Play it lists the program with
"Ready · Play to start". While it runs:

- the listing follows the current line;
- the status line shows Ready, Running, Paused, Held or Completed, the line
  number and the speed override;
- **Hold** and **Resume** pause and continue the feed without pausing time;
- click a line, then **Restart at line N** to continue from there;
- **Speed override** scales the feed from 5% to 200%;
- after a failure the status names the problem and the line, with **Reset
  program** and **Technical details**.

See [Toolpaths](../concepts/process/toolpaths.md) and
[Stock and material removal](../concepts/process/stock.md).

### A real printer

The 3D printer example can talk to a Klipper printer through Moonraker. Set its
address, and optionally a `printer.cfg`, under **Editor Settings > Machine >
Printer**, then use the **Printer:** commands in the command palette: Connect,
Disconnect, Send to printer, Install Klipper config, Pause print, Resume print,
Cancel print, and Mirror the machine in the model (the model follows the real
printer's reported state). The connection state appears in the CNC panel and
the Console.

## Project variants

Some projects offer alternative scenes: machine **configurations**, design
**studies**, or **scene presets**. When a project has them, the card in the 3D
view shows one drop-down per kind. Choose an entry to rebuild the scene with
it. Only one variant is installed at a time; picking a study does not keep the
chosen configuration.

The rebuild runs in the background with a spinner and **Cancel**. A failure or
cancellation keeps the current scene and selection. Variants cannot be changed
while a simulation exists ("Stop simulation to change the project selection").
A scene opened from a prebuilt `.mtrg` has only the variant it was built with.
The chosen variant is saved with the document.

## The inspection camera

In Simulate, **Camera** in the viewport toolbar opens a second live view that
follows one part, docked beside the right-hand panels.

- Choose a preset (projects can define them); without presets, select a part
  first and the camera follows it as **Selected part**.
- Orbit and pan with the usual viewport gestures; the wheel or **Zoom +** and
  **Zoom -** change the lens. **Reset** returns to the preset.
- **Picture in picture** moves the view into a corner of the main viewport;
  **Dock** puts it back in a panel.
- The camera follows the part's position but keeps its own orientation, hides
  its own set of parts, and never changes the selection or the document.

The camera closes when you leave Simulate and reopens when you return.
`--inspection-camera` opens it on entering Simulate.

## Workers

Scenes with human workers show each worker's current step, zones and distance
from each robot in the **Telemetry** panel while the simulation runs. The two
worker examples on the Start page start running as soon as they open.

## Session recordings

`--record` writes a session recording, a JSON-lines file under
`app/build/recordings/`, or `--record=PATH` to choose the file. It records
window events and every command, selection change and property edit, with the
command line and window size at the start and the exit code at the end. It is
for reproducing what someone did in the editor, not a recording of the
simulation: there is no simulation playback or export in the editor.

## See also

- [The editor and projects](../concepts/platform/editor-and-projects.md#modes-and-play)
- [The robot runtime](../concepts/execution/robot-runtime.md)
- [Commands and shortcuts](commands.md)

## Open questions

- **No Demo selector.** `app/README.md:152,171` describes a third variant
  kind, `demo`, with its own **Demo** selector. The card shows only
  Configuration, Study and Scene preset (`app/src/editor/ProjectVariantView.hx:24-26`);
  `demo` variants appear under Configuration.
- **No Simulation panel.** `app/README.md:491,497` refers to diagnostics in "the
  Simulation panel". No such panel is registered (`makeWorkspace` in
  `app/src/Main.hx`); simulation errors appear on the status bar and in the
  Console.
- **"Press Design to edit."** The Hierarchy's hint during simulation names the
  Design button (`app/src/editor/HierarchyPanel.hx:21`), but editing stays
  disabled until Stop (`canEditObjects` in `app/src/Main.hx`).
