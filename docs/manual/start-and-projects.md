---
title: Starting the editor and opening projects
description: Launch the editor, use its command-line flags, open examples from the Start page, and open and save documents.
status: draft
---

# Starting the editor and opening projects

## Launch the editor

From the repository root, after a build:

```sh
./app/run-built.sh
```

The script runs the built editor (`app/build/host/main.hl`) with the runtime
and native libraries it needs, from the repository root. If there is no build
it says so and exits. The window opens at 1600 × 1000.

A plain launch shows the **Start page** beside the 3D view, unless you turned
it off (see [Settings](settings.md)). Launching with a project, a setup script,
a demo or a preview goes straight to the model instead.

## Open a project from the command line

```sh
./app/run-built.sh --project=./machinekit/examples/gantry-picker/materia.project.json
```

`--project` takes a `materia.project.json` manifest or a prebuilt `.mtrg`
artifact. The window appears first, then the project builds in the
background. While it builds, the lower toolbar row shows a spinner, the
project's name, the current phase and the elapsed seconds, with a **Cancel**
button. The rest of the editor stays usable. If the build fails, the same row
shows the error with **Dismiss**, and the message is also in the Console.

The first open of a project compiles its code, which can take a while. Later
opens reuse the cached build and, for most examples, the cached geometry. What
is cached and why is in
[The editor and projects](../concepts/platform/editor-and-projects.md#caching).

Add `--simulate` to press Play as soon as the project has opened, and
`--frame=OBJECT-ID` to frame the camera on one scene object.

## Command-line flags

Flags are passed after the script name, for example
`./app/run-built.sh --dark --project=...`. An unknown flag prints the usage
line and exits with status 2. The parser is `Main.main` and
`ReferenceEditorLaunchOptions.fromArgs` in `app/src/Main.hx`.

### Opening content

| Flag | What it does |
|---|---|
| `--project=PATH` | Open a project manifest or `.mtrg` artifact in the background. Cannot be combined with `--setup-script`. |
| `--setup-script=REFERENCE` | Open a registered setup script, such as `materia.examples.two-robot`. |
| `--demo` | Start with the demo scene: two starter boxes, the BIM example and sample telemetry. New restores the starter objects. |
| `--worker-demo=rack-to-table` | Open the saved worker document and start its simulation. |
| `--worker-demo=gallery` | Open the six-worker gallery and start its simulation. |
| `--project-action=ID` | Send an initial action to the project's UI extension (only projects that have one, such as the picking station). |
| `--robot=HOST:PORT` | Connect to a running headless robot process over TCP. Its robot appears read-only. |

### Window and view

| Flag | What it does |
|---|---|
| `--dark` | Use the dark theme for this run without changing the saved setting. |
| `--width=PX`, `--height=PX` | Window size; at least 480 × 360. |
| `--perspective` | Bring the 3D view to the front at launch. |
| `--inspection-camera` | Open the inspection camera when Simulate is entered. |
| `--msaa=SAMPLES` | Anti-aliasing samples for this run only (`1` turns it off); the saved setting is untouched. |
| `--reset-workspace` | Discard the saved panel arrangement before starting. |
| `--simulate` | Press Play once the launch project has opened. |
| `--frame=OBJECT-ID` | Frame the camera on that object once the project has opened, leaving nothing selected. |

### Recording and capture

| Flag | What it does |
|---|---|
| `--record` | Record the session to `app/build/recordings/materia-<time>-<pid>.jsonl`. |
| `--record=PATH` | Record the session to `PATH`. |
| `--capture-dir=PATH` | Render a bounded run and write `frame.png`, `ui-tree.txt`, `layout.json`, `app-state.json`, `frame-metrics.json`, `events.jsonl` and `frame-timeline.jsonl` there. |
| `--frames=N` | With `--capture-dir`: stop after N frames (default 3). |
| `--capture-seconds=N` | With `--capture-dir`, instead of `--frames`: stop after N seconds. |

A recording is a JSON-lines file. It starts with the window size and the
command line, then holds every window event, every command run (with the
selection and document state at that moment), selection changes and property
edits, and ends with the exit code. It does not record the simulation; see
[Simulating](simulate.md#session-recordings).

### Headless checks and developer previews

These are for automated checks and kit development rather than everyday use.

| Flag | What it does |
|---|---|
| `--snapshot` | Run without a window and print the workspace layout as JSON. |
| `--snapshot --simulate` | Also build and start the simulation and print its state. |
| `--example=ID[,ID...]` | With `--snapshot`: open Start page examples in order. |
| `--example-settle=SECONDS`, `--example-play` | With `--example`: let each example run, or press Play on each. |
| `--worker-demo-step=N`, `--worker-demo-trace` | Step a worker demo N ticks (0 to 3600) and print the result. |
| `--lab`, `--story=ID` | Open the UI Component Lab instead of the editor. |
| `--character=GLTF` and the `--character-*` options | Preview an animated character walking in the scene. |

## The Start page

The Start page is a tab beside the 3D view. It has four parts, top to bottom:

1. **New file**: **Empty scene** (the New command) and **Open file...** (the
   Open command).
2. **Recent files**: the three most recent files that still exist. Click one
   to open it. Files appear here once you open or save them.
3. **Examples**: a search field, category buttons, and one card per example.
   Categories are All, Robotics & handling, Welding, Machining, Gear cutting,
   3D printing, CAD & design, and People & simulation. Related examples (the
   welding cells, the cobots, the mills) appear together under a family
   heading. Search matches titles, families, categories and descriptions;
   **Clear filters** resets both. Only examples whose files exist next to this
   build are listed.
4. **Show this page at startup**: untick it to stop the page opening at
   launch.

Click anywhere on an example card to open it. If the current document has
unsaved changes you are asked first. A project example builds in the
background exactly like `--project`: the Start page shows a spinner, the phase
and the elapsed time with **Cancel**. When it finishes the editor shows the 3D
view, framed on the whole model.

The Start page is not part of any saved layout. To bring it back, run **Show
Start page** from the **More** menu or the command palette.

The examples list is fixed in the editor (`app/src/editor/ExampleCatalog.hx`).
Projects cannot add themselves to it; open any other project with `--project`.

## Open, save and close documents

| Action | Command | Shortcut |
|---|---|---|
| Start an empty scene | New | Ctrl+N |
| Open a file | Open | Ctrl+O |
| Save | Save | Ctrl+S |
| Save under a new name | Save As | Ctrl+Shift+S |

New, Open and Save are on the left of the top toolbar; Save As is in **More**.
Open and Save use the system file chooser. Save suggests `Untitled.materia.json`
for a document that has never been saved.

**Open** accepts:

- a `.materia` scene document saved by the editor;
- a `.mtrg` scene artifact, which opens in the background like a project.

A project manifest (`materia.project.json`) is not a document: open it with
`--project` or from the Start page. Opening it with Open reports
"Unsupported scene document".

**Unsaved changes.** The toolbar shows the document's path, with `*` when it
has unsaved changes. New, Open, opening an example and closing the window ask
**Save**, **Discard** or **Cancel** first. Cancelling the chooser, or a save
that fails, keeps the current document open.

**Running project code.** Opening a saved document that belongs to a project
or a setup script runs that code. The editor asks **Run project code?** and
names the code; choose **Run code and open** or **Cancel**. A document that
refers to a prebuilt `.mtrg` opens without asking.

**Errors.** A file that cannot be read or saved shows a **Scene document
error** dialog; **Close** dismisses it and the previous document stays open.

## What a saved document holds

Saving a plain scene writes its objects, sensors, workers and BIM model.

Saving while a project is open writes a document that points at the project
and holds only your edits on top of it: visibility, mass and collision
settings, joint positions, added mates, the chosen variant. It never copies the
generated geometry. Reopening it rebuilds the project and reapplies the edits.
If the project's code has changed so that some edits no longer apply, the
Console lists them as **Stale project edits** with a **Discard stale edits**
button. See [Data vs. runtime](../concepts/foundations/data-and-runtime.md).

Selection, camera, undo history and panel layout are not saved in documents.

## Files you will see

| File | What it is | Written by |
|---|---|---|
| `materia.project.json` | A project manifest | The project's author |
| `*.materia` | A saved scene or project document | Save |
| `*.mtrg` | A scene artifact produced by a project | The project build; the browser build's exports |
| `build/reference-editor-workspace.json` | The saved Design layout | The editor, automatically |
| `build/settings.json` | Settings, shortcuts and recent files | The editor, automatically |
| `app/build/recordings/*.jsonl` | Session recordings | `--record` |
| `~/.cache/materia/` | Compiled generators, cached artifacts and prepared scenes | The editor; safe to delete |

The two `build/` paths are relative to the directory the editor runs in, which
`run-built.sh` sets to the repository root. Set `REFERENCE_EDITOR_WORKSPACE`
to move the layout file; `settings.json` always sits beside it. The cache
follows `XDG_CACHE_HOME` when it is set.

## See also

- [The editor and projects](../concepts/platform/editor-and-projects.md): what a
  project, an artifact and the caches are
- [The workspace](workspace.md)
- [Simulating](simulate.md)

## Open questions

- **Settings file name.** `app/README.md:292-293` and `app/README.md:671-672`
  say the Start page choice, recent files and anti-aliasing are stored in
  `preferences.json`. The code stores them in `settings.json` beside the
  workspace file and only imports an existing `preferences.json` once
  (`app/src/AppPreferences.hx:39-42`).
- **Opening a project manifest with Open.** `app/README.md:154` says to "Open
  the source `.project.json`" to choose variants. File > Open parses every
  non-`.mtrg` file as a scene document and rejects a manifest
  (`app/src/SceneDocumentController.hx:56-79`, `app/src/SceneCodec.hx:77-81`);
  manifests open only through `--project` or the Start page.
- **Document extension.** Example documents and the concept pages use
  `.materia`, but Save suggests `Untitled.materia.json`
  (`app/src/SceneFileDialogs.hx:24`).
