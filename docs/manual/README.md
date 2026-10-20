---
title: Editor manual
description: How to use the Materia editor without writing code, from launching it to running a simulation.
status: draft
---

# Editor manual

This manual is for using the Materia editor: opening machines, looking at them,
moving their joints, editing scenes and pressing Play. It does not explain why
the editor works the way it does; the [concept pages](../concepts/README.md)
do that, and each manual page links to the one it relies on. It does not cover
writing a machine in code either.

Everything here describes the desktop editor, started with
`./app/run-built.sh`. The browser build uses the same window and commands but
opens only prebuilt examples and artifacts; see
[Web vs. native](../concepts/platform/web-and-native.md).

## Pages

- [Starting the editor and opening projects](start-and-projects.md): launching,
  command-line flags, the Start page and its examples, opening and saving
  documents, and the files the editor reads and writes.
- [The workspace](workspace.md): the toolbar, status bar, dock layout and every
  panel.
- [The 3D viewport](viewport.md): moving the camera, selecting, hovering,
  dragging parts, and the view options.
- [Simulating](simulate.md): Design and Simulate modes, Play, Pause, Step,
  Reset and Stop, missions and CNC jobs, project variants, and session
  recordings.
- [Commands and shortcuts](commands.md): every command, where it appears and
  its default shortcut.
- [Settings](settings.md): the Editor Settings dialog, shortcut rebinding, and
  where settings are stored.

## Before you start

You need a built editor. The [gantry picker tutorial](../tutorials/gantry-picker.md)
walks through building it; in short:

```sh
./haxeon/scripts/bootstrap-tools.sh
./haxeon/scripts/haxeon build --project app/haxeon.json
./app/run-built.sh
```

## Words used in this manual

- **Project**: a machine written as code, described by a
  `materia.project.json` manifest. See
  [The editor and projects](../concepts/platform/editor-and-projects.md).
- **Document**: what the editor has open and saves, a `.materia` scene file.
  A document can be a plain scene or a set of edits layered over a project.
- **Scene artifact**: the `.mtrg` file a project's code produces. The editor
  can open one directly.
- **Command**: a named action such as `editor.save` or `sim.play`. Toolbar
  buttons, menus, the command palette and keyboard shortcuts all run commands.

The [glossary](../concepts/glossary.md) defines the rest.
