---
title: Settings
description: The Editor Settings dialog, rebinding shortcuts, and where the editor keeps settings, recent files and the panel layout.
status: draft
---

# Settings

Open **Editor Settings** with Ctrl+, or from **More**. The dialog has two
tabs, **General** and **Shortcuts**, and a **Close** button. Changes apply and
are saved as soon as you make them; there is no OK or Cancel.

## General

The left side is a tree of categories, the right side the selected category's
settings. **Filter Settings** narrows both to settings whose name or path
matches; **Advanced Settings** shows settings marked advanced (Materia has none
yet). Settings that need a restart are marked with
`*` (none do yet).

| Category | Setting | Default | What it does |
|---|---|---|---|
| Interface › Start Page | Show at Startup | On | Open the Start page when the editor starts without a file to open. Same as the checkbox on the Start page. |
| Interface › Theme | Color Scheme | Light | Light or Dark. **Toggle dark theme** changes it too. `--dark` overrides it for one run. |
| Editors › 3D | Anti-Aliasing Samples | 4 | Samples per pixel for edges in the 3D view, 1 to 16; 1 turns anti-aliasing off. The viewport menu's Off, 2x and 4x change it too. `--msaa` overrides it for one run. |
| Editors › 3D | Lighting | Studio | Studio, Soft or Contrast. |
| Editors › 3D | Show Cosmetic Threads | On | Draw nominal threads on named bore and shank faces. |
| Editors › 3D › Grid | Show Grid | On | The floor grid; same as **Grid** in the viewport. |
| Editors › 3D › Grid | Snap to Grid | Off | Round dragged and placed objects to the grid spacing; same as **Snap**. |
| Editors › 3D › Grid | Spacing | 0.2 m | Grid line spacing and snapping step, 0.01 to 10 m. |
| Editors › 3D › Simulation | Show Simulation Overlays | On | Draw LiDAR rays and a mission's route, costmap, sensed obstacles and odometry while a simulation runs. |
| Editors › 3D › Collision | Show Collisions | On | While editing, tint colliding objects red and near ones amber, and join their closest points. |
| Editors › 3D › Collision | Near Distance | 10 mm | How close two objects must be to count as near, 0 to 1000 mm. |
| Machine › Printer | Moonraker Address | empty | `host` or `host:port` (port 7125 unless given) of the Klipper printer that the Printer commands talk to. |
| Machine › Printer | Klipper Config File | empty | The `printer.cfg` that **Printer: Install Klipper config** uploads. |

The anti-aliasing default is the viewport's standard sample count, 4; a
graphics card that cannot do it uses as many as it supports. The definitions
are in `app/src/AppSettings.hx`.

The toolbar and viewport buttons for the grid, snapping, lighting, collisions,
cosmetic threads, overlays and theme change these same settings, so what you
choose there is remembered too.

## Shortcuts

The **Shortcuts** tab lists every command, grouped by the first part of its ID
(Editor, Scene, Sim, Workspace, Assembly, Printer, Start, Console), with its
current shortcuts. **Filter by name or shortcut** narrows the list.

- **Rebind**: click a command's shortcut button, then press the new key chord.
  Modifier keys alone keep it waiting; Escape cancels.
- **Conflicts**: a chord another command already uses is moved to this one,
  and the tab says which command lost it.
- **Reset** (the arrow icon) restores the command's default shortcut, and warns
  if another command now has the same chord.
- **Remove** (the cross icon) leaves the command without a shortcut.

Only shortcuts that differ from the defaults are saved. The default for every
command is in [Commands and shortcuts](commands.md). Keys handled by a panel
rather than a command, such as the viewport's fly keys, cannot be rebound.

## Where settings are stored

| What | File |
|---|---|
| Settings, changed shortcuts, recent files | `build/settings.json` |
| Design layout of the dock | `build/reference-editor-workspace.json` |

Both paths are relative to the directory the editor runs in; `run-built.sh`
runs it from the repository root. Set `REFERENCE_EDITOR_WORKSPACE` to another
path to move the layout file, and `settings.json` moves with it, into the same
directory.

`settings.json` holds only values that differ from their defaults, under
`values` keyed by setting path (for example `editors/3d/grid/spacing`), plus a
`state` object with `recent` (the last ten files opened or saved) and
`shortcuts` (for example `{"editor.save": ["ctrl+s"]}`). It is rewritten in one
step on every change. A value that no longer fits its setting falls back to the
default, values for settings this build does not know are kept, and a damaged
file only resets these conveniences. Delete the file to return to every
default.

If there is no `settings.json` but an older `preferences.json` exists beside the
layout file, the editor imports its Start page choice, anti-aliasing and recent
files once and leaves the old file in place.

The layout file holds only the Design layout; the Simulate layout lasts for
the session. `--reset-workspace` or **Reset workspace** returns to the default
layout.

Documents never hold settings, and settings never hold document content. The
caches under `~/.cache/materia` are separate again; see
[Starting the editor](start-and-projects.md#files-you-will-see).

## See also

- [The workspace](workspace.md)
- [Commands and shortcuts](commands.md)
- [NativeKit and UIKit](../concepts/platform/nativekit-and-ui.md)

## Open questions

- **Settings file name.** `app/README.md:292-293` and `app/README.md:671-672`
  name `preferences.json` as the store for the Start page choice, recent files
  and anti-aliasing. The code writes `settings.json` and reads
  `preferences.json` only as a one-time import
  (`app/src/AppPreferences.hx:13-19,39-42`).
- **Where the README says settings live.** `app/README.md` does not mention the
  Editor Settings dialog, the Shortcuts tab or the printer settings at all.
