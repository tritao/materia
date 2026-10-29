# Pendant timeline

`PendantCaseDocument` records the pendant's shell, plate and board envelope through `DocumentBuilder`
(helpers in `machinekit/hmi/PanelRecorder.hx`). `tests/haxeon.pendant-document.json` compares each with
the direct `PendantCase` build on volume and bounds, and checks that every feature has a label and that
labels and groups survive a save and load.

Run: `./haxeon/scripts/haxeon run --project=machinekit/tests/haxeon.pendant-document.json`

| Part | Features | Groups |
|---|---|---|
| Shell | 55 | Body, Cable and strap, Fasteners (Reinforcement when the planner places ribs) |
| Plate | 191 | Plate, Bosses, Control cutouts, Fasteners, Perimeter tongue |
| Board | 35 | Board |

## Core changes this needed

- `Sketch.roundedRectangle` and a `rounded-rectangle` sketch profile (with a `sketch.radius` parameter).
- `Feature.label` and `Feature.group`, saved with the document (optional fields; no schema bump). The
  builder has `group(name, body)` and `label(feature, text)`.
- `Dimension` (`recording/Dimension.hx`): builder parameters take a plain number or a shared
  `NamedParameter`. Plain numbers add nothing to the document's named parameters.
- Haxeon: a `Null<T>` argument now converts through an abstract's `@:from(T)` (it used to become a runtime
  cast to the underlying type). Test: `haxeon/tests/programs/abstract-from-nullable.hx`.

## Timeline model and the scene

- `cadkit.parametric.TimelineModel.of(document)`: one row per feature with label, group, kind, active, `rolledBack`, a
  `parent` (a tool, sketch or placement belongs to the boolean that applies it; a nested boolean stays a step of its
  own) and the output faces the step created. Faces are attributed from the `f<id>:` tag in each face's element name.
  Every output face belongs to exactly one top-level step.
- `PendantScene.build()` builds the pendant scene from the recorded plate, shell and board (their face numbers are the
  scene's face numbers; the direct builds have more, coplanar-split faces) and lists the top-level steps in the scene's
  source map, with the group in front of the label (`Fasteners · Screw post`).
- `app/tests/haxeon.pendant-step-links.json` loads that scene in the editor's own tree: 99 steps, each selects and lights
  exactly its faces (221 of 221).

## Rollback

`Document.rollTo(index)` (-1 or the last index shows everything): view state, not saved and not in history. Recompute
leaves out later features, `displayedFeature()`/`displayedShape()` give what a view shows, and adding a feature while
rolled back is rejected. `CadDocumentSession.rollTo` publishes the shape and restores the marker if that fails. Tests:
`RecordingBuilderSmoke` and `app/tests/haxeon.cad-rollback.json`.

## Editing in the app

A recorded part opens as a `cad-part` object (its encoded document is the object's `cadGraph`). History rows read
`Group · Label`, and suppressed or rolled-back rows say so. The Feature menu and the Inspector have **Roll back to
selected feature** and **Roll forward**; both are view state (no undo entry, not saved) and only apply to `cad-part`
objects. Face numbers, size and picking follow the shown shape. Test: `app/tests/haxeon.cad-timeline-editor.json`
(the recorded pendant plate: select a row, roll back, check the shown shape, save and reopen, roll forward).

## Rolling back a generated part

The code stays the source of truth. The scene artifact's `project.histories` carries each part's recorded document
(`SceneArtifactHistory`: occurrence + encoded `cadkit.document`; about 42 KB for the pendant). The app opens a part's
history as a live session the first time it is needed (`app/src/editor/PartHistories.hx`). **Roll back to selected
feature** on a step row shows the part as it stood after that step, in the frame of its static geometry (the part's
centre and the project's units); **Roll forward** restores it. While rolled back, that part's step rows are rewritten to
match the shown shape (faces follow it; later steps read `Rolled back · …` and own no faces), so the existing tree and
face highlights keep working. View state only: nothing is saved or added to undo. Test:
`app/tests/haxeon.pendant-rollback.json`.

## Editing the case's size

The code is still the source of truth; the dimensions that are design intent are an editable recipe, through the
mechanism other generated projects already use (`documentInput` + `reconcilesSavedRecipe`).

- `PendantCasePlate` (`machinekit.pendant.plate`) is a recipe component with inputs `width`, `height`, `thickness`,
  `wall` and `depth` (mm). It is registered in `MachineKitComponents`, so the registry-wide contract tests cover it.
- `PendantScene.build(?savedDocument)` makes the recipe document, applies the saved values, plans the case for that
  `PendantSize`, and returns the scene with `recipeDocument` set. The `pendant` entrypoint is
  `PendantPreview.preview(?savedDocument)`.
- The inputs appear in the Inspector of the **plate** (`case/plate`); the shell and board follow the same values. The
  controls keep their places, so a size that no longer fits them (`PanelLayout.problems()`) is refused and the editor
  keeps the scene it has.
- Test: `machinekit/tests/haxeon.pendant-recipe.json` (default case, an edit regenerating plate, shell and history, a
  deeper case, and a refused one).

## Still open

- The Inspector flow itself (select the plate, edit a value, regenerate) has not been run in the app; the test calls
  `PendantScene.build` with the saved document the way the editor does.
- The controls' positions are not inputs yet; only the case's size is.
- UI-authored features on top of generated parts (a patch layer replayed after regeneration) need stable feature keys in
  the recorder (ids are sequential today). Not started.
- Nobody has looked at it in the running app; the checks are headless. The marker is a command, not a draggable bar,
  and there is no suppress toggle on a row.
- Rolled forward, the part shows the history session's tessellation, not the original static mesh (same faces,
  possibly different triangulation). Collision, mates and physics keep using the static geometry while rolled back.
- Whole-part selection lists no steps: the source map has no member links.
- Int literals do not convert to `Dimension` (`b.cylinder(2, 12)`); pass floats.
