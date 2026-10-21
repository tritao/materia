# Assembly presentation: views, exploded views and procedures

Make complex assemblies easy to understand without hand-hiding parts, moving them
apart or opening several scene files. Moulds, fixtures, machines and maintenance
instructions all use the same infrastructure. Each project supplies its views and
procedures. The viewer supplies the controls.

## Principles

1. **Presentation is a separate layer.** An `AssemblyPresentation` record sits
   beside `AssemblyDefinition` and `AssemblyStateRecord`. It is not stored inside either.
   It references occurrences by id and never changes engineering data. Mates,
   joints, collision, SimKit, BOM and exports never read it.
2. **Display transforms are applied last.** Displayed pose = solved assembly pose
   followed by a presentation offset. The offset is applied only when building
   render and pick transforms. It never writes back into the assembly state.
3. **Viewer state is derived, not edited.** The current view, explode amount and
   procedure step make up a small session cursor. Everything visible is a pure
   function of (document, cursor). Previous and Next recompute that function. They
   don't undo anything. Reset sets the cursor to the default.
4. **Evidence is explicit and goes stale.** Only a planner can mark a movement as
   checked. The check is tied to a geometry fingerprint. When the geometry changes,
   the movement shows as stale and is never silently treated as checked.
5. **One mechanism.** This replaces `SceneArtifactInspectionCamera`,
   `SceneArtifactProject.initialView`, the hobber enclosure presentation mode,
   and every `exploded:Bool` assembly builder. Old fields and old `.materia`
   presentation data are rejected rather than migrated (one schema version).

## Today (what this replaces)

| Existing | Where | Becomes |
| --- | --- | --- |
| Inspection camera presets (`hiddenPrefixes` string match) | `projectkit/.../SceneArtifact.hx:120`, `app/src/editor/InspectionCameraPanel.hx` | `PresentationView` with typed selectors |
| `initialView` yaw/pitch | `SceneArtifactProject` | the `default` view |
| Enclosure visible/transparent/hidden | `app/src/editor/HobberScenePanel.hx`, `Main.setEnclosurePresentation` | per-view visibility rules |
| Viewport-only vertical section plane | `app/src/EditorPerspectiveViewport.hx:156` | persisted general section planes in views |
| `exploded:Bool` assembly builders and separate `*-exploded.mtrg` | `BoxMoldCad`, `CastingMoldCad`, `PanelKit`, `SocketFixtureCad`, `HobberSocketFixture` | stored explode offsets on the single assembly |
| `demoldingSteps`, `removalOrder`, `demoldingMotions` (review JSON only) | `CastingMoldCad`, `SweptRemoval.RemovalReport` | planner-generated `Procedure` with checked motions |

## Data model (`projectkit`, `materia.presentation`)

```haxe
@:wire typedef AssemblyPresentation = {
  schemaVersion:Int;            // exactly one accepted version
  definition:String;            // AssemblyDefinition.id it presents
  defaultView:String;           // must name a view
  views:Array<PresentationView>;
  explode:Array<ExplodeOffset>;
  procedures:Array<Procedure>;
}

// Selectors resolve against flattened occurrence ids. Unknown ids are a load error.
@:wire typedef PartSelector = { occurrence:String; subtree:Bool; }

@:wire typedef PresentationView = {
  id:String; label:String;
  camera:ViewCamera;            // Frame(selectors, yaw, pitch, margin) | Anchored(anchor, offset, span, yaw, pitch)
  visibility:Array<VisibilityRule>; // ordered; later rules win; default Shown
  sections:Array<SectionPlane>; // point + normal, expressed in an occurrence or world frame
  explode:Float;                // 0..1, the slider value this view opens with
  @:optional source:PresentationSource; // Generated(producer) | Authored
}
@:wire typedef VisibilityRule = { select:PartSelector; mode:Shown|Transparent|Hidden; }

// Direction is in the parent occurrence's frame, so offsets follow joint motion.
// Stages run in sequence: slider t is divided across stages 1..n. Nested offsets compose
// parent first, so a sub-assembly moves out as a unit before its parts separate.
@:wire typedef ExplodeOffset = { select:PartSelector; direction:Vec3; distance:Float; stage:Int; }

@:wire typedef Procedure = {
  id:String; label:String; kind:Assembly|Disassembly|Service;
  start:ProcedureStart;         // Assembled | Disassembled (every part removed by the procedure starts out)
  steps:Array<ProcedureStep>;
}
@:wire typedef ProcedureStep = {
  id:String; title:String; instruction:String;
  @:optional view:String;       // a reference, never an embedded view
  highlight:Array<PartSelector>;
  action:StepAction;            // Remove(sel, motion) | Install(sel, motion) | SetJoints(coords) | Note
}
@:wire typedef PresentationMotion = {
  path:Array<Vec3>;             // translation polyline in the parent frame, or joint keys for SetJoints
  evidence:MotionEvidence;      // Illustrative | Checked{checker, status:ToolingCheckStatus, geometryFingerprint, report}
}
```

Notes:

- Typed selectors replace `hiddenPrefixes` string matching. A renamed part then
  fails loudly at load instead of quietly showing everything.
- Procedure state at step *k* is a fold of steps `0..k-1` from `start`:
  removed sets, installed sets and joint overrides. Previous re-folds from the
  start, so every step is reproducible and there's no undo stack to corrupt.
- A procedure whose `start` is `Disassembled` uses the same fold. Assembly is
  disassembly reversed, but both are stored explicitly. A `Procedures.reverse()`
  helper derives one from the other for producers, and the viewer never infers it.
- Units follow the definition's `lengthUnit`.

### Where the record lives

- **Generated:** producers emit `presentation` in `SceneArtifactData`, next to
  `assemblyDefinition`. The record is regenerated on every rebuild, the same way
  recipe output is.
- **Authored:** views, explode offsets and procedures the user adds in the editor
  are saved in the `.materia` document as `Authored` entries. Generated entries
  are read-only in the editor (you can duplicate one to edit it). On reopen,
  authored entries are validated against the new definition, and an entry that
  references a missing occurrence is reported. It is not dropped silently.
- The session cursor (view, explode, step) is **not** saved. Reopening always
  shows the default view, assembled, with no procedure active. That is the reliable reset.

## Viewer (`app`)

- One `PresentationController` owns the cursor and computes a
  `PresentationFrame`: display offsets, per-occurrence visibility and opacity,
  section planes, the highlight set and the camera. The main viewport, the
  inspection camera dock and picture-in-picture all consume the same frame type.
  Docked and PiP views each hold their own cursor.
- Display offsets feed both rendering and picking or hover, so clicking an
  exploded part selects that part.
- While simulation is playing, explode and procedure motions are forced to zero
  and the controls are disabled. Engineering motion and presentation motion never mix.
- Controls: view list, Reset view, explode slider, procedure picker with
  Previous / Next / Restart, step text, and an evidence badge for each movement
  (Checked / Illustrative / Stale / Failed).
- Editor commands (undoable `document.apply`): "Save current view", "Edit explode
  offset" (drag gizmo along a direction), and "Add step".

## Invariant tests (gating, written in P1 and kept forever)

- **Engineering isolation:** fingerprint `AssemblyDefinition`, `AssemblyStateRecord`,
  the mate solve result, the collision broadphase input and the exported BOM and
  STEP. Every value is byte-identical with the explode slider at 0 and 1, and
  at every procedure step.
- **Reset:** after any sequence of view, slider and step operations, Reset yields
  a `PresentationFrame` equal to the one from a freshly opened document.
- **Round trip:** save, reopen, re-encode, and the result is byte-identical. A document
  with an unknown `schemaVersion` or an unknown selector is rejected with a message.
- **Determinism:** step *k* reached by Next×k equals step *k* reached by
  Restart → Next×k, and equals step *k* reached by Next×(k+2) → Previous×2.

## Phases

### AP0 — Data model and validation (projectkit)
Implementation status: implemented in `projectkit/src/materia/presentation`, with
coverage in `projectkit/tests/src/PresentationTests.hx`. Scene artifact version 42
transports the validated optional record. The remaining phases are tracked below.

`materia.presentation` types, codec, a validator (selectors resolve, view ids
unique, `defaultView` exists, stages ≥ 1, checked motions carry a fingerprint) and
the pure `PresentationFrame` evaluator: (definition, solved poses, presentation,
cursor) → frame. No UI.
*Accept:* unit tests for the validator and for the fold and nested explode composition,
plus the determinism invariant.

### AP1 — Views replace inspection cameras
Implementation status: the controller, shared viewport controls, typed render/pick
frames, persisted authored sections, hobber views and camera-field removal are in
place. Supplier axis defaults moved with the field removal. The presentation native
suite passes 20 checks and the ported enclosure runtime passes 10,551 checks.
Acceptance is still pending the explicit BOM/STEP export isolation checks. The
broader viewport-coordinate suite also fails its retained toolbar/status subtree
identity assertion even though both application revision keys are unchanged; that
failure remains unresolved and is not suppressed in that suite.

Viewer controller, view list and Reset in the main viewport and inspection dock.
Sections become persisted general planes. Migrate `HobberInspectionViews` and
the hobber enclosure modes to views. **Delete** `SceneArtifactInspectionCamera`,
`initialView`, `HobberScenePanel` enclosure buttons and `setEnclosurePresentation`.
*Accept:* hobber opens on its default view, every former preset is reproduced, the
engineering-isolation and Reset invariants are in the app test suite, and
`EnclosureRuntimeTests` are ported.

### AP2 — Socket fixture views (first example)
`SocketFixtureCad` emits views: `assembled` (default), `see-inside` (plates
transparent, section through both socket axes) and `socket-mounting` (bed
hidden, camera on the screw/washer stack from below).
*Accept:* the fixture opens assembled, and each view looks right in a screenshot
recorded in the example's README.

### AP3 — Explode offsets replace exploded builders
The slider, staged composition, and display offsets in picking. Producers emit
`ExplodeOffset`s. **Delete** `exploded:Bool` from `PanelKit`, `BoxMoldCad`,
`CastingMoldCad` and `SocketFixtureCad`, plus `socket-fixture-exploded.mtrg` and
the other `*-exploded` outputs. Box-mold and panel-tooling previews move to the slider.
*Accept:* the old exploded scenes are visually matched at t=1, the isolation
invariant holds at t=1, and you can pick parts while exploded.

### AP4 — Procedures
The `Procedure` fold in the viewer: Previous / Next / Restart, step text,
highlight, the referenced view and animated motion, plus evidence badges. Editor
"Add step" authoring stores `Authored` entries with `Illustrative` evidence only.
*Accept:* the determinism and Reset invariants with a procedure active, and an
authored procedure survives save and reopen.

### AP5 — Second, non-mould assembly (generality proof)
`dh-gripper` with custom fingers: views (`assembled`, `jaw-detail` with a
section through the jaw guide), an explode of fingers and screws, and a
**Service** procedure (remove finger screws → withdraw finger → install
replacement → reinstall screws) using `SetJoints` to open the jaws first.
Motions are `Illustrative` unless AP6's checker applies to them.
If anything in AP0–AP4 needs a gripper-specific branch, fix the model. Don't add a special case.
*Accept:* no gripper-specific code in `app/`, and all invariants pass on both examples.

### AP6 — Tooling planner adapter
`machinekit.tooling.ToolingPresentation` builds a `Disassembly` (demoulding)
procedure and an `Assembly` procedure (via `reverse`) from the planner's existing
roles, `dependsOn`, `removalOrder` and withdrawal directions. Steps that
`SweptRemoval` verified get `Checked{checker:"swept-removal", status, fingerprint}`.
Semantic steps (thread release) are `Note`, or `Illustrative`. Explode offsets come
from withdrawal direction × travel, with stages taken from the dependency depth. The planner
decides the order. The viewer only presents it.
*Accept:* the socket fixture demoulding procedure matches `demolding-steps.json`.
The negative control (screws left fitted) marks the plate withdrawal as `Failed`,
not hidden. Changing the plate thickness turns the cached evidence `Stale` until
the planner runs again. Box mold and casting mold get procedures with no new viewer code.

### AP7 — Remaining producers
Move remaining project-specific presets (supplier-fasteners axis previews,
machine-tending, sheet-metal press-brake steps where they are presentation)
onto views and procedures. Remove any leftover ad-hoc presentation fields from
`SceneArtifactProject`.
*Accept:* `grep` finds no inspection-camera, `initialView`, `hiddenPrefixes` or
`exploded` presentation paths outside `materia.presentation`.

## Ordering and dependencies

AP0 → AP1 → AP2 → AP3 → AP4 → AP5 → AP6 → AP7. AP5 comes before AP6 on purpose,
so the format is proven general before the mould planner shapes it. AP6 depends on
the casting tooling slices that own `SweptRemoval` and `SocketFixturePlanner`
(`manufacturingkit/TOOLING_PLAN.md`), and only reads their output.

## Out of scope

- Rendered instruction documents (PDF or web manuals). A later exporter can
  consume `Procedure` without changing it.
- Checking non-translational motions (screw helix, hinged removal). Until a
  checker exists, those stay `Illustrative`.
- Persisting the viewer cursor or offering per-user "last view".
