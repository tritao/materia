package app.editor;

import app.Main.ReferenceEditorApp;
import app.EditorScene.EditorSceneObject;
import haxeon.ui.Insets;
import haxeon.ui.core.TextStyleOverride;
import haxeon.ui.core.View;
import haxeon.ui.core.RetainedVariants;
import haxeon.ui.icons.IconName;
import haxeon.ui.widgets.KeyedView;
import haxeon.ui.widgets.controls.Button;
import haxeon.ui.widgets.layout.Column;
import haxeon.ui.widgets.layout.Row;
import haxeon.ui.widgets.properties.PropertyInspector;
import haxeon.ui.widgets.controls.Select;
import haxeon.ui.widgets.controls.SelectOption;
import materia.project.MaterialLibrary;
import app.editor.SheetWorkflowPanel;
import haxeon.ui.core.BuildContext;
import haxeon.ui.core.RenderNode;
import haxeon.ui.widgets.text.Text;
import haxeon.ui.widgets.text.TextField;

/** Selected-object inspector dock panel. */
@:access(app.Main.ReferenceEditorApp) class InspectorPanel {
  /** Retained by the observable fields read by the inspector, sheet, and project panels. */
  public static function build(app:ReferenceEditorApp, context:BuildContext):View {
    var appearance = app.appearance;
    var scene = app.scene;
    var session = app.session;
    var simulation = app.simulation;
    var perspectiveViewport = app.perspectiveViewport;
    var fillStyle = function() return ReferenceEditorApp.fillStyle();
    var actionRowStyle = function() return ReferenceEditorApp.actionRowStyle();
    var sectionHeading = function(label:String) return app.sectionHeading(label);
    var sceneAction = function(key:String, commandId:String, label:String, icon:Null<IconName>) return app.sceneAction(
      key,
      commandId,
      label,
      icon
    );
    var textLines = function(key:String, lines:Array<String>) return ReferenceEditorApp.textLines(key, lines);
    var refreshScriptMaterialization = function(message:String) app.refreshScriptMaterialization(message);
    var style = fillStyle();
    style.padding = new Insets(10.0, 10.0, 10.0, 10.0);
    style.background = appearance.theme.tokens.surface;
    var groupKey = app.treeModel.selectionKey();
    var group = app.treeModel.groupInfo(groupKey);
    if (group != null) {
      var editable = !app.documents.blocked();
      var noun = group.kind.toLowerCase();
      var shown = app.treeModel.visibilityState(groupKey).own;
      var visibility = new Button(shown ? "Hide " + noun : "Show " + noun,
        null, function() {
        if (!app.documents.blocked()) app.treeModel.toggleVisibility(groupKey);
      }
      );
      var isolate = new Button("Isolate " + noun, null, function() {
        if (!app.documents.blocked()) app.treeModel.isolateSelected();
      }
      );
      visibility.enabled = isolate.enabled = editable;
      var rows:Array<KeyedView> = [
        new KeyedView("heading", sectionHeading(group.label)),
        new KeyedView(
          "assembly-summary",
          new Text(group.kind + " · " + group.count + " part occurrences · " + group.shown + " shown")
        ),
        new KeyedView("assembly-path", new Text(group.path == "" ? "Root assembly" : group.path)),
        new KeyedView("frame", sceneAction("assembly-frame", "scene.frame-selected", "Frame " + noun, null)),
        new KeyedView("visibility", visibility),
        new KeyedView("isolate", isolate)
      ];
      return new Column("inspector-assembly", rows, style);
    }
    var selected = scene.object(scene.selectedId);
    if (selected == null) {
      var emptyRows:Array<KeyedView> = [
        new KeyedView("heading", sectionHeading("INSPECTOR")),
        new KeyedView("hint", new Text("Select an object to edit its properties."))
      ];
      var sheetWorkflow = SheetWorkflowPanel.build(app);
      var projectPanel = app.projectUiPanel();
      if (projectPanel != null) emptyRows.push(new KeyedView("project-ui", projectPanel));
      if (sheetWorkflow != null) emptyRows.push(new KeyedView("sheet-workflow", sheetWorkflow));
      return new Column("inspector-empty", emptyRows, style);
    }
    if (app.sceneInspector == null || app.inspectorSelectionRevision != scene.selectionRevision) {
      var feature = scene.selectedFeatureKey;
      var modelKey = session.generation + ":" + selected.id.length + ":" + selected.id + ":" +
        (feature == null ? "-1:" : feature.length + ":" + feature);
      var cached = app.inspectorModels.get(modelKey);
      if (cached == null || !cached.isPristine()) {
        cached = new PropertyInspector(
          "scene-inspector:" + scene.selectedId,
          scene.properties(),
          style,
          null,
          null,
          null,
          "Selected object inspector"
        );
        app.inspectorModels.set(modelKey, cached);
        app.inspectorModelIds.set(modelKey, app.nextInspectorModelId++);
      }
      app.inspectorModelOrder.remove(modelKey);
      app.inspectorModelOrder.push(modelKey);
      if (app.inspectorModelOrder.length > 4) {
        var oldest = app.inspectorModelOrder.shift();
        if (oldest != null) { app.inspectorModels.remove(oldest); app.inspectorModelIds.remove(oldest); }
      }
      app.sceneInspector = cached;
      var modelId = app.inspectorModelIds.get(modelKey);
      if (modelId == null) throw "Missing retained inspector model identity";
      app.inspectorModelId = modelId;
      app.inspectorSelectionRevision = scene.selectionRevision;
    }
    var inspector = app.sceneInspector;
    inspector.setStyle(style);
    var widthClass = app.viewportWidth < 760.0 ? 0 : app.viewportWidth < 1180.0 ? 1 : 2;
    inspector.labelWidth = widthClass == 0 ? 56.0 : widthClass == 1 ? 76.0 : 108.0;
    var ownership = session.scriptOwnership;
    inspector.enabled = ownership == null && !simulation.isActive()
      && !(app.perspectiveViewport != null && app.perspectiveViewport.dragging());
    var rows:Array<KeyedView> = [new KeyedView("heading", new ObservableInspectorHeading(app, selected))];
    if (simulation.isActive()) rows.push(new KeyedView(
      "simulation-hint",
      new Text(
        HierarchyPanel.SIMULATION_LOCK_HINT,
        null,
        appearance.theme.tokens.textSecondary,
        TextStyleOverride.text(12.0)
      )
    ));
    var assembly = session.projectAssemblyDefinition;
    if (assembly != null && StringTools.startsWith(selected.id, "project:")) {
      var instanceId = selected.id.substr(8);
      var jointLines:Array<String> = [];
      for (joint in assembly.joints) if (joint.parent == instanceId
        || joint.child == instanceId) jointLines.push(joint.id
        + " · " + joint.type + " · " + joint.parentConnector + " → " + joint.childConnector);
      if (jointLines.length > 0) rows.push(new KeyedView(
        "assembly-joints",
        textLines("assembly-joint-lines", jointLines)
      ));
    }
    if (StringTools.startsWith(selected.id, "project:")) {
      // When the part's mates leave it one turn or slide, offer the joint they amount to.
      var mateJoint = session.assemblyMateJoint(selected.id);
      var mateJointType = mateJoint == null ? null : mateJoint.type;
      if (session.assemblyPartStillFree(selected.id)) rows.push(new KeyedView(
        "mate-freedom",
        new Text(
          "Still free to move under its mates",
          null,
          appearance.theme.tokens.textSecondary,
          TextStyleOverride.text(12.0)
        )
      ));
      if (mateJointType != null) rows.push(new KeyedView(
        "convert-mates-to-joint",
        sceneAction(
          "convert-mates-to-joint",
          "assembly.convert-to-joint",
          "Make " + mateJointType + " joint",
          IconName.Plus
        )
      ));
    }
    if (scene.canCreateExtrusion() || scene.canCreateSheetFlange()) {
      var input = scene.sheetAuthoring;
      var entries:Array<{key:String, label:String, read:Void -> String, write:String -> Void}> = [];
      if (scene.canCreateExtrusion()) {
        var grades = MaterialLibrary.sheetMaterials();
        for (grade in session.customMaterials) if (grade.sheet != null) grades.push(grade);
        var options = [for (grade in grades) new SelectOption<String>(grade.id, grade.name, grade.id)];
        rows.push(new KeyedView("sheet-material-select",
          new Select<String>("sheet-material-select", options, input.materialId, function(value) {
          input.materialId = value;
          input.kFactor = "";

        }
        )));
        entries.push({
          key: "material",
          label: "Sheet material identity",
          read: function() return input.materialId,
          write: function(v) input.materialId = v
        });
        entries.push({
          key: "thickness",
          label: "Thickness (mm)",
          read: function() return input.thickness,
          write: function(v) input.thickness = v
        });
        entries.push({
          key: "k",
          label: "ANSI K factor override (blank uses sheet grade)",
          read: function() return input.kFactor,
          write: function(v) input.kFactor = v
        });
        entries.push({
          key: "grain",
          label: "Rolling direction in sketch (deg)",
          read: function() return input.grainDegrees,
          write: function(v) input.grainDegrees = v
        });
      } else {
        entries.push({
          key: "length",
          label: "Flange straight length (mm)",
          read: function() return input.length,
          write: function(v) input.length = v
        });
        entries.push({
          key: "angle",
          label: "Signed bend angle (deg)",
          read: function() return input.angleDegrees,
          write: function(v) input.angleDegrees = v
        });
        entries.push({
          key: "radius",
          label: "Inner radius (mm)",
          read: function() return input.radius,
          write: function(v) input.radius = v
        });
      }
      for (entry in entries) {
        var field = new TextField("sheet-authoring-" + entry.key, entry.read(), entry.write);
        field.label = entry.label;
        rows.push(new KeyedView("sheet-input-" + entry.key, field));
      }
    }
    if (scene.selectedSheetFeature() != null) {
      var bendLines:Array<String> = ["Bend · direction · angle (deg) · radius (mm) · allowance (mm) · length (mm)"];
      for (bend in scene.selectedSheetBends()) bendLines.push("B" + bend.id + " · " + (bend.up()
        ? "up" : "down") + " · " + Std.string(bend.angle * 180 / Math.PI) + " · "
        + Std.string(bend.innerRadius) + " · " + Std.string(bend.allowance) + " · " + Std.string(bend.length()));
      rows.push(new KeyedView("sheet-bend-table", textLines("sheet-bend-rows", bendLines)));
    }
    if (scene.hasActiveSketchEdit()) {
      var summary = scene.sketchEditSummary();
      if (summary != null) rows.push(new KeyedView("sketch-draft-status", new Text(summary)));
      var sketchTools:Array<KeyedView> = [];
      if (scene.canAddSketchDraftRectangle()) sketchTools.push(new KeyedView(
        "add-rectangle",
        sceneAction("add-sketch-rectangle", "scene.add-sketch-rectangle", "Add rectangle", IconName.Plus)
      ));
      if (scene.canClearSketchDraft()) sketchTools.push(new KeyedView(
        "clear-sketch",
        sceneAction("clear-sketch-draft", "scene.clear-sketch-draft", "Clear sketch", IconName.Close)
      ));
      if (sketchTools.length > 0) rows.push(new KeyedView(
        "sketch-draft-tools",
        new Row("sketch-draft-tools-row", sketchTools, actionRowStyle())
      ));
      rows.push(new KeyedView(
        "sketch-draft-actions",
        new Row(
          "sketch-draft-actions-row",
          [
            new KeyedView(
              "apply",
              sceneAction("apply-sketch-draft", "scene.apply-sketch", "Apply sketch", IconName.Save)
            ),
            new KeyedView("cancel", sceneAction("cancel-sketch-draft", "scene.cancel-sketch", "Cancel", IconName.Close))
          ],
          actionRowStyle()
        )
      ));
    } else if (scene.canBeginSelectedSketchEdit()) {
      rows.push(new KeyedView(
        "sketch-edit-action",
        sceneAction("edit-selected-sketch", "scene.edit-sketch", "Edit sketch", IconName.Inspect)
      ));
    }
    // An edit waiting for the user to say which element a reference means (TN7).
    var pending = scene.pendingReferenceChoice();
    if (pending != null) {
      rows.push(new KeyedView("pending-choice-status", new Text(pending.message)));
      for (candidate in 0...pending.candidates.length) rows.push(new KeyedView("pending-choice-" + candidate,
        new Button("Use " + pending.candidates[candidate], null, function() {
        app.runSceneEdit("Could not apply the edit", function() app.scene.resolvePendingReferenceChoice(candidate));

      }, "pending-choice-" + candidate)));
      rows.push(new KeyedView("pending-choice-cancel", new Button("Keep the previous model", null, function() {
        app.scene.cancelPendingReferenceChoice();

      }, "pending-choice-cancel")));
    }
    // References that need a look (TN5): what each lost, with a button per element it could mean now.
    var referenceIssues = scene.selectedReferenceIssues();
    for (issue in referenceIssues) {
      var key = "reference-" + issue.index;
      rows.push(new KeyedView(key + "-status", new Text(issue.message)));
      for (candidate in 0...issue.candidates.length) {
        var referenceIndex = issue.index;
        rows.push(new KeyedView(key + "-candidate-" + candidate,
          new Button("Use " + issue.candidates[candidate], null, function() {
          app.runSceneEdit(
            "Could not repair reference",
            function() app.scene.repairSelectedReference(referenceIndex, candidate)
          );

        }, "repair-reference-" + referenceIndex + "-" + candidate)));
      }
      if (issue.pickable) {
        var pickedIndex = issue.index;
        rows.push(new KeyedView(key + "-picked", new Button("Use the selected " + issue.kind, null, function() {
          app.runSceneEdit(
            "Could not repair reference",
            function() app.scene.repairSelectedReferenceWithPick(pickedIndex)
          );

        }, "repair-reference-" + pickedIndex + "-picked")));
      }
    }
    var supportStatus = scene.selectedSketchSupportStatus();
    if (supportStatus != null) {
      if (referenceIssues.length == 0) rows.push(new KeyedView("sketch-support-status", new Text(supportStatus)));
      if (scene.canRepairSelectedSketchSupportFace()) rows.push(new KeyedView(
        "repair-sketch-support",
        sceneAction(
          "repair-sketch-support-face",
          "scene.repair-sketch-support-face",
          "Repair support face",
          IconName.Inspect
        )
      ));
    }
    var pickedLabel = scene.selectedElementLabel();
    if (pickedLabel != null) rows.push(new KeyedView("picked-element", new Text(pickedLabel)));
    if (scene.isCadPart(selected.id) && scene.hasCadOutput(selected.id)) rows.push(new KeyedView(
      "face-selection",
      new Text(scene.selectedCadEdgeIndex >= 0 ? "Selected edge " + (scene.selectedCadEdgeIndex + 1)
      : scene.selectedCadFaceIndex < 0 ? "Click a CAD face or edge to select it" : selected.kind
      == "cad-plate" ? "Selected face " + (scene.selectedCadFaceIndex + 1) + " · Add hole uses the picked location"
      : "Selected face " + (scene.selectedCadFaceIndex + 1))
    ));
    if (selected.kind == "cad-preview" && scene.selectedCadEdgeIndex >= 0) rows.push(new KeyedView(
      "edge-selection",
      new Text("Selected edge " + (scene.selectedCadEdgeIndex + 1))
    ));
    var kindProvider = ObjectKindRegistry.find(selected.kind);
    if (kindProvider != null) for (commandId in kindProvider.commands(scene, selected.id)) {
      switch (commandId) {
        case "scene.create-sketch":
          rows.push(new KeyedView(
            "create-sketch",
            sceneAction("create-constrained-sketch", commandId, "Create sketch", IconName.Plus)
          ));
        case "scene.create-face-sketch":
          rows.push(new KeyedView(
            "create-face-sketch",
            sceneAction("create-face-sketch", commandId, "Sketch on face", IconName.Plus)
          ));
        case "scene.create-extrusion":
          rows.push(new KeyedView(
            "create-extrusion",
            sceneAction("create-extrusion", commandId, "Extrude", IconName.Plus)
          ));
        case "scene.create-sheet-base":
          rows.push(new KeyedView(
            "create-sheet-base",
            sceneAction("create-sheet-base", commandId, "Create sheet base", IconName.Plus)
          ));
        case "scene.create-sheet-flange":
          rows.push(new KeyedView(
            "create-sheet-flange",
            sceneAction("create-sheet-flange", commandId, "Create edge flange", IconName.Plus)
          ));
        case "scene.export-sheet-dxf", "scene.export-sheet-svg":
          rows.push(new KeyedView(
            commandId,
            sceneAction(
              commandId,
              commandId,
              commandId == "scene.export-sheet-dxf" ? "Export flat DXF" : "Export flat SVG",
              IconName.Save
            )
          ));
        case "scene.roll-back-to-feature":
          rows.push(new KeyedView(
            "roll-back-to-feature",
            sceneAction("roll-back-to-feature", commandId, "Roll back to here", IconName.Undo)
          ));
        case "scene.roll-forward":
          rows.push(new KeyedView("roll-forward", sceneAction("roll-forward", commandId, "Roll forward", IconName.Redo)));
        case "scene.create-pocket":
          rows.push(new KeyedView("create-pocket", sceneAction("create-pocket", commandId, "Pocket", IconName.Plus)));
        case "scene.create-vertical-fillet":
          rows.push(new KeyedView(
            "create-vertical-fillet",
            sceneAction("create-vertical-fillet", commandId, "Fillet vertical edges", IconName.Plus)
          ));
        case "scene.worker-add-step":
          rows.push(new KeyedView(
            "worker-add-step",
            sceneAction("worker-add-step", commandId, "Add step", IconName.Plus)
          ));
        case "scene.worker-remove-step":
          rows.push(new KeyedView(
            "worker-remove-step",
            sceneAction("worker-remove-step", commandId, "Remove step", IconName.Close)
          ));
        case "scene.worker-step-up":
          rows.push(new KeyedView(
            "worker-step-up",
            sceneAction("worker-step-up", commandId, "Move step up", IconName.Plus)
          ));
        case "scene.worker-step-down":
          rows.push(new KeyedView(
            "worker-step-down",
            sceneAction("worker-step-down", commandId, "Move step down", IconName.Plus)
          ));
        default:
      }
    }
    if (ownership != null) {
      rows.push(new KeyedView(
        "origin",
        textLines(
          "script-object-origins",
          ["Script-owned"].concat(ownership.propertyOrigins(
            selected.id,
            ["position", "dimensions", "mass", "collisionEnabled", "dynamicBody", "visible"]
          ))
        )
      ));
      rows.push(new KeyedView("revert", new Button("Revert object overrides", null, function() {
        if (ownership.revertTarget(selected.id)) refreshScriptMaterialization("Object overrides reverted");
      }, "script-object-revert")));
    }
    // A clean selected-object inspector can be restored after visiting another object. Changed detached state
    // is rebuilt in a fresh scope, preserving the previous reset-on-selection behavior for edited controls.
    rows.push(new KeyedView("properties", new RetainedVariants("scene-properties", Std.string(app.inspectorModelId),
      function(_) return inspector,
      function() return inspector.labelWidth + ":" +
        inspector.enabled + ":" + inspector.stackedRows + ":" + inspector.searchQuery,
      4, true)));
    var sheetWorkflow = SheetWorkflowPanel.build(app);
    var projectPanel = app.projectUiPanel();
    if (projectPanel != null) rows.push(new KeyedView("project-ui", projectPanel));
    if (sheetWorkflow != null) rows.push(new KeyedView("sheet-workflow", sheetWorkflow));
    return new Column("inspector-panel", rows, style);
  }

}

/** Rebuilds only the title when the selected object's label changes. */
@:access(app.Main.ReferenceEditorApp) private class ObservableInspectorHeading implements View {
  final app:ReferenceEditorApp;
  final selected:EditorSceneObject;

  public function new(app:ReferenceEditorApp, selected:EditorSceneObject) {
    this.app = app;
    this.selected = selected;
  }

  public function build(context:BuildContext):RenderNode {
    var id = context.id("heading-root");
    return context.selfUpdating(id, function() return new Column("heading", [
      new KeyedView("text", app.sectionHeading(selected.label))
    ]).buildWithId(context, id), false);
  }
}
