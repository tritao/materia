package app.editor;

import app.Main.ReferenceEditorApp;
import Insets;
import nativekit.ui.core.TextStyleOverride;
import nativekit.ui.core.View;
import nativekit.ui.icons.IconName;
import nativekit.ui.widgets.KeyedView;
import nativekit.ui.widgets.controls.Button;
import nativekit.ui.widgets.layout.Column;
import nativekit.ui.widgets.layout.Row;
import nativekit.ui.widgets.properties.PropertyInspector;
import app.editor.SheetWorkflowPanel;
import nativekit.ui.widgets.text.Text;

/** Selected-object inspector dock panel. */
@:access(app.Main.ReferenceEditorApp)
class InspectorPanel {
  public static function build(app:ReferenceEditorApp):View {
    var appearance = app.appearance;
    var scene = app.scene;
    var session = app.session;
    var simulation = app.simulation;
    var perspectiveViewport = app.perspectiveViewport;
    var viewportWidth = app.viewportWidth;
    var fillStyle = function() return ReferenceEditorApp.fillStyle();
    var actionRowStyle = function() return ReferenceEditorApp.actionRowStyle();
    var sectionHeading = function(label:String) return app.sectionHeading(label);
    var sceneAction = function(key:String, commandId:String, label:String, icon:Null<IconName>)
      return app.sceneAction(key, commandId, label, icon);
    var textLines = function(key:String, lines:Array<String>) return ReferenceEditorApp.textLines(key, lines);
    var refreshScriptMaterialization = function(message:String) app.refreshScriptMaterialization(message);
    var style = fillStyle();
    style.padding = new Insets(10.0, 10.0, 10.0, 10.0);
    style.background = appearance.theme.tokens.surface;
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
      app.sceneInspector = new PropertyInspector("scene-inspector:" + scene.selectedId,
        scene.properties(), style, null, null, null, "Selected object inspector");
      app.inspectorSelectionRevision = scene.selectionRevision;
    }
    var inspector = app.sceneInspector;
    inspector.setStyle(style);
    inspector.labelWidth = viewportWidth < 760.0 ? 56.0 :
      viewportWidth < 1180.0 ? 76.0 : 108.0;
    var ownership=session.scriptOwnership;
    inspector.enabled = ownership==null&&!simulation.isActive() &&
      (perspectiveViewport == null || !perspectiveViewport.dragging());
    var rows:Array<KeyedView> = [new KeyedView("heading",sectionHeading(selected.label))];
    if (simulation.isActive()) rows.push(new KeyedView("simulation-hint",
      new Text(HierarchyPanel.SIMULATION_LOCK_HINT, null, appearance.theme.tokens.textSecondary,
        TextStyleOverride.text(12.0))));
    var assembly = session.projectAssembly;
    if (assembly != null && StringTools.startsWith(selected.id, "project:")) {
      var instanceId = selected.id.substr(8);
      var jointLines:Array<String> = [];
      for (joint in assembly.joints) if (joint.parent == instanceId || joint.child == instanceId)
        jointLines.push(joint.id + " · " + joint.kind + " · " +
          joint.parentConnector + " → " + joint.childConnector);
      if (jointLines.length > 0)
        rows.push(new KeyedView("assembly-joints", textLines("assembly-joint-lines", jointLines)));
    }
    if (StringTools.startsWith(selected.id, "project:")) {
      // When the part's mates leave it one turn or slide, offer the joint they amount to.
      var mateJoint = session.assemblyMateJoint(selected.id);
      var mateJointType = mateJoint == null ? null : mateJoint.type;
      if (mateJointType != null)
        rows.push(new KeyedView("convert-mates-to-joint", sceneAction("convert-mates-to-joint", "assembly.convert-to-joint",
          "Make " + mateJointType + " joint", IconName.Plus)));
    }
    if (scene.hasActiveSketchEdit()) {
      var summary = scene.sketchEditSummary();
      if (summary != null)
        rows.push(new KeyedView("sketch-draft-status", new Text(summary)));
      var sketchTools:Array<KeyedView> = [];
      if (scene.canAddSketchDraftRectangle())
        sketchTools.push(new KeyedView("add-rectangle",
          sceneAction("add-sketch-rectangle", "scene.add-sketch-rectangle", "Add rectangle", IconName.Plus)));
      if (scene.canClearSketchDraft())
        sketchTools.push(new KeyedView("clear-sketch",
          sceneAction("clear-sketch-draft", "scene.clear-sketch-draft", "Clear sketch", IconName.Close)));
      if (sketchTools.length > 0)
        rows.push(new KeyedView("sketch-draft-tools", new Row("sketch-draft-tools-row", sketchTools, actionRowStyle())));
      rows.push(new KeyedView("sketch-draft-actions", new Row("sketch-draft-actions-row", [
        new KeyedView("apply", sceneAction("apply-sketch-draft", "scene.apply-sketch", "Apply sketch", IconName.Save)),
        new KeyedView("cancel", sceneAction("cancel-sketch-draft", "scene.cancel-sketch", "Cancel", IconName.Close))
      ], actionRowStyle())));
    } else if (scene.canBeginSelectedSketchEdit()) {
      rows.push(new KeyedView("sketch-edit-action",
        sceneAction("edit-selected-sketch", "scene.edit-sketch", "Edit sketch", IconName.Inspect)));
    }
    var supportStatus = scene.selectedSketchSupportStatus();
    if (supportStatus != null) {
      rows.push(new KeyedView("sketch-support-status", new Text(supportStatus)));
      if (scene.canRepairSelectedSketchSupportFace())
        rows.push(new KeyedView("repair-sketch-support",
          sceneAction("repair-sketch-support-face", "scene.repair-sketch-support-face",
            "Repair support face", IconName.Inspect)));
    }
    if(scene.isCadPart(selected.id) && scene.hasCadOutput(selected.id))rows.push(new KeyedView("face-selection",
      new Text(scene.selectedCadEdgeIndex>=0?"Selected edge "+(scene.selectedCadEdgeIndex+1):
        scene.selectedCadFaceIndex<0?"Click a CAD face or edge to select it":
        selected.kind=="cad-plate"
          ?"Selected face "+(scene.selectedCadFaceIndex+1)+" · Add hole uses the picked location"
          :"Selected face "+(scene.selectedCadFaceIndex+1))));
    if(selected.kind=="cad-preview"&&scene.selectedCadEdgeIndex>=0)
      rows.push(new KeyedView("edge-selection",
        new Text("Selected edge "+(scene.selectedCadEdgeIndex+1))));
    var kindProvider = ObjectKindRegistry.find(selected.kind);
    if (kindProvider != null) for (commandId in kindProvider.commands(scene, selected.id)) {
      switch (commandId) {
        case "scene.create-sketch": rows.push(new KeyedView("create-sketch",
          sceneAction("create-constrained-sketch", commandId, "Create sketch", IconName.Plus)));
        case "scene.create-face-sketch": rows.push(new KeyedView("create-face-sketch",
          sceneAction("create-face-sketch", commandId, "Sketch on face", IconName.Plus)));
        case "scene.create-extrusion": rows.push(new KeyedView("create-extrusion",
          sceneAction("create-extrusion", commandId, "Extrude", IconName.Plus)));
        case "scene.create-pocket": rows.push(new KeyedView("create-pocket",
          sceneAction("create-pocket", commandId, "Pocket", IconName.Plus)));
        case "scene.create-vertical-fillet": rows.push(new KeyedView("create-vertical-fillet",
          sceneAction("create-vertical-fillet", commandId, "Fillet vertical edges", IconName.Plus)));
        case "scene.worker-add-step": rows.push(new KeyedView("worker-add-step",
          sceneAction("worker-add-step", commandId, "Add step", IconName.Plus)));
        case "scene.worker-remove-step": rows.push(new KeyedView("worker-remove-step",
          sceneAction("worker-remove-step", commandId, "Remove step", IconName.Close)));
        case "scene.worker-step-up": rows.push(new KeyedView("worker-step-up",
          sceneAction("worker-step-up", commandId, "Move step up", IconName.Plus)));
        case "scene.worker-step-down": rows.push(new KeyedView("worker-step-down",
          sceneAction("worker-step-down", commandId, "Move step down", IconName.Plus)));
        default:
      }
    }
    if(ownership!=null) {
      rows.push(new KeyedView("origin",textLines("script-object-origins",
        ["Script-owned"].concat(ownership.propertyOrigins(selected.id,["position","dimensions",
          "mass","collisionEnabled","dynamicBody","visible"])))));
      rows.push(new KeyedView("revert",new Button("Revert object overrides",null,function(){
        if(ownership.revertTarget(selected.id))refreshScriptMaterialization("Object overrides reverted");
      },"script-object-revert")));
    }
    rows.push(new KeyedView("properties",inspector));
    var sheetWorkflow = SheetWorkflowPanel.build(app);
    var projectPanel = app.projectUiPanel();
    if (projectPanel != null) rows.push(new KeyedView("project-ui", projectPanel));
    if (sheetWorkflow != null) rows.push(new KeyedView("sheet-workflow", sheetWorkflow));
    return new Column(
      "inspector-panel",
      rows,
      style
    );
  }

}
