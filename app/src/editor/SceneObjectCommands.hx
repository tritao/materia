package app.editor;

import app.Main.ReferenceEditorApp;
import nativekit.ui.core.Command;
import nativekit.ui.core.Shortcut;
import nativekit.ui.core.UiModifier;

/** Object and CAD editing command registrations. */
@:access(app.Main.ReferenceEditorApp)
class SceneObjectCommands {
  public static function install(app:ReferenceEditorApp):Void {
    app.commands.register(new Command("scene.create", "Add rectangle", function() {
      app.scene.createRectangle();
      app.commands.refresh();
    }, null, function() return app.canEditObjects() && app.scene.canCreate()));
    app.commands.register(new Command("scene.create-part", "Add empty CAD part", function() {
      app.runSceneEdit("Could not add CAD part", function() app.scene.createCadPart());
    }, null, function() return app.canEditObjects() && app.scene.canCreate()));
    app.commands.register(new Command("scene.create-sketch", "Create constrained sketch", function() {
      app.runSceneEdit("Could not create sketch", function() app.scene.createSketch());
    }, null, function() return app.canEditObjects() && app.scene.canCreateSketch()));
    app.commands.register(new Command("scene.create-extrusion", "Extrude selected sketch", function() {
      app.runSceneEdit("Could not create extrusion", function() app.scene.createExtrusion());
    }, null, function() return app.canEditObjects() && app.scene.canCreateExtrusion()));
    app.commands.register(new Command("scene.create-face-sketch", "Sketch on selected face", function() {
      app.runSceneEdit("Could not create face sketch", function() app.scene.createFaceSketch());
    }, null, function() return app.canEditObjects() && app.scene.canCreateFaceSketch()));
    app.commands.register(new Command("scene.create-pocket", "Pocket selected face sketch", function() {
      app.runSceneEdit("Could not create pocket", function() app.scene.createPocket());
    }, null, function() return app.canEditObjects() && app.scene.canCreatePocket()));
    app.commands.register(new Command("scene.create-vertical-fillet", "Fillet vertical edges", function() {
      app.runSceneEdit("Could not fillet vertical edges", function() app.scene.createVerticalFillet());
    }, null, function() return app.canEditObjects() && app.scene.canCreateVerticalFillet()));
    app.commands.register(new Command("scene.create-plate", "Add mounting plate", function() {
      app.runSceneEdit("Could not add mounting plate", function() app.scene.createMountingPlate());
    }, null, function() return app.canEditObjects() && app.scene.canCreate()));
    app.commands.register(new Command("scene.create-stock-simulation", "Add stock simulation", function() {
      app.runSceneEdit("Could not add stock simulation", function() app.scene.createStockSimulation());
    }, null, function() return app.canEditObjects() && app.scene.canCreate()));
    app.commands.register(new Command("scene.create-worker", "Add worker", function() {
      app.runSceneEdit("Could not add worker", function() app.scene.createWorker());
    }, null, function() return app.canEditObjects() && app.scene.canCreate()));
    for (command in ["scene.worker-add-step","scene.worker-remove-step",
        "scene.worker-step-up","scene.worker-step-down"]) {
      var selected = command;
      app.commands.register(new Command(selected, switch selected {
        case "scene.worker-add-step": "Add job step";
        case "scene.worker-remove-step": "Remove job step";
        case "scene.worker-step-up": "Move job step up";
        default: "Move job step down";
      }, function() app.runSceneEdit("Could not edit worker job", function()
        return HumanWorkerKind.command(app.scene, selected)), null,
        function() {
          var selectedObject = app.scene.object(app.scene.selectedId);
          return app.canEditObjects() && selectedObject != null && selectedObject.kind == HumanWorkerKind.KIND;
        }));
    }
    app.commands.register(new Command("scene.create-bracket", "Add L bracket", function() {
      app.runSceneEdit("Could not add L bracket", function() app.scene.createBracket());
    }, null, function() return app.canEditObjects() && app.scene.canCreate()));
    app.commands.register(new Command("scene.edit-sketch", "Edit selected sketch", function() {
      try {
        app.scene.beginSelectedSketchEdit();
        } catch (error:Dynamic) app.log("Could not edit sketch: " + Std.string(error));
      app.commands.refresh();
    }, null, function() return app.canEditObjects() && app.scene.canBeginSelectedSketchEdit()));
    app.commands.register(new Command("scene.repair-sketch-support-face", "Repair selected sketch support face", function() {
      app.runSceneEdit("Could not repair sketch support face", function() app.scene.repairSelectedSketchSupportFace());
      app.commands.refresh();
    }, null, function() return app.canEditObjects() && app.scene.canRepairSelectedSketchSupportFace()));
    app.commands.register(new Command("scene.apply-sketch", "Apply sketch draft", function() {
      app.runSceneEdit("Could not apply sketch draft", function() app.scene.applySelectedSketchEdit());
      app.commands.refresh();
    }, null, function() return app.canEditObjects() && app.scene.canApplySelectedSketchEdit()));
    app.commands.register(new Command("scene.cancel-sketch", "Cancel sketch draft", function() {
      app.scene.cancelSelectedSketchEdit();
      app.commands.refresh();
    }, null, function() return app.scene.hasActiveSketchEdit()));
    app.commands.register(new Command("scene.add-sketch-rectangle", "Add starter rectangle to sketch", function() {
      app.runSceneEdit("Could not add sketch rectangle", function() app.scene.addSketchDraftRectangle());
      app.commands.refresh();
    }, null, function() return app.canEditObjects() && app.scene.canAddSketchDraftRectangle()));
    app.commands.register(new Command("scene.clear-sketch-draft", "Clear sketch geometry", function() {
      app.runSceneEdit("Could not clear sketch", function() app.scene.clearSketchDraft());
      app.commands.refresh();
    }, null, function() return app.canEditObjects() && app.scene.canClearSketchDraft()));
    app.commands.register(new Command("scene.import-step", "Import STEP part", function() {
      var chooser=app.files;
      if(chooser==null)return;
      chooser.chooseImport("Import STEP part",function(path,error) {
        if(error!=null){app.log(error);return;}
        if(path==null)return;
        try {
          app.scene.importStep(path);
          app.log("Imported STEP part: "+path);
        } catch(failure:Dynamic) app.log("STEP import failed: "+Std.string(failure));
        app.commands.refresh();
      });
    },null,function() return app.canEditObjects()&&app.files!=null&&app.scene.canCreate()));
    app.commands.register(new Command("scene.add-face-hole", "Add hole on selected face", function() {
      app.runSceneEdit("Could not add hole", function() app.scene.addHoleOnSelectedFace());
    },null,function() return app.canEditObjects()&&app.scene.canAddHoleOnSelectedFace()));
    app.commands.register(new Command("scene.export-step", "Export STEP", function() {
      var chooser=app.files;
      if(chooser==null)return;
      chooser.chooseExport("Export selected CAD part","CAD part.step",function(path,error) {
        if(error!=null){app.log(error);return;}
        if(path==null)return;
        try { app.scene.exportSelectedCad(path); app.log("Exported STEP: "+path); }
        catch(failure:Dynamic) app.log("STEP export failed: "+Std.string(failure));
        app.commands.refresh();
      });
    },null,function() {
      var selected=app.scene.object(app.scene.selectedId);
      return !app.documents.blocked()&&app.files!=null&&selected!=null&&app.scene.isCadPart(selected.id);
    }));
    app.commands.register(new Command("scene.duplicate", "Duplicate", function() {
      app.runSceneEdit("Could not duplicate object", function() app.scene.duplicateSelected());
    }, new Shortcut(68, UiModifier.Control), function() return app.canEditObjects()
      && app.scene.canCreate() && app.scene.object(app.scene.selectedId) != null));
    app.commands.register(new Command("scene.delete", "Delete", function() {
      app.scene.deleteSelected();
      app.commands.refresh();
    }, null, function() return app.canEditObjects() && app.scene.object(app.scene.selectedId) != null));
  }
}
