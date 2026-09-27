package tests;

import app.CadPlateModel;
import app.ProjectDocumentSession;
import app.BimInspectorDescriptors;
import nativekit.ui.core.CommandContext;
import nativekit.ui.editing.EditOperation;
import nativekit.ui.editing.EditorDocument;
import nativekit.ui.properties.PropertyValue;
import sys.FileSystem;
import sys.io.File;
import haxe.Json;
import app.SceneCodec;
import app.EditorScene;

class ProjectDocumentTests {
  static function check(value:Bool, message:String):Void {
    if (!value) throw message;
  }

  static function near(actual:Float, expected:Float, message:String):Void
    check(Math.abs(actual - expected) < 0.000001, message);

  public static function run():Void {
    testCurrentSceneFixture();
    testProjectEditCoordinator();
    var session = new ProjectDocumentSession();
    check(session.scene.document == session.document && session.sensors.document == session.document,
      "scene and sensors share the project history instance");
    check(session.scene.nudgeSelected(0.1, 0.0), "project scene move applies");
    var sensor = session.sensors.add("imu");
    check(sensor != null, "project sensor change applies");
    check(session.scene.createMountingPlate(), "project CAD feature object applies");
    var cadId = session.scene.selectedId;
    var before = session.scene.cadParameters(cadId).holeDiameter;
    session.scene.setCadParameter(cadId, CadPlateModel.HOLE_DIAMETER, before * 1.25);
    check(session.document.history.undoCount == 4,
      "scene, sensor, CAD creation and CAD parameter edits share one ordered history");
    near(session.scene.cadParameters(cadId).holeDiameter, before * 1.25,
      "CAD feature edit publishes before undo");

    check(session.document.undo(), "global project undo restores CAD parameter");
    near(session.scene.cadParameters(cadId).holeDiameter, before,
      "CAD parameter undo restores its prior value");
    check(session.document.undo(), "global project undo removes CAD object");
    check(session.scene.object(cadId) == null, "CAD creation undo removes its object");
    check(session.document.undo(), "global project undo removes sensor");
    check(session.sensors.model.sensors.length == 1, "sensor undo restores the prior configuration");
    check(session.document.undo(), "global project undo restores scene move");
    near(session.scene.info("box").localTransform().element(12), -1.5,
      "scene undo restores its original position");
    check(!session.document.canUndo, "all four project edits were undone");

    check(session.scene.nudgeSelected(0.05, 0.0), "BIM ordering fixture scene edit applies");
    session.sensors.add("imu");
    check(session.scene.createMountingPlate(), "BIM ordering fixture CAD edit applies");
    session.applyBimEdit("Add BIM project", function() { session.bim.createProject("Project"); });
    var bimProject = session.bim.cad.allElements()[0];
    var bimName = BimInspectorDescriptors.forElement(bimProject,
      function(label, change) session.applyBimEdit(label, change))[2];
    bimName.write(new CommandContext(), PropertyValue.Text("Renamed Project"));
    check(session.document.history.undoCount == 5 && session.isDirty(),
      "BIM mutation joins scene, sensor and CAD edits in project history");
    check(bimProject.name == "Renamed Project", "BIM inspector edit is published");
    check(session.document.undo() && bimProject.name == "Project",
      "global undo reverses a BIM inspector edit");
    check(session.document.undo() && session.bim.cad.allElements().length == 0,
      "global undo reverses BIM creation after CAD, sensor and scene edits");
    check(session.document.redo() && session.document.redo() &&
      session.bim.cad.allElements()[0].name == "Renamed Project",
      "global redo reapplies BIM creation and the inspector edit in order");

    if (!FileSystem.exists("build")) FileSystem.createDirectory("build");
    var path = "build/project-document-" + Std.random(100000000) + ".materia.json";
    session.save(path);
    check(!session.isDirty(), "one project save marks all owned state clean");
    check(session.document.undo(), "project history remains undoable after save");
    check(session.isDirty(), "undo away from the shared savepoint marks the project dirty");
    check(session.document.redo() && !session.isDirty(),
      "redo returns the entire project to the saved state");
    var bimElementCount = session.bim.cad.allElements().length;

    var previousDocument = session.document;
    session.open(path);
    check(session.document != previousDocument && session.scene.document == session.document &&
      session.sensors.document == session.document && !session.document.canUndo && !session.isDirty(),
      "Open replaces scene, sensors and project history together");
    check(session.bim.cad.allElements().length == bimElementCount,
      "Open restores the versioned BIM project section");
    previousDocument = session.document;
    session.newDocument();
    check(session.document != previousDocument && session.scene.document == session.document &&
      session.sensors.document == session.document && !session.document.canUndo,
      "New replaces every project-owned history together");
    check(session.bim.cad.allElements().length == 0, "New clears project-owned BIM state");
    session.dispose();
  }

  static function testCurrentSceneFixture():Void {
    var objects = SceneCodec.decode(File.getContent("app/tests/fixtures/scene-current.json"));
    check(objects.length == 1 && objects[0].appearance != null,
      "scene fixture has current appearance data");
    var scene = new EditorScene(objects);
    try {
      var encoded = SceneCodec.encode(scene);
      var root:Dynamic = Json.parse(encoded);
      check(Reflect.field(root, "version") == SceneCodec.VERSION &&
        SceneCodec.decode(encoded).length == 1, "scene fixture re-encodes in current format");
      check(encoded.indexOf('"materialId"') >= 0 && encoded.indexOf('"visualOverrides"') >= 0 &&
        encoded.indexOf('"red"') < 0 && encoded.indexOf('"appearance"') < 0,
        "saved scene uses material references and sparse visual values");
      var custom:materia.project.MaterialDef = {id: "user-copper", name: "Copper",
        visual: {baseColor: [0.7, 0.35, 0.2], metallic: 0.9, roughness: 0.3},
        physical: {density: 8960, spec: "C110"}};
      var withCustom = SceneCodec.encode(scene, null, null, null, null, null, [custom]);
      check(SceneCodec.decodeCustomMaterialsRoot(SceneCodec.parse(withCustom)).length == 1,
        "project material library preserves custom materials");
    } catch (error:Dynamic) {
      scene.dispose();
      throw error;
    }
    scene.dispose();
  }

  static function testProjectEditCoordinator():Void {
    var document = new EditorDocument("compound-test");
    var coordinator = new app.ProjectEditCoordinator(document);
    var value = 0;
    var failFirstUndo = false;
    var steps = [
      new EditOperation("add one", function() { value += 1; }, function() {
        if (failFirstUndo) throw "injected undo failure";
        value -= 1;
      }),
      new EditOperation("add ten", function() { value += 10; }, function() { value -= 10; })
    ];
    coordinator.applyCompound("compound", steps);
    check(value == 11 && document.history.undoCount == 1,
      "compound project edit creates one history entry");
    failFirstUndo = true;
    var undoRejected = false;
    try document.undo() catch (_:Dynamic) undoRejected = true;
    check(undoRejected && value == 11 && document.canUndo,
      "failed compound undo compensates already undone components");
    failFirstUndo = false;
    check(document.undo() && value == 0 && document.redo() && value == 11,
      "compound project undo and redo preserve operation order");

    var applyFailed = false;
    try coordinator.applyCompound("failing compound", [
      new EditOperation("temporary", function() { value += 3; }, function() { value -= 3; }),
      new EditOperation("fail", failProjectApply, function() {})
    ]) catch (_:Dynamic) applyFailed = true;
    check(applyFailed && value == 11 && document.history.undoCount == 1,
      "failed compound apply compensates completed components and records no entry");
  }

  static function failProjectApply():Void throw "injected apply failure";

  public static function main():Int {
    run();
    return 0;
  }
}
