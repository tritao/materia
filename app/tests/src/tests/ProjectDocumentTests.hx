package tests;

import app.CadPlateModel;
import app.ProjectDocumentSession;
import sys.FileSystem;

class ProjectDocumentTests {
  static function check(value:Bool, message:String):Void {
    if (!value) throw message;
  }

  static function near(actual:Float, expected:Float, message:String):Void
    check(Math.abs(actual - expected) < 0.000001, message);

  public static function run():Void {
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

    check(session.scene.nudgeSelected(0.2, 0.0), "savepoint fixture scene edit applies");
    session.sensors.add("imu");
    if (!FileSystem.exists("build")) FileSystem.createDirectory("build");
    var path = "build/project-document-" + Std.random(100000000) + ".materia.json";
    session.save(path);
    check(!session.isDirty(), "one project save marks all owned state clean");
    check(session.document.undo(), "project history remains undoable after save");
    check(session.isDirty(), "undo away from the shared savepoint marks the project dirty");
    check(session.document.redo() && !session.isDirty(),
      "redo returns the entire project to the saved state");

    var previousDocument = session.document;
    session.open(path);
    check(session.document != previousDocument && session.scene.document == session.document &&
      session.sensors.document == session.document && !session.document.canUndo && !session.isDirty(),
      "Open replaces scene, sensors and project history together");
    previousDocument = session.document;
    session.newDocument();
    check(session.document != previousDocument && session.scene.document == session.document &&
      session.sensors.document == session.document && !session.document.canUndo,
      "New replaces every project-owned history together");
    session.dispose();
  }

  public static function main():Int {
    run();
    return 0;
  }
}
