package tests;

import app.EditorScene;

/** Failure-injection regressions for the editor's SceneKit publication boundary. */
class SceneAtomicityTests {
  static function check(value:Bool, message:String):Void {
    if (!value) throw message;
  }

  static function rejects(action:Void->Void, message:String):Void {
    var rejected = false;
    try action() catch (_:Dynamic) rejected = true;
    check(rejected, message);
  }

  static function runtimeNodeCount(scene:EditorScene):Int {
    var snapshot = scene.bridge.scene.snapshot();
    try {
      var result = snapshot.nodeCount();
      snapshot.dispose();
      return result;
    } catch (error:Dynamic) {
      snapshot.dispose();
      throw error;
    }
  }

  static function runtimeRevision(scene:EditorScene):haxe.Int64 {
    var snapshot = scene.bridge.scene.snapshot();
    try {
      var result = snapshot.revision();
      snapshot.dispose();
      return result;
    } catch (error:Dynamic) {
      snapshot.dispose();
      throw error;
    }
  }

  static function creationFailure(point:String):Void {
    var scene = new EditorScene();
    var oldObjects = scene.items().length;
    var oldNodes = runtimeNodeCount(scene);
    var oldUndo = scene.document.history.undoCount;
    scene.failureInjection = function(current) {
      if (current == point) throw 'injected failure at $point';
    };
    var cachePublicationFailure = StringTools.startsWith(point, "publish.");
    if (cachePublicationFailure) {
      check(scene.createRectangle(), 'cache failure keeps the committed edit at ' + point);
    } else {
      rejects(function() scene.createRectangle(), 'creation fails at ' + point);
    }
    scene.failureInjection = null;

    var present = scene.object("rectangle-1") != null;
    if (present) {
      check(scene.items().length == oldObjects + 1, point + ": committed object is listed");
      check(scene.document.history.undoCount == oldUndo + 1,
        point + ": committed object has its undo entry");
      check(runtimeNodeCount(scene) == oldNodes + 1,
        point + ": committed object has its SceneKit node");
      check(scene.renderSnapshot().nodeCount() == oldNodes + 1,
        point + ": stale render snapshot rebuilds from the committed edit");
      if (cachePublicationFailure)
        check(scene.pick(0.0, 0.0) == "rectangle-1",
          point + ": stale spatial index rebuilds before picking");
      check(scene.document.undo(), point + ": committed object can be undone");
      check(scene.object("rectangle-1") == null && runtimeNodeCount(scene) == oldNodes,
        point + ": undo removes the object and SceneKit node together");
      check(scene.document.redo(), point + ": committed object can be redone");
      check(scene.object("rectangle-1") != null && runtimeNodeCount(scene) == oldNodes + 1,
        point + ": redo restores the object and SceneKit node together");
    } else {
      check(scene.items().length == oldObjects,
        point + ": rejected object is absent from the object list");
      check(scene.document.history.undoCount == oldUndo,
        point + ": rejected object has no undo entry");
      check(runtimeNodeCount(scene) == oldNodes,
        point + ": rejected object has no SceneKit node");
      check(scene.bridge.runtime("rectangle-1") == null,
        point + ": rejected object has no bridge mapping");
    }
    scene.dispose();
  }

  static function existingObjectFailure(point:String, edit:EditorScene->Void):Void {
    var scene = new EditorScene();
    var before = scene.object("box");
    if (before == null) throw "atomicity fixture is missing box";
    var oldWidth = before.width;
    var oldRed = before.red;
    var oldUndo = scene.document.history.undoCount;
    var oldNativeRevision = runtimeRevision(scene);
    scene.failureInjection = function(current) {
      if (current == point) throw 'injected failure at $point';
    };
    rejects(function() edit(scene), 'existing-object edit fails at ' + point);
    scene.failureInjection = null;

    var after = scene.object("box");
    if (after == null) throw point + ": edit removed the box";
    check(after.width == oldWidth && after.red == oldRed,
      point + ": rejected edit preserves candidate object fields");
    check(scene.document.history.undoCount == oldUndo,
      point + ": rejected edit does not create a history entry");
    check(runtimeNodeCount(scene) == scene.items().length,
      point + ": rejected edit preserves SceneKit/object correspondence");
    check(runtimeRevision(scene) == oldNativeRevision,
      point + ": rejected edit preserves the SceneKit resource revision");
    scene.dispose();
  }

  static function cadPreparationFailure():Void {
    var scene = new EditorScene();
    var oldObjects = scene.items().length;
    var oldNodes = runtimeNodeCount(scene);
    var oldUndo = scene.document.history.undoCount;
    scene.failureInjection = function(point) {
      if (point == "prepare.cad-session") throw "injected CAD session preparation failure";
    };
    rejects(function() scene.createMountingPlate(), "CAD session preparation failure is injected");
    scene.failureInjection = null;
    check(scene.items().length == oldObjects && scene.object("plate-1") == null,
      "failed CAD preparation publishes no object");
    check(runtimeNodeCount(scene) == oldNodes && scene.bridge.runtime("plate-1") == null,
      "failed CAD preparation publishes no SceneKit node or bridge mapping");
    check(scene.document.history.undoCount == oldUndo,
      "failed CAD preparation creates no undo entry");
    scene.dispose();
  }

  public static function run():Void {
    for (point in ["prepare.new-geometry", "prepare.new-material", "prepare.bridge-attach",
        "prepare.new-object", "transaction.before-commit",
        "publish.snapshot", "publish.spatial-index"])
      creationFailure(point);
    cadPreparationFailure();

    existingObjectFailure("prepare.existing-geometry", function(scene)
      scene.setDimensions("box", 2.0, 1.2));
    existingObjectFailure("prepare.existing-material", function(scene)
      scene.setColour("box", 0.4, 0.5, 0.6));
    existingObjectFailure("prepare.existing-object", function(scene)
      scene.setDimensions("box", 2.0, 1.2));
  }

  public static function main():Int {
    run();
    return 0;
  }
}
