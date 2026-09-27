package tests;

import app.EditorScene;
import app.SceneObjectData;
import nativekit.scene.SceneRenderer;
import nativekit.scene.SceneView;

/** Structural regression gate for large editor scenes; no timing threshold. */
@:access(app.EditorScene)
class EditorEditPerformanceTests {
  static function check(value:Bool, message:String):Void {
    if (!value) throw message;
  }

  public static function main():Int {
    var records:Array<SceneObjectData> = [];
    for (index in 0...1000) records.push({
      id: "object-" + index, label: "Object " + index, type: "rectangle",
      x: index * 2.0, y: 0.0, z: 0.0, width: 1.0, height: 1.0, depth: 1.0,
      collisionEnabled: true, dynamicBody: false, mass: 1.0,
      red: 0.2, green: 0.4, blue: 0.6, visible: true
    });
    var scene = new EditorScene(records);
    var renderer = SceneRenderer.createHeadless();
    try {
      var view = new SceneView();
      renderer.render(scene.renderSnapshot(), view);
      var reconciliations = scene.fullReconciliationCount;
      var spatialRebuilds = scene.spatialFullRebuildCount;
      var environment = scene.environmentRevision;
      scene.setColour("object-500", 0.8, 0.3, 0.1);
      check(scene.fullReconciliationCount == reconciliations,
        "colour edit does not reconcile all scene objects");
      check(scene.spatialFullRebuildCount == spatialRebuilds,
        "colour edit does not rebuild the spatial index");
      check(scene.environmentRevision == environment,
        "colour edit does not change the physics environment");
      var changes = scene.takeRenderChanges();
      check(changes != null, "colour edit emits a SceneKit change set");
      renderer.render(scene.renderSnapshot(), view, changes);
      var update = renderer.lastUpdate();
      check(update != null && update.get_plan_rebuilt() == 0 &&
        haxe.Int64.toInt(update.get_updated_material_resources()) <= 1,
        "renderer applies one incremental material update");
      changes.dispose();
      var content = scene.revision;
      check(scene.select("object-500"), "selection changes");
      check(scene.revision == content && scene.fullReconciliationCount == reconciliations &&
        scene.spatialFullRebuildCount == spatialRebuilds && scene.takeRenderChanges() == null,
        "selection does not rebuild scene content or renderer resources");
    } catch (error:Dynamic) { renderer.dispose(); scene.dispose(); throw error; }
    renderer.dispose();
    scene.dispose();
    Sys.println("Editor edit performance counters passed");
    return 0;
  }
}
