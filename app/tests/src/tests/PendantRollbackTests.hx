package tests;

import NativeKitScene;

import app.EditorSceneTree;
import materia.project.SceneArtifact;
import nativekit.scene.GeometryData;

/** A generated pendant part rolls back through its recorded steps, in the frame of its static geometry. */
@:access(app.EditorScene) class PendantRollbackTests {
  static var checks = 0;
  static function check(ok:Bool, message:String):Void {
    checks++;
    if (!ok) throw message;
  }
  /** Lowest and highest z of a geometry's positions (float32 triples). */
  static function zRange(geometry:GeometryData):{low:Float, high:Float} {
    var stream = geometry.stream(NksceneVertexSemantic.Position);
    if (stream == null) throw "Geometry has no positions";
    var stride = stream.stride == 0 ? 12 : stream.stride, low = 1e30, high = -1e30;
    for (index in 0...stream.count) {
      var z = stream.bytes.getFloat(index * stride + 8);
      low = Math.min(low, z);
      high = Math.max(high, z);
    }
    return {low: low, high: high};
  }
  public static function main():Int {
    run();
    return 0;
  }
  public static function run():Void {
    var bytes = SceneArtifact.encode(machinekit.pendant.PendantScene.build());
    var generated = new app.ProjectArtifactLoader().load(bytes);
    var centre = generated.localCentersByDefinition.get("case/plate");
    if (centre == null) throw "The plate has no local centre";
    var session = new app.ProjectDocumentSession(null, false);
    session.openGeneratedProject(generated, "/tmp/pendant-rollback.mtrg");
    var scene = session.scene, tree = new EditorSceneTree(scene, session.projectAssemblyDefinition);
    var map = scene.sourceMap;
    if (map == null) throw "The pendant scene has no source map";
    try {
      var plate = "project:case/plate";
      check(scene.histories.has(plate) && scene.histories.has("project:case/shell"), "Parts carry their recorded histories");
      var base:Null<materia.project.SceneSourceMap.SceneSourceFeature> = null, later:Null<materia.project.SceneSourceMap.SceneSourceFeature> = null;
      for (feature in map.features) {
        if (feature.occurrence != "case/plate") continue;
        if (base == null && StringTools.endsWith(feature.label, "Face plate")) base = feature;
        if (later == null && StringTools.startsWith(feature.label, "Bosses · ")) later = feature;
      }
      if (base == null || later == null) throw "The plate has a base step and a later step";
      var baseId = base.id, laterLabel = later.label, laterFaces = later.faces.copy(), baseFaces = base.faces.copy();
      check(laterFaces.length > 0 && !scene.isHistoryRolledBack(plate), "The whole plate is shown at first");
      check(scene.canRollBackToStep(baseId), "The base step can be rolled back to");
      var full = scene.histories.session(plate).geometry();
      var fullVertices = full.vertexCount(), fullZ = zRange(full);

      check(scene.rollBackToStep(baseId), "Rolling back to a step succeeds");
      check(scene.isHistoryRolledBack(plate), "The part reports being rolled back");
      var rolled = scene.histories.session(plate).geometry();
      check(rolled.vertexCount() < fullVertices, "The shown geometry is the smaller, earlier shape");
      var scale = generated.metresPerUnit, rolledZ = zRange(rolled);
      check(Math.abs(rolledZ.low - (0.0 - centre[2]) * scale) < 1e-6 && Math.abs(rolledZ.high - (3.0 - centre[2]) * scale) < 1e-6, "The earlier shape sits in the static geometry's frame: " + rolledZ.low + ".." + rolledZ.high);
      check(fullZ.low < rolledZ.low - 1e-6, "The full plate reaches below the base, where the bosses are");
      check(StringTools.startsWith(later.label, "Rolled back · ") && later.faces.length == 0, "Later steps are marked and own no faces");
      var shown = scene.histories.session(plate).copyPublishedShape();
      var shownFaces = shown.subshapeCount(CadKit.ShapeKind.Face);
      shown.close();
      check(base.faces.length == shownFaces, "The base step owns all " + shownFaces + " faces of the shown shape, not " + base.faces.length);
      tree.selectKey("source:" + baseId);
      scene.renderSnapshot();
      check(scene.presentation.sourceFaces.active.length == shownFaces, "Selecting the base step lights the shown shape's faces");
      check(!scene.canRollBackToStep(baseId) && scene.isHistoryRolledBack(plate), "Rolling to the shown step again is not offered");

      check(scene.rollHistoryForward(plate), "Rolling forward succeeds");
      check(!scene.isHistoryRolledBack(plate), "The marker is cleared");
      check(later.label == laterLabel && later.faces.length == laterFaces.length, "Later steps are restored");
      check(base.faces.length == baseFaces.length, "The base step's faces are restored");
      var forward = zRange(scene.histories.session(plate).geometry());
      check(Math.abs(forward.low - fullZ.low) < 1e-6 && Math.abs(forward.high - fullZ.high) < 1e-6, "The full plate is shown again");
    } catch (error:Dynamic) {
      session.dispose();
      throw error;
    }
    session.dispose();
    Sys.println("Pendant rollback: " + checks + " checks passed");
  }
}
