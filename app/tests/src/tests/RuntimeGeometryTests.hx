package tests;

import app.EditorScene;
import haxe.io.Bytes;
import nativekit.scene.GeometryData;

/** Runtime geometry shown in parts, such as stock a simulated machine is cutting, and how it ends. */
@:access(app.EditorScene)
class RuntimeGeometryTests {
  static function check(value:Bool, message:String):Void if (!value) throw message;

  static function triangle(offset:Float):GeometryData {
    var positions = Bytes.alloc(36);
    for (vertex in 0...3) {
      positions.setFloat(vertex * 12, offset + (vertex == 1 ? 0.01 : 0.0));
      positions.setFloat(vertex * 12 + 4, vertex == 2 ? 0.01 : 0.0);
      positions.setFloat(vertex * 12 + 8, 0.0);
    }
    var indices = Bytes.alloc(12);
    for (index in 0...3) indices.setInt32(index * 4, index);
    var data = new GeometryData();
    data.addStream(1, 2, positions, 3, 12);
    data.setIndexBuffer(indices, 3);
    data.setBounds(offset, 0, 0, offset + 0.01, 0.01, 0);
    return data;
  }

  static function live(scene:EditorScene, node:nativekit.scene.NodeId):Bool
    return scene.renderSnapshot().findNode(node) != null;

  public static function main():Int {
    var scene = new EditorScene([]);
    try {
      check(scene.createRectangle(), "runtime geometry test creates an object");
      var id = scene.selectedId;
      var authored = scene.runtimeFor(id).geometry;
      check(scene.createRectangle(), "batch runtime geometry test creates a second object");
      var second = scene.selectedId, secondAuthored = scene.runtimeFor(second).geometry;
      var revision = scene.revision, visualRevision = scene.visualRevision, environmentRevision = scene.environmentRevision;
      scene.setRuntimeGeometries([id => triangle(0.04), second => triangle(0.08)]);
      check(scene.revision == revision && scene.environmentRevision == environmentRevision &&
        scene.visualRevision == visualRevision + 1,
        "related runtime meshes publish one visual revision without invalidating document or physics caches");
      check(scene.runtimeFor(id).geometry != authored && scene.runtimeFor(second).geometry != secondAuthored,
        "a batch gives each object independent runtime geometry");
      var firstBounds = scene.info(id).bounds(), secondBounds = scene.info(second).bounds();
      check(firstBounds.valid && secondBounds.valid && Math.abs(firstBounds.maxX - firstBounds.minX - 0.01) < 1e-6 &&
        Math.abs(secondBounds.maxX - secondBounds.minX - 0.01) < 1e-6,
        "the batch snapshot contains both updated meshes");
      revision = scene.revision; visualRevision = scene.visualRevision;
      scene.setRuntimeGeometries([id => triangle(0.12), second => triangle(0.16)]);
      check(scene.revision == revision && scene.visualRevision == visualRevision + 1 &&
        scene.info(id).bounds().minX > firstBounds.maxX,
        "subsequent batched updates refresh the retained geometry and bounds");
      scene.clearRuntimeGeometry(id); scene.clearRuntimeGeometry(second);
      check(scene.runtimeFor(id).geometry == authored && scene.runtimeFor(second).geometry == secondAuthored,
        "both batched meshes restore their authored geometry");
      scene.select(id);
      scene.setRuntimeGeometryParts(id, 2, [0 => triangle(0), 1 => triangle(0.02)]);
      var parts = scene.runtimePartNodes(id);
      check(parts.length == 2 && live(scene, parts[0]) && live(scene, parts[1]),
        "each runtime part is a node of its own");
      check(scene.bridge.idForNode(parts[1]) == id, "a runtime part picks as its object");
      check(scene.runtimeFor(id).geometry != authored, "the object's own geometry is hidden behind its parts");
      scene.setRuntimeGeometryParts(id, 2, [1 => triangle(0.03)]);
      check(scene.runtimePartNodes(id).join(",") == parts.join(","), "an update replaces a part's geometry in place");

      scene.clearRuntimeGeometry(id);
      check(scene.revision == revision && scene.environmentRevision == environmentRevision,
        "runtime part updates and restoration keep document and physics revisions stable");
      check(scene.runtimePartNodes(id).length == 0 && !live(scene, parts[0]) && !live(scene, parts[1]),
        "clearing removes the part nodes");
      check(scene.runtimeFor(id).geometry == authored && scene.bridge.idForNode(parts[0]) == null,
        "clearing gives the object its own geometry back");

      // Deleting the object takes its parts along; the scene does not destroy children with a parent.
      scene.setRuntimeGeometryParts(id, 1, [0 => triangle(0)]);
      var orphan = scene.runtimePartNodes(id)[0];
      check(scene.deleteSelected(), "the object deletes");
      check(!live(scene, orphan) && scene.runtimePartNodes(id).length == 0 &&
        !scene.ownedRuntimeGeometry.exists(id) && !scene.runtimeOriginalGeometry.exists(id),
        "deleting an object removes its runtime parts and forgets its runtime geometry");
      scene.dispose();
    } catch (error:Dynamic) {
      scene.dispose();
      Sys.println('Runtime geometry tests failed: $error');
      return 1;
    }
    Sys.println("Runtime geometry tests passed");
    return 0;
  }
}
