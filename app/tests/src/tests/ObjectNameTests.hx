package tests;

import app.EditorSceneTree;
import app.ProjectDocumentSession;
import app.ProjectSceneRecord.ProjectFieldOverride;

/** Generated names and explicit document overrides have one visible value. */
@:access(app
  .ProjectDocumentSession) @:access(app.EditorSceneTree) @:access(app.Main.ReferenceEditorApp) class ObjectNameTests {
  static var assertions = 0;
  static function check(value:Bool, message:String):Void {
    assertions++;
    if (!value) throw message;
  }

  public static function main():Void {
    var editor = new app.Main.ReferenceEditorApp();
    var session = editor.session;
    var fixture = "/tmp/materia-object-name-fixture-" + Std.random(1000000000) + ".mtrg";
    try {
      var vertices = haxe.io.Bytes.alloc(96), normals = haxe.io.Bytes.alloc(96);
      var indices = haxe.io.Bytes.alloc(48);
      vertices.setDouble(24, 1);
      vertices.setDouble(56, 1);
      vertices.setDouble(88, 1);
      var triangles = [0, 2, 1, 0, 1, 3, 0, 3, 2, 1, 2, 3];
      for (i in 0...4) normals.setDouble(i * 24 + 16, 1);
      for (i in 0...triangles.length) indices.setInt32(i * 4, triangles[i]);
      var artifact:materia.project.SceneArtifact.SceneArtifactData = {
        metresPerUnit: 1,
        parts: [{
          id: "internal-frame-id",
          name: "Source steel frame",
          red: 0.5,
          green: 0.5,
          blue: 0.5,
          vertexCount: 4,
          indexCount: 12,
          vertices: vertices,
          normals: normals,
          indices: indices,
          faceRanges: []
        }]
      };
      var bytes = materia.project.SceneArtifact.encode(artifact);
      sys.io.File.saveBytes(fixture, bytes);
      var generated = new app.ProjectArtifactLoader().load(bytes);
      session.openGeneratedProject(generated, fixture);
      editor.documentChanged();
      var source = session.scene.records()[0];
      var tree = editor.treeModel;
      var id = source.id;
      check(tree.displayLabel(id) == "Source steel frame", "Tree uses the source name, never a prettified ID");
      var item = session.scene.object(id);
      if (item == null) throw "Missing test object";
      check(tree.visibilityLabel(id) == item.label, "Visibility and Rename use the same name");
      editor.startRename(id);
      check(editor.renameValue == tree.displayLabel(id), "Rename starts with exactly the tree name");
      editor.renameValue = "My frame";
      editor.finishRename();
      check(tree.displayLabel(id) == "My frame", "Rename changes the tree immediately");
      check(item.id == id, "Rename preserves identity");
      var edits:Array<ProjectFieldOverride> = [];
      ProjectDocumentSession.addDeltas(edits, id, session.scene.records()[0], source, true);
      check(
        edits.length == 1 && edits[0].property == "label" && edits[0].value == "My frame",
        "A rename persists as a label override on the stable ID"
      );
      var regenerated = session.scene.records()[0];
      regenerated.label = "Updated source frame";
      var diagnostics:Array<String> = [];
      var restored = ProjectDocumentSession.applyDeltas(regenerated, id, edits, diagnostics, []);
      check(restored.label == "My frame" && diagnostics.length == 0, "Rebuilding preserves the explicit name override");
      session.projectBaseline = [regenerated];
      session.scene.reconcileRecords([restored]);
      check(session.resetName(id), "A generated name can be reset");
      check(tree.displayLabel(id) == "Updated source frame", "Reset restores the current source name");
      edits = [];
      ProjectDocumentSession.addDeltas(edits, id, session.scene.records()[0], regenerated, true);
      check(edits.length == 0, "Reset removes the persisted override");
      check(!session.resetName("unknown"), "Reset never invents a source name");
      session.scene.setName(id, "Internal frame id");
      check(tree.displayLabel(id) == "Internal frame id", "An override resembling the ID remains literal");
      session.scene.select("scene");
      tree.setFilter("internal frame");
      check(tree.childCount("scene") == 1, "Search uses the same visible name");
      tree.setFilter("Source steel frame");
      check(tree.childCount("scene") == 0, "Search does not use a stale generated name");
      editor.dispose();
      if (sys.FileSystem.exists(fixture)) sys.FileSystem.deleteFile(fixture);
    } catch (error:Dynamic) {
      editor.dispose();
      if (sys.FileSystem.exists(fixture)) sys.FileSystem.deleteFile(fixture);
      throw error;
    }
    Sys.println('Object names: $assertions assertions passed');
  }
}
