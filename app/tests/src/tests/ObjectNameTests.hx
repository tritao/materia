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
      check(tree.displayLabel(id) == "Internal frame id", "A non-assembly artifact also uses the occurrence role");
      check(session.scene.objectDetail(id) == "Source steel frame · internal-frame-id",
        "Detail retains the part number and full occurrence path");
      check(session.scene.partNumbers.get(id) == "Source steel frame", "Part identity is separate from its role");
      session.scene.select(id);
      var properties = session.scene.properties();
      var numberFields = properties.filter(p -> p.label == "Part number");
      var pathFields = properties.filter(p -> p.label == "Occurrence path");
      check(numberFields.length == 1 && numberFields[0].readOnly
        && (switch numberFields[0].readValue() {case Text(value): value == "Source steel frame"; default: false;}),
        "Inspector exposes the source part number as read-only");
      check(pathFields.length == 1 && pathFields[0].readOnly
        && (switch pathFields[0].readValue() {case Text(value): value == "internal-frame-id"; default: false;}),
        "Inspector exposes the occurrence path as read-only");
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
      check(tree.childCount("scene") == 1, "Search retains the part number after renaming the role");
      tree.setFilter("internal-frame-id");
      check(tree.childCount("scene") == 1, "Search retains the complete occurrence path");
      artifact.assemblyDefinition = {
        schemaVersion:materia.assembly.AssemblyDefinitionCodec.VERSION, id:"role-fixture", lengthUnit:"m",
        definitions:[{id:"internal-frame-id", connectors:[]}],
        occurrences:[for (role in ["left/bolt", "right/bolt", "third/bolt", "machine/rightHanger1"]) {
          id:role, definition:"internal-frame-id", initialPose:materia.assembly.AssemblyFrames.identity(),
          groupPath:role == "third/bolt" ? "separate" : "hardware"
        }], joints:[]
      };
      var shared = new app.ProjectArtifactLoader().load(materia.project.SceneArtifact.encode(artifact));
      function roleLabel(role:String):String {
        for (record in shared.objects) if (record.id == "project:" + role) return record.label;
        throw "Missing role fixture " + role;
      }
      check(roleLabel("left/bolt") == "Bolt (1)" && roleLabel("right/bolt") == "Bolt (2)",
        "Duplicate siblings are distinct without full paths");
      check(roleLabel("third/bolt") == "Bolt", "Labels in separate authored groups need no suffix");
      check(roleLabel("machine/rightHanger1") == "Right hanger 1", "Reused definitions still name the role");
      var sharedState = shared.assemblyState;
      if (sharedState == null) throw "Missing shared assembly state";
      var moved = app.MateriaProjectRunner.evaluateAssemblyState(shared, sharedState);
      check(moved.partNumbers != null && moved.partNumbers.get("project:left/bolt") == "Source steel frame",
        "Assembly reevaluation retains source identity");
      session.openGeneratedProject(shared, fixture);
      editor.documentChanged();
      session.scene.select("scene");
      tree = editor.treeModel;
      tree.setFilter("right/bolt");
      check(tree.matches("project:right/bolt") && !tree.matches("project:left/bolt"),
        "Search distinguishes complete occurrence paths for shared definitions");
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
