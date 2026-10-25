import app.MateriaProjectRunner;
import app.ProjectLoadControl;
import materia.project.SceneArtifact;
import haxe.io.Bytes;
import sys.io.File;
import sys.FileSystem;

class LinkedArtifactTests {
  static var assertions = 0;
  static function check(value:Bool, message:String):Void {
    assertions++;
    if (!value) throw message;
  }
  static function rejects(action:Void->Void, message:String):Void {
    var error = "";
    try action() catch (value:Dynamic) error = Std.string(value);
    check(error.length > 0 && error.indexOf(message) >= 0, "Expected rejection: " + message + "; got " + error);
  }
  static function fixture(label:String):Bytes {
    var vertices = Bytes.alloc(96), indices = Bytes.alloc(48);
    var coordinates = [0.0,0.0,0.0,1.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,1.0];
    var triangles = [0,2,1,0,1,3,0,3,2,1,2,3];
    for (i in 0...12) {
      vertices.setDouble(i * 8, coordinates[i]);
      indices.setInt32(i * 4, triangles[i]);
    }
    return SceneArtifact.encode({metresPerUnit:0.001, recipeDocument:label, parts:[{
      id:"tetra", name:"Tetra", red:0.5, green:0.5, blue:0.5,
      vertexCount:4, indexCount:12, vertices:vertices, normals:Bytes.alloc(96), indices:indices, faceRanges:[]
    }]});
  }
  static function main():Void {
    var root = "/tmp/materia-linked-artifact-test-" + Sys.getPid();
    FileSystem.createDirectory(root);
    FileSystem.createDirectory(root + "/generated");
    File.saveContent(root + "/haxeon.json", "{}");
    var artifact = fixture("first");
    var path = root + "/generated/mold.mtrg";
    File.saveBytes(path, artifact);
    var entry:Dynamic = {kind:"artifact", path:"generated/mold.mtrg"};
    var project = {format:"materia.project", version:1, defaultEntrypoint:"mold",
      build:{system:"haxeon", manifest:"haxeon.json"}, entrypoints:{mold:entry}};
    var manifest = root + "/materia.project.json";
    function save():Void File.saveContent(manifest, haxe.Json.stringify(project));
    save();
    check(MateriaProjectRunner.executionRequirement(manifest).kind == "prebuilt-artifact", "No code execution");
    check(MateriaProjectRunner.generateArtifact(manifest).compare(artifact) == 0, "Linked bytes preserved");
    var scene = MateriaProjectRunner.loadProject(manifest);
    check(scene.objects.length == 1, "Linked scene uses the shared artifact materializer");
    var cancelled = new ProjectLoadControl(); cancelled.cancel();
    rejects(() -> MateriaProjectRunner.generateArtifact(manifest, null, cancelled), ProjectLoadControl.CANCELLED);
    File.saveBytes(path, fixture("replacement"));
    check(SceneArtifact.decode(MateriaProjectRunner.generateArtifact(manifest)).recipeDocument == "replacement",
      "Regenerated output replaces old artifact without a generator cache");
    rejects(() -> MateriaProjectRunner.generateArtifact(manifest, "source"), "cannot accept an editable document");
    entry.documentInput = true; save();
    rejects(() -> MateriaProjectRunner.executionRequirement(manifest), "cannot accept editable source");
    Reflect.deleteField(entry, "documentInput");
    entry.path = "generated/missing.mtrg"; save();
    rejects(() -> MateriaProjectRunner.generateArtifact(manifest), "regenerate it first");
    entry.path = "haxeon.json"; save();
    rejects(() -> MateriaProjectRunner.generateArtifact(manifest), "requires an .mtrg");
    entry.path = "generated/mold.mtrg"; save();
    File.saveBytes(path, Bytes.ofString("broken"));
    rejects(() -> MateriaProjectRunner.generateArtifact(manifest), "");
    FileSystem.deleteFile(path);
    FileSystem.deleteFile(manifest);
    FileSystem.deleteFile(root + "/haxeon.json");
    FileSystem.deleteDirectory(root + "/generated");
    FileSystem.deleteDirectory(root);
    Sys.println("Linked artifact tests: " + assertions + " assertions passed");
  }
}
