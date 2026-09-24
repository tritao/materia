package app;

import app.MateriaProjectRunner;
import app.ProjectDocumentSession;
import haxe.Json;
import sys.FileSystem;
import sys.io.File;

/** Save and reopen a generated project without persisting its mesh buffers. */
class ProjectSourceTests {
  static function check(value:Bool, message:String):Void {
    if (!value) throw message;
  }

  public static function main():Int {
    var root = FileSystem.exists("cadkit/examples/modeling/materia.project.json")
      ? FileSystem.fullPath(".") : FileSystem.exists("../cadkit/examples/modeling/materia.project.json")
      ? FileSystem.fullPath("..") : FileSystem.fullPath("../../..");
    Sys.setCwd(root);
    var manifest = root + "/cadkit/examples/modeling/materia.project.json";
    var generated = MateriaProjectRunner.load(manifest);
    check(generated.length == 13, "project generates all excavator parts");
    var base = generated[0], removed = generated[1];
    var session = new ProjectDocumentSession();
    var output = "/tmp/materia-project-source-" + Sys.getPid() + ".materia.json";
    var stage = "open generated scene";
    try {
      session.openGeneratedScene(generated, manifest);
      session.scene.select(base.id);
      check(session.scene.nudgeSelected(0.1, 0.0), "generated part can be moved");
      check(session.scene.duplicateSelected(), "generated part can be instanced");
      var copyId = session.scene.selectedId;
      session.scene.select(removed.id);
      check(session.scene.deleteSelected(), "generated part can be removed");
      check(session.scene.createRectangle(), "authored object can join project");
      stage = "save scene";
      session.save(output);
      var saved = File.getContent(output);
      var document:Dynamic = Json.parse(saved);
      var project:Dynamic = Reflect.field(document, "project");
      check(project != null && Reflect.field(project, "reference") != null,
        "saved scene keeps its project reference");
      check(saved.indexOf(base.meshSnapshot) < 0,
        "saved project excludes generated mesh buffers");
      var authored:Array<Dynamic> = cast Reflect.field(document, "objects");
      check(authored.length == 1, "saved scene keeps authored objects separately");
      stage = "reopen scene";
      session.open(output);
      stage = "check reopened scene";
      check(session.projectReference == manifest, "reopen restores the source manifest");
      check(session.scene.items().length == 14, "reopen restores generated and authored membership");
      check(session.scene.object(removed.id) == null, "reopen keeps the generated removal");
      check(session.scene.object(copyId) != null, "reopen restores a linked preview copy");
      check(Math.abs(session.scene.info(base.id).localTransform().element(12) - (base.x + 0.1)) < 0.000001,
        "reopen restores the generated part's authored position");
      check(!session.isDirty(), "reopened project starts clean");
    } catch (error:Dynamic) {
      session.dispose();
      if (FileSystem.exists(output)) FileSystem.deleteFile(output);
      throw '$stage: $error';
    }
    session.dispose();
    if (FileSystem.exists(output)) FileSystem.deleteFile(output);
    return 0;
  }
}
