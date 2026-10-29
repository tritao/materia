import haxe.io.Bytes;
import machinekit.assembly.AssemblyPreview;
import materia.project.SceneArtifact;

/** Materia project entrypoint for the table-top XYZ stage. */
class XyzStagePreview {
  public static function stage():Bytes return SceneArtifact.encode(AssemblyPreview.scene(new XyzStage(), "xyz-stage"));

  public static function main():Void {
    var problems = new XyzStage().check();
    if (problems.hasErrors()) throw "The XYZ stage is not a valid machine";
    var bytes = stage();
    sys.FileSystem.createDirectory("build");
    sys.io.File.saveBytes("build/xyz-stage.scene", bytes);
    Sys.println("XYZ stage scene written: " + bytes.length + " bytes");
  }
}
