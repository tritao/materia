import haxe.io.Bytes;
import machinekit.assembly.AssemblyPreview;
import machinekit.pendant.PendantScene;
import materia.project.SceneArtifact;

/** Materia project entrypoint for the table-top XYZ stage. */
class XyzStagePreview {
  public static function stage():Bytes return SceneArtifact.encode(AssemblyPreview.scene(new XyzStage(), "xyz-stage"));

  /**
   * The stage with a jog pendant beside it, lying on the bench face up: its operator profile is the plain XYZ one and its wheel jogs
   * the stage's joints. `savedDocument` is the pendant case's recipe as the editor last saved it.
   */
  public static function withPendant(?savedDocument:String):Bytes {
    return SceneArtifact.encode(PendantScene.attach(new XyzStage(), PROFILE, materia.assembly.AssemblyFrames.translation(-150, 75, 50), savedDocument));
  }

  public static inline var PROFILE:String = "xyz";

  public static function main():Void {
    var problems = new XyzStage().check();
    if (problems.hasErrors()) throw "The XYZ stage is not a valid machine";
    var bytes = stage();
    sys.FileSystem.createDirectory("build");
    sys.io.File.saveBytes("build/xyz-stage.scene", bytes);
    Sys.println("XYZ stage scene written: " + bytes.length + " bytes");
  }
}
