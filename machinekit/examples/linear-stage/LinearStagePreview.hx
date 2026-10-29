import haxe.io.Bytes;
import machinekit.assembly.AssemblyPreview;
import machinekit.pendant.PendantScene;
import materia.assembly.AssemblyFrames;
import materia.project.SceneArtifact;

/** Materia project entrypoints for the single-axis stage. */
class LinearStagePreview {
  public static function stage():Bytes return SceneArtifact.encode(AssemblyPreview.scene(LinearStage.axis(), "linear-stage"));

  /** The stage with a jog pendant beside it, whose wheel jogs the carriage. `savedDocument` is the pendant case's saved recipe. */
  public static function withPendant(?savedDocument:String):Bytes {
    return SceneArtifact.encode(PendantScene.attach(
      LinearStage.axis(),
      AssemblyFrames.translation(-260, 0, 200),
      savedDocument,
      LinearStage.operator(),
      LinearStage.axisJoints()
    ));
  }

  public static function main():Void {
    var bytes = stage();
    sys.FileSystem.createDirectory("build");
    sys.io.File.saveBytes("build/linear-stage.scene", bytes);
    Sys.println("Linear stage scene written: " + bytes.length + " bytes");
  }
}
