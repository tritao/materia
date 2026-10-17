import haxe.io.Bytes;
import machinekit.assembly.AssemblyPreview;
import materia.project.SceneArtifact;

class FlexsplineHobberPreview {
  public static function hobber():Bytes return SceneArtifact.encode(AssemblyPreview.scene(new FlexsplineHobber(),"flexspline-hobber"));
}
