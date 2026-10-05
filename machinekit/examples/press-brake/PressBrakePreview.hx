import haxe.io.Bytes;
import machinekit.assembly.AssemblyPreview;
import materia.project.SceneArtifact;

/** The press alone, with its mechanical axes available to the editor. */
class PressBrakePreview {
	public static function pressBrake():Bytes return SceneArtifact.encode(AssemblyPreview.scene(new PressBrake(), "press-brake"));
}
