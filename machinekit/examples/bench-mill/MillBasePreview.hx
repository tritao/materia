import haxe.io.Bytes;
import machinekit.assembly.MachineAssembly;
import machinekit.assembly.AssemblyPreview;

/** The actual enclosed mill support assembly, isolated for visual inspection. */
class MillBasePreview {
	public static function base():Bytes {
		var cell = new EnclosedBenchMill(false), state = cell.state();
		var base = new MachineAssembly();
		for (member in cell.components()) if (cell.standIds.indexOf(member.id) >= 0 || member.id == "chipTray")
			base.addComponent(member.id.split("/").join("-"), member.component, state.worldPose(member.id));
		return materia.project.SceneArtifact.encode(AssemblyPreview.scene(base, "mill-base"));
	}
	public static function main():Void {
		var root = Sys.getCwd();
		var relative = "machinekit/examples/bench-mill/build/mill-base.mtrg";
		var path = sys.FileSystem.exists(root + "/machinekit") ? root + "/" + relative : root + "/build/mill-base.mtrg";
		sys.io.File.saveBytes(path, base());
		Sys.println('Mill base preview: $path');
	}
}
