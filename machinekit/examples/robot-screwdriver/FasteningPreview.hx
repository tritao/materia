import machinekit.assembly.AssemblyPreview;
import machinekit.screwdriving.FastenedAssembly;
import machinekit.screwdriving.FlangeBearingMount;
import machinekit.screwdriving.NemaMotorMount;
import machinekit.standard.DeepGrooveBearing;
import materia.project.SceneArtifact;
import haxe.io.Bytes;

/** CAD workpieces only: seated screws and named taps determine the connections. */
class FasteningPreview {
	static function preview(assembly:FastenedAssembly, id:String):Bytes {
		var fastenings = assembly.fastenings().require();
		if (fastenings.length != 4) throw '$id must have four fastenings';
		return SceneArtifact.encode(AssemblyPreview.scene(assembly, id));
	}
	public static function flange():Bytes
		return preview(new FlangeBearingMount(DeepGrooveBearing.metric("608")), "flange-mount");
	public static function motor():Bytes return preview(new NemaMotorMount(), "motor-mount");
}

function main():Void {
	var flange = SceneArtifact.decode(FasteningPreview.flange());
	var motor = SceneArtifact.decode(FasteningPreview.motor());
	if (flange.assemblyDefinition == null || motor.assemblyDefinition == null) throw "Workpiece lost its assembly";
	trace("Fastening previews passed");
}
