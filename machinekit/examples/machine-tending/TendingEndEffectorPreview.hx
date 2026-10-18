import haxe.io.Bytes;
import machinekit.assembly.AssemblyPreview;
import machinekit.assembly.MachineAssembly;
import machinekit.pneumatic.AirSupply;
import machinekit.robotics.RobotFlange;
import materia.assembly.AssemblyFrames;
import materia.project.SceneArtifact;

/** The tending cell's actual tool, isolated for close inspection of its services and guides. */
class TendingEndEffectorPreview {
	public static function preview():Bytes {
		var assembly = new MachineAssembly();
		assembly.include("tool", TendingReach.tendingTool().build(new RobotFlange(50)),
			AssemblyFrames.compose(AssemblyFrames.translation(0, 0, 160), AssemblyFrames.turnY(Math.PI)));
		// Compile the real pneumatic bindings with their regulated pressure. The
		// remote FRL is a service source, not part of the displayed wrist tool.
		assembly.addComponent("air", new AirSupply(), AssemblyFrames.translation(0, 0, -1000));
		assembly.connectPorts("air", "air", "air", "tool", "compressedAir");
		var scene = AssemblyPreview.scene(assembly, "tending-end-effector");
		var definition = scene.assemblyDefinition;
		if (definition == null) throw "End-effector preview lost its assembly";
		var airDefinitions = [for (item in definition.occurrences) if (item.id == "air") item.definition];
		definition.occurrences = [for (item in definition.occurrences) if (item.id != "air") item];
		definition.definitions = [for (item in definition.definitions) if (airDefinitions.indexOf(item.id) < 0) item];
		if (scene.assemblyState != null) scene.assemblyState.rootPoses =
			[for (root in scene.assemblyState.rootPoses) if (root.occurrence != "air") root];
		scene.parts = [for (part in scene.parts) if (airDefinitions.indexOf(part.id) < 0) part];
		return SceneArtifact.encode(scene);
	}

	public static function main():Void {
		var path = Sys.args().length > 0 ? Sys.args()[0] : "end-effector.mtrg";
		sys.io.File.saveBytes(path, preview());
		Sys.println('Exported $path');
	}
}
