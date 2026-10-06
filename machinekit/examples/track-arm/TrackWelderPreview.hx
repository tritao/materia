import haxe.io.Bytes;
import machinekit.assembly.AssemblyPreview;
import machinekit.robot.RobotScene;
import machinekit.welding.WeldingRecipe;
import machinekit.welding.WeldMetal;
import cadkit.modeling.AssemblyState;
import materia.project.SceneArtifact;

/** Coordinated process mode: the torch welds farther than twice the arm's reach. */
class TrackWelderPreview {
	public static function cell():Bytes {
		var cell = new TrackWelder();
		cell.check().throwIfErrors();
		var scene = AssemblyPreview.scene(cell, "track-welder");
		scene.robotTools = RobotScene.robotTools(cell.arm.tool, "arm/tool", cell.equipment());
		var state = new AssemblyState(scene.assemblyDefinition);
		var ready = [0.0, 0.25, Math.PI / 2 - 1.4, 0.0, Math.PI - 0.25 - 1.4, 0.0];
		for (i in 0...ready.length) state.setJoint("arm/" + cell.arm.specs[i].id, ready[i]);
		state.forwardKinematics();
		scene.assemblyState = state.record();
		var weldment = cell.weldment();
		var found = weldment.findIn(cell, cell.solvedPoses(state)).require();
		var seams = [for (seam in found) if (seam.name() == "work/basePlate:box.+z|work/upright:box.+y") seam];
		if (seams.length != 1) throw "The long T-joint must have one near-side seam";
		var feeder = cell.feeder;
		scene.mission = {steps: [WeldingRecipe.passStep(5,
			{diameterMm: feeder.wireDiameterMm, depositionEfficiency: feeder.depositionEfficiency,
				maxSpeedMPerMin: feeder.maxSpeedMPerMin}, seams, weldment.reference,
			WeldMetal.carrierOf(cell, weldment), scene.metresPerUnit)]};
		return SceneArtifact.encode(scene);
	}
}
