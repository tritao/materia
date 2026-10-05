import haxe.io.Bytes;
import machinekit.assembly.AssemblyPreview;
import machinekit.robot.RobotScene;
import machinekit.robot.EndEffectorControls;
import machinekit.welding.WeldingMission;
import machinekit.welding.WeldMetal;
import cadkit.modeling.AssemblyState;
import cadkit.modeling.Vector;
import materia.assembly.AssemblyFrames;
import materia.project.SceneArtifact;

/** Whole plate T-joint and tube-frame mission on a five-axis process gantry. */
class GantryWelderPreview {
	public static function cell():Bytes {
		var cell = new GantryWelder();
		cell.check().throwIfErrors();
		var scene = AssemblyPreview.scene(cell, "gantry-welder");
		scene.robotTools = RobotScene.robotTools(cell.tool, "tool", cell.equipment());
		var weldment = cell.weldment();
		var state = new AssemblyState(cell.definition());
		var found = weldment.findIn(cell, cell.solvedPoses(state));
		var arc = EndEffectorControls.derive(cell.tool, "tool").arcs[0];
		var tip = state.worldConnector("tool/" + arc.member, arc.tcpConnector);
		var work = AssemblyFrames.inverse(state.worldPose(weldment.reference));
		var at = AssemblyFrames.transformPoint(work, tip.x, tip.y, tip.z);
		var wire = AssemblyFrames.transformVector(AssemblyFrames.compose(work, tip), 0, 0, 1);
		var home:machinekit.welding.WeldingMission.TorchPlace = {
			position: new Vector(at.x, at.y, at.z), wire: new Vector(wire.x, wire.y, wire.z)};
		var feeder = cell.feeder;
		var mission = WeldingMission.generate(weldment, found,
			{diameterMm: feeder.wireDiameterMm, depositionEfficiency: feeder.depositionEfficiency,
				maxSpeedMPerMin: feeder.maxSpeedMPerMin}, WeldMetal.carrierOf(cell, weldment), scene.metresPerUnit, home);
		scene.mission = {steps: mission.require()};
		return SceneArtifact.encode(scene);
	}
}

function main():Void {
	var scene = SceneArtifact.decode(GantryWelderPreview.cell());
	var mission:materia.project.SceneArtifact.SceneArtifactMission = cast scene.mission;
	var seams = 0;
	for (step in mission.steps) {
		var weld:materia.project.SceneArtifact.SceneArtifactWeld = cast step.weld;
		seams += weld.path.length;
	}
	if (mission.steps.length != 4 || seams != 10) throw "The gantry must weld all ten seams in four runs";
	Sys.println("gantry welder: XYZ + CA, four runs, ten CAD-derived seams");
}
