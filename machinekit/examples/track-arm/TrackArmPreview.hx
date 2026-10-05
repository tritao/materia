import haxe.io.Bytes;
import machinekit.assembly.AssemblyPreview;
import machinekit.robot.RobotScene;
import materia.project.SceneArtifact;
import materia.project.SceneArtifact.SceneArtifactMissionStep;
import materia.project.SceneArtifact.SceneArtifactJointTarget;

class TrackArmPreview {
	public static function cell():Bytes {
		var cell = new TrackArm();
		var scene = AssemblyPreview.scene(cell, "track-arm");
		scene.robotTools = RobotScene.robotTools(cell.arm.tool, "arm/tool");
		function ready(coordinate:Float):SceneArtifactMissionStep {
			var targets:Array<SceneArtifactJointTarget> = [{joint: "track", position: coordinate / 1000}];
			for (joint in cell.arm.specs) targets.push({joint: "arm/" + joint.id, position: joint.initial});
			return {kind: "moveJoints", at: {occurrence: "arm/tool/cup", connector: "contact"}, joints: targets};
		}
		scene.mission = {loop: false, steps: [ready(0),
			{kind: "pick", at: {occurrence: "workpiece", connector: "top"}}, ready(0), ready(3000),
			{kind: "place", at: {occurrence: "padPlace", connector: "top"}}, ready(3000)]};
		return SceneArtifact.encode(scene);
	}
}

class TrackArmChecks {
	public static function run():Void {
		var scene = SceneArtifact.decode(TrackArmPreview.cell());
		if (scene.mission == null || scene.mission.steps.length != 6 || scene.robotTools == null || scene.robotTools.length != 1)
			throw "Track arm needs a sensed suction tool and six positioning steps";
		var definition = scene.assemblyDefinition;
		if (definition == null || definition.actuators == null || definition.actuators.length != 7)
			throw "Track arm must retain six arm drives and the rack drive";
		var arm = [for (occurrence in definition.occurrences) if (occurrence.id == "arm/toolFlange") occurrence];
		if (arm.length != 1 || arm[0].includePath != "arm") throw "The arm must own its tool flange";
		Sys.println("Track arm: 3 m rack axis, whole six-axis arm, two table stations");
	}
}

function main():Void TrackArmChecks.run();
