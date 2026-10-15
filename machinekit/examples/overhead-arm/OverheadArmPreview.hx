import haxe.io.Bytes;
import machinekit.assembly.AssemblyPreview;
import machinekit.robot.RobotScene;
import materia.project.SceneArtifact;
import materia.project.SceneArtifact.SceneArtifactMissionStep;
import materia.project.SceneArtifact.SceneArtifactJointTarget;
import materia.assembly.AssemblyFrames;
import cadkit.modeling.AssemblyState;

class OverheadArmPreview {
	public static function cell():Bytes {
		var cell = new OverheadArm();
		var scene = AssemblyPreview.scene(cell, "overhead-arm");
		scene.robotTools = RobotScene.robotTools(cell.arm.tool, "arm/tool");
		function ready(x:Float, y:Float):SceneArtifactMissionStep {
			var targets:Array<SceneArtifactJointTarget> = [
				{joint: "x", position: x / 1000}, {joint: "y", position: y / 1000}, {joint: "z", position: 0.15}];
			for (index in 0...cell.arm.specs.length)
				targets.push({joint: "arm/" + cell.arm.specs[index].id, position: cell.ready[index]});
			return {kind: "moveJoints", at: {occurrence: "arm/tool/cup", connector: "contact"}, joints: targets};
		}
		scene.missions = [{loop: false, steps: [ready(200, 0),
			{kind: "pick", at: {occurrence: "workpiece", connector: "top"}}, ready(200, 0), ready(2800, 1500),
			{kind: "place", at: {occurrence: "padPlace", connector: "top"}}, ready(2800, 1500)]}];
		return SceneArtifact.encode(scene);
	}
}

class OverheadArmChecks {
	public static function run():Void {
		var cell = new OverheadArm();
		var definition = cell.definition();
		cell.validate();
		var state = new AssemblyState(definition);
		var floor = state.worldPose("arm/pedestal");
		var up = AssemblyFrames.transformVector(floor, 0, 0, 1);
		if (up.z > -0.999999) throw "The overhead arm must actually hang inverted";
		var logical = ["x", "y", "z"].concat([for (index in 1...7) "arm/j" + index]);
		for (id in logical) if ([for (joint in definition.joints) if (joint.id == id) joint].length != 1)
			throw 'The overhead arm lacks logical joint $id';
		var scene = SceneArtifact.decode(OverheadArmPreview.cell());
		if (scene.missions == null || scene.missions[0].steps.length != 6 || scene.robotTools == null || scene.robotTools.length != 1)
			throw "The overhead arm needs two stations and its sensed suction tool";
		Sys.println('Overhead arm: inverted industrial arm, XYZ external axes, 3 x 1.5 m table travel; mount ${floor.z} mm');
	}
}

function main():Void OverheadArmChecks.run();
