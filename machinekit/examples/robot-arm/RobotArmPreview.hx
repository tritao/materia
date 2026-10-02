import machinekit.assembly.AssemblyPreview;
import haxe.io.Bytes;
import cadkit.modeling.AssemblyModel;
import cadkit.modeling.AssemblyState;
import materia.assembly.AssemblyFrames;
import materia.project.SceneArtifact;

/** Materia project entrypoint for the six-axis robot arm. */
class RobotArmPreview {
	public static inline var ASSEMBLY_ID:String = "robot-arm";

	/**
	 * Geometry, joints and initial pose of the arm in its cell, its suction tool, and its work: carry
	 * the workpiece from one pad to the other and back, on repeat.
	 */
	public static function arm():Bytes {
		var robot = new RobotArm();
		var scene = AssemblyPreview.scene(robot, ASSEMBLY_ID);
		scene.robotTools = AssemblyPreview.robotTools(robot.tool, "tool");
		function onto(pad:String):Array<materia.project.SceneArtifact.SceneArtifactMissionStep>
			return [{kind: "pick", at: {occurrence: "workpiece", connector: "top"}},
				{kind: "place", at: {occurrence: pad, connector: "top"}}];
		scene.mission = {loop: true, steps: onto("padPlace").concat(onto("padPick"))};
		return SceneArtifact.encode(scene);
	}
}

/** Geometry builds, and forward kinematics puts the tool where the layout says. */
class RobotArmChecks {
	static function near(actual:Float, expected:Float, message:String, tolerance:Float = 1e-6):Void {
		if (!(Math.abs(actual - expected) <= tolerance))
			throw '$message: expected $expected, got $actual';
	}

	public static function run():Void {
		var scene = SceneArtifact.decode(RobotArmPreview.arm());
		var definition = scene.assemblyDefinition;
		if (scene.robotTools == null || scene.robotTools.length != 1 || scene.robotTools[0].sensor == null)
			throw "Robot arm should carry one suction tool with a vacuum sensor";
		if (scene.mission == null || scene.mission.steps.length != 4)
			throw "Robot arm should carry the workpiece to the other pad and back";
		var robot = new RobotArm();
		if (definition == null || definition.occurrences.length != robot.components().length)
			throw "Robot arm preview has the wrong number of occurrences";
		for (part in scene.parts) if (part.volume == null || part.volume <= 0 || part.inertia == null)
			throw 'Robot arm part "${part.id}" has no mass properties';
		var revolutes = [for (joint in definition.joints) if (Std.string(joint.type) == "revolute") joint.id];
		if (revolutes.join(",") != "j1,j2,j3,j4,j5,j6") throw "Robot arm should have joints j1 to j6";

		var model = new AssemblyModel("mm");
		robot.addTo(model, "");
		var straight = new AssemblyState(model.definition(RobotArmPreview.ASSEMBLY_ID));
		for (spec in robot.specs) straight.setJoint(spec.id, 0);
		straight.forwardKinematics();
		var base = straight.worldPose("joint1");
		near(base.z, RobotArm.PEDESTAL_HEIGHT + robot.flange.thickness, "arm base sits on the pedestal flange", 1e-3);
		// At zero the arm points straight up: shoulder offset 45, elbow back to the axis, wrist offset 35.
		var tool = straight.worldPose("toolFlange");
		near(tool.x, 35, "tool flange x at the straight pose", 1e-3);
		near(tool.y, 0, "tool flange y at the straight pose", 1e-3);
		near(tool.z, 1299 + robot.toolFlange.thickness, "tool flange z at the straight pose", 1e-3);
		near(AssemblyFrames.transformVector(tool, 0, 0, 1).z, 1, "tool axis points up when straight", 1e-6);

		var ready = new AssemblyState(model.definition(RobotArmPreview.ASSEMBLY_ID));
		for (spec in robot.specs) ready.setJoint(spec.id, spec.initial);
		ready.forwardKinematics();
		var readyTool = ready.worldPose("toolFlange");
		near(AssemblyFrames.transformVector(readyTool, 0, 0, 1).z, -1, "tool axis points down in the ready pose", 1e-6);
		// The cup's contact face hangs below the flange in the ready pose, facing the work below.
		var contact = ready.worldConnector("tool/cup", "contact");
		var lift = readyTool.z - contact.z;
		if (!(lift > 80)) throw 'Suction cup should hang well below the tool flange, got $lift mm';
		var approach = AssemblyFrames.transformVector(contact, 0, 1, 0);
		Sys.println('robot arm: cup contact at ${Math.round(contact.x)}, ${Math.round(contact.y)}, ${Math.round(contact.z)} mm, ' +
			'axis ${Math.round(approach.x * 100) / 100}, ${Math.round(approach.y * 100) / 100}, ${Math.round(approach.z * 100) / 100}');
		var moving = robot.toolFlange.massProperties().mass;
		for (joint in robot.joints) moving += joint.massProperties().mass;
		for (link in robot.links) moving += link.massProperties().mass;
		moving += robot.tool.massPropertiesAtMount().mass;
		Sys.println('robot arm: ${Math.round(moving * 10) / 10} kg above the base flange, ' +
			'${Math.round(robot.massProperties().mass * 10) / 10} kg in all');
		Sys.println('robot arm: ${scene.parts.length} definitions, ${definition.occurrences.length} occurrences, tool at ' +
			'${Math.round(readyTool.x)}, ${Math.round(readyTool.y)}, ${Math.round(readyTool.z)} mm in the ready pose');
	}
}

/** Standalone check of the arm example. */
function main():Void RobotArmChecks.run();
