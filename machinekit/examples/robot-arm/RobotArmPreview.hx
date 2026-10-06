import machinekit.motion.ServoMotor;
import machinekit.assembly.AssemblyPreview;
import haxe.io.Bytes;
import cadkit.modeling.AssemblyModel;
import cadkit.modeling.AssemblyState;
import materia.assembly.AssemblyFrames;
import materia.project.SceneArtifact;
import machinekit.robot.RobotScene;

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
		scene.robotTools = RobotScene.robotTools(robot.tool, "tool");
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

	static function sizeClasses():Void {
		for (cls in [machinekit.robotics.IndustrialArmClass.Reach700,
			machinekit.robotics.IndustrialArmClass.Reach900, machinekit.robotics.IndustrialArmClass.Reach1300]) {
			var arm = new RobotArm(false, null, cls);
			arm.check().throwIfErrors();
			var model = new AssemblyModel("mm");
			arm.addTo(model, "");
			var state = new AssemblyState(model.definition("industrial-arm"));
			for (spec in arm.specs) state.setJoint(spec.id, 0);
			state.forwardKinematics();
			var upper = AssemblyFrames.transformVector(state.worldPose("upperArm"), 0, 0, 1);
			var forearm = AssemblyFrames.transformVector(state.worldPose("forearm"), 0, 0, 1);
			near(upper.z, 1, "industrial upper arm is vertical at zero");
			near(forearm.y, -1, "industrial forearm is horizontal and forward at zero");
			var centre = state.worldPose("hand");
			for (index in 3...6) {
				var frame = state.worldConnector('joint${index + 1}', index == 5 ? "tool" : "rotor");
				var axis = AssemblyFrames.transformVector(frame, 0, 1, 0);
				var dx = centre.x - frame.x, dy = centre.y - frame.y, dz = centre.z - frame.z;
				var cx = dy * axis.z - dz * axis.y, cy = dz * axis.x - dx * axis.z, cz = dx * axis.y - dy * axis.x;
				near(Math.sqrt(cx * cx + cy * cy + cz * cz), 0, "all wrist axes pass through the clevis centre", 1e-6);
			}
			var actuators:Array<materia.assembly.AssemblyDefinition.AssemblyActuator> = cast model.definition("industrial-arm").actuators;
			if (actuators == null || actuators.length != 6) throw "Industrial class must retain all six drive actuators";
			Sys.println('industrial arm ${arm.reference.designation}: zero and spherical axes passed');
		}
	}

	public static function run():Void {
		sizeClasses();
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

		// Every joint is driven by a servo through a gearbox, and its limits are what that drive gives: the servo's maximum
		// speed over the ratio and its peak torque through the gearbox at its efficiency.
		var actuators = definition.actuators;
		if (actuators == null || actuators.length != 6) throw "Robot arm should have a servo on each of its six joints";
		for (spec in robot.specs) {
			var drive = [for (actuator in actuators) if (actuator.joint == spec.id) actuator];
			if (drive.length != 1 || drive[0].drive != "servo") throw 'Robot arm joint ${spec.id} should have one servo drive';
			var peakValue = drive[0].peakTorque, topValue = drive[0].maxSpeed, ratioValue = drive[0].gearRatio, efficiencyValue = drive[0].gearEfficiency;
			if (peakValue == null || topValue == null || ratioValue == null || efficiencyValue == null) throw 'Robot arm joint ${spec.id} has an incomplete drive';
			var peak:Float = peakValue, top:Float = topValue, ratio:Float = ratioValue, efficiency:Float = efficiencyValue;
			near(ratio, spec.gearbox.ratio, '${spec.id} gearbox ratio');
			near(efficiency, RobotArm.GEARBOX_EFFICIENCY, '${spec.id} gearbox efficiency');
			near(spec.gearbox.jointSpeed(ServoMotor.model(spec.servo).rating.maxSpeed), top / ratio, '${spec.id} speed limit is the servo\'s over the ratio', 1e-9);
			near(spec.gearbox.jointTorque(ServoMotor.model(spec.servo).rating.peakTorque), peak * ratio * efficiency, '${spec.id} torque limit is the servo\'s through the gearbox', 1e-9);
			var edge = [for (joint in definition.joints) if (joint.id == spec.id) joint][0];
			var edgeSpeed = edge.limits.velocity, edgeEffort = edge.limits.effort;
			if (edgeSpeed != null || edgeEffort != null) throw 'Joint ${spec.id} should derive its drive limits at compilation';
		}
		near(robot.specs[0].gearbox.jointSpeed(ServoMotor.model(robot.specs[0].servo).rating.maxSpeed), 2.094, "j1 runs at about 2.1 rad/s", 1e-3);
		near(robot.specs[0].gearbox.jointTorque(ServoMotor.model(robot.specs[0].servo).rating.peakTorque), 405.9, "and carries about 406 N m", 0.1);
		var model = new AssemblyModel("mm");
		robot.addTo(model, "");
		var straight = new AssemblyState(model.definition(RobotArmPreview.ASSEMBLY_ID));
		for (spec in robot.specs) straight.setJoint(spec.id, 0);
		straight.forwardKinematics();
		var base = straight.worldPose("joint1");
		near(base.z, RobotArm.PEDESTAL_HEIGHT + robot.flange.thickness, "arm base sits on the pedestal flange", 1e-3);
		// Industrial zero: upper arm vertical, forearm and tool forward (-Y).
		var tool = straight.worldPose("toolFlange");
		near(tool.x, 0, "spherical tool flange x at the straight pose", 1e-3);
		near(tool.y, -(robot.reference.forearm + robot.joints[3].length + robot.reference.wristBody +
			robot.reference.hand + robot.joints[5].length + robot.toolFlange.thickness), "tool flange forward reach at zero", 1e-3);
		near(tool.z, base.z + robot.joints[0].length + robot.reference.turret + robot.reference.upperArm,
			"tool flange height at industrial zero", 1e-3);
		near(AssemblyFrames.transformVector(tool, 0, 0, 1).y, -1, "tool axis points forward at industrial zero", 1e-6);

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
		for (spec in robot.specs) moving += spec.gearbox.massProperties().mass;
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
