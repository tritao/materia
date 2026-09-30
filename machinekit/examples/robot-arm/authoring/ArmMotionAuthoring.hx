import cadbridge.AssemblySimulationBridge;
import cadkit.modeling.AssemblyState;
import cadbridge.AssemblySimulationBridge.AssemblyPhysicalData;
import haxe.Json;
import materia.assembly.AssemblyDefinition;
import materia.project.SceneArtifact;
import materia.project.SceneArtifact.SceneArtifactData;
import motionkit.kinematics.IkTolerance;
import motionkit.kinematics.Pose3;
import motionkit.robot.ManipulatorKinematics;
import robotkit.manipulation.Manipulator;
import materia.assembly.AssemblyRecord.AssemblyFrame;
import robotkit.model.Frame;
import robotkit.model.Link;
import robotkit.model.RobotModel;
import sys.io.File;

/** Where the tool contact point goes, in millimetres, and how long it stays there. */
typedef Waypoint = {
	var name:String;
	var x:Float;
	var y:Float;
	var z:Float;
	/** Seconds held at the waypoint (the vacuum comes on or off here). */
	var dwell:Float;
}

typedef Keyframe = {
	var time:Float;
	var q:Array<Float>;
}

/**
 * Authors the arm's pick-and-place motion in Cartesian space. The tool contact point follows straight
 * lines between waypoints at a fixed, downward orientation; inverse kinematics turns each sample into
 * joint positions, and the result is written as the project's `robot-arm.motion.json`.
 *
 * `write FILE` regenerates the motion file; `check FILE` (the default) verifies that the committed file
 * is what the waypoints produce and that the tool reaches every waypoint.
 */
class ArmMotionAuthoring {
	/** Free moves and vertical approach moves, in metres per second. */
	static inline var TRAVEL_SPEED:Float = 0.30;
	static inline var APPROACH_SPEED:Float = 0.10;
	/** Sample spacing along a straight move, in metres. */
	static inline var STEP:Float = 0.02;
	/** Fraction of each joint's velocity limit the plan may use. */
	static inline var JOINT_SPEED_FRACTION:Float = 0.5;

	static function waypoints(home:Pose3):Array<Waypoint> {
		var approach = RobotArm.TABLE_TOP + RobotArm.WORKPIECE_HEIGHT + 120;
		var contact = RobotArm.TABLE_TOP + RobotArm.WORKPIECE_HEIGHT + 2;
		var y = RobotArm.WORK_Y;
		return [
			{name: "home", x: home.x * 1000, y: home.y * 1000, z: home.z * 1000, dwell: 0.5},
			{name: "above pick", x: RobotArm.PICK_X, y: y, z: approach, dwell: 0},
			{name: "pick", x: RobotArm.PICK_X, y: y, z: contact, dwell: 1.0},
			{name: "lift", x: RobotArm.PICK_X, y: y, z: approach, dwell: 0},
			{name: "above place", x: RobotArm.PLACE_X, y: y, z: approach, dwell: 0},
			{name: "place", x: RobotArm.PLACE_X, y: y, z: contact, dwell: 1.0},
			{name: "retreat", x: RobotArm.PLACE_X, y: y, z: approach, dwell: 0},
			{name: "home", x: home.x * 1000, y: home.y * 1000, z: home.z * 1000, dwell: 0.5}
		];
	}

	static function require(value:Bool, message:String):Void {
		if (!value) throw message;
	}

	static function round(value:Float):Float return Math.round(value * 1e6) / 1e6;

	static function cupLinkOf(model:RobotModel):Link {
		for (link in model.links) if (link.id == "tool/cup") return link;
		throw "The arm has no suction cup link";
	}

	/** The cup's contact connector in the cup's own frame, in the assembly's length unit. */
	static function contactFrame(definition:AssemblyDefinition):AssemblyFrame {
		for (occurrence in definition.occurrences) if (occurrence.id == "tool/cup")
			for (component in definition.definitions) if (component.id == occurrence.definition)
				for (connector in component.connectors) if (connector.name == "contact") return connector.frame;
		throw "The suction cup has no contact connector";
	}

	/** The arm as a kinematic RobotModel: its assembly joints, with the cup contact face as the tool frame. */
	static function solver(robot:RobotArm):ManipulatorKinematics {
		var scene = SceneArtifact.decode(RobotArmPreview.arm());
		var definition:AssemblyDefinition = cast(scene.assemblyDefinition, AssemblyDefinition);
		// Only kinematics matter here, so every part carries placeholder mass properties.
		var physical:AssemblyPhysicalData = {metresPerUnit: scene.metresPerUnit, parts: [for (part in scene.parts)
			{id: part.id, materialId: "neutral", volume: 1.0, centerOfMass: [0.0, 0.0, 0.0],
				inertia: [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0], density: 1.0}]};
		var model = AssemblySimulationBridge.toRobotModel(definition, physical, scene.assemblyState).model;
		var cupLink = cupLinkOf(model);
		var contact = contactFrame(definition);
		var tcp = model.addFrame(new Frame("tcp", cupLink));
		tcp.position = [contact.x * scene.metresPerUnit, contact.y * scene.metresPerUnit, contact.z * scene.metresPerUnit];
		tcp.rotation = [contact.qx, contact.qy, contact.qz, contact.qw];
		var arm = new Manipulator(model, "assembly-root", tcp.id);
		var ids = arm.jointIds();
		require(ids.join(",") == [for (spec in robot.specs) spec.id].join(","),
			"The kinematic chain should turn joints " + [for (spec in robot.specs) spec.id].join(",") + ", got " + ids.join(","));
		verifyModelsAgree(robot, definition, scene, arm);
		return new ManipulatorKinematics(arm, 1e-8);
	}

	/**
	 * The arm exists twice: as the CAD assembly and as the RobotModel cadbridge derives from it. Both
	 * must put the cup contact in the same place for any joint values, or the motion authored here would
	 * not be the motion the assembly shows. Robot joint values are relative to the scene's posed state.
	 */
	static function verifyModelsAgree(robot:RobotArm, definition:AssemblyDefinition, scene:SceneArtifactData,
			arm:Manipulator):Void {
		var assembly = new AssemblyState(definition, scene.assemblyState);
		var initial = [for (spec in robot.specs) assembly.joint(spec.id)];
		var seed = 12345;
		function next():Float {
			seed = (seed * 1103515245 + 12345) & 0x7fffffff;
			return seed / 2147483647.0;
		}
		var worstPosition = 0.0, worstAngle = 0.0;
		for (_ in 0...25) {
			var q = [for (index in 0...robot.specs.length) {
				var spec = robot.specs[index];
				var value = spec.lower + (spec.upper - spec.lower) * next();
				value - initial[index];
			}];
			for (index in 0...robot.specs.length) assembly.setJoint(robot.specs[index].id, initial[index] + q[index]);
			var cad = assembly.worldConnector("tool/cup", "contact");
			var tcp = arm.forwardKinematics(q);
			var dx = cad.x * scene.metresPerUnit - tcp.translation.x, dy = cad.y * scene.metresPerUnit - tcp.translation.y,
				dz = cad.z * scene.metresPerUnit - tcp.translation.z;
			worstPosition = Math.max(worstPosition, Math.sqrt(dx * dx + dy * dy + dz * dz));
			var dot = Math.abs(cad.qx * tcp.rotation.x + cad.qy * tcp.rotation.y + cad.qz * tcp.rotation.z + cad.qw * tcp.rotation.w);
			worstAngle = Math.max(worstAngle, 2 * Math.acos(Math.min(1.0, dot)));
		}
		require(worstPosition < 1e-9 && worstAngle < 1e-6,
			'The CAD assembly and the robot model disagree on the cup contact by $worstPosition m, $worstAngle rad');
	}

	/** Joint keyframes for the whole cycle; positions are relative to the arm's ready pose. */
	static function plan(robot:RobotArm, kinematics:ManipulatorKinematics):Array<Keyframe> {
		var tolerance = new IkTolerance(1e-5, 1e-4, 300, 0.02, 1e-3);
		var zero = [for (_ in 0...6) 0.0];
		var home = kinematics.forward(zero);
		var stops = waypoints(home);
		var frames:Array<Keyframe> = [{time: 0.0, q: zero}];
		var last = zero;
		var here = {x: home.x, y: home.y, z: home.z};
		var time = 0.0;
		for (index in 1...stops.length) {
			var stop = stops[index - 1];
			if (stop.dwell > 0) {
				time += stop.dwell;
				frames.push({time: time, q: last.copy()});
			}
			var target = stops[index];
			var goal = {x: target.x / 1000, y: target.y / 1000, z: target.z / 1000};
			var dx = goal.x - here.x, dy = goal.y - here.y, dz = goal.z - here.z;
			var distance = Math.sqrt(dx * dx + dy * dy + dz * dz);
			var vertical = Math.abs(dz) > 0.9 * distance;
			var samples = Std.int(Math.max(1, Math.ceil(distance / STEP)));
			for (sample in 1...samples + 1) {
				var fraction = sample / samples;
				// The tool keeps its ready-pose orientation: contact face down.
				var pose = new Pose3(here.x + dx * fraction, here.y + dy * fraction, here.z + dz * fraction,
					home.qx, home.qy, home.qz, home.qw);
				var solved = kinematics.solvePose(pose, last, tolerance);
				require(solved != null, 'No inverse kinematics solution for "${target.name}" at sample $sample of $samples');
				var q:Array<Float> = cast solved;
				var seconds = distance / samples / (vertical ? APPROACH_SPEED : TRAVEL_SPEED);
				for (joint in 0...6) seconds = Math.max(seconds,
					Math.abs(q[joint] - last[joint]) / (robot.specs[joint].velocity * JOINT_SPEED_FRACTION));
				time += Math.max(seconds, 0.02);
				frames.push({time: time, q: q});
				last = q;
			}
			here = goal;
		}
		var lastStop = stops[stops.length - 1];
		if (lastStop.dwell > 0) {
			time += lastStop.dwell;
			frames.push({time: time, q: last.copy()});
		}
		// A looping track must end where it starts: the last waypoint is home, so close the residual.
		var closing = kinematics.solvePose(home, last, tolerance);
		require(closing != null, "The cycle should end back at home");
		var settle:Array<Float> = cast closing;
		for (joint in 0...6) require(Math.abs(settle[joint]) < 1e-3, 'Joint ${robot.specs[joint].id} should return to its ready position');
		frames[frames.length - 1].q = zero;
		return frames;
	}

	static function encode(robot:RobotArm, frames:Array<Keyframe>):String {
		var tracks:Array<String> = [];
		for (joint in 0...6) {
			var keys = [for (frame in frames)
				'\t\t\t\t{"time": ${round(frame.time)}, "position": ${round(frame.q[joint])}}'];
			tracks.push('\t\t{\n\t\t\t"joint": "${robot.specs[joint].id}",\n\t\t\t"loop": true,\n\t\t\t"keys": [\n' +
				keys.join(",\n") + '\n\t\t\t]\n\t\t}');
		}
		return '{\n\t"version": 1,\n\t"tracks": [\n' + tracks.join(",\n") + '\n\t]\n}\n';
	}

	/** The tool must actually reach every waypoint at the times the plan visits them. */
	static function verify(robot:RobotArm, kinematics:ManipulatorKinematics, frames:Array<Keyframe>):Void {
		var home = kinematics.forward([for (_ in 0...6) 0.0]);
		var lowest = 1e9;
		var pickTouched = false, placeTouched = false;
		for (frame in frames) {
			var pose = kinematics.forward(frame.q);
			lowest = Math.min(lowest, pose.z);
			if (Math.abs(pose.x * 1000 - RobotArm.PICK_X) < 0.5 && Math.abs(pose.y * 1000 - RobotArm.WORK_Y) < 0.5 &&
				Math.abs(pose.z * 1000 - (RobotArm.TABLE_TOP + RobotArm.WORKPIECE_HEIGHT + 2)) < 0.5) pickTouched = true;
			if (Math.abs(pose.x * 1000 - RobotArm.PLACE_X) < 0.5 && Math.abs(pose.y * 1000 - RobotArm.WORK_Y) < 0.5 &&
				Math.abs(pose.z * 1000 - (RobotArm.TABLE_TOP + RobotArm.WORKPIECE_HEIGHT + 2)) < 0.5) placeTouched = true;
			// Orientation stays put: the contact face keeps pointing down.
			var dot = Math.abs(pose.qx * home.qx + pose.qy * home.qy + pose.qz * home.qz + pose.qw * home.qw);
			require(dot > 1 - 1e-6, "The tool tilted away from the ready orientation");
			for (joint in 0...6) require(Math.abs(frame.q[joint] - robot.specs[joint].initial) <= 100,
				"Joint position is out of range");
		}
		require(pickTouched && placeTouched, "The plan should reach both the pick and the place point");
		require(lowest * 1000 >= RobotArm.TABLE_TOP + RobotArm.WORKPIECE_HEIGHT + 2 - 0.5,
			"The tool contact point must stay above the workpiece top");
	}

	static function main():Void {
		var args = Sys.args();
		var mode = args.length > 0 ? args[0] : "check";
		var file = args.length > 1 ? args[1] : "robot-arm.motion.json";
		var robot = new RobotArm();
		var kinematics = solver(robot);
		var frames = plan(robot, kinematics);
		verify(robot, kinematics, frames);
		var text = encode(robot, frames);
		if (mode == "write") {
			File.saveContent(file, text);
			Sys.println('Wrote $file: ${frames.length} keyframes over ${round(frames[frames.length - 1].time)} s');
			return;
		}
		var saved:Dynamic = Json.parse(File.getContent(file));
		var tracks:Array<Dynamic> = Reflect.field(saved, "tracks");
		require(tracks.length == 6, "The motion file should have one track per joint");
		for (joint in 0...6) {
			var keys:Array<Dynamic> = Reflect.field(tracks[joint], "keys");
			require(Reflect.field(tracks[joint], "joint") == robot.specs[joint].id && keys.length == frames.length,
				'Track ${robot.specs[joint].id} does not match the plan');
			for (index in 0...frames.length) {
				var time:Float = Reflect.field(keys[index], "time"), position:Float = Reflect.field(keys[index], "position");
				require(Math.abs(time - round(frames[index].time)) < 1e-5 && Math.abs(position - round(frames[index].q[joint])) < 1e-5,
					'Track ${robot.specs[joint].id} key $index differs from the plan');
			}
		}
		Sys.println('$file matches the plan: ${frames.length} keyframes, tool reaches pick and place');
	}
}
