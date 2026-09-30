import cadbridge.AssemblySimulationBridge;
import cadbridge.AssemblySimulationBridge.AssemblyPhysicalData;
import haxe.Json;
import materia.assembly.AssemblyDefinition;
import materia.assembly.AssemblyFrames;
import materia.project.SceneArtifact;
import motionkit.kinematics.IkTolerance;
import motionkit.kinematics.Pose3;
import motionkit.robot.ManipulatorKinematics;
import robotkit.manipulation.ChainTip;
import robotkit.manipulation.KinematicChain;
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
	/** Seconds held at the waypoint. */
	var dwell:Float;
	/** 1 turns the vacuum on once the tool has settled here, -1 turns it off, 0 leaves it alone. */
	var vacuum:Int;
}

/** A vacuum command, in seconds from the start of the cycle. */
typedef Grip = {
	var time:Float;
	var grip:Bool;
}

typedef Plan = {
	var frames:Array<Keyframe>;
	var grips:Array<Grip>;
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

	/** Seconds after the tool arrives before the vacuum switches, so the cup has stopped moving. */
	static inline var VACUUM_DELAY:Float = 0.4;
	/** The link that carries the suction cup. */
	static inline var CUP_LINK:String = "tool/cup";

	/** Carry the workpiece from one pad to the other, come home, then carry it back. */
	static function waypoints(home:Pose3):Array<Waypoint> {
		var approach = RobotArm.WORKPIECE_TOP + 120;
		var contact = RobotArm.WORKPIECE_TOP - RobotArm.GRIP_PRESS;
		var y = RobotArm.WORK_Y;
		var stops:Array<Waypoint> = [];
		function at(name:String, x:Float, z:Float, dwell:Float, vacuum:Int):Void
			stops.push({name: name, x: x, y: y, z: z, dwell: dwell, vacuum: vacuum});
		function atHome():Void
			stops.push({name: "home", x: home.x * 1000, y: home.y * 1000, z: home.z * 1000, dwell: 0.5, vacuum: 0});
		atHome();
		for (leg in [[RobotArm.PICK_X, RobotArm.PLACE_X], [RobotArm.PLACE_X, RobotArm.PICK_X]]) {
			at("above pick", leg[0], approach, 0, 0);
			at("pick", leg[0], contact, 1.0, 1);
			at("lift", leg[0], approach, 0, 0);
			at("above place", leg[1], approach, 0, 0);
			at("place", leg[1], contact, 1.0, -1);
			at("retreat", leg[1], approach, 0, 0);
			atHome();
		}
		return stops;
	}

	static function require(value:Bool, message:String):Void {
		if (!value) throw message;
	}

	static function round(value:Float):Float return Math.round(value * 1e6) / 1e6;


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
		var converted = AssemblySimulationBridge.toRobotModel(definition, physical, scene.assemblyState);
		var model = converted.model;
		// The cup rides the hand's link; its contact face is the cup's offset there, then the connector.
		var cup = converted.partLinks.get("tool/cup");
		if (cup == null) throw "The arm has no suction cup";
		var contact = contactFrame(definition);
		var metres = scene.metresPerUnit;
		var tip = AssemblyFrames.compose(cup.offset, {x: contact.x * metres, y: contact.y * metres, z: contact.z * metres,
			qx: contact.qx, qy: contact.qy, qz: contact.qz, qw: contact.qw});
		var tcp = model.addFrame(new Frame("tcp", model.links[cup.link]));
		tcp.position = [tip.x, tip.y, tip.z];
		tcp.rotation = [tip.qx, tip.qy, tip.qz, tip.qw];
		var chain = new KinematicChain(model, "assembly-root", ChainTip.Frame(tcp.id));
		var ids = chain.dofJointIds();
		require(ids.join(",") == [for (spec in robot.specs) spec.id].join(","),
			"The kinematic chain should turn joints " + [for (spec in robot.specs) spec.id].join(",") + ", got " + ids.join(","));
		return new ManipulatorKinematics(new Manipulator(model, chain), 1e-8);
	}

	/** Joint keyframes for the whole cycle; positions are relative to the arm's ready pose. */
	static function plan(robot:RobotArm, kinematics:ManipulatorKinematics):Plan {
		var tolerance = new IkTolerance(1e-5, 1e-4, 300, 0.02, 1e-3);
		var zero = [for (_ in 0...6) 0.0];
		var home = kinematics.forward(zero);
		var stops = waypoints(home);
		var frames:Array<Keyframe> = [{time: 0.0, q: zero}];
		var grips:Array<Grip> = [];
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
			if (target.vacuum != 0) grips.push({time: time + VACUUM_DELAY, grip: target.vacuum > 0});
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
		return {frames: frames, grips: grips};
	}

	static function encode(robot:RobotArm, plan:Plan):String {
		var tracks:Array<String> = [];
		for (joint in 0...6) {
			var keys = [for (frame in plan.frames)
				'\t\t\t\t{"time": ${round(frame.time)}, "position": ${round(frame.q[joint])}}'];
			tracks.push('\t\t{\n\t\t\t"joint": "${robot.specs[joint].id}",\n\t\t\t"loop": true,\n\t\t\t"keys": [\n' +
				keys.join(",\n") + '\n\t\t\t]\n\t\t}');
		}
		var grips = [for (grip in plan.grips)
			'\t\t{"time": ${round(grip.time)}, "link": "$CUP_LINK", "action": "${grip.grip ? "grip" : "release"}"}'];
		return '{\n\t"version": 1,\n\t"tracks": [\n' + tracks.join(",\n") + '\n\t],\n\t"grips": [\n' +
			grips.join(",\n") + '\n\t]\n}\n';
	}

	/** The tool must actually reach every waypoint at the times the plan visits them. */
	static function verify(robot:RobotArm, kinematics:ManipulatorKinematics, plan:Plan):Void {
		var home = kinematics.forward([for (_ in 0...6) 0.0]);
		var contact = RobotArm.WORKPIECE_TOP - RobotArm.GRIP_PRESS;
		var lowest = 1e9;
		var pickTouched = false, placeTouched = false;
		for (frame in plan.frames) {
			var pose = kinematics.forward(frame.q);
			lowest = Math.min(lowest, pose.z);
			var atContact = Math.abs(pose.y * 1000 - RobotArm.WORK_Y) < 0.5 && Math.abs(pose.z * 1000 - contact) < 0.5;
			if (atContact && Math.abs(pose.x * 1000 - RobotArm.PICK_X) < 0.5) pickTouched = true;
			if (atContact && Math.abs(pose.x * 1000 - RobotArm.PLACE_X) < 0.5) placeTouched = true;
			// Orientation stays put: the contact face keeps pointing down.
			var dot = Math.abs(pose.qx * home.qx + pose.qy * home.qy + pose.qz * home.qz + pose.qw * home.qw);
			require(dot > 1 - 1e-6, "The tool tilted away from the ready orientation");
			for (joint in 0...6) require(Math.abs(frame.q[joint] - robot.specs[joint].initial) <= 100,
				"Joint position is out of range");
		}
		require(pickTouched && placeTouched, "The plan should reach both the pick and the place point");
		require(lowest * 1000 >= contact - 0.5, "The tool contact point must stay above the workpiece top");
		// Two legs, each a grip then a release, and every grip lands while the tool is at rest.
		require(plan.grips.length == 4, "Two legs need four vacuum commands, got " + plan.grips.length);
		for (index in 0...plan.grips.length) {
			require(plan.grips[index].grip == (index % 2 == 0), "Vacuum commands should alternate grip and release");
			if (index > 0) require(plan.grips[index].time > plan.grips[index - 1].time, "Vacuum commands should be in time order");
		}
	}

	static function main():Void {
		var args = Sys.args();
		var mode = args.length > 0 ? args[0] : "check";
		var file = args.length > 1 ? args[1] : "robot-arm.motion.json";
		var robot = new RobotArm();
		var kinematics = solver(robot);
		var motion = plan(robot, kinematics);
		verify(robot, kinematics, motion);
		var frames = motion.frames;
		var text = encode(robot, motion);
		if (mode == "write") {
			File.saveContent(file, text);
			Sys.println('Wrote $file: ${frames.length} keyframes, ${motion.grips.length} vacuum commands over ${round(frames[frames.length - 1].time)} s');
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
		var savedGrips:Array<Dynamic> = Reflect.field(saved, "grips");
		require(savedGrips != null && savedGrips.length == motion.grips.length, "The motion file's vacuum commands do not match the plan");
		for (index in 0...motion.grips.length) {
			var time:Float = Reflect.field(savedGrips[index], "time");
			require(Math.abs(time - round(motion.grips[index].time)) < 1e-5 &&
				Reflect.field(savedGrips[index], "action") == (motion.grips[index].grip ? "grip" : "release") &&
				Reflect.field(savedGrips[index], "link") == CUP_LINK, 'Vacuum command $index differs from the plan');
		}
		Sys.println('$file matches the plan: ${frames.length} keyframes, ${motion.grips.length} vacuum commands, tool reaches pick and place');
	}
}
