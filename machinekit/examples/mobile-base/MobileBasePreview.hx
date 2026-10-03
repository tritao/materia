import machinekit.assembly.AssemblyPreview;
import haxe.io.Bytes;
import cadkit.modeling.AssemblyModel;
import cadkit.modeling.AssemblyState;
import cadkit.modeling.Location;
import cadkit.modeling.Part;
import cadkit.modeling.Plane;
import cadkit.modeling.Vector;
import machinekit.assembly.MachineAssembly;
import machinekit.component.ComponentDetail;
import materia.assembly.AssemblyFrames;
import materia.project.SceneArtifact;
import materia.project.SceneArtifact.SceneArtifactMobileBase;

/** Materia project entrypoint for the differential-drive mobile base. */
class MobileBasePreview {
	public static inline var ASSEMBLY_ID:String = "mobile-base";
	public static inline var CELL_ID:String = "mobile-base-cell";

	/**
	 * Geometry, joints and initial pose of the base (parts with equal designations share geometry), and
	 * its drive: the wheel joints, with the wheel radius and track measured from the assembly.
	 */
	public static function base():Bytes {
		var robot = new MobileBase();
		var scene = AssemblyPreview.scene(robot, ASSEMBLY_ID);
		scene.mobileBase = drive(robot, "");
		scene.robotSensors = AssemblyPreview.robotSensors(robot, "");
		return SceneArtifact.encode(scene);
	}

	/** The base at work in its room, driving its round of the shelves and the dock on its own. */
	public static function cell():Bytes {
		var cell = new MobileBaseCell();
		var scene = AssemblyPreview.scene(cell, CELL_ID);
		var section = drive(cell.robot, "robot/");
		section.robot = "robot";
		section.origin = metres(MobileBaseCell.ORIGIN);
		scene.mobileBase = section;
		scene.robotTools = robotTools(cell.robot, "robot/");
		scene.robotSensors = AssemblyPreview.robotSensors(cell.robot, "robot/");
		scene.mission = {loop: true,
			steps: [for (step in MobileBaseCell.ROUND) switch step {
				case GoTo(pose): {kind: "goTo", pose: metres(pose)};
				case Pick(part, grasp): {kind: "pick", at: {occurrence: part, connector: grasp}};
				case Place(table, seat): {kind: "place", at: {occurrence: table, connector: seat}};
			}]};
		return SceneArtifact.encode(scene);
	}

	/** The tools of the arm the base carries, if any. */
	static function robotTools(robot:MobileBase, prefix:String):Array<materia.project.SceneArtifact.SceneArtifactRobotTool>
		return robot.arm == null ? [] : AssemblyPreview.robotTools(robot.arm.tool, prefix + "arm/tool");

	/** The drive of `robot`, whose joints carry `prefix`: wheel radius and track measured from its assembly. */
	static function drive(robot:MobileBase, prefix:String):SceneArtifactMobileBase
		return {leftWheel: prefix + "wheel_l", rightWheel: prefix + "wheel_r",
			wheelRadius: robot.wheel.radius / 1000, trackWidth: robot.trackWidth() / 1000,
			maxLinearSpeed: MobileBase.MAX_LINEAR_SPEED, maxAngularSpeed: MobileBase.MAX_ANGULAR_SPEED,
			maxLinearAcceleration: MobileBase.MAX_LINEAR_ACCELERATION, maxAngularAcceleration: MobileBase.MAX_ANGULAR_ACCELERATION,
			footprintLength: MobileBase.LENGTH / 1000, footprintWidth: MobileBase.WIDTH / 1000};

	static function metres(pose:MobileBaseCell.FloorPose):materia.project.SceneArtifact.SceneArtifactFloorPose
		return {x: pose.x / 1000, y: pose.y / 1000, yaw: pose.yaw};
}

/** Geometry builds, the base stands on its wheels and casters, the wheels roll, and nothing collides. */
class MobileBaseChecks {
	static function near(actual:Float, expected:Float, message:String, tolerance:Float = 1e-6):Void {
		if (!(Math.abs(actual - expected) <= tolerance))
			throw '$message: expected $expected, got $actual';
	}

	public static function run():Void {
		var scene = SceneArtifact.decode(MobileBasePreview.base());
		var definition = scene.assemblyDefinition;
		var robot = new MobileBase();
		if (definition == null || definition.occurrences.length != robot.components().length)
			throw "Mobile base preview has the wrong number of occurrences";
		for (part in scene.parts) if (part.volume == null || part.volume <= 0 || part.inertia == null)
			throw 'Mobile base part "${part.id}" has no mass properties';
		var continuous = [for (joint in definition.joints) if (Std.string(joint.type) == "continuous") joint.id];
		if (continuous.join(",") != "wheel_l,wheel_r") throw 'Mobile base should have wheel joints wheel_l, wheel_r, got $continuous';
		if (definition.joints.length != robot.components().length - 1) throw "Every member but the base plate should hang off one joint";

		var model = new AssemblyModel("mm");
		robot.addTo(model, "");
		var state = new AssemblyState(model.definition(MobileBasePreview.ASSEMBLY_ID));
		state.forwardKinematics();
		// It stands on the floor: wheel axles a radius up, caster wheels touching it.
		for (id in ["wheelLeft", "wheelRight"]) {
			var centre = state.worldConnector(id, "centre");
			near(centre.z, robot.wheel.radius, '$id axle height', 1e-9);
			near(centre.x, 0, '$id sits on the centre line of the base', 1e-9);
		}
		for (id in ["casterFront", "casterRear"]) near(state.worldConnector(id, "floor").z, 0, '$id touches the floor', 1e-9);
		near(robot.trackWidth(), 380, "track width", 1e-9);
		near(state.worldConnector("wheelLeft", "centre").y, -state.worldConnector("wheelRight", "centre").y,
			"the wheels are symmetric about the centre line", 1e-9);
		var drive = scene.mobileBase;
		if (drive == null) throw "Mobile base preview should declare its drive";
		near(drive.wheelRadius, robot.wheel.radius / 1000, "declared wheel radius", 1e-12);
		near(drive.trackWidth, 0.38, "declared track width", 1e-12);
		// The wheel limits come from the wheels' drive: each stepper through its gearhead.
		var wheelDrives = scene.assemblyDefinition == null ? null : scene.assemblyDefinition.actuators;
		if (wheelDrives == null || wheelDrives.length != 2) throw "Mobile base should drive each wheel with a motor";
		for (motor in wheelDrives) {
			var ratioValue = motor.gearRatio, efficiencyValue = motor.gearEfficiency;
			if (ratioValue == null || efficiencyValue == null || (motor.joint != "wheel_l" && motor.joint != "wheel_r"))
				throw 'Mobile base motor ${motor.id} should drive a wheel through a gearhead';
			var ratio:Float = ratioValue, efficiency:Float = efficiencyValue;
			near(ratio, MobileBase.WHEEL_GEARBOX.ratio, "wheel gearhead ratio");
			near(motor.maxRate / ratio, MobileBase.WHEEL_SPEED, "wheel speed is the stepper's over the gearhead ratio", 1e-9);
			near(motor.maxEffort * ratio * efficiency, MobileBase.WHEEL_TORQUE, "wheel torque is the stepper's through the gearhead", 1e-9);
		}
		near(MobileBase.WHEEL_SPEED, 13.71, "the wheels turn at most about 13.7 rad/s", 0.01);
		near(MobileBase.WHEEL_SPEED * drive.wheelRadius, 1.03, "which is about 1 m/s on the ground", 0.01);
		// The wheels' top speed covers the drive's: straight, and turning in place.
		if (!(MobileBase.MAX_LINEAR_SPEED / drive.wheelRadius <= MobileBase.WHEEL_SPEED &&
				MobileBase.MAX_ANGULAR_SPEED * drive.trackWidth / 2 / drive.wheelRadius <= MobileBase.WHEEL_SPEED))
			throw "Mobile base speed limits ask more of the wheels than their joints allow";
		var scan = state.worldConnector("lidar", "scan");
		near(scan.z, MobileBase.DECK_Z + MobileBase.DECK_THICKNESS + LidarPuck.SCAN_HEIGHT, "lidar scan height", 1e-9);
		near(scan.x, MobileBase.LIDAR_X, "lidar position along the robot", 1e-9);
		var zero = AssemblyFrames.transformVector(scan, 1, 0, 0), up = AssemblyFrames.transformVector(scan, 0, 0, 1);
		near(zero.x, 1, "lidar zero bearing points ahead", 1e-9);
		near(up.z, 1, "lidar scans a horizontal plane", 1e-9);
		// The scene mounts the lidar where the puck says, scanning the full circle of its connector's plane.
		var sensors = scene.robotSensors;
		if (sensors == null || sensors.length != 1) throw "Mobile base preview should declare its lidar";
		if (sensors[0].mount.occurrence != "lidar" || sensors[0].mount.connector != "scan" || sensors[0].kind != "lidar")
			throw "Mobile base lidar should be mounted at the puck's scan connector";
		near(sensors[0].maxRange, LidarPuck.RANGE, "declared lidar range", 1e-12);

		// Each wheel joint turns its wheel about its own motor's shaft, which points outward. Spinning
		// about +Y rolls a wheel forward, so a positive speed drives the left wheel forward and the right one back.
		for (side in [{joint: "wheel_l", wheel: "wheelLeft", motor: "motorLeft", forward: 1},
				{joint: "wheel_r", wheel: "wheelRight", motor: "motorRight", forward: -1}]) {
			var shaft = AssemblyFrames.transformVector(state.worldConnector(side.motor, "shaftAxis"), 0, 1, 0);
			near(shaft.y, side.forward, '${side.joint} shaft points outward', 1e-9);
			var start = AssemblyFrames.transformVector(state.worldPose(side.wheel), 1, 0, 0);
			for (angle in [Math.PI / 2, 2.0]) {
				state.setJoint(side.joint, angle);
				state.forwardKinematics();
				var turned = AssemblyFrames.transformVector(state.worldPose(side.wheel), 1, 0, 0);
				// Rodrigues: start turned by `angle` about the shaft (start is perpendicular to it).
				var cross = {x: shaft.y * start.z - shaft.z * start.y, y: shaft.z * start.x - shaft.x * start.z,
					z: shaft.x * start.y - shaft.y * start.x};
				near(turned.x, Math.cos(angle) * start.x + Math.sin(angle) * cross.x, '${side.joint} at $angle, x', 1e-9);
				near(turned.y, Math.cos(angle) * start.y + Math.sin(angle) * cross.y, '${side.joint} at $angle, y', 1e-9);
				near(turned.z, Math.cos(angle) * start.z + Math.sin(angle) * cross.z, '${side.joint} at $angle, z', 1e-9);
				near(state.worldConnector(side.wheel, "centre").z, robot.wheel.radius, '${side.joint} at $angle keeps its axle', 1e-9);
			}
			state.setJoint(side.joint, 0);
		}

		// No two members intersect, with the wheels at rest and turned.
		var ids = [for (entry in robot.components()) entry.id];
		for (angle in [0.0, 0.7]) {
			state.setJoint("wheel_l", angle);
			state.setJoint("wheel_r", -angle);
			state.forwardKinematics();
			var solids = [for (id in ids) posed(robot, state, id)];
			for (i in 0...ids.length) for (j in i + 1...ids.length) {
				var common = solids[i].intersect(solids[j]);
				var volume = common.volume();
				common.close();
				if (volume > 1e-3) throw '${ids[i]} collides with ${ids[j]} at wheel angle $angle: ${Math.round(volume)} mm³';
			}
			for (solid in solids) solid.close();
		}

		checkCell();
		var mass = robot.massProperties().mass;
		var bom = robot.billOfMaterials().lines();
		Sys.println('mobile base: ${scene.parts.length} definitions, ${definition.occurrences.length} occurrences, ' +
			'${bom.length} BOM lines, ${Math.round(mass * 10) / 10} kg, track ${robot.trackWidth()} mm');
	}

	/**
	 * In its room the robot starts clear of everything, and every goal leaves a disc as wide as the chassis'
	 * corners, plus margin, clear of the room's fixed blocks: the planner can stand the robot there.
	 */
	static function checkCell():Void {
		var scene = SceneArtifact.decode(MobileBasePreview.cell());
		if (scene.mobileBase == null || scene.mission == null) throw "Mobile base cell should carry its drive and mission";
		if (scene.robotSensors == null || scene.robotSensors[0].mount.occurrence != "robot/lidar")
			throw "Mobile base cell should carry its lidar, mounted on the robot's puck";
		var cell = new MobileBaseCell();
		var model = new AssemblyModel("mm");
		cell.addTo(model, "");
		var state = new AssemblyState(model.definition(MobileBasePreview.CELL_ID));
		state.forwardKinematics();
		var plate = state.worldPose("robot/basePlate");
		near(plate.x, MobileBaseCell.ORIGIN.x, "the robot stands at its origin, x", 1e-9);
		near(plate.y, MobileBaseCell.ORIGIN.y, "the robot stands at its origin, y", 1e-9);
		var blocks = [for (entry in cell.components()) if (!StringTools.startsWith(entry.id, "robot/")) entry];
		var robotIds = [for (entry in cell.components()) if (StringTools.startsWith(entry.id, "robot/")) entry.id];
		for (block in blocks) for (id in robotIds) {
			var a = posed(cell, state, block.id), b = posed(cell, state, id);
			var common = a.intersect(b);
			var volume = common.volume();
			common.close(); a.close(); b.close();
			if (volume > 1e-3) throw 'At its origin the robot\'s ${id} hits ${block.id}';
		}
		// The chassis' corner radius plus the planner's half-cell margin.
		var radius = Math.sqrt(Math.pow(MobileBase.LENGTH / 2, 2) + Math.pow(MobileBase.WIDTH / 2, 2)) + 40;
		for (goal in MobileBaseCell.GOALS) for (block in blocks) {
			if (block.id == "workpiece") continue;
			var solid = posed(cell, state, block.id);
			var box = solid.shape.bounds();
			// The floor slab lies under the robot, not in its way.
			if (box.get_max().get_z() <= 1e-6) { solid.close(); continue; }
			var minX = box.get_min().get_x(), maxX = box.get_max().get_x();
			var minY = box.get_min().get_y(), maxY = box.get_max().get_y();
			solid.close();
			var dx = Math.max(0, Math.max(minX - goal.x, goal.x - maxX));
			var dy = Math.max(0, Math.max(minY - goal.y, goal.y - maxY));
			if (!(Math.sqrt(dx * dx + dy * dy) >= radius))
				throw 'Goal at ${goal.x}, ${goal.y} is within ${Math.round(radius)} mm of ${block.id}';
		}
		// At each table the place seat is where the arm reaches, on the table top.
		for (table in ["tableNorth", "tableEast"]) {
			var seat = state.worldConnector(table, "placeSeat");
			near(seat.z, MobileBaseCell.TABLE_TOP, '$table seat height', 1e-9);
		}
		checkArm(cell.robot);
		Sys.println('mobile base cell: ${scene.parts.length} definitions, ${MobileBaseCell.GOALS.length} goals');
	}

	/**
	 * The arm stands on the deck working ahead of the robot: in its ready pose the cup hangs ahead of the
	 * chassis on the centre line, and no arm part meets a part of the base.
	 */
	static function checkArm(robot:MobileBase):Void {
		var model = new AssemblyModel("mm");
		robot.addTo(model, "");
		var state = new AssemblyState(model.definition("mobile-manipulator"));
		state.forwardKinematics();
		var cup = state.worldConnector("arm/tool/cup", "contact");
		if (!(cup.x > MobileBase.LENGTH / 2) || !(Math.abs(cup.y) < 50))
			throw 'The arm should work ahead of the robot, its cup is at ${Math.round(cup.x)}, ${Math.round(cup.y)} mm';
		var floor = state.worldConnector("arm/pedestal", "floor");
		near(floor.z, MobileBase.DECK_Z + MobileBase.DECK_THICKNESS, "the arm stands on the deck", 1e-9);
		var armIds = [for (entry in robot.components()) if (StringTools.startsWith(entry.id, "arm/")) entry.id];
		var baseIds = [for (entry in robot.components()) if (!StringTools.startsWith(entry.id, "arm/")) entry.id];
		for (a in armIds) for (b in baseIds) {
			if (a == "arm/pedestal" && b == "deck") continue;
			var first = posed(robot, state, a), second = posed(robot, state, b);
			var common = first.intersect(second);
			var volume = common.volume();
			common.close(); first.close(); second.close();
			if (volume > 1e-3) throw '$a collides with $b on the mobile manipulator: ${Math.round(volume)} mm³';
		}
		Sys.println('mobile manipulator: cup ahead at ${Math.round(cup.x)}, ${Math.round(cup.y)}, ${Math.round(cup.z)} mm');
	}

	static function posed(robot:MachineAssembly, state:AssemblyState, id:String):Part {
		for (entry in robot.components()) if (entry.id == id) {
			var pose = state.worldPose(id);
			var x = AssemblyFrames.transformVector(pose, 1, 0, 0), z = AssemblyFrames.transformVector(pose, 0, 0, 1);
			var local = entry.component.geometry(ComponentDetail.Preview);
			var placed = local.placed(new Location(new Plane(new Vector(pose.x, pose.y, pose.z), new Vector(x.x, x.y, x.z),
				new Vector(z.x, z.y, z.z))));
			local.close();
			return placed;
		}
		throw 'Mobile base has no member "$id"';
	}
}

/** Standalone check of the mobile base example. */
function main():Void MobileBaseChecks.run();
