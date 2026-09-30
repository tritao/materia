import cadkit.modeling.Part;
import cadkit.modeling.Vector;
import machinekit.assembly.MachineAssembly;
import machinekit.component.ComponentDetail;
import machinekit.component.Dimension;
import machinekit.component.MachineComponent;
import machinekit.component.Solids;
import machinekit.pneumatic.schmalz.SchmalzPushInFitting;
import machinekit.pneumatic.schmalz.SchmalzSuctionCup;
import machinekit.pneumatic.schmalz.SchmalzVacuumGenerator;
import machinekit.pneumatic.schmalz.SchmalzVacuumHose;
import machinekit.robotics.EndEffector;
import machinekit.robotics.EndEffectorPlate;
import machinekit.robotics.FrameBar;
import machinekit.robotics.Pedestal;
import machinekit.robotics.RobotFlange;
import materia.assembly.AssemblyFrames;

/** Direction of a joint axis, in the frame of the link that carries the joint. */
enum ArmAxis {
	PlusX;
	MinusX;
	PlusZ;
}

/** Joint module: a cylindrical housing along local +Z with a fixed `stator` face at z=0 and the
 * rotating output `rotor` face at z=length. The housing belongs to the link before the joint;
 * the next link mates its `start` connector to `rotor` on a revolute joint.
 *
 * A module given a `flange` is the last joint of the arm. Its output carries the `RobotFlange`
 * plate against the housing end, so it has a `tool` connector at the flange's mounting face
 * instead of `rotor`: mate the flange's `face` to it, and a tool mates to the flange's pilot boss.
 */
class ArmJoint extends MachineComponent {
	public final diameter:Float;
	public final length:Float;
	public final flange:Null<RobotFlange>;

	public function new(diameter:Float, length:Float, ?flange:RobotFlange) {
		if (!(diameter > 0) || !(length > 0)) throw "Arm joint needs a positive diameter and length";
		var text = '${Dimension.format(diameter)}x${Dimension.format(length)}';
		super(flange == null ? 'ARM-JOINT-D$text' : 'ARM-JOINT-D$text-${flange.designation}',
			'Arm joint module, $text mm', "steel 12.9");
		this.diameter = diameter;
		this.length = length;
		this.flange = flange;
		addConnector("stator", Mount, Solids.axial(0, 0, 0));
		if (flange == null) {
			addConnector("rotor", Mount, Solids.axial(0, 0, length));
		} else {
			if (!(diameter >= flange.flangeDiameter + 2)) throw "Arm joint is too narrow for its tool flange";
			addConnector("tool", Mount, flange.pinAlignedFrame(length + flange.thickness));
		}
	}

	override public function hasGeometry():Bool return true;

	override public function geometry(detail:ComponentDetail = Preview):Part
		return Part.cylinderSpan(diameter / 2, 0, length);
}

/** Hollow tube link along local +Z, closed at both ends, with a collar where it meets the
 * previous joint. Its `start` connector sits at the origin with the joint axis given by
 * `startAxis`; its `end` connector carries the next joint module, whose axis is `endAxis`. A
 * lateral end joint is centred on the tube's end point, so `end` is offset by half the module
 * length against the axis direction.
 */
class ArmLink extends MachineComponent {
	public final length:Float;
	public final diameter:Float;
	public final wall:Float;
	public final collarDiameter:Float;
	public final startAxis:ArmAxis;
	public final endAxis:ArmAxis;

	static inline var COLLAR_THICKNESS:Float = 6;

	public function new(length:Float, diameter:Float, wall:Float, collarDiameter:Float, startAxis:ArmAxis,
			endAxis:ArmAxis, endJointLength:Float) {
		if (!(length > 2 * wall) || !(diameter > 2 * wall) || !(wall > 0))
			throw "Arm link needs a wall thinner than its radius and length";
		var text = '${Dimension.format(diameter)}x${Dimension.format(length)}';
		super('ARM-LINK-D$text-W${Dimension.format(wall)}', 'Arm link tube, $text mm', "aluminium 6061");
		this.length = length;
		this.diameter = diameter;
		this.wall = wall;
		this.collarDiameter = collarDiameter;
		this.startAxis = startAxis;
		this.endAxis = endAxis;
		var start = direction(startAxis);
		addConnector("start", Mount, AssemblyFrames.alongY(0, 0, 0, start.x, start.y, start.z));
		var end = direction(endAxis);
		var offset = endAxis == PlusZ ? 0 : endJointLength / 2;
		addConnector("end", Mount, AssemblyFrames.alongY(-offset * end.x, -offset * end.y,
			length - offset * end.z, end.x, end.y, end.z));
	}

	public static function direction(axis:ArmAxis):{x:Float, y:Float, z:Float}
		return switch axis {
			case PlusX: {x: 1, y: 0, z: 0};
			case MinusX: {x: -1, y: 0, z: 0};
			case PlusZ: {x: 0, y: 0, z: 1};
		};

	override public function hasGeometry():Bool return true;

	override public function geometry(detail:ComponentDetail = Preview):Part {
		var start = direction(startAxis);
		var collar = Part.cylinderAlong(collarDiameter / 2, new Vector(0, 0, 0),
			new Vector(start.x, start.y, start.z), COLLAR_THICKNESS);
		var body = Solids.union([Part.cylinderSpan(diameter / 2, 0, length), collar]);
		if (detail == Envelope) return body;
		return Solids.cut(body, [Part.cylinderSpan(diameter / 2 - wall, wall, length - wall)]);
	}
}

/** Suction tool for the arm's ISO 9409-1 style tool flange: an adapter plate, a frame bar, and a
 * catalog ejector, cup, fitting and hose, arranged like the fixed EOAT in `examples/eoat`.
 * Its `contact` working frame is the cup's contact face.
 */
class ArmSuctionTool {
	public static function build(flange:RobotFlange):EndEffector {
		var result = new EndEffector();
		result.addComponent("plate", new EndEffectorPlate(flange));
		result.addComponent("bar", new FrameBar(30, 20, 90));
		result.addComponent("ejector", new SchmalzVacuumGenerator("10.02.01.00563"));
		result.addComponent("cup", new SchmalzSuctionCup("10.01.01.11401"));
		result.addComponent("fitting", new SchmalzPushInFitting("10.08.02.00203"));
		result.addComponent("hose", new SchmalzVacuumHose("10.07.09.00001", [
			new Vector(20, 0, 40), new Vector(35, 0, 60),
			new Vector(35, 0, 100), new Vector(-30, 0, 100), new Vector(0, 0, 90)]));
		result.mount("plate", "robot");
		result.addMate("bar-mate", "fixed", "plate", "tool", "bar", "base");
		result.addMemberConnector("bar", "ejector-seat", Solids.axial(20, 0, 20));
		result.addMate("ejector-mate", "fixed", "bar", "ejector-seat", "ejector", "mount");
		result.addMate("cup-mate", "fixed", "bar", "end", "cup", "mount");
		result.addMate("fitting-mate", "fixed", "cup", "mount", "fitting", "mount");
		result.addMate("hose-mate", "fixed", "bar", "base", "hose", "mount");
		result.connectPorts("ejector-hose", "ejector", "vacuum", "hose", "input");
		result.connectPorts("hose-fitting", "hose", "output", "fitting", "hose");
		result.connectPorts("fitting-cup", "fitting", "thread", "cup", "vacuum");
		result.exposePort("compressedAir", "ejector", "air");
		result.workingFrame("contact", "cup", "contact", true);
		return result;
	}
}

/** One joint's motion limits and the pose it starts in, in radians and rad/s. */
typedef ArmJointSpec = {
	var id:String;
	var lower:Float;
	var upper:Float;
	var velocity:Float;
	/** Peak joint torque in N·m. */
	var effort:Float;
	var initial:Float;
}

/** Six-axis serial arm on a pedestal with a suction tool, in the classic shoulder/elbow/spherical-wrist layout.
 *
 * At zero on every joint the arm points straight up. Joints `j1`, `j4` and `j6` turn about the
 * vertical (j6 about the tool axis), while `j2`, `j3` and `j5` pitch about a horizontal axis.
 * Every housing belongs to the link before it, so each revolute mate joins a housing's rotor to
 * the next link's start.
 */
class RobotArm extends MachineAssembly {
	public static inline var PEDESTAL_HEIGHT:Float = 300;

	public final flange = new RobotFlange(63);
	public final pedestal:Pedestal;
	public final toolFlange = new RobotFlange(31.5);
	public final tool:EndEffector;
	public final joints:Array<ArmJoint>;
	public final links:Array<ArmLink>;
	public final specs:Array<ArmJointSpec>;

	public function new() {
		super();
		pedestal = new Pedestal(flange, PEDESTAL_HEIGHT, 100);
		var j1 = new ArmJoint(100, 70), j2 = new ArmJoint(100, 90), j3 = new ArmJoint(80, 90);
		var j4 = new ArmJoint(70, 60), j5 = new ArmJoint(60, 70), j6 = new ArmJoint(55, 40, toolFlange);
		joints = [j1, j2, j3, j4, j5, j6];
		links = [
			new ArmLink(90, 90, 5, 100, PlusZ, PlusX, j2.length),
			new ArmLink(320, 80, 5, 100, PlusX, MinusX, j3.length),
			new ArmLink(260, 70, 4, 80, MinusX, PlusZ, j4.length),
			new ArmLink(90, 60, 4, 70, PlusZ, PlusX, j5.length),
			new ArmLink(60, 50, 4, 60, PlusX, PlusZ, j6.length)
		];
		var pi = Math.PI;
		// j3 turns about -X, so a positive j3 folds the forearm the opposite way from a positive j2.
		specs = [
			{id: "j1", lower: -2.9, upper: 2.9, velocity: 2.0, effort: 300, initial: 0},
			{id: "j2", lower: -1.9, upper: 1.9, velocity: 2.0, effort: 400, initial: 0.4},
			{id: "j3", lower: -2.4, upper: 2.4, velocity: 2.4, effort: 250, initial: -1.4},
			{id: "j4", lower: -3.1, upper: 3.1, velocity: 3.0, effort: 80, initial: 0},
			{id: "j5", lower: -2.1, upper: 2.1, velocity: 3.0, effort: 60, initial: pi - 0.4 - 1.4},
			{id: "j6", lower: -6.2, upper: 6.2, velocity: 4.0, effort: 30, initial: 0}
		];
		addComponent("pedestal", pedestal);
		addComponent("baseFlange", flange);
		addMate("base-flange", "fixed", "pedestal", "top", "baseFlange", "face");
		// The flange turns over on the pedestal, so its plate top is the flange's z=-thickness face
		// looking along local -Z.
		addMemberConnector("baseFlange", "plateTop", AssemblyFrames.alongY(0, 0, -flange.thickness, 0, 0, -1));
		addComponent("joint1", j1);
		addMate("base-joint1", "fixed", "baseFlange", "plateTop", "joint1", "stator");
		var linkNames = ["turret", "upperArm", "forearm", "wristBody", "hand"];
		for (i in 0...5) {
			addComponent(linkNames[i], links[i]);
			revolute(specs[i], 'joint${i + 1}', "rotor", linkNames[i], "start");
			addComponent('joint${i + 2}', joints[i + 1]);
			addMate('${linkNames[i]}-joint${i + 2}', "fixed", linkNames[i], "end", 'joint${i + 2}', "stator");
		}
		addComponent("toolFlange", toolFlange);
		revolute(specs[5], "joint6", "tool", "toolFlange", "face");
		tool = ArmSuctionTool.build(toolFlange);
		include("tool", tool);
		addMate("tool-mount", "fixed", "toolFlange", "face", "tool/plate", "robot");
		exposeConnector("toolFace", "toolFlange", "face");
		exposeConnector("toolContact", "tool/cup", "contact");
		// The ejector's compressed-air inlet is the arm's own service input.
		exposePort("compressedAir", "tool/ejector", "air");
	}

	function revolute(spec:ArmJointSpec, housing:String, rotorConnector:String, child:String,
			childConnector:String):Void {
		addMateOnAxis(spec.id, "revolute", housing, rotorConnector, child, childConnector,
			{x: 0, y: 1, z: 0}, spec.initial,
			{lower: spec.lower, upper: spec.upper, velocity: spec.velocity, effort: spec.effort});
	}
}
