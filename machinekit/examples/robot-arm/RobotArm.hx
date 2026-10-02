import cadkit.modeling.Vector;
import cadkit.modeling.Part;
import machinekit.assembly.MachineAssembly;
import machinekit.component.ComponentDetail;
import machinekit.component.Dimension;
import machinekit.component.MachineComponent;
import machinekit.component.Solids;
import machinekit.pneumatic.VacuumPressureSensor;
import machinekit.pneumatic.schmalz.SchmalzPushInFitting;
import machinekit.pneumatic.schmalz.SchmalzSuctionCup;
import machinekit.pneumatic.schmalz.SchmalzVacuumGenerator;
import machinekit.pneumatic.schmalz.SchmalzVacuumHose;
import machinekit.robotics.ArmJoint;
import machinekit.robotics.ArmLink;
import machinekit.robotics.ArmLink.ArmAxis;
import machinekit.robotics.EndEffector;
import machinekit.robotics.EndEffectorPlate;
import machinekit.robotics.FrameBar;
import machinekit.robotics.Pedestal;
import machinekit.robotics.RobotFlange;
import materia.assembly.AssemblyFrames;
import materia.assembly.AssemblyRecord.AssemblyFrame;

/** Suction tool for the arm's ISO 9409-1 style tool flange: an adapter plate, a frame bar, and a
 * catalog ejector, cup, fitting and hose, arranged like the fixed EOAT in `examples/eoat`, with an
 * inline vacuum sensor between the ejector and the hose that tells a sealed cup from an open one.
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
		result.addComponent("sensor", new VacuumPressureSensor(4));
		result.addComponent("hose", new SchmalzVacuumHose("10.07.09.00001", [
			new Vector(20, 0, 40), new Vector(35, 0, 60),
			new Vector(35, 0, 100), new Vector(-30, 0, 100), new Vector(0, 0, 90)]));
		result.mount("plate", "robot");
		result.addMate("bar-mate", "fixed", "plate", "tool", "bar", "base");
		result.addMemberConnector("bar", "ejector-seat", Solids.axial(20, 0, 20));
		result.addMate("ejector-mate", "fixed", "bar", "ejector-seat", "ejector", "mount");
		result.addMemberConnector("bar", "sensor-seat", Solids.axial(-25, 0, 20));
		result.addMate("sensor-mate", "fixed", "bar", "sensor-seat", "sensor", "mount");
		result.addMate("cup-mate", "fixed", "bar", "end", "cup", "mount");
		result.addMate("fitting-mate", "fixed", "cup", "mount", "fitting", "mount");
		result.addMate("hose-mate", "fixed", "bar", "base", "hose", "mount");
		result.connectPorts("ejector-sensor", "ejector", "vacuum", "sensor", "vacuumIn");
		result.connectPorts("sensor-hose", "sensor", "vacuumOut", "hose", "input");
		result.connectPorts("hose-fitting", "hose", "output", "fitting", "hose");
		result.connectPorts("fitting-cup", "fitting", "thread", "cup", "vacuum");
		result.exposePort("compressedAir", "ejector", "air");
		result.workingFrame("contact", "cup", "contact", true);
		return result;
	}
}

/** Work table: a slab on four legs, standing on the floor (z=0) with its top at `height`. The origin
 * is the centre of the footprint. */
class ArmTable extends MachineComponent {
	public final width:Float;
	public final depth:Float;
	public final height:Float;
	public final thickness:Float;

	public function new(width:Float, depth:Float, height:Float, thickness:Float = 30) {
		if (!(width > 0) || !(depth > 0) || !(thickness > 0) || !(height > thickness) || width < 200 || depth < 200)
			throw "Table needs positive dimensions and legs under its top";
		super('TABLE-${Dimension.format(width)}x${Dimension.format(depth)}x${Dimension.format(height)}',
			"Work table", "steel", true);
		this.width = width;
		this.depth = depth;
		this.height = height;
		this.thickness = thickness;
		addConnector("base", Mount, Solids.axial(0, 0, 0));
	}

	override public function hasGeometry():Bool return true;

	override public function geometry(detail:ComponentDetail = Preview):Part {
		var parts = [Part.box(width, depth, thickness).translated(new Vector(0, 0, height - thickness))];
		var leg = 40.0, inset = 30.0;
		for (sx in [-1, 1]) for (sy in [-1, 1])
			parts.push(Part.box(leg, leg, height - thickness)
				.translated(new Vector(sx * (width / 2 - inset - leg / 2), sy * (depth / 2 - inset - leg / 2), 0)));
		return Solids.union(parts);
	}
}

/** A plain block standing on its base (z=0), centred on its origin. */
class ArmBlock extends MachineComponent {
	public final width:Float;
	public final depth:Float;
	public final height:Float;

	public function new(width:Float, depth:Float, height:Float, material:String, name:String) {
		if (!(width > 0) || !(depth > 0) || !(height > 0)) throw "Block needs positive dimensions";
		super('${name.toUpperCase()}-${Dimension.format(width)}x${Dimension.format(depth)}x${Dimension.format(height)}',
			name, material, true);
		this.width = width;
		this.depth = depth;
		this.height = height;
		addConnector("base", Mount, Solids.axial(0, 0, 0));
		addConnector("top", Mount, Solids.axial(0, 0, height));
	}

	override public function hasGeometry():Bool return true;

	override public function geometry(detail:ComponentDetail = Preview):Part
		return Part.box(width, depth, height);
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
 *
 * Built `withCell`, it stands in its own work cell: a table, two pads and a workpiece in front of it
 * (-Y). Without, it is the arm alone, to stand on something else by its pedestal's `floor`.
 */
class RobotArm extends MachineAssembly {
	public static inline var PEDESTAL_HEIGHT:Float = 300;
	/** Work cell in front of the arm (-Y), in millimetres from the pedestal axis and the floor. */
	public static inline var TABLE_TOP:Float = 250;
	public static inline var TABLE_CENTRE_Y:Float = -650;
	public static inline var WORKPIECE_WIDTH:Float = 60;
	public static inline var WORKPIECE_HEIGHT:Float = 50;
	public static inline var PAD_HEIGHT:Float = 2;
	/** Two pads, side by side; the workpiece starts on the first and is carried to the second and back. */
	public static inline var PICK_X:Float = -150;
	public static inline var PLACE_X:Float = 150;
	public static inline var WORK_Y:Float = -600;

	public final flange = new RobotFlange(63);
	public final pedestal:Pedestal;
	public final toolFlange = new RobotFlange(31.5);
	public final tool:EndEffector;
	public final joints:Array<ArmJoint>;
	public final links:Array<ArmLink>;
	public final specs:Array<ArmJointSpec>;

	public function new(withCell:Bool = true) {
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
		exposeConnector("floor", "pedestal", "floor");
		if (withCell) addCell();
	}

	/**
	 * Table, two pads and a workpiece in the arm's reach. The table and pads are fixed roots; the
	 * project's `dynamicParts` frees the workpiece so the suction cup can carry it.
	 */
	function addCell():Void {
		function at(x:Float, y:Float, z:Float):AssemblyFrame return AssemblyFrames.translation(x, y, z);
		addComponent("table", new ArmTable(800, 500, TABLE_TOP), at(0, TABLE_CENTRE_Y, 0));
		var pad = new ArmBlock(WORKPIECE_WIDTH + 20, WORKPIECE_WIDTH + 20, PAD_HEIGHT, "rubber", "Pad");
		addComponent("padPick", pad, at(PICK_X, WORK_Y, TABLE_TOP));
		addComponent("padPlace", pad, at(PLACE_X, WORK_Y, TABLE_TOP));
		addComponent("workpiece", new ArmBlock(WORKPIECE_WIDTH, WORKPIECE_WIDTH, WORKPIECE_HEIGHT, "birch plywood",
			"Workpiece"), at(PICK_X, WORK_Y, TABLE_TOP + PAD_HEIGHT));
	}

	function revolute(spec:ArmJointSpec, housing:String, rotorConnector:String, child:String,
			childConnector:String):Void {
		addMateOnAxis(spec.id, "revolute", housing, rotorConnector, child, childConnector,
			{x: 0, y: 1, z: 0}, spec.initial,
			{lower: spec.lower, upper: spec.upper, velocity: spec.velocity, effort: spec.effort});
	}
}
