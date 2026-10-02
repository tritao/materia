import cadkit.modeling.Part;
import machinekit.assembly.MachineAssembly;
import machinekit.component.ComponentDetail;
import machinekit.component.Dimension;
import machinekit.component.MachineComponent;
import machinekit.component.Solids;
import materia.assembly.AssemblyFrames;
import materia.assembly.AssemblyRecord.AssemblyFrame;

/** A block standing on the floor (z=0), centred on its origin: a wall, shelf, pillar or dock. */
class RoomBlock extends MachineComponent {
	public final length:Float;
	public final width:Float;
	public final height:Float;

	public function new(length:Float, width:Float, height:Float, material:String, name:String) {
		if (!(length > 0) || !(width > 0) || !(height > 0)) throw "Room block needs positive dimensions";
		super('${name.toUpperCase()}-${Dimension.format(length)}x${Dimension.format(width)}x${Dimension.format(height)}',
			name, material, true);
		this.length = length;
		this.width = width;
		this.height = height;
		addConnector("base", Mount, Solids.axial(0, 0, 0));
	}

	override public function hasGeometry():Bool return true;

	override public function geometry(detail:ComponentDetail = Preview):Part
		return Part.box(length, width, height);
}

/** A pose on the floor in millimetres and radians. */
typedef FloorPose = {x:Float, y:Float, yaw:Float};

/** One step of the cell's round: drive to a pose, pick the workpiece, or place it on a table's seat. */
enum CellStep {
	GoTo(pose:FloorPose);
	Pick(part:String, grasp:String);
	Place(table:String, seat:String);
}

/**
 * The mobile manipulator at work in a small walled room: two tables, a pillar in the middle and a
 * charging dock on the west wall. The robot, the base carrying the arm, is included as `robot` at
 * `ORIGIN`. A workpiece starts on the north table; `ROUND` carries it to the east table and back,
 * then rests at the dock. Every goal leaves the chassis half a metre or so from what it faces, and
 * at a table it leaves the place seat in the arm's reach ahead. The room's floor is the assembly's
 * z = 0, centred on its origin, with +X east.
 */
class MobileBaseCell extends MachineAssembly {
	public static inline var ROOM_LENGTH:Float = 5000;
	public static inline var ROOM_WIDTH:Float = 4000;
	public static inline var WALL:Float = 100;
	public static inline var WALL_HEIGHT:Float = 400;

	public static final ORIGIN:FloorPose = {x: -1500, y: -1000, yaw: 0};
	/** Facing the north table, the east table and the dock. */
	public static final AT_NORTH:FloorPose = {x: 800, y: 1000, yaw: Math.PI / 2};
	public static final AT_EAST:FloorPose = {x: 1450, y: -800, yaw: 0};
	public static final AT_DOCK:FloorPose = {x: -1750, y: 1000, yaw: Math.PI};
	public static final GOALS:Array<FloorPose> = [AT_NORTH, AT_EAST, AT_DOCK];
	public static final ROUND:Array<CellStep> = [
		GoTo(AT_NORTH), Pick("workpiece", "top"),
		GoTo(AT_EAST), Place("tableEast", "placeSeat"), Pick("workpiece", "top"),
		GoTo(AT_NORTH), Place("tableNorth", "placeSeat"),
		GoTo(AT_DOCK)
	];
	/** The tables stand as tall as the arm's own cell, raised by the deck the arm stands on. */
	public static final TABLE_TOP:Float = RobotArm.TABLE_TOP + RobotArm.PAD_HEIGHT + MobileBase.DECK_Z + MobileBase.DECK_THICKNESS;

	public final robot = new MobileBase(new RobotArm(false));

	public function new() {
		super();
		include("robot", robot, floorFrame(ORIGIN));
		exposePort("compressedAir", "robot/arm/tool/ejector", "air");
		var hx = ROOM_LENGTH / 2, hy = ROOM_WIDTH / 2;
		var wallX = new RoomBlock(ROOM_LENGTH + 2 * WALL, WALL, WALL_HEIGHT, "painted steel", "Wall");
		var wallY = new RoomBlock(WALL, ROOM_WIDTH, WALL_HEIGHT, "painted steel", "Wall");
		addComponent("wallNorth", wallX, AssemblyFrames.translation(0, hy + WALL / 2, 0));
		addComponent("wallSouth", wallX, AssemblyFrames.translation(0, -hy - WALL / 2, 0));
		addComponent("wallEast", wallY, AssemblyFrames.translation(hx + WALL / 2, 0, 0));
		addComponent("wallWest", wallY, AssemblyFrames.translation(-hx - WALL / 2, 0, 0));
		table("tableNorth", 800, 500, AT_NORTH);
		table("tableEast", 500, 800, AT_EAST);
		var seat = seatAhead(AT_NORTH);
		addComponent("workpiece", new ArmBlock(RobotArm.WORKPIECE_WIDTH, RobotArm.WORKPIECE_WIDTH, RobotArm.WORKPIECE_HEIGHT,
			"birch plywood", "Workpiece"), AssemblyFrames.translation(seat.x, seat.y, TABLE_TOP));
		addComponent("pillar", new RoomBlock(300, 300, 1000, "painted steel", "Pillar"), AssemblyFrames.translation(0, 0, 0));
		addComponent("dock", new RoomBlock(200, 500, 300, "plastic", "Dock"), AssemblyFrames.translation(-2350, 1000, 0));
	}

	/**
	 * A table whose near edge stands `REACH - SEAT_INSET` ahead of `at`, centred across it, with its
	 * `placeSeat` on the top where the arm reaches from there.
	 */
	function table(id:String, width:Float, depth:Float, at:FloorPose):Void {
		var ahead = width < depth ? width : depth;
		var c = Math.cos(at.yaw), s = Math.sin(at.yaw);
		var centre = REACH - SEAT_INSET + ahead / 2;
		var x = at.x + c * centre, y = at.y + s * centre;
		addComponent(id, new ArmTable(width, depth, TABLE_TOP), AssemblyFrames.translation(x, y, 0));
		var seat = seatAhead(at);
		addMemberConnector(id, "placeSeat", Solids.axial(seat.x - x, seat.y - y, TABLE_TOP));
	}

	/** How far ahead of the robot's centre the arm reaches, and how far inside a table's edge that is. */
	public static inline var REACH:Float = 600;
	public static inline var SEAT_INSET:Float = 100;

	/** The point the arm reaches when the robot stands at `at`. */
	public static function seatAhead(at:FloorPose):{x:Float, y:Float}
		return {x: at.x + Math.cos(at.yaw) * (MobileBase.PAYLOAD_X + REACH), y: at.y + Math.sin(at.yaw) * (MobileBase.PAYLOAD_X + REACH)};

	/** The assembly frame of a floor pose. */
	public static function floorFrame(pose:FloorPose):AssemblyFrame
		return {x: pose.x, y: pose.y, z: 0, qx: 0, qy: 0, qz: Math.sin(pose.yaw / 2), qw: Math.cos(pose.yaw / 2)};
}
