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

/**
 * The mobile base at work in a small walled room: two shelves, a pillar in the middle and a charging
 * dock on the west wall. The robot is included as `robot` at `ORIGIN`; `GOALS` is the round it
 * drives, facing each shelf and the dock in turn and coming back to where it started. Every goal
 * leaves the chassis half a metre from what it faces. The room's floor is the assembly's z = 0,
 * centred on its origin, with +X east.
 */
class MobileBaseCell extends MachineAssembly {
	public static inline var ROOM_LENGTH:Float = 5000;
	public static inline var ROOM_WIDTH:Float = 4000;
	public static inline var WALL:Float = 100;
	public static inline var WALL_HEIGHT:Float = 400;

	public static final ORIGIN:FloorPose = {x: -1500, y: -1000, yaw: 0};
	public static final GOALS:Array<FloorPose> = [
		{x: 800, y: 1000, yaw: Math.PI / 2},
		{x: 1450, y: -800, yaw: 0},
		{x: -1750, y: 1000, yaw: Math.PI},
		{x: -1500, y: -1000, yaw: 0}
	];

	public final robot = new MobileBase();

	public function new() {
		super();
		include("robot", robot, floorFrame(ORIGIN));
		var hx = ROOM_LENGTH / 2, hy = ROOM_WIDTH / 2;
		var wallX = new RoomBlock(ROOM_LENGTH + 2 * WALL, WALL, WALL_HEIGHT, "painted steel", "Wall");
		var wallY = new RoomBlock(WALL, ROOM_WIDTH, WALL_HEIGHT, "painted steel", "Wall");
		addComponent("wallNorth", wallX, AssemblyFrames.translation(0, hy + WALL / 2, 0));
		addComponent("wallSouth", wallX, AssemblyFrames.translation(0, -hy - WALL / 2, 0));
		addComponent("wallEast", wallY, AssemblyFrames.translation(hx + WALL / 2, 0, 0));
		addComponent("wallWest", wallY, AssemblyFrames.translation(-hx - WALL / 2, 0, 0));
		addComponent("shelfNorth", new RoomBlock(1200, 400, 1200, "birch plywood", "Shelf"),
			AssemblyFrames.translation(800, 1700, 0));
		addComponent("shelfEast", new RoomBlock(400, 1200, 1200, "birch plywood", "Shelf"),
			AssemblyFrames.translation(2200, -800, 0));
		addComponent("pillar", new RoomBlock(300, 300, 1000, "painted steel", "Pillar"), AssemblyFrames.translation(0, 0, 0));
		addComponent("dock", new RoomBlock(200, 500, 300, "plastic", "Dock"), AssemblyFrames.translation(-2350, 1000, 0));
	}

	/** The assembly frame of a floor pose. */
	public static function floorFrame(pose:FloorPose):AssemblyFrame
		return {x: pose.x, y: pose.y, z: 0, qx: 0, qy: 0, qz: Math.sin(pose.yaw / 2), qw: Math.cos(pose.yaw / 2)};
}
