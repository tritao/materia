import cadkit.modeling.Align;
import cadkit.modeling.Part;
import cadkit.modeling.Vector;
import machinekit.assembly.PosedAssembly;
import machinekit.component.ComponentDetail;
import machinekit.component.Dimension;
import machinekit.component.MachineComponent;
import machinekit.component.Solids;
import machinekit.motion.CasterWheel;
import machinekit.motion.DriveWheel;
import machinekit.motion.NemaStepper;
import machinekit.structural.RectTube;
import materia.assembly.AssemblyFrames;
import materia.assembly.AssemblyRecord.AssemblyFrame;

/** A flat plate lying on its base (z=0), centred on its origin, with optional rectangular slots through it. */
class ChassisPlate extends MachineComponent {
	public final length:Float;
	public final width:Float;
	public final thickness:Float;
	/** Slots as centre x, centre y, length along X and width along Y. */
	public final slots:Array<{x:Float, y:Float, length:Float, width:Float}>;

	public function new(length:Float, width:Float, thickness:Float, name:String,
			?slots:Array<{x:Float, y:Float, length:Float, width:Float}>) {
		if (!(length > 0) || !(width > 0) || !(thickness > 0)) throw "Chassis plate needs positive dimensions";
		this.slots = slots == null ? [] : slots;
		var slotText = this.slots.length == 0 ? "" : '-S${this.slots.length}';
		super('${name.toUpperCase()}-${Dimension.format(length)}x${Dimension.format(width)}x${Dimension.format(thickness)}$slotText',
			name, "aluminium 6061", true);
		this.length = length;
		this.width = width;
		this.thickness = thickness;
		addConnector("base", Mount, Solids.axial(0, 0, 0));
		addConnector("top", Mount, Solids.axial(0, 0, thickness));
	}

	override public function hasGeometry():Bool return true;

	override public function geometry(detail:ComponentDetail = Preview):Part {
		var plate = Part.box(length, width, thickness);
		if (detail == Envelope || slots.length == 0) return plate;
		return Solids.cut(plate, [for (slot in slots)
			Part.box(slot.length, slot.width, thickness + 2).translated(new Vector(slot.x, slot.y, -1))]);
	}
}

/**
 * L-shaped motor bracket hanging under the base plate. Origin on the motor axis at the plate's inner
 * face; the axis runs along local +Z through the plate (z = 0..thickness) and local +Y points up to
 * the top flange, which runs inward (toward -Z) under the base plate.
 */
class MotorBracket extends MachineComponent {
	public final motor:NemaStepper;
	public final width:Float;
	public final below:Float;
	public final above:Float;
	public final thickness:Float;
	public final flangeDepth:Float;
	public final flangeThickness:Float;

	public function new(motor:NemaStepper, width:Float, below:Float, above:Float, thickness:Float, flangeDepth:Float,
			flangeThickness:Float) {
		if (!(width > motor.variant.bodyFace) || !(below > motor.variant.bodyFace / 2) ||
				!(above - flangeThickness > motor.variant.bodyFace / 2) || !(thickness > 0) || !(flangeDepth > 0))
			throw "Motor bracket must clear its motor's body";
		super('MOTOR-BRACKET-${motor.spec.frame}-${Dimension.format(width)}x${Dimension.format(below + above)}x${Dimension.format(thickness)}',
			'Motor bracket for NEMA ${motor.spec.frame}', "aluminium 6061", true);
		this.motor = motor;
		this.width = width;
		this.below = below;
		this.above = above;
		this.thickness = thickness;
		this.flangeDepth = flangeDepth;
		this.flangeThickness = flangeThickness;
	}

	override public function hasGeometry():Bool return true;

	override public function geometry(detail:ComponentDetail = Preview):Part {
		var plate = Part.box(width, below + above, thickness, Align.Center, Align.Min, Align.Min).translated(new Vector(0, -below, 0));
		var flange = Part.box(width, flangeThickness, flangeDepth + thickness, Align.Center, Align.Max, Align.Max)
			.translated(new Vector(0, above, thickness));
		var body = Solids.union([plate, flange]);
		if (detail == Envelope) return body;
		return Solids.cut(body, [motor.mountingCutout(thickness + 0.2).translated(new Vector(0, 0, -0.1))]);
	}
}

/** Battery pack: a box standing on its base (z=0), centred on its origin. */
class BatteryPack extends MachineComponent {
	public final length:Float;
	public final width:Float;
	public final height:Float;

	public function new(length:Float, width:Float, height:Float) {
		if (!(length > 0) || !(width > 0) || !(height > 0)) throw "Battery needs positive dimensions";
		super('BATTERY-${Dimension.format(length)}x${Dimension.format(width)}x${Dimension.format(height)}',
			"Battery pack", "plastic", true);
		this.length = length;
		this.width = width;
		this.height = height;
		addConnector("base", Mount, Solids.axial(0, 0, 0));
	}

	override public function hasGeometry():Bool return true;

	override public function geometry(detail:ComponentDetail = Preview):Part
		return Part.box(length, width, height);
}

/** A cut length of rectangular tube standing along local +Z from z=0, its section centred. */
class TubePost extends MachineComponent {
	public final profile:RectTube;
	public final length:Float;

	public function new(profile:RectTube, length:Float) {
		if (!(length > 0)) throw "Tube post needs a positive length";
		super('${profile.designation}-L${Dimension.format(length)}', '${profile.description}, ${Dimension.format(length)} mm long',
			"aluminium 6061", true);
		this.profile = profile;
		this.length = length;
	}

	override public function hasGeometry():Bool return true;

	override public function geometry(detail:ComponentDetail = Preview):Part {
		if (detail == Envelope) return Part.box(profile.width, profile.height, length);
		return profile.geometry(length);
	}
}

/**
 * Planar scanning lidar: a puck standing on its base (z=0) with a window band. Its `scan` connector
 * is the optical centre, with the scan plane horizontal and its zero bearing along +X.
 */
class LidarPuck extends MachineComponent {
	public static inline var DIAMETER:Float = 76;
	public static inline var HEIGHT:Float = 70;
	public static inline var SCAN_HEIGHT:Float = 50;

	public function new() {
		super("LIDAR-D76-H70", "Planar scanning lidar", "plastic", true);
		addConnector("base", Mount, Solids.axial(0, 0, 0));
		addConnector("scan", Mount, AssemblyFrames.translation(0, 0, SCAN_HEIGHT));
	}

	override public function hasGeometry():Bool return true;

	override public function geometry(detail:ComponentDetail = Preview):Part {
		if (detail == Envelope) return Part.cylinderSpan(DIAMETER / 2, 0, HEIGHT);
		return Solids.union([Part.cylinderSpan(DIAMETER / 2, 0, 40), Part.cylinderSpan(DIAMETER / 2 - 5, 40, 60),
			Part.cylinderSpan(DIAMETER / 2, 60, HEIGHT)]);
	}
}

/**
 * Differential-drive mobile base: an aluminium base plate with two wheel slots, two NEMA 23 steppers
 * on brackets under it driving 150 mm wheels directly, a swivel caster front and rear, a battery,
 * and an upper deck on four tube posts carrying a lidar at the front.
 *
 * The robot drives along +X with its axles along Y; the assembly origin is on the floor (z = 0)
 * midway between the wheels' floor contacts. Joints `wheel_l` (+Y side) and `wheel_r` are
 * continuous about +Y, so a positive speed on both rolls the robot forward. Every other member is
 * fixed to the base plate, the root.
 */
class MobileBase extends PosedAssembly {
	public static inline var LENGTH:Float = 600;
	public static inline var WIDTH:Float = 440;
	public static inline var WHEEL_DIAMETER:Float = 150;
	public static inline var WHEEL_WIDTH:Float = 40;
	/** Underside of the base plate. */
	public static inline var BASE_Z:Float = 110;
	public static inline var BASE_THICKNESS:Float = 10;
	/** Underside of the upper deck. */
	public static inline var DECK_Z:Float = 320;
	public static inline var DECK_THICKNESS:Float = 8;
	/** Inner face of each motor bracket, where the motor's face sits, from the centre line. */
	public static inline var BRACKET_Y:Float = 110;
	public static inline var BRACKET_THICKNESS:Float = 8;
	/** Gap between a bracket's outer face and its wheel's hub. */
	public static inline var HUB_GAP:Float = 2;
	public static inline var CASTER_X:Float = 230;
	/** Wheel speed limit in rad/s (about 0.9 m/s) and the stepper's holding torque in N·m. */
	public static inline var WHEEL_SPEED:Float = 12;
	public static inline var WHEEL_TORQUE:Float = 1.2;

	public final motor:NemaStepper;
	public final wheel:DriveWheel;
	public final caster:CasterWheel;
	public final lidar = new LidarPuck();

	public function new() {
		super();
		motor = NemaStepper.frame(23);
		wheel = new DriveWheel(WHEEL_DIAMETER, WHEEL_WIDTH, motor.variant.shaftDiameter, 40, 10);
		caster = new CasterWheel(75, 25, BASE_Z, 30, 60);
		var axleZ = wheel.radius;
		var up = [0.0, 0, 1];

		// Base plate, with a slot for each wheel to turn through.
		var wheelCentreY = BRACKET_Y + BRACKET_THICKNESS + HUB_GAP + wheel.hubLength + WHEEL_WIDTH / 2;
		var slot = {length: 2 * Math.sqrt(Math.pow(wheel.radius + 4, 2) - Math.pow(BASE_Z - axleZ, 2)), width: WHEEL_WIDTH + 12};
		place("basePlate", new ChassisPlate(LENGTH, WIDTH, BASE_THICKNESS, "Base plate", [
			{x: 0, y: wheelCentreY, length: slot.length, width: slot.width},
			{x: 0, y: -wheelCentreY, length: slot.length, width: slot.width}]), AssemblyFrames.translation(0, 0, BASE_Z));

		// Drives: bracket, motor and wheel on each side, the motor's shaft pointing outward into the hub.
		var bracket = new MotorBracket(motor, 80, axleZ - 40, BASE_Z - axleZ, BRACKET_THICKNESS, 40, 5);
		for (side in [1, -1]) {
			var name = side > 0 ? "Left" : "Right";
			var outward = [0.0, side, 0];
			attach('bracket$name', bracket, PosedAssembly.orient(0, side * BRACKET_Y, axleZ, up, outward), "basePlate");
			attach('motor$name', side > 0 ? motor : NemaStepper.frame(23),
				PosedAssembly.orient(0, side * BRACKET_Y, axleZ, up, outward), 'bracket$name');
			var hubY = BRACKET_Y + BRACKET_THICKNESS + HUB_GAP;
			move(side > 0 ? "wheel_l" : "wheel_r", "continuous", 'wheel$name', wheel,
				PosedAssembly.orient(0, side * hubY, axleZ, up, outward), 'motor$name', {x: 0, y: 1, z: 0}, 0,
				{lower: null, upper: null, velocity: WHEEL_SPEED, effort: WHEEL_TORQUE});
		}

		// Casters, front and rear, their plates against the underside of the base plate.
		attach("casterFront", caster, AssemblyFrames.translation(CASTER_X, 0, BASE_Z), "basePlate");
		attach("casterRear", caster, AssemblyFrames.translation(-CASTER_X, 0, BASE_Z), "basePlate");

		// Battery between the wheel slots, posts and the upper deck, and the lidar at the front of the deck.
		var top = BASE_Z + BASE_THICKNESS;
		attach("battery", new BatteryPack(260, 180, 110), AssemblyFrames.translation(0, 0, top), "basePlate");
		var post = new TubePost(new RectTube(40, 40, 3), DECK_Z - top);
		for (corner in [{id: "postFrontLeft", x: 1, y: 1}, {id: "postFrontRight", x: 1, y: -1},
				{id: "postRearLeft", x: -1, y: 1}, {id: "postRearRight", x: -1, y: -1}])
			attach(corner.id, post, AssemblyFrames.translation(corner.x * (LENGTH / 2 - 40), corner.y * (WIDTH / 2 - 40), top),
				"basePlate");
		attach("deck", new ChassisPlate(LENGTH, WIDTH, DECK_THICKNESS, "Deck"), AssemblyFrames.translation(0, 0, DECK_Z),
			"postFrontLeft");
		attach("lidar", lidar, AssemblyFrames.translation(LENGTH / 2 - 80, 0, DECK_Z + DECK_THICKNESS), "deck");
		exposeConnector("lidar", "lidar", "scan");
		exposeConnector("deckTop", "deck", "top");
	}

	/** Assembly-frame pose of a member connector with every joint at zero. */
	public function connectorAtZero(id:String, connector:String):AssemblyFrame
		return AssemblyFrames.compose(zeroPose(id), memberConnectorFrame(id, connector));

	/** Distance between the wheels' mid-tread planes, measured from the assembly. */
	public function trackWidth():Float
		return connectorAtZero("wheelLeft", "centre").y - connectorAtZero("wheelRight", "centre").y;

	/** Outline of the chassis on the floor, counter-clockwise, in millimetres from the assembly origin. */
	public static function footprint():Array<{x:Float, y:Float}> {
		var hx = LENGTH / 2, hy = WIDTH / 2;
		return [{x: hx, y: hy}, {x: -hx, y: hy}, {x: -hx, y: -hy}, {x: hx, y: -hy}];
	}
}
