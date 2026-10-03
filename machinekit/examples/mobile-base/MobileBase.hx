import machinekit.motion.MotorDriver;
import cadkit.modeling.Align;
import cadkit.modeling.Part;
import cadkit.modeling.Vector;
import machinekit.assembly.MachineAssembly;
import machinekit.component.ComponentDetail;
import machinekit.component.Dimension;
import machinekit.component.MachineComponent;
import machinekit.component.Solids;
import machinekit.motion.CasterWheel;
import machinekit.motion.DriveWheel;
import machinekit.motion.Gearbox;
import machinekit.motion.NemaStepper;
import machinekit.structural.RectTube;
import materia.assembly.AssemblyFrames;
import materia.assembly.AssemblyRecord.AssemblyFrame;

/** Where something mounts on a plate: a connector frame and the through-holes it bolts through, in plate coordinates. */
typedef PlateSeat = {
	var name:String;
	var frame:AssemblyFrame;
	var holes:Array<{x:Float, y:Float, diameter:Float}>;
}

/**
 * A flat plate lying on its base (z=0), centred on its origin. It owns the layout of what mounts on it:
 * each seat is a connector and the holes for its fasteners, so moving a seat moves both. Slots
 * (centre x, centre y, length along X, width along Y) are cut clean through.
 */
class ChassisPlate extends MachineComponent {
	public final length:Float;
	public final width:Float;
	public final thickness:Float;
	public final slots:Array<{x:Float, y:Float, length:Float, width:Float}>;
	public final seats:Array<PlateSeat>;

	public function new(length:Float, width:Float, thickness:Float, name:String, seats:Array<PlateSeat>,
			?slots:Array<{x:Float, y:Float, length:Float, width:Float}>) {
		if (!(length > 0) || !(width > 0) || !(thickness > 0)) throw "Chassis plate needs positive dimensions";
		this.slots = slots == null ? [] : slots;
		this.seats = seats;
		super('${name.toUpperCase()}-${Dimension.format(length)}x${Dimension.format(width)}x${Dimension.format(thickness)}',
			name, "aluminium 6061", true);
		this.length = length;
		this.width = width;
		this.thickness = thickness;
		for (seat in seats) addConnector(seat.name, Mount, seat.frame);
	}

	/** Seat frame on the top face at (x, y), facing up. */
	public static function onTop(x:Float, y:Float, thickness:Float):AssemblyFrame return Solids.axial(x, y, thickness);

	/** Seat frame on the underside at (x, y), facing up like the parts hung from it. */
	public static function underneath(x:Float, y:Float):AssemblyFrame return Solids.axial(x, y, 0);

	override public function hasGeometry():Bool return true;

	override public function geometry(detail:ComponentDetail = Preview):Part {
		var plate = Solids.named(Part.box(length, width, thickness), "plate");
		if (detail == Envelope) return plate;
		var tools = [for (slot in slots) Part.box(slot.length, slot.width, thickness + 2).translated(new Vector(slot.x, slot.y, -1))];
		for (seat in seats) for (hole in seat.holes) tools.push(Part.cylinderSpan(hole.diameter / 2, -1, thickness + 1, hole.x, hole.y));
		return Solids.cut(plate, tools);
	}
}

/**
 * L-shaped motor bracket hanging under the base plate. Origin on the motor axis at the plate's inner
 * face; the axis runs along local +Z through the plate (z = 0..thickness) and local +Y points up to
 * the top flange, which runs inward (toward -Z) under the base plate. Connectors: `motorSeat` (the
 * motor's face, the same frame as the motor's `mountFace`) and `top` (the flange's top face, with
 * the bracket's own axes), plus two flange holes `holeOffset` either side of the axis along X.
 */
class MotorBracket extends MachineComponent {
	public final motor:NemaStepper;
	public final width:Float;
	public final below:Float;
	public final above:Float;
	public final thickness:Float;
	public final flangeDepth:Float;
	public final flangeThickness:Float;
	public final holeOffset:Float = 25;
	public final holeDiameter:Float = 5.5;

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
		addConnector("motorSeat", Mount, Solids.axial(0, 0, 0));
		addConnector("top", Mount, AssemblyFrames.translation(0, above, 0));
	}

	/** Centre of the flange's bolt line, along the motor axis from the bracket's inner face. */
	public function flangeHoleZ():Float return (thickness - flangeDepth) / 2;

	override public function hasGeometry():Bool return true;

	override public function geometry(detail:ComponentDetail = Preview):Part {
		var plate = Part.box(width, below + above, thickness, Align.Center, Align.Min, Align.Min).translated(new Vector(0, -below, 0));
		var flange = Part.box(width, flangeThickness, flangeDepth + thickness, Align.Center, Align.Max, Align.Max)
			.translated(new Vector(0, above, thickness));
		var body = Solids.union([plate, flange]);
		if (detail == Envelope) return body;
		var tools = [motor.mountingCutout(thickness + 0.2).translated(new Vector(0, 0, -0.1))];
		for (x in [-holeOffset, holeOffset])
			tools.push(Part.cylinderAlongY(holeDiameter / 2, above - flangeThickness - 1, above + 1, x, flangeHoleZ()));
		return Solids.cut(body, tools);
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

/** A cut length of rectangular tube standing along local +Z from its `base` (z=0) to its `top`, its section centred. */
class TubePost extends MachineComponent {
	public final profile:RectTube;
	public final length:Float;

	public function new(profile:RectTube, length:Float) {
		if (!(length > 0)) throw "Tube post needs a positive length";
		super('${profile.designation}-L${Dimension.format(length)}', '${profile.description}, ${Dimension.format(length)} mm long',
			"aluminium 6061", true);
		this.profile = profile;
		this.length = length;
		addConnector("base", Mount, Solids.axial(0, 0, 0));
		addConnector("top", Mount, Solids.axial(0, 0, length));
	}

	override public function hasGeometry():Bool return true;

	override public function geometry(detail:ComponentDetail = Preview):Part {
		if (detail == Envelope) return Part.box(profile.width, profile.height, length);
		return profile.geometry(length);
	}
}

/**
 * Planar scanning lidar: a puck standing on its `base` (z=0) with a window band. Its `scan`
 * connector is the optical centre, with the scan plane horizontal and its zero bearing along +X.
 */
class LidarPuck extends MachineComponent {
	public static inline var DIAMETER:Float = 76;
	public static inline var HEIGHT:Float = 70;
	public static inline var SCAN_HEIGHT:Float = 50;
	/** One return per degree (the most a RobotKit sensor reports), ten scans a second, out to six metres. */
	public static inline var RAYS:Int = 360;
	public static inline var RANGE:Float = 6;
	public static inline var RATE:Float = 10;

	public function new() {
		super("LIDAR-D76-H70", "Planar scanning lidar", "plastic", true);
		addConnector("base", Mount, Solids.axial(0, 0, 0));
		addConnector("scan", Mount, AssemblyFrames.translation(0, 0, SCAN_HEIGHT));
		addCapability(PlanarScanner("scan", RAYS, RANGE, RATE));
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
 * midway between the wheels' floor contacts. The plates own the layout: every part mates to a
 * named seat on the base plate or the deck through its own connector, and each wheel's bore mates
 * to its motor's shaft on continuous joint `wheel_l` (+Y side) or `wheel_r`. Each joint turns about
 * its own shaft, which points outward, so a positive speed rolls the left wheel forward and the
 * right wheel backward, as each motor's encoder counts it.
 */
class MobileBase extends MachineAssembly {
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
	public static inline var LIDAR_X:Float = 220;
	/** The payload seat, where an arm stands: on the centre line, turned so the arm works ahead (+X). */
	public static inline var PAYLOAD_X:Float = 0;
	/** Post centres, inset from the plate corners. */
	public static inline var POST_X:Float = 260;
	public static inline var POST_Y:Float = 180;
	/**
	 * The wheel drive: each NEMA 23 turns its wheel through a 10:1 gearhead at 90% efficiency, on a 24 V
	 * supply with half the holding torque relied on (`NemaStepper.actuator`'s default margin). The gearhead
	 * ratio, its efficiency and the supply are assumptions (a planetary gearhead on a NEMA 23 is typically 3:1
	 * to 100:1 at 0.8 to 0.95); the gearhead is not drawn, the wheel sits on the motor's shaft.
	 */
	public static final WHEEL_GEARBOX = new Gearbox(10, 0.9);
	public static inline var WHEEL_SUPPLY:Float = 24;
	static function wheelDriver():MotorDriver return new MotorDriver("GENERIC-DM542", 2.8, 16, WHEEL_SUPPLY);
	/** The motor's actuator as `MachineAssembly.addMotor` makes it, before the gearbox. */
	static function wheelMotor():materia.assembly.AssemblyDefinition.AssemblyActuator {
		var driver = wheelDriver();
		var voltage = driver.statedVoltage;
		if (voltage == null) throw "The nominal wheel driver needs a stated supply voltage";
		return NemaStepper.frame(23).actuator("wheel", "wheel", voltage, 0.5, driver.current);
	}
	/**
	 * Wheel speed limit in rad/s and torque in N·m, from the drive: the stepper's usable speed over the gearhead
	 * ratio (about 13.7 rad/s, a little over 1 m/s on the 75 mm wheel) and its usable torque through the gearhead.
	 */
	public static final WHEEL_SPEED:Float = WHEEL_GEARBOX.jointSpeed(wheelMotor().maxRate);
	public static final WHEEL_TORQUE:Float = WHEEL_GEARBOX.jointTorque(wheelMotor().maxEffort);
	/**
	 * Drive limits the base is run at: m/s, rad/s, m/s², rad/s². Operating limits, assumed, set under what the
	 * wheels can do (`WHEEL_SPEED` times the wheel radius is the ground speed they top out at).
	 */
	public static inline var MAX_LINEAR_SPEED:Float = 0.8;
	public static inline var MAX_ANGULAR_SPEED:Float = 2.0;
	public static inline var MAX_LINEAR_ACCELERATION:Float = 0.5;
	public static inline var MAX_ANGULAR_ACCELERATION:Float = 1.5;
	/** Turn of the payload seat about its up axis. */
	public static final ARM_TURN:Float = Math.PI / 2;

	public final motor:NemaStepper;
	public final wheel:DriveWheel;
	public final caster:CasterWheel;
	public final bracket:MotorBracket;
	public final post:TubePost;
	public final lidar = new LidarPuck();
	/** The arm on the payload seat, or null for the bare base. */
	public final arm:Null<RobotArm>;

	static final SIDES:Array<{name:String, sign:Int, joint:String}> = [{name: "Left", sign: 1, joint: "wheel_l"}, {name: "Right", sign: -1, joint: "wheel_r"}];
	static final CORNERS:Array<{name:String, x:Int, y:Int}> = [{name: "FrontLeft", x: 1, y: 1}, {name: "FrontRight", x: 1, y: -1},
		{name: "RearLeft", x: -1, y: 1}, {name: "RearRight", x: -1, y: -1}];

	/**
	 * The bare base, or with `arm` standing on the deck's payload seat by its pedestal's `floor`; the arm
	 * then joins the robot as `arm/...`, its joints `arm/j1`..`arm/j6`.
	 */
	public function new(?arm:RobotArm) {
		super();
		this.arm = arm;
		motor = NemaStepper.frame(23);
		wheel = new DriveWheel(WHEEL_DIAMETER, WHEEL_WIDTH, motor.variant.shaftDiameter, 40, 10);
		caster = new CasterWheel(75, 25, BASE_Z, 30, 60);
		var axleZ = wheel.radius;
		bracket = new MotorBracket(motor, 80, axleZ - 40, BASE_Z - axleZ, BRACKET_THICKNESS, 40, 5);
		post = new TubePost(new RectTube(40, 40, 3), DECK_Z - BASE_Z - BASE_THICKNESS);

		addComponent("basePlate", new ChassisPlate(LENGTH, WIDTH, BASE_THICKNESS, "Base plate", baseSeats(),
			wheelSlots()), AssemblyFrames.translation(0, 0, BASE_Z));

		// Drives: the bracket hangs from its seat, the motor sits on the bracket, and the wheel turns on the motor's shaft.
		for (side in SIDES) {
			addComponent('bracket${side.name}', bracket);
			addMate('bracket${side.name}-mount', "fixed", "basePlate", 'bracket${side.name}', 'bracket${side.name}', "top");
			addComponent('motor${side.name}', side.sign > 0 ? motor : NemaStepper.frame(23));
			addMate('motor${side.name}-mount', "fixed", 'bracket${side.name}', "motorSeat", 'motor${side.name}', "mountFace");
			// The hub stands off the shaft's root by the bracket and the gap past it.
			addMemberConnector('motor${side.name}', "wheelSeat", Solids.axial(0, 0, BRACKET_THICKNESS + HUB_GAP));
			addComponent('wheel${side.name}', wheel);
			addMateOnAxis(side.joint, "continuous", 'motor${side.name}', "wheelSeat", 'wheel${side.name}', "bore",
				{x: 0, y: 1, z: 0}, 0, {lower: null, upper: null, velocity: WHEEL_SPEED, effort: WHEEL_TORQUE});
			// The wheel's drive: the stepper through the gearhead, which is where those limits come from.
			var driver = 'driver${side.name}';
			addComponent(driver, wheelDriver());
			addMemberConnector("basePlate", driver, Solids.axial(side.sign * 210, 0, BASE_THICKNESS));
			addMate('$driver-mount', "fixed", "basePlate", driver, driver, "mount");
			addMotor('drive${side.name}', side.joint, 'motor${side.name}', driver, 0.5, WHEEL_GEARBOX);
		}

		for (end in ["Front", "Rear"]) {
			addComponent('caster$end', caster);
			addMate('caster$end-mount', "fixed", "basePlate", 'caster$end', 'caster$end', "mount");
		}
		addComponent("battery", new BatteryPack(260, 180, 110));
		addMate("battery-mount", "fixed", "basePlate", "battery", "battery", "base");
		for (corner in CORNERS) {
			addComponent('post${corner.name}', post);
			addMate('post${corner.name}-mount', "fixed", "basePlate", 'post${corner.name}', 'post${corner.name}', "base");
		}
		// The deck rests on all four posts; one carries it in the mate tree.
		addComponent("deck", new ChassisPlate(LENGTH, WIDTH, DECK_THICKNESS, "Deck", deckSeats()));
		addMate("deck-mount", "fixed", "postFrontLeft", "top", "deck", "postFrontLeft");
		addComponent("lidar", lidar);
		addMate("lidar-mount", "fixed", "deck", "lidar", "lidar", "base");
		exposeConnector("lidar", "lidar", "scan");
		exposeConnector("deckTop", "deck", "payload");
		if (arm != null) {
			include("arm", arm);
			addMate("arm-mount", "fixed", "deck", "payload", "arm/pedestal", "floor");
			// The arm's tool runs on compressed air, which the base passes on as its own service input.
			exposePort("compressedAir", "arm/tool/ejector", "air");
		}
	}

	/** Seats on the base plate: brackets and casters underneath, the battery and posts on top. */
	function baseSeats():Array<PlateSeat> {
		var seats:Array<PlateSeat> = [];
		for (side in SIDES) {
			// The bracket's `top` carries the bracket's own axes: up is +Z, the motor axis points outward.
			var y = side.sign * BRACKET_Y;
			var holeY = y + side.sign * bracket.flangeHoleZ();
			seats.push({name: 'bracket${side.name}', frame: frameFacing(0, y, 0, side.sign),
				holes: [for (x in [-bracket.holeOffset, bracket.holeOffset]) {x: x, y: holeY, diameter: bracket.holeDiameter}]});
		}
		var casterHoles = caster.plateSize / 2 - 8;
		for (end in [{name: "Front", x: CASTER_X}, {name: "Rear", x: -CASTER_X}])
			seats.push({name: 'caster${end.name}', frame: ChassisPlate.underneath(end.x, 0),
				holes: [for (dx in [-1, 1]) for (dy in [-1, 1]) {x: end.x + dx * casterHoles, y: dy * casterHoles, diameter: 6.6}]});
		seats.push({name: "battery", frame: ChassisPlate.onTop(0, 0, BASE_THICKNESS), holes: []});
		for (corner in CORNERS) {
			var x = corner.x * POST_X, y = corner.y * POST_Y;
			seats.push({name: 'post${corner.name}', frame: ChassisPlate.onTop(x, y, BASE_THICKNESS),
				holes: [{x: x, y: y, diameter: 8.4}]});
		}
		return seats;
	}

	/** Seats on the deck: the post tops underneath, the lidar and a payload mount on top. */
	function deckSeats():Array<PlateSeat> {
		var seats:Array<PlateSeat> = [for (corner in CORNERS) {name: 'post${corner.name}',
			frame: ChassisPlate.underneath(corner.x * POST_X, corner.y * POST_Y),
			holes: [{x: corner.x * POST_X, y: corner.y * POST_Y, diameter: 8.4}]}];
		seats.push({name: "lidar", frame: ChassisPlate.onTop(LIDAR_X, 0, DECK_THICKNESS), holes: []});
		// The arm's own cell lies along its -Y; a quarter turn about the seat's up axis brings that ahead.
		seats.push({name: "payload", frame: AssemblyFrames.compose(ChassisPlate.onTop(PAYLOAD_X, 0, DECK_THICKNESS),
			AssemblyFrames.turnY(ARM_TURN)), holes: []});
		return seats;
	}

	/** A slot for each wheel to turn through, a few millimetres clear of its tread. */
	function wheelSlots():Array<{x:Float, y:Float, length:Float, width:Float}> {
		var centreY = BRACKET_Y + BRACKET_THICKNESS + HUB_GAP + wheel.hubLength + WHEEL_WIDTH / 2;
		var length = 2 * Math.sqrt(Math.pow(wheel.radius + 4, 2) - Math.pow(BASE_Z - wheel.radius, 2));
		return [for (side in SIDES) {x: 0, y: side.sign * centreY, length: length, width: WHEEL_WIDTH + 12}];
	}

	/** Frame at (x, y, z) with +Y up and +Z along `sign`·Y: the axes of a bracket whose motor points that way. */
	static function frameFacing(x:Float, y:Float, z:Float, sign:Int):AssemblyFrame
		return AssemblyFrames.fromRotationMatrix(x, y, z, [-sign, 0, 0, 0, 0, sign, 0, 1, 0]);

	/** Distance between the wheels' mid-tread planes, measured from the solved assembly. */
	public function trackWidth():Float {
		var poses = solvedPoses();
		function centreY(id:String):Float
			return AssemblyFrames.compose(poses.get(id), memberConnectorFrame(id, "centre")).y;
		return centreY("wheelLeft") - centreY("wheelRight");
	}

	/** Outline of the chassis on the floor, counter-clockwise, in millimetres from the assembly origin. */
	public static function footprint():Array<{x:Float, y:Float}> {
		var hx = LENGTH / 2, hy = WIDTH / 2;
		return [{x: hx, y: hy}, {x: -hx, y: hy}, {x: -hx, y: -hy}, {x: hx, y: -hy}];
	}
}
