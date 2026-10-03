import cadkit.modeling.Location;
import cadkit.modeling.Part;
import cadkit.modeling.Plane;
import cadkit.modeling.Vector;
import machinekit.assembly.Transmission;
import machinekit.assembly.MachineAssembly;
import machinekit.component.ComponentDetail;
import machinekit.component.Dimension;
import machinekit.component.MachineComponent;
import machinekit.component.Solids;
import machinekit.motion.LeadScrew;
import machinekit.motion.LeadScrewNut;
import machinekit.assembly.Transmission;
import machinekit.assembly.Sense.SenseTools;
import machinekit.motion.LeadScrewThread;
import machinekit.motion.LeadScrewThread.LeadScrewThreadFamily;
import machinekit.motion.ScrewSupport;
import machinekit.motion.LinearRail;
import machinekit.motion.LinearRailBlock;
import machinekit.motion.NemaStepper;
import machinekit.motion.ShaftCoupling;
import machinekit.standard.ClearanceFit;
import machinekit.transmission.TimingBelt;
import machinekit.transmission.TimingBeltProfile;
import machinekit.transmission.TimingPulley;
import machinekit.structural.TSlotExtrusion;
import materia.assembly.AssemblyFrames;
import materia.assembly.AssemblyRecord.AssemblyFrame;
import toolpathkit.tool.CutterProfile;
import toolpathkit.tool.Tool;

/** A cut length of T-slot extrusion, running along local +Z from z=0 with its section centred. */
class ExtrusionMember extends MachineComponent {
	public final profile:TSlotExtrusion;
	public final length:Float;

	public function new(profile:TSlotExtrusion, length:Float) {
		if (!(length > 0)) throw "Extrusion member needs a positive length";
		super('${profile.designation}-L${Dimension.format(length)}',
			'${profile.description}, ${Dimension.format(length)} mm long', "aluminium 6061", true);
		this.profile = profile;
		this.length = length;
	}

	override public function hasGeometry():Bool return true;

	override public function geometry(detail:ComponentDetail = Preview):Part {
		if (detail == Envelope) return Part.box(profile.size, profile.height, length);
		return profile.geometry(length);
	}
}

/** A rectangular plate or block standing on its base (z=0), centred on its origin. */
class RouterPlate extends MachineComponent {
	public final width:Float;
	public final depth:Float;
	public final height:Float;
	/** The motor this plate mounts, or null: its pilot and bolt holes go through the plate. */
	public final motor:Null<NemaStepper>;
	/** The motor's face in the plate's frame, its shaft pointing into the plate. */
	public final motorFace:Null<AssemblyFrame>;

	public function new(width:Float, depth:Float, height:Float, material:String, name:String, ?motor:NemaStepper,
			?motorFace:AssemblyFrame) {
		if (!(width > 0) || !(depth > 0) || !(height > 0)) throw "Plate needs positive dimensions";
		if ((motor == null) != (motorFace == null)) throw "A motor mount needs both the motor and where its face sits";
		super('${name.toUpperCase()}-${Dimension.format(width)}x${Dimension.format(depth)}x${Dimension.format(height)}' +
			(motor == null ? "" : '-NEMA${motor.spec.frame}'), name, material, true);
		this.width = width;
		this.depth = depth;
		this.height = height;
		this.motor = motor;
		this.motorFace = motorFace;
	}

	override public function hasGeometry():Bool return true;

	override public function geometry(detail:ComponentDetail = Preview):Part {
		var box = Part.box(width, depth, height);
		var mounted = motor, face = motorFace;
		if (mounted == null || face == null || detail == Envelope) return box;
		var cutout = mounted.mountingCutout(width + depth + height);
		var x = AssemblyFrames.transformVector(face, 1, 0, 0), z = AssemblyFrames.transformVector(face, 0, 0, 1);
		var placed = cutout.placed(new Location(new Plane(new Vector(face.x, face.y, face.z), new Vector(x.x, x.y, x.z),
			new Vector(z.x, z.y, z.z))));
		cutout.close();
		return Solids.cut(box, [placed]);
	}
}

/**
 * L-shaped bracket that hangs from a gantry upright, outside the frame, and carries the Y lead
 * nut. Origin: below the upright's centre on the floor; `side` is +1 for the right (+X) side and
 * -1 for the left. The screw passes along Y through the bore at x = side·35, z = 30.
 */
class YNutBracket extends MachineComponent {
	public final side:Int;

	public function new(side:Int) {
		if (side != 1 && side != -1) throw "Nut bracket side must be 1 or -1";
		super('YNUT-BRACKET-${side > 0 ? "R" : "L"}', 'Y lead-nut bracket, ${side > 0 ? "right" : "left"}',
			"aluminium 6061", true);
		this.side = side;
	}

	override public function hasGeometry():Bool return true;

	override public function geometry(detail:ComponentDetail = Preview):Part {
		var tab = Part.box(54, 40, 10).translated(new Vector(side * 33, 0, 53));
		var leg = Part.box(38, 40, 48).translated(new Vector(side * 41, 0, 15));
		var body = Solids.union([tab, leg]);
		if (detail == Envelope) return body;
		return Solids.cut(body, [Part.cylinderAlongY(CncRouter.SCREW_CLEARANCE, -25, 25, side * 35, CncRouter.Y_SCREW_Z)]);
	}
}

/** Block on the back of the X carriage plate that carries the X lead nut between the gantry beams. */
class XNutBracket extends MachineComponent {
	public function new() {
		super("XNUT-BRACKET", "X lead-nut bracket", "aluminium 6061", true);
	}

	override public function hasGeometry():Bool return true;

	override public function geometry(detail:ComponentDetail = Preview):Part {
		// From the carriage plate's back face (y = -33) to past the screw (y = 0), inside the beam gap.
		var body = Part.box(40, 45, 20).translated(new Vector(0, -10.5, CncRouter.X_SCREW_Z - 10));
		if (detail == Envelope) return body;
		return Solids.cut(body, [Part.cylinderAlong(CncRouter.SCREW_CLEARANCE, new Vector(-25, 0, CncRouter.X_SCREW_Z),
			new Vector(1, 0, 0), 50)]);
	}
}

/**
 * Block on the back of the X carriage plate that clamps the lower strand of the X belt between the
 * gantry beams. The strand runs through it at height `strandZ`; it ends `far` behind the gantry's
 * centre plane, past the belt's far edge. The upper strand passes above it.
 */
class XBeltBracket extends MachineComponent {
	public final far:Float;
	public final strandZ:Float;

	public function new(far:Float, strandZ:Float) {
		super('XBELT-BRACKET-F${Dimension.format(far)}-Z${Dimension.format(strandZ)}', "X belt clamp bracket", "aluminium 6061", true);
		this.far = far;
		this.strandZ = strandZ;
	}

	override public function hasGeometry():Bool return true;

	override public function geometry(detail:ComponentDetail = Preview):Part
		// From the carriage plate's back face (y = -33) to past the belt, 13 mm tall about the strand.
		return Part.box(40, far + 33, 13).translated(new Vector(0, (far - 33) / 2, strandZ - 6));
}

/**
 * Bracket on a gantry upright like `YNutBracket`, but clamping the lower strand of the Y belt on its
 * side: the strand runs through the leg, and a slot lets the upper strand, at height `upperZ`,
 * pass. The belt runs along Y at x = side·35 from the upright.
 */
class YBeltBracket extends MachineComponent {
	public final side:Int;
	public final upperZ:Float;

	public function new(side:Int, upperZ:Float) {
		if (side != 1 && side != -1) throw "Belt bracket side must be 1 or -1";
		super('YBELT-BRACKET-${side > 0 ? "R" : "L"}-Z${Dimension.format(upperZ)}', 'Y belt bracket, ${side > 0 ? "right" : "left"}',
			"aluminium 6061", true);
		this.side = side;
		this.upperZ = upperZ;
	}

	override public function hasGeometry():Bool return true;

	override public function geometry(detail:ComponentDetail = Preview):Part {
		var tab = Part.box(54, 40, 10).translated(new Vector(side * 33, 0, 53));
		var leg = Part.box(38, 40, 48).translated(new Vector(side * 41, 0, 15));
		var body = Solids.union([tab, leg]);
		if (detail == Envelope) return body;
		return Solids.cut(body, [Part.box(10, 60, 5).translated(new Vector(side * 35, 0, upperZ - 2.5))]);
	}
}

/** Pin an idler pulley turns on. Origin: its base, with the pin along +Z. */
class BeltAxle extends MachineComponent {
	public final diameter:Float;
	public final length:Float;

	public function new(diameter:Float, length:Float) {
		if (!(diameter > 0) || !(length > 0)) throw "Belt axle needs a positive diameter and length";
		super('AXLE-D${Dimension.format(diameter)}-L${Dimension.format(length)}', 'Idler axle, ${Dimension.format(diameter)} mm', "steel", true);
		this.diameter = diameter;
		this.length = length;
	}

	override public function hasGeometry():Bool return true;

	override public function geometry(detail:ComponentDetail = Preview):Part
		return Part.cylinderSpan(diameter / 2, 0, length);
}

/** Block clamping the spindle to the front of the Z plate. Origin on the spindle axis at the clamp's base. */
class SpindleClamp extends MachineComponent {
	public final spindleDiameter:Float;
	/** Distance from the spindle axis back to the Z plate's front face. */
	public final reach:Float;

	public function new(spindleDiameter:Float, reach:Float) {
		super('SPINDLE-CLAMP-D${Dimension.format(spindleDiameter)}', 'Spindle clamp for a ${Dimension.format(spindleDiameter)} mm spindle',
			"aluminium 6061", true);
		this.spindleDiameter = spindleDiameter;
		this.reach = reach;
	}

	override public function hasGeometry():Bool return true;

	override public function geometry(detail:ComponentDetail = Preview):Part {
		var wall = 10.0, front = spindleDiameter / 2 + wall;
		var body = Part.box(spindleDiameter + 2 * wall + 8, reach + front, 50)
			.translated(new Vector(0, (reach - front) / 2, 0));
		if (detail == Envelope) return body;
		return Solids.cut(body, [Part.cylinderSpan(spindleDiameter / 2, -1, 51)]);
	}
}

/**
 * Air-cooled router spindle with an ER11 collet nut. Origin at the bottom of the collet nut on the
 * spindle axis, which runs along +Z; the `nose` connector is there and points down the tool.
 */
class RouterSpindle extends MachineComponent {
	public static inline var DIAMETER:Float = 52;
	public static inline var NUT_LENGTH:Float = 20;
	public static inline var BODY_LENGTH:Float = 120;

	public function new() {
		super("SPINDLE-D52-ER11", "Router spindle, 52 mm, ER11 collet", "steel", true);
		addConnector("nose", Mount, Solids.axial(0, 0, 0));
	}

	override public function hasGeometry():Bool return true;

	/** `cutter` in this spindle: the collet nut above the nose rides with the tool, in metres. */
	public function holding(cutter:CutterProfile):CutterProfile
		return cutter.withHolder(0.018, 0.008).withHolder(0.024, (NUT_LENGTH - 8) / 1000);

	override public function geometry(detail:ComponentDetail = Preview):Part {
		var body = Part.cylinderSpan(DIAMETER / 2, NUT_LENGTH, NUT_LENGTH + BODY_LENGTH);
		if (detail == Envelope) return body;
		return Solids.union([
			Part.cylinderSpan(9, 0, 8), Part.cylinderSpan(12, 8, NUT_LENGTH), body,
			Part.cylinderSpan(DIAMETER / 2 - 6, NUT_LENGTH + BODY_LENGTH, NUT_LENGTH + BODY_LENGTH + 10)
		]);
	}
}

/** Flat end mill. Origin at the tip on the tool axis, which runs up along +Z; `tip` is there. */
class EndMill extends MachineComponent {
	public final diameter:Float;
	public final fluteLength:Float;
	/** Length out of the collet. */
	public final stickout:Float;

	public function new(diameter:Float, fluteLength:Float, stickout:Float) {
		if (!(diameter > 0) || !(fluteLength > 0) || !(stickout >= fluteLength))
			throw "End mill needs a positive diameter and flutes within its stickout";
		super('ENDMILL-D${Dimension.format(diameter)}-F${Dimension.format(fluteLength)}-L${Dimension.format(stickout)}',
			'Flat end mill, ${Dimension.format(diameter)} mm', "steel", true);
		this.diameter = diameter;
		this.fluteLength = fluteLength;
		this.stickout = stickout;
		addConnector("tip", Mount, Solids.axial(0, 0, 0));
	}

	/** The cutter as the stock simulation sees it, in metres: flutes, then the relieved shank. */
	public function cutter():CutterProfile
		return CutterProfile.flat(diameter / 1000, fluteLength / 1000)
			.withShank((diameter - 0.4) / 1000, (stickout - fluteLength) / 1000);

	override public function hasGeometry():Bool return true;

	override public function geometry(detail:ComponentDetail = Preview):Part {
		// The shank continues into the collet; only the visible part is modelled.
		if (detail == Envelope) return Part.cylinderSpan(diameter / 2, 0, stickout);
		return Solids.union([Part.cylinderSpan(diameter / 2, 0, fluteLength),
			Part.cylinderSpan(diameter / 2 - 0.2, fluteLength, stickout)]);
	}
}

/** Twist drill with a 118° point. Origin at the point on the tool axis, which runs up along +Z. */
class TwistDrill extends MachineComponent {
	public static inline var POINT_ANGLE:Float = 118;
	public final diameter:Float;
	public final fluteLength:Float;
	/** Length out of the collet. */
	public final stickout:Float;

	public function new(diameter:Float, fluteLength:Float, stickout:Float) {
		if (!(diameter > 0) || !(fluteLength > pointHeight(diameter)) || !(stickout >= fluteLength))
			throw "Twist drill needs a positive diameter and flutes past its point, within its stickout";
		super('DRILL-D${Dimension.format(diameter)}-F${Dimension.format(fluteLength)}-L${Dimension.format(stickout)}',
			'Twist drill, ${Dimension.format(diameter)} mm', "steel", true);
		this.diameter = diameter;
		this.fluteLength = fluteLength;
		this.stickout = stickout;
		addConnector("tip", Mount, Solids.axial(0, 0, 0));
	}

	/** Height of the conical point. */
	public static function pointHeight(diameter:Float):Float
		return diameter / 2 / Math.tan(POINT_ANGLE / 2 * Math.PI / 180);

	/** The cutter as the stock simulation sees it, in metres: point and flutes, then the shank. */
	public function cutter():CutterProfile
		return CutterProfile.vee(diameter / 1000, POINT_ANGLE * Math.PI / 180, fluteLength / 1000)
			.withShank(diameter / 1000, (stickout - fluteLength) / 1000);

	override public function hasGeometry():Bool return true;

	override public function geometry(detail:ComponentDetail = Preview):Part {
		if (detail == Envelope) return Part.cylinderSpan(diameter / 2, 0, stickout);
		var r = diameter / 2;
		return Part.revolve([{r: 0, z: 0}, {r: r, z: pointHeight(diameter)}, {r: r, z: stickout}, {r: 0, z: stickout}]);
	}
}

/** Round spacer standing a motor off its mount, bored for the screw that holds it. Origin: one end, along +Z. */
class Standoff extends MachineComponent {
	public final diameter:Float;
	public final length:Float;
	public final bore:Float;

	public function new(diameter:Float, length:Float, bore:Float) {
		if (!(bore > 0) || !(diameter > bore) || !(length > 0)) throw "Standoff needs a bore inside a positive diameter and length";
		super('STANDOFF-D${Dimension.format(diameter)}-L${Dimension.format(length)}', 'Standoff ${Dimension.format(diameter)} x ${Dimension.format(length)} mm',
			"aluminium 6061", true);
		this.diameter = diameter;
		this.length = length;
		this.bore = bore;
	}

	override public function hasGeometry():Bool return true;

	override public function geometry(detail:ComponentDetail = Preview):Part {
		var body = Part.cylinderSpan(diameter / 2, 0, length);
		if (detail == Envelope) return body;
		return Solids.cut(body, [Part.cylinderSpan(bore / 2, -0.1, length + 0.1)]);
	}
}

/**
 * Step clamp holding the stock's edge to the spoilboard. Origin on the spoilboard under the stock
 * edge; the clamp reaches 8 mm over the stock along -X and stands on a riser outboard along +X.
 */
class ToeClamp extends MachineComponent {
	public final stockHeight:Float;

	public function new(stockHeight:Float) {
		super('TOE-CLAMP-H${Dimension.format(stockHeight)}', "Step clamp", "steel", true);
		this.stockHeight = stockHeight;
	}

	override public function hasGeometry():Bool return true;

	override public function geometry(detail:ComponentDetail = Preview):Part {
		var bar = Part.box(50, 25, 10).translated(new Vector(17, 0, stockHeight));
		var riser = Part.box(12, 25, stockHeight).translated(new Vector(36, 0, 0));
		if (detail == Envelope) return Solids.union([bar, riser]);
		return Solids.union([bar, riser, Part.cylinderSpan(4, 0, stockHeight + 18, 17, 0),
			Part.cylinderSpan(7, stockHeight + 10, stockHeight + 16, 17, 0)]);
	}
}

/**
 * Travel of one linear axis, in millimetres. Its speed and acceleration are its motors' and
 * screws', worked out from the parts.
 */
typedef RouterAxisSpec = {
	var id:String;
	var lower:Float;
	var upper:Float;
	var initial:Float;
}

/**
 * Desktop three-axis gantry router: a 4040 T-slot frame with a plywood spoilboard, a gantry on
 * MGN12 rails along Y, an X carriage along the gantry beams and a Z slide carrying a 52 mm
 * spindle with a 6 mm end mill. A block of aluminium stock is clamped in the middle of the bed.
 *
 * Joints `x`, `y` and `z` are prismatic and read in machine coordinates, in millimetres: the
 * machine origin is the front-left corner of travel with Z at the top, so the spindle nose sits at
 * `noseAt(x, y, z)` in the assembly frame (floor at z = 0, bed centred on the origin).
 * Each motor turns its lead screw through a shaft coupling, on a continuous joint coupled to its
 * axis by the screw's lead: Y has two, one on each side of the gantry. The motors are the screw
 * joints' actuators, so each axis is as fast as its motors turn its screws, and accelerates as
 * hard as their torque, through the screws' efficiency, moves its mass and turns its screws.
 *
 * With `belts`, X and Y run on GT2 belts instead: a 20-tooth pulley on each motor and an idler of the
 * same size at the far end, Y with one belt per side, each clamped on one strand to the carriage.
 * The pulleys turn on continuous joints coupled to their axes by their pitch radius, so the same
 * motors give the speed and acceleration a belt does. Z keeps its screw.
 */
class CncRouter extends MachineAssembly {
	public static inline var PROFILE:String = "HFS5-4040";
	public static inline var RAIL:String = "MGN12C";
	/** Frame: side members along Y at x = ±SIDE_X, cross members along X; 40 mm tall, on the floor. */
	public static inline var SIDE_X:Float = 260;
	public static inline var FRAME_LENGTH:Float = 640;
	public static inline var SPOILBOARD_TOP:Float = 58;
	/** Stock block, centred on the bed. */
	public static inline var STOCK_WIDTH:Float = 120;
	public static inline var STOCK_DEPTH:Float = 90;
	public static inline var STOCK_HEIGHT:Float = 20;
	public static final STOCK_TOP:Float = SPOILBOARD_TOP + STOCK_HEIGHT;
	/**
	 * The spindle nose at machine zero, in the assembly frame. Machine coordinates name where the
	 * nose (the gauge line) is; each tool hangs its own length below it, applied by G43.
	 */
	public static inline var MACHINE_ZERO_X:Float = -150;
	public static inline var MACHINE_ZERO_Y:Float = -150;
	public static inline var MACHINE_ZERO_Z:Float = 162;
	/** Offset of the tool axis in front of the gantry's centre plane. */
	public static inline var TOOL_OFFSET_Y:Float = 116;
	public static inline var SCREW_CLEARANCE:Float = 5.5;
	public static inline var Y_SCREW_Z:Float = 30;
	public static inline var X_SCREW_Z:Float = 198;
	/** Belt drive: teeth on every pulley, belt width, and motor plate thickness. */
	public static inline var BELT_TEETH:Int = 20;
	public static inline var BELT_WIDTH:Float = 6;
	static inline var BELT_PLATE:Float = 8;
	/** Y belts run in the vertical plane x = ±Y_BELT_X, pulley centres ±Y_BELT_END from the middle in y. */
	public static inline var Y_BELT_X:Float = 295;
	public static inline var Y_BELT_END:Float = 320;
	/** The X belt runs in the gantry's gap, pulley centres ±X_BELT_END from the middle in x. */
	public static inline var X_BELT_END:Float = 222;

	public final motorY:NemaStepper;
	/** True when X and Y run on belts instead of lead screws. */
	public final belts:Bool;
	public final spindle = new RouterSpindle();
	/** The tool in the spindle, tool 1. */
	public final tool = new EndMill(6, 22, 30);
	/** Tool 2, loaded by hand when a program calls for it. */
	public final drill = new TwistDrill(5.5, 28, 40);
	public final specs:Array<RouterAxisSpec> = [
		{id: "x", lower: 0, upper: 300, initial: 150},
		{id: "y", lower: 0, upper: 300, initial: 150},
		{id: "z", lower: -80, upper: 0, initial: 0}
	];
	/** The stepper drivers' supply, as on most desktop routers. */
	public static inline var SUPPLY_VOLTS:Float = 24;
	/**
	 * Nominal wiring of the stepper drivers: 16 microsteps per full step, the usual setting of
	 * desktop drivers, on a controller generating 40 kHz step edges (the RKD6 board's software step
	 * tick, `robotkit/runtime/DEVICE_PROTOCOL.md`). Together they cap each axis: a NEMA 23 at 16
	 * microsteps is 3200 steps a turn, so 40 kHz turns it 78.5 rad/s, 12.5 turns a second.
	 */
	public static inline var MICROSTEPS:Int = 16;
	public static inline var STEP_TICK_HZ:Int = 40000;

	/** Room past each axis's travel before its rail blocks reach the rail ends, in millimetres. */
	final overtravel = new Map<String, Float>();

	/** Pose of every member with all axes at zero, used to derive mate connectors. */
	final zeroPoses = new Map<String, AssemblyFrame>();

	public function new(belts:Bool = false) {
		super();
		this.belts = belts;
		motorY = NemaStepper.frame(23);
		var shaft = motorY.variant.shaftLength;
		var profile = TSlotExtrusion.forProfile(PROFILE);
		var railSpec = LinearRailBlock.metric(RAIL).spec;
		var railTop = 40 + railSpec.railHeight;
		var thread = new LeadScrewThread(MetricTrapezoidal, 10, 2);

		// Fixed frame and bed.
		var up = [0.0, 0, 1];
		var alongX = [1.0, 0, 0], alongY = [0.0, 1, 0];
		var halfFrame = FRAME_LENGTH / 2;
		for (side in [-1, 1]) {
			var name = side < 0 ? "Left" : "Right";
			place('side$name', new ExtrusionMember(profile, FRAME_LENGTH), orient(side * SIDE_X, -halfFrame, 20, up, alongY));
			place('railY$name', LinearRail.metric(RAIL, FRAME_LENGTH - 40), orient(side * SIDE_X, -halfFrame + 20, railTop, up, alongY));
			if (belts) {
				beltY(side, name, shaft);
				continue;
			}
			// The motor stands behind its plate with the shaft pointing forward through it.
			var motorPose = orient(side * 295, halfFrame + 8, Y_SCREW_Z, up, [0, -1, 0]);
			var platePose = AssemblyFrames.translation(side * 295, halfFrame + 4, 0);
			place('motorPlateY$name', new RouterPlate(66, 8, 62, "aluminium 6061", "Motor plate", motorY,
				AssemblyFrames.compose(AssemblyFrames.inverse(platePose), motorPose)), platePose);
			place('motorY$name', side < 0 ? motorY : NemaStepper.frame(23), motorPose);
			driveScrew(specs[1], 'screwY$name', 'motorY$name', new LeadScrew(thread, FRAME_LENGTH + 8 - shaft - 10),
				orient(side * 295, halfFrame + 8 - shaft, Y_SCREW_Z, up, [0, -1, 0]), [0, -1, 0], [0, 1, 0]);
		}
		var inner = 2 * (SIDE_X - 20);
		for (entry in [{id: "crossFront", y: -halfFrame + 20}, {id: "crossMiddle", y: 0.0}, {id: "crossBack", y: halfFrame - 20}])
			place(entry.id, new ExtrusionMember(profile, inner), orient(-inner / 2, entry.y, 20, up, alongX));
		place("spoilboard", new RouterPlate(inner - 10, FRAME_LENGTH - 40, SPOILBOARD_TOP - 40, "birch plywood", "Spoilboard"),
			AssemblyFrames.translation(0, 0, 40));
		place("stock", new RouterPlate(STOCK_WIDTH, STOCK_DEPTH, STOCK_HEIGHT, "aluminium 6061", "Stock"),
			AssemblyFrames.translation(0, 0, SPOILBOARD_TOP));
		var clamp = new ToeClamp(STOCK_HEIGHT);
		place("clampRight", clamp, AssemblyFrames.translation(STOCK_WIDTH / 2, 0, SPOILBOARD_TOP));
		place("clampLeft", clamp, {x: -STOCK_WIDTH / 2, y: 0, z: SPOILBOARD_TOP, qx: 0, qy: 0, qz: 1, qw: 0});

		// Gantry, riding the Y rails. yb is the gantry's centre plane at y = 0.
		var yb = MACHINE_ZERO_Y + TOOL_OFFSET_Y;
		var ySpec = specs[1];
		slide(ySpec, "railYLeft", "blockYLeft", LinearRailBlock.metric(RAIL), orient(-SIDE_X, yb, railTop, up, alongY), {x: 0, y: 1, z: 0});
		var uprightTop = railTop + railSpec.blockHeight - railSpec.railHeight;
		var upright = new RouterPlate(12, 100, 200, "aluminium 6061", "Gantry upright");
		attach("uprightLeft", upright, AssemblyFrames.translation(-SIDE_X, yb, uprightTop), "blockYLeft");
		var beamLength = 2 * (SIDE_X - 6);
		var beamZ = [233.0, 163];
		attach("beamUpper", new ExtrusionMember(profile, beamLength), orient(-beamLength / 2, yb, beamZ[0], up, alongX), "uprightLeft");
		attach("beamLower", new ExtrusionMember(profile, beamLength), orient(-beamLength / 2, yb, beamZ[1], up, alongX), "uprightLeft");
		// The right upright carries the X motor, so it has the motor's pilot and bolt holes.
		var outside = SIDE_X + 6;
		var uprightRightPose = AssemblyFrames.translation(SIDE_X, yb, uprightTop);
		var motorXPose = orient(outside, yb, X_SCREW_Z, up, [-1, 0, 0]);
		attach("uprightRight", belts ? new RouterPlate(12, 100, 200, "aluminium 6061", "Gantry upright")
			: new RouterPlate(12, 100, 200, "aluminium 6061", "Gantry upright", motorY,
			AssemblyFrames.compose(AssemblyFrames.inverse(uprightRightPose), motorXPose)), uprightRightPose, "beamUpper");
		attach("blockYRight", LinearRailBlock.metric(RAIL), orient(SIDE_X, yb, railTop, up, alongY), "uprightRight");
		if (belts) {
			for (side in [-1, 1])
				attach('beltBracketY${side < 0 ? "Left" : "Right"}', new YBeltBracket(side, Y_SCREW_Z + beltRadius()),
					AssemblyFrames.translation(side * SIDE_X, yb, 0), side < 0 ? "uprightLeft" : "uprightRight");
		} else {
			attach("nutBracketYLeft", new YNutBracket(-1), AssemblyFrames.translation(-SIDE_X, yb, 0), "uprightLeft");
			attach("nutBracketYRight", new YNutBracket(1), AssemblyFrames.translation(SIDE_X, yb, 0), "uprightRight");
		}
		var front = [0.0, -1, 0];
		var railX = 2 * (SIDE_X - 20);
		var railXFace = yb - 20 - railSpec.railHeight;
		attach("railXUpper", LinearRail.metric(RAIL, railX), orient(-railX / 2, railXFace, beamZ[0], front, alongX), "beamUpper");
		attach("railXLower", LinearRail.metric(RAIL, railX), orient(-railX / 2, railXFace, beamZ[1], front, alongX), "beamLower");
		if (belts) beltX(yb, shaft);
		else {
			attach("motorX", NemaStepper.frame(23), motorXPose, "uprightRight");
			driveScrew(specs[0], "screwX", "motorX", new LeadScrew(thread, outside - shaft + beamLength / 2 - 4),
				orient(outside - shaft, yb, X_SCREW_Z, up, [-1, 0, 0]), [-1, 0, 0], [1, 0, 0]);
		}

		// X carriage, riding the X rails on the beams' front faces.
		var xc = MACHINE_ZERO_X;
		var blockFace = railXFace - (railSpec.blockHeight - railSpec.railHeight);
		slide(specs[0], "railXUpper", "blockXUpper", LinearRailBlock.metric(RAIL), orient(xc, railXFace, beamZ[0], front, alongX),
			{x: 1, y: 0, z: 0});
		var plateThickness = 12.0;
		var xPlateBottom = 128.0, xPlateHeight = 140.0;
		attach("xPlate", new RouterPlate(120, plateThickness, xPlateHeight, "aluminium 6061", "X carriage plate"),
			AssemblyFrames.translation(xc, blockFace - plateThickness / 2, xPlateBottom), "blockXUpper");
		attach("blockXLower", LinearRailBlock.metric(RAIL), orient(xc, railXFace, beamZ[1], front, alongX), "xPlate");
		if (belts) attach("beltBracketX", new XBeltBracket(BELT_FAR_X(shaft), X_SCREW_Z - beltRadius()), AssemblyFrames.translation(xc, yb, 0), "xPlate");
		else attach("nutBracketX", new XNutBracket(), AssemblyFrames.translation(xc, yb, 0), "xPlate");
		var xPlateFront = blockFace - plateThickness;
		var railZFace = xPlateFront - railSpec.railHeight;
		var railZ = xPlateHeight;
		var vertical = [0.0, 0, 1];
		for (side in [-1, 1]) {
			var name = side < 0 ? "Left" : "Right";
			attach('railZ$name', LinearRail.metric(RAIL, railZ), orient(xc + side * 40, railZFace, xPlateBottom, front, vertical), "xPlate");
		}
		var xPlateTop = xPlateBottom + xPlateHeight;
		var screwZY = railZFace + 1;
		// The Z screw runs in the narrow gap between the X and Z plates, too narrow for its coupling, so
		// the motor stands on four spacers above its bracket and the coupling turns between them.
		var bracketPose = AssemblyFrames.translation(xc, blockFace - 24.5, xPlateTop);
		var bracketTop = xPlateTop + 8;
		var coupling = new ShaftCoupling(motorY.variant.shaftDiameter, thread.screwDiameter);
		var zFace = bracketTop + 1 + coupling.length / 2 + shaft;
		var motorZPose = orient(xc, screwZY, zFace, [0, 1, 0], [0, 0, -1]);
		attach("motorBracketZ", new RouterPlate(70, 49, 8, "aluminium 6061", "Z motor bracket", motorY,
			AssemblyFrames.compose(AssemblyFrames.inverse(bracketPose), motorZPose)), bracketPose, "xPlate");
		var standoff = new Standoff(8, zFace - bracketTop, motorY.mountScrew(10).clearanceDiameter(ClearanceFit.Medium));
		var corner = 1;
		for (bolt in motorY.boltPattern()) {
			var at = AssemblyFrames.transformPoint(motorZPose, bolt.x, bolt.y, 0);
			attach('standoffZ${corner++}', standoff, AssemblyFrames.translation(at.x, at.y, bracketTop), "motorBracketZ");
		}
		attach("motorZ", NemaStepper.frame(23), motorZPose, "motorBracketZ");
		driveScrew(specs[2], "screwZ", "motorZ", new LeadScrew(thread, zFace - shaft - (xPlateBottom + 2)),
			orient(xc, screwZY, zFace - shaft, [0, 1, 0], [0, 0, -1]), [0, 0, -1], [0, 0, 1]);

		// Z slide and spindle. The Z blocks sit at the top of their rails at z = 0.
		var zBlock = xPlateTop - railSpec.railEndMargin - railSpec.blockLength / 2 - 0.5;
		var zBlockFace = railZFace - (railSpec.blockHeight - railSpec.railHeight);
		slide(specs[2], "railZLeft", "blockZLeft", LinearRailBlock.metric(RAIL), orient(xc - 40, railZFace, zBlock, front, vertical),
			{x: 0, y: 0, z: 1});
		var zPlateBottom = zBlock - 50;
		attach("zPlate", new RouterPlate(120, plateThickness, 75, "aluminium 6061", "Z plate"),
			AssemblyFrames.translation(xc, zBlockFace - plateThickness / 2, zPlateBottom), "blockZLeft");
		attach("blockZRight", LinearRailBlock.metric(RAIL), orient(xc + 40, railZFace, zBlock, front, vertical), "zPlate");
		var toolY = yb - TOOL_OFFSET_Y;
		var reach = (zBlockFace - plateThickness) - toolY;
		attach("spindleClamp", new SpindleClamp(RouterSpindle.DIAMETER, reach),
			AssemblyFrames.translation(xc, toolY, zPlateBottom + 5), "zPlate");
		attach("spindle", spindle, AssemblyFrames.translation(xc, toolY, MACHINE_ZERO_Z), "spindleClamp");
		attach("tool", tool, AssemblyFrames.translation(xc, toolY, MACHINE_ZERO_Z - tool.stickout), "spindle");
		if (!belts) {
			mountNut("screwYLeft", "nutBracketYLeft", orient(-295, yb + 20, Y_SCREW_Z, up, [0, -1, 0]));
			mountNut("screwYRight", "nutBracketYRight", orient(295, yb + 20, Y_SCREW_Z, up, [0, -1, 0]));
			mountNut("screwX", "nutBracketX", orient(xc + 20, yb, X_SCREW_Z, up, [-1, 0, 0]));
		}
		mountNut("screwZ", "zPlate", orient(xc, screwZY, zPlateBottom + 30, [0, 1, 0], [0, 0, 1]));
		exposeConnector("nose", "spindle", "nose");
		exposeConnector("toolTip", "tool", "tip");
	}

	/** Room past the travel of axis `id` before its rail blocks reach the rail ends, in millimetres. */
	public function axisOvertravel(id:String):Float {
		var room = overtravel.get(id);
		if (room == null) throw 'CNC router has no axis "$id"';
		return room;
	}

	/** Assembly-frame position of the spindle nose at machine coordinates (x, y, z), in millimetres. */
	public static function noseAt(x:Float, y:Float, z:Float):{x:Float, y:Float, z:Float}
		return {x: MACHINE_ZERO_X + x, y: MACHINE_ZERO_Y + y, z: MACHINE_ZERO_Z + z};

	/**
	 * The tool table, numbered as programs call the tools: each tool's length below the nose and
	 * shape in the spindle, in metres.
	 */
	public function tools():Array<Tool>
		return [Tool.shaped(1, tool.stickout / 1000, spindle.holding(tool.cutter())),
			Tool.shaped(2, drill.stickout / 1000, spindle.holding(drill.cutter()))];

	/** Work zero (G54) in machine coordinates, in millimetres: the stock's front-left top corner. */
	public static function workOffset():Array<Float>
		return [-STOCK_WIDTH / 2 - MACHINE_ZERO_X, -STOCK_DEPTH / 2 - MACHINE_ZERO_Y, STOCK_TOP - MACHINE_ZERO_Z];

	/** Frame whose local +Y points along `up` and local +Z along `along`. */
	static function orient(x:Float, y:Float, z:Float, up:Array<Float>, along:Array<Float>):AssemblyFrame {
		var xx = up[1] * along[2] - up[2] * along[1];
		var xy = up[2] * along[0] - up[0] * along[2];
		var xz = up[0] * along[1] - up[1] * along[0];
		return AssemblyFrames.fromRotationMatrix(x, y, z, [xx, up[0], along[0], xy, up[1], along[1], xz, up[2], along[2]]);
	}

	/** A fixed root member at its world pose. */
	function place(id:String, component:MachineComponent, pose:AssemblyFrame):Void {
		addComponent(id, component, pose);
		zeroPoses.set(id, pose);
	}

	/** A member fixed to `parent`, at its world pose with every axis at zero. */
	function attach(id:String, component:MachineComponent, pose:AssemblyFrame, parent:String):Void {
		addComponent(id, component);
		zeroPoses.set(id, pose);
		connect(parent, id);
		addMate('$id-mount', "fixed", parent, 'to-$id', id, 'attach-$id');
	}

	/**
	 * Motor `motor` turns lead screw `id` (at `pose`, its input end on the shaft tip, pointing
	 * along world direction `along`) through a shaft coupling. Coupling and screw turn together on
	 * a continuous joint `id-turn`, which coupling `id-lead` ties to `axis` (moving along world
	 * `axisDirection`) by the screw's lead. The coupling is centred on the shaft tip and turns
	 * inside the mount's pilot bore.
	 */
	function driveScrew(axis:RouterAxisSpec, id:String, motor:String, screw:LeadScrew, pose:AssemblyFrame,
			along:Array<Float>, axisDirection:Array<Float>):Void {
		var shaft = cast(component(motor), NemaStepper).variant.shaftDiameter;
		var coupling = new ShaftCoupling(shaft, screw.thread.screwDiameter);
		var grip = coupling.length / 2;
		var couplingId = id + "Coupling";
		addComponent(couplingId, coupling);
		zeroPoses.set(couplingId, {x: pose.x - along[0] * grip, y: pose.y - along[1] * grip, z: pose.z - along[2] * grip,
			qx: pose.qx, qy: pose.qy, qz: pose.qz, qw: pose.qw});
		connect(motor, couplingId);
		attach(id, screw, pose, couplingId);
		// The screw's thread sets the ratio; the joint starts where the axis puts it.
		var alongAxis = along[0] * axisDirection[0] + along[1] * axisDirection[1] + along[2] * axisDirection[2];
		addComponent(id + "Nut", new LeadScrewNut(screw.thread, 4, axis.id != "z"));
		var ratio = addTransmission('$id-lead', axis.id, '$id-turn', Transmission.LeadScrew(id, id + "Nut"),
			SenseTools.fromAlignment(alongAxis));
		addMateOnAxis('$id-turn', "continuous", motor, 'to-$couplingId', couplingId, 'attach-$couplingId',
			{x: along[0], y: along[1], z: along[2]}, ratio * axis.initial);
		// The motor holds the screw's input end through the coupling. Nothing holds the far end, and the
		// nut floats on the carriage, so it is no support: fixed at the motor, free at the far end, over
		// the whole screw. That is what sets the screw's top speed.
		supportScrew('$id-lead', Fixed, Free);
		addMotor(motor, '$id-turn', motor, SUPPLY_VOLTS);
	}

	/** Attach the source nut to its carriage once the bracket exists. */
	function mountNut(screw:String, parent:String, face:AssemblyFrame):Void {
		var id = screw + "Nut";
		var nut:LeadScrewNut = cast component(id);
		zeroPoses.set(id, AssemblyFrames.compose(face, AssemblyFrames.translation(0, 0, -nut.bodyLength - nut.flangeThickness)));
		connect(parent, id);
		addMate('$id-mount', "fixed", parent, 'to-$id', id, 'attach-$id');
	}

	/** Pitch radius of the 20-tooth GT2 belt pulleys, in millimetres. */
	static function beltRadius():Float
		return TimingPulley.profileDimensions(GT2).pitch * BELT_TEETH / (2 * Math.PI);

	/** Where the X belt's far edge lies behind the gantry's centre plane, with a millimetre of margin. */
	static function BELT_FAR_X(shaft:Float):Float
		return BELT_PLATE + 20 - shaft + BELT_WIDTH + 1;

	/** A GT2 pulley of the router's size, bored for the motor shaft. */
	function beltPulley():TimingPulley
		return new TimingPulley(GT2, BELT_TEETH, motorY.variant.shaftDiameter, BELT_WIDTH);

	/**
	 * Pulley member `id`, already added at `pose` with its origin on its axis, turns on a continuous
	 * joint `id-turn` about world direction `about` (the belt plane's normal, so the belt's own
	 * rotation sign applies), coupled to `axis` through its belt. `rotation` is +1 when it turns
	 * counter-clockwise about `about` as the carriage moves positively.
	 */
	function turnWithBelt(axis:RouterAxisSpec, id:String, parent:String, about:Array<Float>, rotation:Int,
			beltId:String):Void {
		var ratio = addTransmission('$id-belt', axis.id, '$id-turn', Transmission.TimingBelt(beltId, id, 0),
			SenseTools.fromAlignment(rotation));
		addMateOnAxis('$id-turn', "continuous", parent, 'to-$id', id, 'attach-$id', {x: about[0], y: about[1], z: about[2]},
			ratio * axis.initial);
	}

	/** Adds `component` at `pose` as a child of `parent` without a mate: the caller adds the joint. */
	function hang(id:String, component:MachineComponent, pose:AssemblyFrame, parent:String):Void {
		addComponent(id, component);
		zeroPoses.set(id, pose);
		connect(parent, id);
	}

	/**
	 * X on a belt. The motor sits behind the gantry beams on a plate bolted to their back, its shaft
	 * pointing forward into the gap between them, where its pulley takes the belt; the idler turns on
	 * an axle in a second plate at the other end. The belt's lower strand (strand 0) is clamped to the
	 * carriage's bracket.
	 */
	function beltX(yb:Float, shaft:Float):Void {
		var up = [0.0, 0, 1];
		var xm = X_BELT_END, tip = yb + BELT_PLATE + 20 - shaft;
		var plateY = yb + 20 + BELT_PLATE / 2, plateZ = X_SCREW_Z - 32;
		var motorPose = orient(xm, yb + 20 + BELT_PLATE, X_SCREW_Z, up, [0, -1, 0]);
		var platePose = AssemblyFrames.translation(xm, plateY, plateZ);
		attach("motorPlateX", new RouterPlate(60, BELT_PLATE, 64, "aluminium 6061", "Motor plate", motorY,
			AssemblyFrames.compose(AssemblyFrames.inverse(platePose), motorPose)), platePose, "beamUpper");
		attach("motorX", NemaStepper.frame(23), motorPose, "motorPlateX");
		var belt = TimingBelt.twoPulley(GT2, BELT_TEETH, BELT_TEETH, 2 * xm, BELT_WIDTH);
		var plane = orient(xm, tip, X_SCREW_Z, up, [0, 1, 0]);
		attach("beltX", belt, plane, "beamUpper");
		var turn = belt.rotation(0, 0, -1, 0);
		hang("pulleyX", beltPulley(), plane, "motorX");
		turnWithBelt(specs[0], "pulleyX", "motorX", [0, 1, 0], turn, "beltX");
		var idlerPlate = AssemblyFrames.translation(-xm, plateY, plateZ);
		attach("idlerPlateX", new RouterPlate(60, BELT_PLATE, 64, "aluminium 6061", "Idler plate"), idlerPlate, "beamUpper");
		attach("axleX", new BeltAxle(motorY.variant.shaftDiameter, shaft - BELT_PLATE), orient(-xm, yb + 20, X_SCREW_Z, up, [0, -1, 0]),
			"idlerPlateX");
		hang("idlerX", beltPulley(), orient(-xm, tip, X_SCREW_Z, up, [0, 1, 0]), "axleX");
		turnWithBelt(specs[0], "idlerX", "axleX", [0, 1, 0], belt.rotation(1, 0, -1, 0), "beltX");
		addMotor("motorX", "pulleyX-turn", "motorX", SUPPLY_VOLTS);
	}

	/**
	 * Y on a belt, one per side, in the vertical plane outside the frame. The motor stands on a plate
	 * behind the frame with its shaft pointing inward through it to the pulley; the idler turns on an
	 * axle at the front. The lower strand (strand 0) is clamped to the gantry's bracket.
	 */
	function beltY(side:Int, name:String, shaft:Float):Void {
		var up = [0.0, 0, 1], s:Float = side;
		var inner = Y_BELT_X - BELT_WIDTH / 2;
		var faceX = s * (inner + shaft), plateX = s * (inner + shaft - BELT_PLATE / 2);
		var end = Y_BELT_END;
		var motorPose = orient(faceX, end, Y_SCREW_Z, up, [-s, 0, 0]);
		var platePose = AssemblyFrames.translation(plateX, end, 0);
		place('motorPlateY$name', new RouterPlate(BELT_PLATE, 60, 60, "aluminium 6061", "Motor plate", motorY,
			AssemblyFrames.compose(AssemblyFrames.inverse(platePose), motorPose)), platePose);
		place('motorY$name', side < 0 ? motorY : NemaStepper.frame(23), motorPose);
		var belt = TimingBelt.twoPulley(GT2, BELT_TEETH, BELT_TEETH, 2 * end, BELT_WIDTH);
		var plane = orient(s * Y_BELT_X + BELT_WIDTH / 2, end, Y_SCREW_Z, up, [-1, 0, 0]);
		place('beltY$name', belt, plane);
		hang('pulleyY$name', beltPulley(), plane, 'motorY$name');
		turnWithBelt(specs[1], 'pulleyY$name', 'motorY$name', [-1, 0, 0], belt.rotation(0, 0, -1, 0), 'beltY$name');
		var idlerPlate = AssemblyFrames.translation(plateX, -end, 0);
		place('idlerPlateY$name', new RouterPlate(BELT_PLATE, 60, 60, "aluminium 6061", "Idler plate"), idlerPlate);
		attach('axleY$name', new BeltAxle(motorY.variant.shaftDiameter, shaft - BELT_PLATE),
			orient(s * (inner + shaft - BELT_PLATE), -end, Y_SCREW_Z, up, [-s, 0, 0]), 'idlerPlateY$name');
		hang('idlerY$name', beltPulley(), orient(s * Y_BELT_X + BELT_WIDTH / 2, -end, Y_SCREW_Z, up, [-1, 0, 0]), 'axleY$name');
		turnWithBelt(specs[1], 'idlerY$name', 'axleY$name', [-1, 0, 0], belt.rotation(1, 0, -1, 0), 'beltY$name');
		addMotor('motorY$name', 'pulleyY$name-turn', 'motorY$name', SUPPLY_VOLTS);
	}

	function component(id:String):MachineComponent {
		for (entry in components()) if (entry.id == id) return entry.component;
		throw 'CNC router has no member "$id" yet';
	}

	/**
	 * A rail block sliding on its rail (`parent`) along a world axis; `pose` is where it sits at
	 * coordinate zero. The axis's overtravel is the room its block has left on the rail at either end
	 * of travel, where the rail's end stops are.
	 */
	function slide(spec:RouterAxisSpec, parent:String, id:String, component:MachineComponent, pose:AssemblyFrame,
			axis:{x:Float, y:Float, z:Float}):Void {
		var rail = [for (entry in components()) if (entry.id == parent) entry.component][0];
		if (!Std.isOfType(rail, LinearRail)) throw 'Axis ${spec.id} must slide on a rail';
		var guide:LinearRail = cast rail;
		var railFrame = AssemblyFrames.inverse(zeroPose(parent));
		function alongRail(coordinate:Float):Float
			return AssemblyFrames.transformPoint(railFrame, pose.x + axis.x * coordinate, pose.y + axis.y * coordinate,
				pose.z + axis.z * coordinate).z;
		var reach = guide.spec.railEndMargin + guide.spec.blockLength / 2;
		var first = alongRail(spec.lower), last = alongRail(spec.upper);
		var room = Math.min(Math.min(first, last) - reach, guide.length - reach - Math.max(first, last));
		if (room < 0) throw 'Axis ${spec.id} runs its block off its rail by ${Dimension.format(-room)} mm';
		overtravel.set(spec.id, room);
		addComponent(id, component);
		zeroPoses.set(id, pose);
		connect(parent, id);
		addMateOnAxis(spec.id, "prismatic", parent, 'to-$id', id, 'attach-$id', axis, spec.initial,
			{lower: spec.lower, upper: spec.upper, velocity: null, effort: null, overtravel: room});
	}

	/**
	 * Connectors meeting at the child's origin with world-aligned axes, so joint axes are world
	 * directions. Names carry the member ids, so members that share geometry can share one
	 * definition holding all of their connectors.
	 */
	function connect(parent:String, child:String):Void {
		var childPose = zeroPose(child);
		var meeting = AssemblyFrames.translation(childPose.x, childPose.y, childPose.z);
		addMemberConnector(parent, 'to-$child', AssemblyFrames.compose(AssemblyFrames.inverse(zeroPose(parent)), meeting));
		addMemberConnector(child, 'attach-$child', AssemblyFrames.compose(AssemblyFrames.inverse(childPose), meeting));
	}

	function zeroPose(id:String):AssemblyFrame {
		var pose = zeroPoses.get(id);
		if (pose == null) throw 'CNC router has no member "$id" yet';
		return pose;
	}
}
