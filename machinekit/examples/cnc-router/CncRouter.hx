import machinekit.motion.PowerSupply;
import machinekit.motion.MotorDriver;
import cadkit.modeling.Location;
import cadkit.modeling.Part;
import cadkit.modeling.Plane;
import cadkit.modeling.Vector;
import machinekit.assembly.Transmission;
import machinekit.assembly.AxisBuilder;
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
import machinekit.standard.DeepGrooveBearing;
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

/** Bearing seat at the top of the Z carriage, bored for the screw's 6000 bearing. */
class FoldedZBearingPlate extends MachineComponent {
	public final holeY:Float;
	public function new(holeY:Float) {
		super("FOLDED-Z-BEARING-PLATE", "Folded Z screw bearing plate", "aluminium 6061", true);
		this.holeY = holeY;
	}
	override public function hasGeometry():Bool return true;
	override public function geometry(detail:ComponentDetail = Preview):Part {
		var body = Part.box(70, 49, 8);
		if (detail == Envelope) return body;
		return Solids.cut(body, [Part.cylinderSpan(13, -1, 10).translated(new Vector(0, holeY, 0))]);
	}
}

/** Two short webs lead from the screw bearing to a motor pad forward and to its right. */
class FoldedZMotorPlate extends MachineComponent {
	/** Total X adjustment; its projection onto the screw-to-motor line tensions the loop. */
	public static inline var TENSION_TRAVEL:Float = 5.0;
	public final motor:NemaStepper;
	public final x:Float;
	public final y:Float;
	public function new(motor:NemaStepper, x:Float, y:Float) {
		super("FOLDED-Z-MOTOR-PLATE", "Folded Z motor plate", "aluminium 6061", true);
		this.motor = motor; this.x = x; this.y = y;
	}
	override public function hasGeometry():Bool return true;
	override public function geometry(detail:ComponentDetail = Preview):Part {
		var body = Solids.union([
			Part.box(x + 12, 16, 8).translated(new Vector(x / 2, 20, 0)),
			Part.box(16, -y + 32, 8).translated(new Vector(x, (y + 20) / 2, 0)),
			Part.box(70, 70, 8).translated(new Vector(x, y, 0))]);
		if (detail == Envelope) return body;
		var holes:Array<Part> = [slot(x, y, (motor.spec.pilotDiameter + 0.2) / 2)];
		for (bolt in motor.boltPattern()) holes.push(slot(x + bolt.x, y + bolt.y,
			motor.mountScrew(10).clearanceDiameter(ClearanceFit.Medium) / 2));
		return Solids.cut(body, holes);
	}
	static function slot(x:Float, y:Float, radius:Float):Part
		return Solids.union([
			Part.cylinderSpan(radius, -1, 10).translated(new Vector(x - TENSION_TRAVEL / 2, y, 0)),
			Part.cylinderSpan(radius, -1, 10).translated(new Vector(x + TENSION_TRAVEL / 2, y, 0)),
			Part.box(TENSION_TRAVEL, 2 * radius, 11).translated(new Vector(x, y, -1))]);
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
	@:optional var rackingTolerance:Null<Float>;
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
class CncRouter extends AxisBuilder {
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
	/** A 2:1 shaft-belt Z drive with its motor below the gantry. */
	public final foldedZ:Bool;
	public final spindle = new RouterSpindle();
	/** The tool in the spindle, tool 1. */
	public final tool = new EndMill(6, 22, 30);
	/** Tool 2, loaded by hand when a program calls for it. */
	public final drill = new TwistDrill(5.5, 28, 40);
	public final specs:Array<RouterAxisSpec> = [
		{id: "x", lower: 0, upper: 300, initial: 150},
		{id: "y", lower: 0, upper: 300, initial: 150, rackingTolerance: 0.5},
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

	public function new(belts:Bool = false, foldedZ:Bool = false) {
		super();
		this.belts = belts;
		this.foldedZ = foldedZ;
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
			place('side$name', new ExtrusionMember(profile, FRAME_LENGTH), AxisBuilder.orient(side * SIDE_X, -halfFrame, 20, up, alongY));
			place('railY$name', LinearRail.metric(RAIL, FRAME_LENGTH - 40), AxisBuilder.orient(side * SIDE_X, -halfFrame + 20, railTop, up, alongY));
			if (belts) {
				mountBeltY(side, name, shaft);
				continue;
			}
			// The motor stands behind its plate with the shaft pointing forward through it.
			var motorPose = AxisBuilder.orient(side * 295, halfFrame + 8, Y_SCREW_Z, up, [0, -1, 0]);
			var platePose = AssemblyFrames.translation(side * 295, halfFrame + 4, 0);
			place('motorPlateY$name', new RouterPlate(66, 8, 62, "aluminium 6061", "Motor plate", motorY,
				AssemblyFrames.compose(AssemblyFrames.inverse(platePose), motorPose)), platePose);
			place('motorY$name', side < 0 ? motorY : NemaStepper.frame(23), motorPose);
			driveRouterScrew(specs[1], 'screwY$name', 'motorY$name', new LeadScrew(thread, FRAME_LENGTH + 8 - shaft - 10),
				AxisBuilder.orient(side * 295, halfFrame + 8 - shaft, Y_SCREW_Z, up, [0, -1, 0]), [0, -1, 0], [0, 1, 0]);
		}
		var inner = 2 * (SIDE_X - 20);
		for (entry in [{id: "crossFront", y: -halfFrame + 20}, {id: "crossMiddle", y: 0.0}, {id: "crossBack", y: halfFrame - 20}])
			place(entry.id, new ExtrusionMember(profile, inner), AxisBuilder.orient(-inner / 2, entry.y, 20, up, alongX));
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
		slide(ySpec, "railYLeft", "blockYLeft", LinearRailBlock.metric(RAIL), AxisBuilder.orient(-SIDE_X, yb, railTop, up, alongY), {x: 0, y: 1, z: 0});
		var uprightTop = railTop + railSpec.blockHeight - railSpec.railHeight;
		var upright = new RouterPlate(12, 100, 200, "aluminium 6061", "Gantry upright");
		attach("uprightLeft", upright, AssemblyFrames.translation(-SIDE_X, yb, uprightTop), "blockYLeft");
		var beamLength = 2 * (SIDE_X - 6);
		var beamZ = [233.0, 163];
		attach("beamUpper", new ExtrusionMember(profile, beamLength), AxisBuilder.orient(-beamLength / 2, yb, beamZ[0], up, alongX), "uprightLeft");
		attach("beamLower", new ExtrusionMember(profile, beamLength), AxisBuilder.orient(-beamLength / 2, yb, beamZ[1], up, alongX), "uprightLeft");
		// The right upright carries the X motor, so it has the motor's pilot and bolt holes.
		var outside = SIDE_X + 6;
		var uprightRightPose = AssemblyFrames.translation(SIDE_X, yb, uprightTop);
		var motorXPose = AxisBuilder.orient(outside, yb, X_SCREW_Z, up, [-1, 0, 0]);
		attach("uprightRight", belts ? new RouterPlate(12, 100, 200, "aluminium 6061", "Gantry upright")
			: new RouterPlate(12, 100, 200, "aluminium 6061", "Gantry upright", motorY,
			AssemblyFrames.compose(AssemblyFrames.inverse(uprightRightPose), motorXPose)), uprightRightPose, "beamUpper");
		attach("blockYRight", LinearRailBlock.metric(RAIL), AxisBuilder.orient(SIDE_X, yb, railTop, up, alongY), "uprightRight");
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
		attach("railXUpper", LinearRail.metric(RAIL, railX), AxisBuilder.orient(-railX / 2, railXFace, beamZ[0], front, alongX), "beamUpper");
		attach("railXLower", LinearRail.metric(RAIL, railX), AxisBuilder.orient(-railX / 2, railXFace, beamZ[1], front, alongX), "beamLower");
		if (belts) mountBeltX(yb, shaft);
		else {
			attach("motorX", NemaStepper.frame(23), motorXPose, "uprightRight");
			driveRouterScrew(specs[0], "screwX", "motorX", new LeadScrew(thread, outside - shaft + beamLength / 2 - 4),
				AxisBuilder.orient(outside - shaft, yb, X_SCREW_Z, up, [-1, 0, 0]), [-1, 0, 0], [1, 0, 0]);
		}

		// X carriage, riding the X rails on the beams' front faces.
		var xc = MACHINE_ZERO_X;
		var blockFace = railXFace - (railSpec.blockHeight - railSpec.railHeight);
		slide(specs[0], "railXUpper", "blockXUpper", LinearRailBlock.metric(RAIL), AxisBuilder.orient(xc, railXFace, beamZ[0], front, alongX),
			{x: 1, y: 0, z: 0});
		var plateThickness = 12.0;
		var xPlateBottom = 128.0, xPlateHeight = 140.0;
		attach("xPlate", new RouterPlate(120, plateThickness, xPlateHeight, "aluminium 6061", "X carriage plate"),
			AssemblyFrames.translation(xc, blockFace - plateThickness / 2, xPlateBottom), "blockXUpper");
		attach("blockXLower", LinearRailBlock.metric(RAIL), AxisBuilder.orient(xc, railXFace, beamZ[1], front, alongX), "xPlate");
		if (belts) attach("beltBracketX", new XBeltBracket(BELT_FAR_X(shaft), X_SCREW_Z - beltRadius()), AssemblyFrames.translation(xc, yb, 0), "xPlate");
		else attach("nutBracketX", new XNutBracket(), AssemblyFrames.translation(xc, yb, 0), "xPlate");
		var xPlateFront = blockFace - plateThickness;
		var railZFace = xPlateFront - railSpec.railHeight;
		var railZ = xPlateHeight;
		var vertical = [0.0, 0, 1];
		for (side in [-1, 1]) {
			var name = side < 0 ? "Left" : "Right";
			attach('railZ$name', LinearRail.metric(RAIL, railZ), AxisBuilder.orient(xc + side * 40, railZFace, xPlateBottom, front, vertical), "xPlate");
		}
		var xPlateTop = xPlateBottom + xPlateHeight;
		var screwZY = railZFace + 1;
		// The conventional Z motor stands over its screw. The folded variant puts the same motor
		// below the gantry, on a second plate forward and to the right of the screw bearing.
		var bracketPose = AssemblyFrames.translation(xc, blockFace - 24.5, xPlateTop);
		var bracketTop = xPlateTop + 8;
		var coupling = new ShaftCoupling(motorY.variant.shaftDiameter, thread.screwDiameter);
		var zFace = bracketTop + 1 + coupling.length / 2 + shaft;
		if (foldedZ) foldedZDrive(specs[2], thread, xc, screwZY, xPlateBottom, bracketPose, bracketTop, shaft);
		else {
			var motorZPose = AxisBuilder.orient(xc, screwZY, zFace, [0, 1, 0], [0, 0, -1]);
			attach("motorBracketZ", new RouterPlate(70, 49, 8, "aluminium 6061", "Z motor bracket", motorY,
				AssemblyFrames.compose(AssemblyFrames.inverse(bracketPose), motorZPose)), bracketPose, "xPlate");
			var standoff = new Standoff(8, zFace - bracketTop, motorY.mountScrew(10).clearanceDiameter(ClearanceFit.Medium));
			var corner = 1;
			for (bolt in motorY.boltPattern()) {
				var at = AssemblyFrames.transformPoint(motorZPose, bolt.x, bolt.y, 0);
				attach('standoffZ${corner++}', standoff, AssemblyFrames.translation(at.x, at.y, bracketTop), "motorBracketZ");
			}
			attach("motorZ", NemaStepper.frame(23), motorZPose, "motorBracketZ");
			driveRouterScrew(specs[2], "screwZ", "motorZ", new LeadScrew(thread, zFace - shaft - (xPlateBottom + 2)),
				AxisBuilder.orient(xc, screwZY, zFace - shaft, [0, 1, 0], [0, 0, -1]), [0, 0, -1], [0, 0, 1]);
		}
		// Z slide and spindle. The Z blocks sit at the top of their rails at z = 0.
		var zBlock = xPlateTop - railSpec.railEndMargin - railSpec.blockLength / 2 - 0.5;
		var zBlockFace = railZFace - (railSpec.blockHeight - railSpec.railHeight);
		slide(specs[2], "railZLeft", "blockZLeft", LinearRailBlock.metric(RAIL), AxisBuilder.orient(xc - 40, railZFace, zBlock, front, vertical),
			{x: 0, y: 0, z: 1});
		var zPlateBottom = zBlock - 50;
		attach("zPlate", new RouterPlate(120, plateThickness, 75, "aluminium 6061", "Z plate"),
			AssemblyFrames.translation(xc, zBlockFace - plateThickness / 2, zPlateBottom), "blockZLeft");
		attach("blockZRight", LinearRailBlock.metric(RAIL), AxisBuilder.orient(xc + 40, railZFace, zBlock, front, vertical), "zPlate");
		var toolY = yb - TOOL_OFFSET_Y;
		var reach = (zBlockFace - plateThickness) - toolY;
		attach("spindleClamp", new SpindleClamp(RouterSpindle.DIAMETER, reach),
			AssemblyFrames.translation(xc, toolY, zPlateBottom + 5), "zPlate");
		attach("spindle", spindle, AssemblyFrames.translation(xc, toolY, MACHINE_ZERO_Z), "spindleClamp");
		attach("tool", tool, AssemblyFrames.translation(xc, toolY, MACHINE_ZERO_Z - tool.stickout), "spindle");
		if (!belts) {
			mountNut("screwYLeft", "nutBracketYLeft", AxisBuilder.orient(-295, yb + 20, Y_SCREW_Z, up, [0, -1, 0]));
			mountNut("screwYRight", "nutBracketYRight", AxisBuilder.orient(295, yb + 20, Y_SCREW_Z, up, [0, -1, 0]));
			mountNut("screwX", "nutBracketX", AxisBuilder.orient(xc + 20, yb, X_SCREW_Z, up, [-1, 0, 0]));
		}
		mountNut("screwZ", "zPlate", AxisBuilder.orient(xc, screwZY, zPlateBottom + 30, [0, 1, 0], [0, 0, 1]));
		if (belts) {
			attachBeltPath("beltX", "beltBracketX", "pulleyX", "idlerX");
			for (name in ["Left", "Right"]) attachBeltPath("beltY" + name, "beltBracketY" + name, "pulleyY" + name, "idlerY" + name);
		}
		// Independent Y contacts support squaring; Z homes upward before lateral travel.
		buildHome(specs[1], "YLeft", "sideLeft", "uprightLeft", [-SIDE_X - 18.0, yb, 70.0],
			[40.0, 6.0, 6.0], [0.0, 1, 0], -1, (belts ? "pulley" : "screw") + "YLeft-turn");
		buildHome(specs[1], "YRight", "sideRight", "uprightRight", [SIDE_X + 18.0, yb, 70.0],
			[40.0, 6.0, 6.0], [0.0, 1, 0], -1, (belts ? "pulley" : "screw") + "YRight-turn");
		buildHome(specs[0], "X", "beamUpper", "xPlate", [xc, blockFace - 30.0, 250.0],
			[6.0, 40.0, 6.0], [1.0, 0, 0], -1, (belts ? "pulleyX-turn" : "screwX-turn"));
		buildHome(specs[2], "Z", "xPlate", "zPlate", [xc + 75.0, zBlockFace, zPlateBottom + 35.0],
			[40.0, 6.0, 6.0], [0.0, 0, 1], 1, foldedZ ? "motorZ-turn" : "screwZ-turn");
		exposeConnector("nose", "spindle", "nose");
		exposeConnector("toolTip", "tool", "tip");
	}

	/** Physical outboard steel target and supported proximity sensor at a guide's home end. */
	function buildHome(axis:RouterAxisSpec, suffix:String, fixed:String, moving:String,
			point:Array<Float>, dimensions:Array<Float>, direction:Array<Float>, side:Int, shaft:String):Void {
		var room = axisOvertravel(axis.id);
		var sensor = new machinekit.motion.ProximitySwitch();
		if (!Math.isFinite(room) || room <= 4 * sensor.switchRepeatability())
			throw "Router home uncertainty does not fit its physical guide overtravel";
		var trigger = "homeTrigger" + suffix;
		// Each target's trip connector follows its axis; distinct definitions retain those frames.
		attach(trigger, new RouterPlate(dimensions[0], dimensions[1], dimensions[2], "steel", "Home trigger " + suffix),
			AssemblyFrames.translation(point[0], point[1], point[2] - dimensions[2] / 2), moving);
		var half = 0.0;
		for (i in 0...3) half += Math.abs(direction[i]) * dimensions[i] / 2;
		addMemberConnector(trigger, "trip", AssemblyFrames.translation(direction[0] * side * half,
			direction[1] * side * half, dimensions[2] / 2 + direction[2] * side * half));
		var travel = (side < 0 ? axis.lower : axis.upper) + side * room * 0.25;
		var normal = [for (value in direction) -side * value];
		var face = [for (i in 0...3) point[i] + direction[i] * (travel + side * half)];
		var origin = [for (i in 0...3) face[i] - normal[i] * (sensor.spec.length + sensor.spec.sensingDistance)];
		var transverse = suffix == "X" ? [0.0, 1, 0] : [1.0, 0, 0];
		var pose = AxisBuilder.orient(origin[0], origin[1], origin[2], transverse, normal);
		var host = switchMountBounds(fixed);
		var anchor = [for (i in 0...3) Math.max(host.min[i], Math.min(host.max[i], origin[i]))];
		var lo = [for (i in 0...3) Math.min(anchor[i], origin[i]) - 3];
		var hi = [for (i in 0...3) Math.max(anchor[i], origin[i]) + 3];
		var id = "home" + suffix, mount = id + "Mount";
		attach(mount, new RouterPlate(hi[0] - lo[0], hi[1] - lo[1], hi[2] - lo[2], "aluminium 6061", "Home switch mount"),
			AssemblyFrames.translation((lo[0] + hi[0]) / 2, (lo[1] + hi[1]) / 2, lo[2]), fixed);
		attach(id, sensor, pose, mount);
		addTripSwitch(id, axis.id, id, {instanceId: trigger, connectorName: "trip"}, side, "home",
			suffix == "YRight" ? 2 : 1, shaft);
	}

	function switchMountBounds(id:String):{min:Array<Float>, max:Array<Float>} {
		var member = component(id), min:Array<Float>, max:Array<Float>;
		if (Std.isOfType(member, ExtrusionMember)) {
			var frame:ExtrusionMember = cast member;
			min = [-frame.profile.size / 2, -frame.profile.height / 2, 0.0];
			max = [frame.profile.size / 2, frame.profile.height / 2, frame.length];
		} else if (Std.isOfType(member, RouterPlate)) {
			var plate:RouterPlate = cast member;
			min = [-plate.width / 2, -plate.depth / 2, 0.0]; max = [plate.width / 2, plate.depth / 2, plate.height];
		} else throw "Router switch mount requires a frame or plate";
		var lo = [Math.POSITIVE_INFINITY, Math.POSITIVE_INFINITY, Math.POSITIVE_INFINITY];
		var hi = [Math.NEGATIVE_INFINITY, Math.NEGATIVE_INFINITY, Math.NEGATIVE_INFINITY];
		for (x in [min[0], max[0]]) for (y in [min[1], max[1]]) for (z in [min[2], max[2]]) {
			var point = AssemblyFrames.transformPoint(zeroPose(id), x, y, z), values = [point.x, point.y, point.z];
			for (i in 0...3) { lo[i] = Math.min(lo[i], values[i]); hi[i] = Math.max(hi[i], values[i]); }
		}
		return {min: lo, max: hi};
	}

	/** The Z screw and motor are two distinct shafts joined by one pretensioned belt loop. */
	function foldedZDrive(axis:RouterAxisSpec, thread:LeadScrewThread, xc:Float, screwY:Float,
			xPlateBottom:Float, bracketPose:AssemblyFrame, bracketTop:Float, shaft:Float):Void {
		var motor = NemaStepper.frame(23);
		var driverTeeth = 40, motorTeeth = 20;
		var first = TimingBelt.twoPulley(GT2, driverTeeth, motorTeeth, Math.sqrt(70 * 70 + 70 * 70), BELT_WIDTH);
		var target = first.toothLength(), low = 96.0, high = 102.0;
		for (_ in 0...40) {
			var middle = (low + high) / 2;
			if (TimingBelt.twoPulley(GT2, driverTeeth, motorTeeth, middle, BELT_WIDTH).length < target) low = middle;
			else high = middle;
		}
		var centre = (low + high) / 2, offsetX = 70.0, offsetY = -Math.sqrt(centre * centre - offsetX * offsetX);
		var loop = TimingBelt.twoPulley(GT2, driverTeeth, motorTeeth, centre, BELT_WIDTH);
		var bearing = DeepGrooveBearing.metric("6000");
		attach("motorBracketZ", new FoldedZBearingPlate(screwY - bracketPose.y), bracketPose, "xPlate");
		attach("bearingZ", bearing, AssemblyFrames.translation(xc, screwY, bracketPose.z), "motorBracketZ");
		attach("foldedMotorPlateZ", new FoldedZMotorPlate(motor, offsetX, offsetY),
			AssemblyFrames.translation(xc, screwY, bracketTop), "motorBracketZ");
		var screwInput = bracketTop + shaft + 8;
		var beltHeight = screwInput - 11;
		var motorPose = AxisBuilder.orient(xc + offsetX, screwY + offsetY, bracketTop, [0, 1, 0], [0, 0, 1]);
		attach("motorZ", motor, motorPose, "foldedMotorPlateZ");
		var screwPose = AxisBuilder.orient(xc, screwY, screwInput, [0, 1, 0], [0, 0, -1]);
		var screw = new LeadScrew(thread, screwInput - (xPlateBottom + 2));
		addComponent("screwZ", screw);
		zeroPoses.set("screwZ", screwPose);
		connect("motorBracketZ", "screwZ");
		addComponent("screwZNut", new LeadScrewNut(thread, 4, false));
		var lead = addTransmission("screwZ-lead", axis.id, "screwZ-turn", Transmission.LeadScrew("screwZ", "screwZNut"), SenseTools.fromAlignment(-1));
		addMateOnAxis("screwZ-turn", "continuous", "motorBracketZ", "to-screwZ", "screwZ", "attach-screwZ",
			{x: 0, y: 0, z: -1}, lead * axis.initial);
		supportScrew("screwZ-lead", Fixed, Free);
		var screwPulleyPose = AxisBuilder.orient(xc, screwY, beltHeight + 3, [0, 1, 0], [0, 0, -1]);
		attach("pulleyScrewZ", new TimingPulley(GT2, driverTeeth, thread.screwDiameter, BELT_WIDTH), screwPulleyPose, "screwZ");
		var motorPulleyPose = AxisBuilder.orient(xc + offsetX, screwY + offsetY, beltHeight - 3, [0, 1, 0], [0, 0, 1]);
		addComponent("pulleyMotorZ", new TimingPulley(GT2, motorTeeth, motor.variant.shaftDiameter, BELT_WIDTH));
		zeroPoses.set("pulleyMotorZ", motorPulleyPose);
		connect("motorZ", "pulleyMotorZ");
		addMateOnAxis("motorZ-turn", "continuous", "motorZ", "to-pulleyMotorZ", "pulleyMotorZ", "attach-pulleyMotorZ",
			{x: 0, y: 0, z: 1});
		var cosine = offsetX / centre, sine = offsetY / centre;
		var beltPose = AssemblyFrames.fromRotationMatrix(xc, screwY, beltHeight,
			[cosine, -sine, 0, sine, cosine, 0, 0, 0, 1]);
		attach("beltZ", loop, beltPose, "motorBracketZ");
		addBeltPath({belt: "beltZ", wraps: [{instanceId: "pulleyScrewZ", connectorName: "axis"},
			{instanceId: "pulleyMotorZ", connectorName: "axis"}]});
		addTransmission("screwZ-belt", "screwZ-turn", "motorZ-turn",
			Transmission.BeltReduction("beltZ", "pulleyScrewZ", "pulleyMotorZ"), SenseTools.fromAlignment(1));
		addMotor("motorZ", "motorZ-turn", "motorZ", addDriver("motorZ"));
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

	var driverIndex:Int = 0;
	var supplyAdded:Bool = false;
	/** Drivers live on the fixed frame, so their envelopes do not add carriage mass. */
	function addDriver(motor:String):String {
		var id = motor + "Driver";
		if (!supplyAdded) {
			attach("powerSupply", new PowerSupply(SUPPLY_VOLTS, 20, 4), AssemblyFrames.translation(0, -320, 0), "sideLeft");
			supplyAdded = true;
		}
		attach(id, new MotorDriver("GENERIC-DM542", 2.8, MICROSTEPS),
			AssemblyFrames.translation(-180 + 125 * driverIndex++, -200, 0), "sideLeft");
		connectPorts('$id-power', "powerSupply", 'power$driverIndex', id, "power");
		return id;
	}

	/** Router nut shape and frame-mounted electrical driver. */
	function driveRouterScrew(axis:RouterAxisSpec, id:String, motor:String, screw:LeadScrew, pose:AssemblyFrame,
			along:Array<Float>, axisDirection:Array<Float>):Void
		driveScrew(axis, id, motor, screw, pose, along, axisDirection, axis.id != "z", addDriver);

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
	function attachBeltPath(beltId:String, clampId:String, driver:String, idler:String):Void {
		var belt:TimingBelt = cast component(beltId);
		var run = belt.strands()[0];
		var world = AssemblyFrames.transformPoint(zeroPoses.get(beltId), (run.startX + run.endX) / 2, (run.startY + run.endY) / 2, 0);
		if (beltId == "beltX") world.x -= specs[0].initial;
		else world.y -= specs[1].initial;
		addMemberConnector(clampId, "beltClamp", AssemblyFrames.compose(AssemblyFrames.inverse(zeroPoses.get(clampId)), AssemblyFrames.translation(world.x, world.y, world.z)));
		addBeltPath({belt: beltId, clamp: {instanceId: clampId, connectorName: "beltClamp"},
			wraps: [{instanceId: driver, connectorName: "attach-" + driver}, {instanceId: idler, connectorName: "attach-" + idler}]});
	}

	/** Mount the router's X motor and idler plates; the builder supplies the belt drive. */
	function mountBeltX(yb:Float, shaft:Float):Void {
		var up = [0.0, 0, 1];
		var xm = X_BELT_END, tip = yb + BELT_PLATE + 20 - shaft;
		var plateY = yb + 20 + BELT_PLATE / 2, plateZ = X_SCREW_Z - 32;
		var motorPose = AxisBuilder.orient(xm, yb + 20 + BELT_PLATE, X_SCREW_Z, up, [0, -1, 0]);
		var platePose = AssemblyFrames.translation(xm, plateY, plateZ);
		attach("motorPlateX", new RouterPlate(60, BELT_PLATE, 64, "aluminium 6061", "Motor plate", motorY,
			AssemblyFrames.compose(AssemblyFrames.inverse(platePose), motorPose)), platePose, "beamUpper");
		attach("motorX", NemaStepper.frame(23), motorPose, "motorPlateX");
		var belt = TimingBelt.twoPulley(GT2, BELT_TEETH, BELT_TEETH, 2 * xm, BELT_WIDTH);
		var plane = AxisBuilder.orient(xm, tip, X_SCREW_Z, up, [0, 1, 0]);
		twoPulleyAxis({axis: specs[0], beltId: "beltX", belt: belt, beltPose: plane, beltParent: "beamUpper",
			motorId: "motorX", pulleyId: "pulleyX", pulley: beltPulley(), idlerId: "idlerX", idler: beltPulley(),
			idlerPose: AxisBuilder.orient(-xm, tip, X_SCREW_Z, up, [0, 1, 0]), about: [0.0, 1, 0],
			strand: 0, travelX: -1.0, travelY: 0.0, driverForMotor: addDriver,
			mountIdler: () -> {
				var plate = AssemblyFrames.translation(-xm, plateY, plateZ);
				attach("idlerPlateX", new RouterPlate(60, BELT_PLATE, 64, "aluminium 6061", "Idler plate"), plate, "beamUpper");
				attach("axleX", new BeltAxle(motorY.variant.shaftDiameter, shaft - BELT_PLATE),
					AxisBuilder.orient(-xm, yb + 20, X_SCREW_Z, up, [0, -1, 0]), "idlerPlateX");
				return "axleX";
			}});
	}

	/** Mount one side's router Y motor and idler plates. */
	function mountBeltY(side:Int, name:String, shaft:Float):Void {
		var up = [0.0, 0, 1], sign:Float = side;
		var inner = Y_BELT_X - BELT_WIDTH / 2;
		var faceX = sign * (inner + shaft), plateX = sign * (inner + shaft - BELT_PLATE / 2);
		var end = Y_BELT_END;
		var motorPose = AxisBuilder.orient(faceX, end, Y_SCREW_Z, up, [-sign, 0, 0]);
		var platePose = AssemblyFrames.translation(plateX, end, 0);
		place('motorPlateY$name', new RouterPlate(BELT_PLATE, 60, 60, "aluminium 6061", "Motor plate", motorY,
			AssemblyFrames.compose(AssemblyFrames.inverse(platePose), motorPose)), platePose);
		place('motorY$name', side < 0 ? motorY : NemaStepper.frame(23), motorPose);
		var belt = TimingBelt.twoPulley(GT2, BELT_TEETH, BELT_TEETH, 2 * end, BELT_WIDTH);
		var plane = AxisBuilder.orient(sign * Y_BELT_X + BELT_WIDTH / 2, end, Y_SCREW_Z, up, [-1, 0, 0]);
		twoPulleyAxis({axis: specs[1], beltId: 'beltY$name', belt: belt, beltPose: plane, beltParent: null,
			motorId: 'motorY$name', pulleyId: 'pulleyY$name', pulley: beltPulley(), idlerId: 'idlerY$name', idler: beltPulley(),
			idlerPose: AxisBuilder.orient(sign * Y_BELT_X + BELT_WIDTH / 2, -end, Y_SCREW_Z, up, [-1, 0, 0]),
			about: [-1.0, 0, 0], strand: 0, travelX: -1.0, travelY: 0.0, driverForMotor: addDriver,
			mountIdler: () -> {
				var plate = AssemblyFrames.translation(plateX, -end, 0);
				place('idlerPlateY$name', new RouterPlate(BELT_PLATE, 60, 60, "aluminium 6061", "Idler plate"), plate);
				attach('axleY$name', new BeltAxle(motorY.variant.shaftDiameter, shaft - BELT_PLATE),
					AxisBuilder.orient(sign * (inner + shaft - BELT_PLATE), -end, Y_SCREW_Z, up, [-sign, 0, 0]), 'idlerPlateY$name');
				return 'axleY$name';
			}});
	}

}
