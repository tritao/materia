import cadkit.modeling.Part;
import cadkit.modeling.Vector;
import machinekit.assembly.MachineAssembly;
import machinekit.component.ComponentDetail;
import machinekit.component.Dimension;
import machinekit.component.MachineComponent;
import machinekit.component.Solids;
import machinekit.motion.LeadScrew;
import machinekit.motion.LeadScrewThread;
import machinekit.motion.LeadScrewThread.LeadScrewThreadFamily;
import machinekit.motion.LinearRail;
import machinekit.motion.LinearRailBlock;
import machinekit.motion.NemaStepper;
import machinekit.structural.TSlotExtrusion;
import materia.assembly.AssemblyFrames;
import materia.assembly.AssemblyRecord.AssemblyFrame;

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

	public function new(width:Float, depth:Float, height:Float, material:String, name:String) {
		if (!(width > 0) || !(depth > 0) || !(height > 0)) throw "Plate needs positive dimensions";
		super('${name.toUpperCase()}-${Dimension.format(width)}x${Dimension.format(depth)}x${Dimension.format(height)}',
			name, material, true);
		this.width = width;
		this.depth = depth;
		this.height = height;
	}

	override public function hasGeometry():Bool return true;

	override public function geometry(detail:ComponentDetail = Preview):Part
		return Part.box(width, depth, height);
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

	override public function hasGeometry():Bool return true;

	override public function geometry(detail:ComponentDetail = Preview):Part {
		// The shank continues into the collet; only the visible part is modelled.
		if (detail == Envelope) return Part.cylinderSpan(diameter / 2, 0, stickout);
		return Solids.union([Part.cylinderSpan(diameter / 2, 0, fluteLength),
			Part.cylinderSpan(diameter / 2 - 0.2, fluteLength, stickout)]);
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

/** Travel limit and speed of one linear axis, in millimetres and mm/s. */
typedef RouterAxisSpec = {
	var id:String;
	var lower:Float;
	var upper:Float;
	var velocity:Float;
	/** Peak axis force in newtons. */
	var effort:Float;
	/** Largest axis acceleration in mm/s². */
	var acceleration:Float;
	var initial:Float;
}

/**
 * Desktop three-axis gantry router: a 4040 T-slot frame with a plywood spoilboard, a gantry on
 * MGN12 rails along Y, an X carriage along the gantry beams and a Z slide carrying a 52 mm
 * spindle with a 6 mm end mill. A block of aluminium stock is clamped in the middle of the bed.
 *
 * Joints `x`, `y` and `z` are prismatic and read in machine coordinates, in millimetres: the
 * machine origin is the front-left corner of travel with Z at the top, so the tool tip sits at
 * `toolTipAt(x, y, z)` in the assembly frame (floor at z = 0, bed centred on the origin).
 * Lead screws and motors are placed as fixed parts; the screws do not turn with the axes.
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
	/** Tool tip at machine zero, in the assembly frame. */
	public static inline var MACHINE_ZERO_X:Float = -150;
	public static inline var MACHINE_ZERO_Y:Float = -150;
	public static inline var MACHINE_ZERO_Z:Float = 132;
	/** Offset of the tool axis in front of the gantry's centre plane. */
	public static inline var TOOL_OFFSET_Y:Float = 116;
	public static inline var SCREW_CLEARANCE:Float = 5.5;
	public static inline var Y_SCREW_Z:Float = 30;
	public static inline var X_SCREW_Z:Float = 198;

	public final motorY:NemaStepper;
	public final spindle = new RouterSpindle();
	public final tool = new EndMill(6, 22, 30);
	public final specs:Array<RouterAxisSpec> = [
		{id: "x", lower: 0, upper: 300, velocity: 80, effort: 400, acceleration: 500, initial: 150},
		{id: "y", lower: 0, upper: 300, velocity: 80, effort: 600, acceleration: 400, initial: 150},
		{id: "z", lower: -80, upper: 0, velocity: 40, effort: 400, acceleration: 300, initial: 0}
	];

	/** Room past each axis's travel before its rail blocks reach the rail ends, in millimetres. */
	final overtravel = new Map<String, Float>();

	/** Pose of every member with all axes at zero, used to derive mate connectors. */
	final zeroPoses = new Map<String, AssemblyFrame>();

	public function new() {
		super();
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
			place('motorPlateY$name', new RouterPlate(66, 8, 62, "aluminium 6061", "Motor plate"),
				AssemblyFrames.translation(side * 295, halfFrame + 4, 0));
			// The motor stands behind its plate with the shaft pointing forward through it.
			place('motorY$name', side < 0 ? motorY : NemaStepper.frame(23), orient(side * 295, halfFrame + 8, Y_SCREW_Z, up, [0, -1, 0]));
			place('screwY$name', new LeadScrew(thread, FRAME_LENGTH + 8 - shaft - 10),
				orient(side * 295, halfFrame + 8 - shaft, Y_SCREW_Z, up, [0, -1, 0]));
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
		attach("uprightRight", upright, AssemblyFrames.translation(SIDE_X, yb, uprightTop), "beamUpper");
		attach("blockYRight", LinearRailBlock.metric(RAIL), orient(SIDE_X, yb, railTop, up, alongY), "uprightRight");
		attach("nutBracketYLeft", new YNutBracket(-1), AssemblyFrames.translation(-SIDE_X, yb, 0), "uprightLeft");
		attach("nutBracketYRight", new YNutBracket(1), AssemblyFrames.translation(SIDE_X, yb, 0), "uprightRight");
		var front = [0.0, -1, 0];
		var railX = 2 * (SIDE_X - 20);
		var railXFace = yb - 20 - railSpec.railHeight;
		attach("railXUpper", LinearRail.metric(RAIL, railX), orient(-railX / 2, railXFace, beamZ[0], front, alongX), "beamUpper");
		attach("railXLower", LinearRail.metric(RAIL, railX), orient(-railX / 2, railXFace, beamZ[1], front, alongX), "beamLower");
		var outside = SIDE_X + 6;
		attach("motorX", NemaStepper.frame(23), orient(outside, yb, X_SCREW_Z, up, [-1, 0, 0]), "uprightRight");
		attach("screwX", new LeadScrew(thread, outside - shaft + beamLength / 2 - 4),
			orient(outside - shaft, yb, X_SCREW_Z, up, [-1, 0, 0]), "uprightRight");

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
		attach("nutBracketX", new XNutBracket(), AssemblyFrames.translation(xc, yb, 0), "xPlate");
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
		attach("motorBracketZ", new RouterPlate(70, 49, 8, "aluminium 6061", "Z motor bracket"),
			AssemblyFrames.translation(xc, blockFace - 24.5, xPlateTop), "xPlate");
		attach("motorZ", NemaStepper.frame(23), orient(xc, screwZY, xPlateTop + 8, [0, 1, 0], [0, 0, -1]), "motorBracketZ");
		attach("screwZ", new LeadScrew(thread, xPlateTop + 8 - shaft - (xPlateBottom + 2)),
			orient(xc, screwZY, xPlateTop + 8 - shaft, [0, 1, 0], [0, 0, -1]), "motorBracketZ");

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
		attach("spindle", spindle, AssemblyFrames.translation(xc, toolY, MACHINE_ZERO_Z + tool.stickout), "spindleClamp");
		attach("tool", tool, AssemblyFrames.translation(xc, toolY, MACHINE_ZERO_Z), "spindle");
		exposeConnector("toolTip", "tool", "tip");
	}

	/** Room past the travel of axis `id` before its rail blocks reach the rail ends, in millimetres. */
	public function axisOvertravel(id:String):Float {
		var room = overtravel.get(id);
		if (room == null) throw 'CNC router has no axis "$id"';
		return room;
	}

	/** Assembly-frame position of the tool tip at machine coordinates (x, y, z), in millimetres. */
	public static function toolTipAt(x:Float, y:Float, z:Float):{x:Float, y:Float, z:Float}
		return {x: MACHINE_ZERO_X + x, y: MACHINE_ZERO_Y + y, z: MACHINE_ZERO_Z + z};

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

	/** A member sliding on `parent` along a world axis; `pose` is where it sits at coordinate zero. */
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
			{lower: spec.lower, upper: spec.upper, velocity: spec.velocity, effort: spec.effort, overtravel: room,
				acceleration: spec.acceleration});
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
