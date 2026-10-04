import machinekit.motion.PowerSupply;
import machinekit.motion.MotorDriver;
import cadkit.modeling.Part;
import machinekit.assembly.Transmission;
import machinekit.assembly.Sense.SenseTools;
import machinekit.assembly.MachineAssembly;
import machinekit.component.ComponentDetail;
import machinekit.component.ConnectorRole;
import machinekit.component.Dimension;
import machinekit.component.MachineComponent;
import machinekit.component.Solids;
import machinekit.motion.LinearRail;
import machinekit.motion.LinearRailBlock;
import machinekit.motion.NemaStepper;
import machinekit.transmission.TimingBelt;
import machinekit.transmission.TimingBelt.BeltWrap;
import machinekit.transmission.TimingBeltProfile;
import machinekit.transmission.TimingPulley;
import materia.assembly.AssemblyFrames;
import materia.assembly.AssemblyRecord.AssemblyFrame;

/** A rectangular plate or block standing on its base (z=0), centred on its origin. */
class PlotterBlock extends MachineComponent {
	public final width:Float;
	public final depth:Float;
	public final height:Float;

	public function new(width:Float, depth:Float, height:Float, material:String, name:String) {
		if (!(width > 0) || !(depth > 0) || !(height > 0)) throw "Plotter block needs positive dimensions";
		super('${name.toUpperCase()}-${Dimension.format(width)}x${Dimension.format(depth)}x${Dimension.format(height)}', name, material, true);
		this.width = width;
		this.depth = depth;
		this.height = height;
	}

	override public function hasGeometry():Bool return true;

	override public function geometry(detail:ComponentDetail = Preview):Part
		return Part.box(width, depth, height);
}

/** A round pin standing on its base (z=0): an idler's axle, or the pen, whose `tip` is the pin's far end. */
class PlotterPin extends MachineComponent {
	public final diameter:Float;
	public final length:Float;

	public function new(diameter:Float, length:Float, name:String, material:String = "steel") {
		if (!(diameter > 0) || !(length > 0)) throw "Plotter pin needs a positive diameter and length";
		super('${name.toUpperCase().split(" ").join("-")}-D${Dimension.format(diameter)}-L${Dimension.format(length)}', name, material, true);
		this.diameter = diameter;
		this.length = length;
		addConnector("tip", ConnectorRole.Axis, Solids.axial(0, 0, 0));
	}

	override public function hasGeometry():Bool return true;

	override public function geometry(detail:ComponentDetail = Preview):Part
		return Part.cylinderSpan(diameter / 2, 0, length);
}

/** One axis's range of travel, in millimetres, and where it starts. */
typedef PlotterAxisSpec = {id:String, lower:Float, upper:Float, initial:Float};

/**
 * A small CoreXY pen plotter: the head moves along X on a gantry that moves along Y, and both
 * motors sit on the frame. Joints `x` and `y` are prismatic and read in millimetres from the middle of
 * the bed, so the pen is over the bed's centre at (0, 0).
 *
 * Two belts, each one loop of five pulleys, close through the carriage, which clamps their gantry
 * strands. They sit on two levels, belt A above belt B. Belt A's driving pulley is on the rear right,
 * belt B's on the rear left (each stepper's shaft carries it), and each belt turns four idlers:
 * two on the frame (a front corner and the rear corner of the other side) and two on the gantry's ends.
 *
 * A pulley turns with the belt that runs round it. The belt moves as far as the carriage travels along
 * the gantry strand, and as far as the gantry's end idler is from the frame idler, in the sense
 * of its heading. That makes the motor pulleys (angles in radians, pitch radius `R`):
 *
 *     motor A = (x - y) / R        motor B = (x + y) / R
 *
 * A frame idler turns with its belt like its motor does, so it follows both axes; a gantry end idler
 * sees only the carriage moving along the gantry, so follows `x` alone. Each is one coupling per
 * leader, written from the belts' geometry (`TimingBelt.strands()` and `BeltWrap.side`).
 *
 * The belts are drawn at their home position. The bed's travel stays clear of the rear corners.
 */
class CoreXyPlotter extends MachineAssembly {
	public static inline var RAIL:String = "MGN12C";
	public static inline var BASE_WIDTH:Float = 300;
	public static inline var BASE_DEPTH:Float = 260;
	public static inline var BASE_TOP:Float = 8;
	/** GT2 belts, 20-tooth pulleys: pitch radius 6.366 mm. */
	public static inline var TEETH:Int = 20;
	public static inline var BELT_WIDTH:Float = 6;
	/** Motor columns (the rear motors and the front idlers) at x = ±COLUMN_X, and the rear and front idlers at y = ±CORNER_Y. */
	public static inline var COLUMN_X:Float = 100;
	public static inline var CORNER_Y:Float = 90;
	/** The idler of the other belt next to each rear motor, this far in from the motor's column. */
	public static inline var INNER_X:Float = 66;
	/** The Y rails, outside the motors. */
	public static inline var RAIL_X:Float = 135;
	public static inline var RAIL_LENGTH:Float = 200;
	/** Supply of the stepper drivers, as on most desktop machines. */
	public static inline var SUPPLY_VOLTS:Float = 24;
	/** Pen tip height above the bed's top at (0, 0), set by the pen's length. */
	public static inline var PEN_BASE:Float = 14;

	public final motor:NemaStepper;
	public final specs:Array<PlotterAxisSpec> = [
		{id: "x", lower: -35, upper: 35, initial: 0},
		{id: "y", lower: -35, upper: 35, initial: 0}
	];
	/** Pitch radius of the pulleys, in millimetres. */
	public final pulleyRadius:Float;
	/** Pulley bottoms (z) of belts A and B. */
	public final levelA:Float;
	public final levelB:Float;

	final overtravel = new Map<String, Float>();
	final zeroPoses = new Map<String, AssemblyFrame>();

	public function new() {
		super();
		motor = NemaStepper.frame(17);
		var shaft = motor.variant.shaftLength;
		pulleyRadius = TimingPulley.profileDimensions(GT2).pitch * TEETH / (2 * Math.PI);
		var up = [0.0, 0, 1], alongY = [0.0, 1, 0], front = [0.0, -1, 0], alongX = [1.0, 0, 0];
		var railSpec = LinearRailBlock.metric(RAIL).spec;
		var motorFace = BASE_TOP + motor.bodyLength;
		levelA = motorFace + shaft - BELT_WIDTH - 1;
		levelB = levelA - BELT_WIDTH - 1;
		var R = pulleyRadius;

		place("base", new PlotterBlock(BASE_WIDTH, BASE_DEPTH, BASE_TOP, "aluminium 6061", "Base plate"), AssemblyFrames.translation(0, 0, 0));
		var railTop = BASE_TOP + railSpec.railHeight;
		var blockTop = BASE_TOP + railSpec.blockHeight;
		for (side in [-1, 1]) {
			var name = side < 0 ? "Left" : "Right";
			attach('railY$name', LinearRail.metric(RAIL, RAIL_LENGTH), orient(side * RAIL_X, -RAIL_LENGTH / 2, railTop, up, alongY), "base");
		}

		// The belts' loops in their plane, with the gantry's idlers at y = 0, where the gantry starts.
		var beltA = new TimingBelt(GT2, BELT_WIDTH, [
			new BeltWrap(-INNER_X, 0, R, 1), new BeltWrap(COLUMN_X - 2 * R, -2 * R, R, -1),
			new BeltWrap(COLUMN_X, -CORNER_Y, R, 1), new BeltWrap(COLUMN_X, CORNER_Y, R, 1),
			new BeltWrap(-INNER_X, CORNER_Y, R, 1)]);
		var beltB = new TimingBelt(GT2, BELT_WIDTH, [
			new BeltWrap(INNER_X, 0, R, -1), new BeltWrap(-(COLUMN_X - 2 * R), -2 * R, R, 1),
			new BeltWrap(-COLUMN_X, -CORNER_Y, R, -1), new BeltWrap(-COLUMN_X, CORNER_Y, R, -1),
			new BeltWrap(INNER_X, CORNER_Y, R, -1)]);
		attach("beltA", beltA, AssemblyFrames.translation(0, 0, levelA), "base");
		attach("beltB", beltB, AssemblyFrames.translation(0, 0, levelB), "base");

		// Motors on the frame, one belt each, the shaft carrying the belt's driving pulley.
		attach("motorA", motor, orient(COLUMN_X, CORNER_Y, motorFace, [0.0, 1, 0], up), "base");
		attach("motorB", NemaStepper.frame(17), orient(-COLUMN_X, CORNER_Y, motorFace, [0.0, 1, 0], up), "base");
		var pinLength = levelA + BELT_WIDTH + 2 - BASE_TOP;
		var wraps = [beltA.wraps(), beltB.wraps()];
		var beltIds = ["A", "B"], belts = [beltA, beltB], levels = [levelA, levelB];
		// Wrap 0 and 1 of each belt are the gantry's idlers, 2 the front idler, 3 the driving pulley, 4 the rear idler.
		var frameWraps = [[2, 3, 4], [2, 3, 4]];
		for (b in 0...2) {
			var letter = beltIds[b], belt = belts[b], level = levels[b], loop = wraps[b];
			var zero = AssemblyFrames.translation(0, 0, level);
			// Frame pulleys: the driving one on its motor's shaft, the idlers on pins stood on the base.
			for (w in frameWraps[b]) {
				var wrap = loop[w];
				var id = w == 3 ? 'pulley$letter' : 'idler$letter${w == 2 ? "Front" : "Rear"}';
				var pose = AssemblyFrames.translation(wrap.x, wrap.y, level);
				var parent = 'motor$letter';
				if (w != 3) {
					parent = '${id}Pin';
					attach(parent, new PlotterPin(motor.variant.shaftDiameter, pinLength, "Idler pin"),
						AssemblyFrames.translation(wrap.x, wrap.y, BASE_TOP), "base");
				}
				hang(id, beltPulley(), pose, parent);
				turnWithBelt(id, parent, 'belt$letter', belt, wrap, true);
			}
		}

		// Gantry: a bracket on each rail block carrying the idlers of both belts at that end, and the crossbar
		// between them. Only the left block's joint is the axis: the right block is bolted to the gantry.
		var ySpec = specs[1], xSpec = specs[0];
		var bracketBase = blockTop;
		slide(ySpec, "railYLeft", "blockYLeft", LinearRailBlock.metric(RAIL), orient(-RAIL_X, 0, railTop, up, alongY), {x: 0, y: 1, z: 0});
		attach("bracketLeft", new PlotterBlock(80, 36, 6, "aluminium 6061", "Gantry bracket"),
			AssemblyFrames.translation(-95, 0, bracketBase), "blockYLeft");
		var barHeight = 20.0;
		attach("crossbar", new PlotterBlock(190, 20, barHeight, "aluminium 6061", "Gantry crossbar"),
			AssemblyFrames.translation(0, 20, bracketBase), "bracketLeft");
		attach("bracketRight", new PlotterBlock(80, 36, 6, "aluminium 6061", "Gantry bracket"),
			AssemblyFrames.translation(95, 0, bracketBase), "crossbar");
		attach("blockYRight", LinearRailBlock.metric(RAIL), orient(RAIL_X, 0, railTop, up, alongY), "bracketRight");
		for (b in 0...2) {
			var letter = beltIds[b], loop = wraps[b], level = levels[b];
			for (w in 0...2) {
				var wrap = loop[w];
				var id = 'idler$letter${w == 0 ? "Start" : "End"}';
				var pin = '${id}Pin';
				attach(pin, new PlotterPin(motor.variant.shaftDiameter, pinLength - (bracketBase + 6 - BASE_TOP), "Idler pin"),
					AssemblyFrames.translation(wrap.x, wrap.y, bracketBase + 6), wrap.x < 0 ? "bracketLeft" : "bracketRight");
				hang(id, beltPulley(), AssemblyFrames.translation(wrap.x, wrap.y, level), pin);
				turnWithBelt(id, pin, 'belt$letter', belts[b], wrap, false);
			}
		}

		// Carriage on a rail along the crossbar's front face (y = 10), clamping both belts' gantry strands.
		var railXFace = 10 - railSpec.railHeight;
		var railLength = 160.0, railZ = bracketBase + barHeight / 2;
		attach("railX", LinearRail.metric(RAIL, railLength), orient(-railLength / 2, railXFace, railZ, front, alongX), "crossbar");
		slide(xSpec, "railX", "blockX", LinearRailBlock.metric(RAIL), orient(0, railXFace, railZ, front, alongX), {x: 1, y: 0, z: 0});
		var blockFront = railXFace - (railSpec.blockHeight - railSpec.railHeight);
		var plateBase = bracketBase - 2, plateHeight = levelB - 3 - plateBase;
		attach("carriagePlate", new PlotterBlock(40, 6, plateHeight, "aluminium 6061", "Carriage plate"),
			AssemblyFrames.translation(0, blockFront - 3, plateBase), "blockX");
		// The clamp holds the belts' gantry strands, which run at y = -R from the gantry's middle line.
		var padBase = plateBase + plateHeight;
		var padTop = levelA + BELT_WIDTH + 2;
		attach("clamp", new PlotterBlock(24, 9, padTop - padBase, "aluminium 6061", "Belt clamp"),
			AssemblyFrames.translation(0, blockFront - 4.5, padBase), "carriagePlate");
		for (b in 0...2) {
			var letter = b == 0 ? "A" : "B";
			addMemberConnector("clamp", "belt" + letter, AssemblyFrames.compose(
				AssemblyFrames.inverse(zeroPoses.get("clamp")), AssemblyFrames.translation(0, -R, b == 0 ? levelA : levelB)));
			addBeltPath({belt: "belt" + letter, clamp: {instanceId: "clamp", connectorName: "belt" + letter},
				wraps: [for (id in ["idler" + letter + "Start", "idler" + letter + "End", "idler" + letter + "Front", "pulley" + letter, "idler" + letter + "Rear"])
					{instanceId: id, connectorName: "attach-" + id}]});
		}
		var penLength = padBase - PEN_BASE;
		attach("pen", new PlotterPin(8, penLength, "Pen", "plastic"),
			AssemblyFrames.translation(0, blockFront - 6 - 4, PEN_BASE), "carriagePlate");
		exposeConnector("penTip", "pen", "tip");

		// Supply and drivers stay on the fixed frame; the supply's service port sets their voltage.
		attach("powerSupply", new PowerSupply(SUPPLY_VOLTS, 10, 2),
			AssemblyFrames.translation(BASE_WIDTH / 2 + 70, 0, BASE_TOP), "base");
		// Each stepper turns its belt's driving pulley.
		for (entry in [{id: "A", x: -30.0}, {id: "B", x: 30.0}]) {
			var driver = 'driver${entry.id}';
			attach(driver, new MotorDriver("GENERIC-TMC2209", 1.68, 16),
				AssemblyFrames.translation(entry.x, -BASE_DEPTH / 2 + 20, BASE_TOP), "base");
			connectPorts('$driver-power', "powerSupply", entry.id == "A" ? "power1" : "power2", driver, "power");
			addMotor('motor${entry.id}', 'pulley${entry.id}-turn', 'motor${entry.id}', driver);
		}
	}

	/** Room past the travel of axis `id` before its rail block reaches the rail's end, in millimetres. */
	public function axisOvertravel(id:String):Float {
		var room = overtravel.get(id);
		if (room == null) throw 'CoreXY plotter has no axis "$id"';
		return room;
	}

	/** A GT2 pulley of the plotter's size, bored for the motor shaft. */
	function beltPulley():TimingPulley
		return new TimingPulley(GT2, TEETH, motor.variant.shaftDiameter, BELT_WIDTH);

	/**
	 * Pulley `id`, already added with its origin on its axis, turns on a continuous joint `id-turn`
	 * about +Z and follows the axes through its belt. A belt moves along its heading as far as the carriage
	 * travels along the gantry strand (the belt's strand 0), and a frame pulley also sees the strand that
	 * leaves the gantry's end idler for the frame (strand 1) lengthen as the gantry moves, so a pulley
	 * on frame follows both axes and one on the gantry only x. The pulley turns the way its belt goes
	 * round it (`BeltWrap.side`).
	 */
	function turnWithBelt(id:String, parent:String, beltId:String, belt:TimingBelt, wrap:BeltWrap, onFrame:Bool):Void {
		var strands = belt.strands();
		var signs = [sign(strands[0].dx)];
		if (onFrame) signs.push(sign(strands[1].dy));
		var axes = ["x", "y"];
		for (index in 0...signs.length) if (signs[index] != 0)
			addTransmission('$id-${axes[index]}', axes[index], '$id-turn', (id == "pulleyA" || id == "pulleyB" ? Transmission.TimingBelt(beltId, id) : Transmission.BeltIdler(beltId, id)),
				SenseTools.fromAlignment(wrap.side * signs[index]));
		addMateOnAxis('$id-turn', "continuous", parent, 'to-$id', id, 'attach-$id', {x: 0, y: 0, z: 1}, 0);
	}

	static function sign(value:Float):Int return value > 1e-9 ? 1 : value < -1e-9 ? -1 : 0;

	function component(id:String):MachineComponent {
		for (entry in components()) if (entry.id == id) return entry.component;
		throw 'CoreXY plotter has no member "$id" yet';
	}

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

	/** Adds `component` at `pose` as a child of `parent` without a mate: the caller adds the joint. */
	function hang(id:String, component:MachineComponent, pose:AssemblyFrame, parent:String):Void {
		addComponent(id, component);
		zeroPoses.set(id, pose);
		connect(parent, id);
	}

	/**
	 * A rail block sliding on its rail (`parent`) along a world axis; `pose` is where it sits at coordinate
	 * zero. The axis's overtravel is the room its block has left on the rail at either end of travel.
	 */
	function slide(spec:PlotterAxisSpec, parent:String, id:String, block:MachineComponent, pose:AssemblyFrame,
			axis:{x:Float, y:Float, z:Float}):Void {
		var rail = component(parent);
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
		addComponent(id, block);
		zeroPoses.set(id, pose);
		connect(parent, id);
		addMateOnAxis(spec.id, "prismatic", parent, 'to-$id', id, 'attach-$id', axis, spec.initial,
			{lower: spec.lower, upper: spec.upper, velocity: null, effort: null, overtravel: room});
	}

	/**
	 * Connectors meeting at the child's origin with world-aligned axes, so joint axes are world
	 * directions. Names carry the member ids, so members that share geometry can share one definition.
	 */
	function connect(parent:String, child:String):Void {
		var childPose = zeroPose(child);
		var meeting = AssemblyFrames.translation(childPose.x, childPose.y, childPose.z);
		addMemberConnector(parent, 'to-$child', AssemblyFrames.compose(AssemblyFrames.inverse(zeroPose(parent)), meeting));
		addMemberConnector(child, 'attach-$child', AssemblyFrames.compose(AssemblyFrames.inverse(childPose), meeting));
	}

	function zeroPose(id:String):AssemblyFrame {
		var pose = zeroPoses.get(id);
		if (pose == null) throw 'CoreXY plotter has no member "$id" yet';
		return pose;
	}
}
