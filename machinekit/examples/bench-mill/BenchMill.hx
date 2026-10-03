import machinekit.assembly.MachineAssembly;
import machinekit.assembly.Transmission;
import cadkit.modeling.Part;
import cadkit.modeling.Vector;
import machinekit.component.ComponentDetail;
import machinekit.component.MachineComponent;
import machinekit.component.Solids;
import machinekit.milling.MillCasting.MillBase;
import machinekit.milling.MillCasting.MillColumn;
import machinekit.milling.MillCasting.MillSaddle;
import machinekit.milling.MillCasting.MillTable;
import machinekit.milling.MillCasting.MillHead;
import machinekit.milling.SpindleCartridge;
import machinekit.milling.SpindleCartridge.Er20Holder;
import machinekit.milling.SpindleMotor;
import machinekit.motion.BallScrew;
import machinekit.motion.BallNut;
import machinekit.motion.ScrewSupportUnit;
import machinekit.motion.LinearRail;
import machinekit.motion.LinearRailBlock;
import machinekit.motion.ServoMotor;
import machinekit.motion.MotorDriver;
import machinekit.motion.PowerSupply;
import machinekit.motion.ShaftCoupling;
import machinekit.motion.ShaftEncoder;
import machinekit.transmission.TimingBelt;
import machinekit.transmission.TimingPulley;
import materia.assembly.AssemblyFrames;
import materia.assembly.AssemblyRecord.AssemblyFrame;
import toolpathkit.tool.Tool;
import toolpathkit.setup.Fixture;
import CncRouter.EndMill;
import CncRouter.RouterPlate;
import CncRouter.ToeClamp;

/** Machine coordinates, mm; brake is a mechanical property of the vertical axis. */
typedef MillAxisSpec = {
	var id:String;
	var lower:Float;
	var upper:Float;
	var initial:Float;
	var holdingBrake:Bool;
}

/** Table X and saddle Y move opposite the tool's work coordinates; the head moves along Z.
 * All mates are solved from the zero poses, as on the router. The rails, screw supports and
 * current motor parts decide room, speed and acceleration every time the machine is compiled.
 */
class BenchMill extends MachineAssembly {
	public static inline var RAIL:String = "HGR15";
	public static inline var STOCK_WIDTH:Float = 60;
	public static inline var STOCK_DEPTH:Float = 40;
	public static inline var STOCK_HEIGHT:Float = 20;
	public static inline var STEP_TICK_HZ:Int = 40000;
	public final vise:Null<machinekit.milling.PneumaticVise>;
	public final base = new MillBase();
	public final column = new MillColumn();
	public final saddle = new MillSaddle();
	public final table = new MillTable();
	public final head = new MillHead();
	public final spindle = new SpindleCartridge();
	public final holder = new Er20Holder();
	public final tool = new EndMill(Er20Holder.BORE_DIAMETER, 22, 35);
	public final specs:Array<MillAxisSpec> = [
		{id: "x", lower: 0, upper: 250, initial: 125, holdingBrake: false},
		{id: "y", lower: 0, upper: 150, initial: 75, holdingBrake: false},
		{id: "z", lower: -250, upper: 0, initial: 0, holdingBrake: true}
	];
	public final tableTop:Float;
	public final gaugeZero:AssemblyFrame;
	final zeroPoses = new Map<String, AssemblyFrame>();
	final overtravel = new Map<String, Float>();

	public function new(withVise:Bool = false) {
		super();
		vise = withVise ? new machinekit.milling.PneumaticVise(STOCK_DEPTH, STOCK_WIDTH) : null;
		var railSpec = LinearRailBlock.metric(RAIL).spec;
		var up = [0.0, 0, 1], alongX = [1.0, 0, 0], alongY = [0.0, 1, 0];
		var baseTop = base.height;
		var saddleBottom = baseTop + railSpec.blockHeight;
		var saddleTop = saddleBottom + saddle.height;
		var tableBottom = saddleTop + railSpec.blockHeight;
		tableTop = tableBottom + table.height;
		var columnSeat = base.connector("column").frame;
		var columnPose = AssemblyFrames.translation(columnSeat.x, columnSeat.y, columnSeat.z);
		var columnFront = columnPose.y - column.depth / 2;
		var headBack = columnFront - railSpec.blockHeight;
		var headY = headBack - head.depth / 2;
		var spindleY = headY + head.connector("spindle").frame.y;
		gaugeZero = AssemblyFrames.translation(0, spindleY, tableTop + tool.stickout - specs[2].lower);
		var saddleY = spindleY + specs[1].upper / 2;
		var tableX = specs[0].upper / 2;
		place("base", base, AssemblyFrames.translation(0, 0, 0));
		attach("column", column, columnPose, "base");
		var yRailStart = -base.depth / 2 + 5;
		var yRailLength = columnFront - 5 - yRailStart;
		for (side in [-1, 1]) {
			var name = side < 0 ? "Left" : "Right";
			attach('railY$name', LinearRail.metric(RAIL, yRailLength),
				orient(side * base.railSpacing / 2, yRailStart, baseTop + railSpec.railHeight, up, alongY), "base");
		}
		slide(specs[1], "railYLeft", "blockYLeft", LinearRailBlock.metric(RAIL),
			orient(-base.railSpacing / 2, saddleY, baseTop + railSpec.railHeight, up, alongY), [0.0, -1, 0]);
		attach("saddle", saddle, AssemblyFrames.translation(0, saddleY, saddleBottom), "blockYLeft");
		attach("blockYRight", LinearRailBlock.metric(RAIL),
			orient(base.railSpacing / 2, saddleY, baseTop + railSpec.railHeight, up, alongY), "saddle");

		var xRailLength = saddle.width - 10;
		for (side in [-1, 1]) {
			var name = side < 0 ? "Front" : "Back";
			attach('railX$name', LinearRail.metric(RAIL, xRailLength),
				orient(-xRailLength / 2, saddleY + side * saddle.railSpacing / 2, saddleTop + railSpec.railHeight, up, alongX), "saddle");
		}
		slide(specs[0], "railXFront", "blockXFront", LinearRailBlock.metric(RAIL),
			orient(tableX, saddleY - saddle.railSpacing / 2, saddleTop + railSpec.railHeight, up, alongX), [-1.0, 0, 0]);
		attach("table", table, AssemblyFrames.translation(tableX, saddleY, tableBottom), "blockXFront");
		attach("blockXBack", LinearRailBlock.metric(RAIL),
			orient(tableX, saddleY + saddle.railSpacing / 2, saddleTop + railSpec.railHeight, up, alongX), "table");

		var headZ = gaugeZero.z + spindle.connector("mount").frame.z;
		var zRailStart = columnPose.z + railSpec.railEndMargin;
		var zRailLength = column.height - 2 * railSpec.railEndMargin;
		var zBlock = headZ + head.height / 2;
		for (side in [-1, 1]) {
			var name = side < 0 ? "Left" : "Right";
			attach('railZ$name', LinearRail.metric(RAIL, zRailLength),
				orient(side * column.railSpacing / 2, columnFront - railSpec.railHeight, zRailStart, [0.0, -1, 0], [0.0, 0, 1]), "column");
		}
		slide(specs[2], "railZLeft", "blockZLeft", LinearRailBlock.metric(RAIL),
			orient(-column.railSpacing / 2, columnFront - railSpec.railHeight, zBlock, [0.0, -1, 0], [0.0, 0, 1]), [0.0, 0, 1]);
		attach("head", head, AssemblyFrames.translation(0, headY, headZ), "blockZLeft");
		attach("blockZRight", LinearRailBlock.metric(RAIL),
			orient(column.railSpacing / 2, columnFront - railSpec.railHeight, zBlock, [0.0, -1, 0], [0.0, 0, 1]), "head");
		// MT2 keeps the cartridge at zero speed; MT5 binds its continuous joint to spindle.speed.
		addComponent("spindle", spindle);
		zeroPoses.set("spindle", gaugeZero);
		connect("head", "spindle");
		addMateOnAxis("spindle-turn", "continuous", "head", "to-spindle", "spindle", "attach-spindle", {x: 0, y: 0, z: 1});
		attach("holder", holder, gaugeZero, "spindle");
		attach("tool", tool, AssemblyFrames.translation(0, spindleY, gaugeZero.z - tool.stickout), "spindle");
		if (vise == null) {
			attach("stock", new RouterPlate(STOCK_WIDTH, STOCK_DEPTH, STOCK_HEIGHT, "aluminium 6061", "Bearing block blank"),
				AssemblyFrames.translation(tableX, saddleY, tableTop), "table");
			// Both clamps hold the back corners, clear of the bearing seat and fastening recesses.
			for (side in [-1, 1]) attach(side < 0 ? "clampLeft" : "clampRight", new ToeClamp(STOCK_HEIGHT),
				AssemblyFrames.fromRotationMatrix(tableX + side * (STOCK_WIDTH / 2 - 8), saddleY + STOCK_DEPTH / 2, tableTop,
					[0.0, -1, 0, 1.0, 0, 0, 0.0, 0, 1]), "table");
		} else {
			include("vise", vise, AssemblyFrames.translation(tableX, saddleY, tableTop));
			zeroPoses.set("vise/body", AssemblyFrames.translation(tableX, saddleY, tableTop));
			connect("table", "vise/body");
			addMate("vise-mount", "fixed", "table", "to-vise/body", "vise/body", "attach-vise/body");
			attach("stock", new RouterPlate(STOCK_WIDTH, STOCK_DEPTH, STOCK_HEIGHT, "aluminium 6061", "Bearing block blank"),
				AssemblyFrames.translation(tableX, saddleY, tableTop + vise.datum.z), "vise/body");
		}

		exposeConnector("gaugeLine", "spindle", "gaugeLine");
		exposeConnector("toolTip", "tool", "tip");
		attach("powerSupply", new PowerSupply(48, 40, 4), AssemblyFrames.translation(-190, columnPose.y, 0), "base");
		var xScrewLength = table.width + specs[0].upper - specs[0].lower +
			2 * (BallScrew.INPUT_JOURNAL_LENGTH + ScrewSupportUnit.bk12().length);
		driveScrew(specs[0], "saddle", "table", [-xScrewLength / 2, saddleY, tableBottom - railSpec.blockHeight / 2],
			[1.0, 0, 0], [0.0, 0, 1], xScrewLength, [tableX, saddleY, tableBottom - railSpec.blockHeight / 2]);
		driveScrew(specs[1], "base", "saddle", [0.0, yRailStart, saddleBottom + saddle.screwHeight],
			[0.0, 1, 0], [0.0, 0, 1], yRailLength, [0.0, saddleY, saddleBottom + saddle.screwHeight]);
		var zScrewTop = columnPose.z + column.height + railSpec.railEndMargin;
		var zScrewX = column.width / 2 + new BallNut().flangeDiameter / 2 + 2;
		driveScrew(specs[2], "column", "head", [zScrewX, (headBack + columnFront) / 2, zScrewTop],
			[0.0, 0, -1], [1.0, 0, 0], column.height,
			[zScrewX, (headBack + columnFront) / 2, headZ + head.height / 2]);
		var pulley = new TimingPulley(HTD5M, 20, 20, 15);
		var motorPulley = new TimingPulley(HTD5M, 20, 20, 15);
		var belt = TimingBelt.twoPulley(HTD5M, 20, 20, 150, 15);
		var beltZ = gaugeZero.z + spindle.connector("pulley").frame.z;
		attach("spindlePulley", pulley, AssemblyFrames.translation(0, spindleY, beltZ), "spindle");
		var spindleDrive = new SpindleMotor();
		var motorFaceZ = beltZ + motorPulley.thickness;
		attach("spindleMotor", spindleDrive, orient(150, spindleY, motorFaceZ, [0.0, 1, 0], [0.0, 0, -1]), "head");
		attach("spindleMotorShaft", new MillDriveShaft(20, motorPulley.thickness),
			orient(150, spindleY, motorFaceZ, [0.0, 1, 0], [0.0, 0, -1]), "spindleMotor");
		var shelfBottom = motorFaceZ - 8;
		attach("spindleMotorShelf", new MillMotorPlate(spindleDrive.rating.bodyDiameter + 20, 8, motorPulley.outsideDiameter + 2),
			AssemblyFrames.translation(150, spindleY, shelfBottom), "head");
		attach("spindleMotorRiser", new MillDriveBox(20, 100, shelfBottom - headZ - head.height),
			AssemblyFrames.translation(head.width / 2, spindleY + 5, headZ + head.height), "head");
		attach("spindleMotorPulley", motorPulley, AssemblyFrames.translation(150, spindleY, beltZ), "spindleMotor");
		attach("spindleBelt", belt, AssemblyFrames.translation(0, spindleY, beltZ), "head");
	}

	/** Each drive reads its lead and bearing boundary from its screw, nut and support parts. */
	function driveScrew(spec:MillAxisSpec, parent:String, carriage:String, start:Array<Float>, along:Array<Float>,
			up:Array<Float>, length:Float, nutCenter:Array<Float>):Void {
		var id = "screw" + spec.id.toUpperCase();
		var motorId = "motor" + spec.id.toUpperCase();
		var motor = ServoMotor.model(spec.id == "z" ? "GENERIC-SERVO-750W" : "GENERIC-SERVO-400W");
		var screw = new BallScrew(length), nut = new BallNut();
		var coupling = new ShaftCoupling(12, BallScrew.JOURNAL_DIAMETER, 26, 36);
		var shaftLength = 15.0; // Generic servo shaft extension, assumed.
		var motorOffset = coupling.length / 2 + shaftLength;
		var motorPose = orient(start[0] - along[0] * motorOffset, start[1] - along[1] * motorOffset,
			start[2] - along[2] * motorOffset, up, along);
		var couplingPose = orient(start[0] - along[0] * coupling.length / 2, start[1] - along[1] * coupling.length / 2,
			start[2] - along[2] * coupling.length / 2, up, along);
		attach(motorId, motor, motorPose, parent);
		addComponent(id, screw);
		zeroPoses.set(id, orient(start[0], start[1], start[2], up, along));
		connect(motorId, id);

		attach(id + "Coupling", coupling, couplingPose, id);
		attach(motorId + "Shaft", new MillDriveShaft(12, shaftLength), motorPose, id);
		var nutOffset = nut.bodyLength / 2;
		attach(id + "Nut", nut, orient(nutCenter[0] - along[0] * nutOffset,
			nutCenter[1] - along[1] * nutOffset, nutCenter[2] - along[2] * nutOffset, up, along), carriage);
		var faceDistance = nut.bodyLength / 2 + nut.flangeThickness;
		attach(id + "NutBracket", new MillNutBracket(nut, spec.id == "z"),
			orient(nutCenter[0] + along[0] * faceDistance, nutCenter[1] + along[1] * faceDistance,
				nutCenter[2] + along[2] * faceDistance, up, along), carriage);
		var near = ScrewSupportUnit.bk12(), far = ScrewSupportUnit.bf12();
		for (end in 0...2) {
			var support = end == 0 ? near : far;
			var offset = end == 0 ? BallScrew.INPUT_JOURNAL_LENGTH - near.length / 2 : length - BallScrew.JOURNAL_LENGTH / 2;
			attach(id + (end == 0 ? "Fixed" : "Floating"), support,
				orient(start[0] + along[0] * offset, start[1] + along[1] * offset,
					start[2] + along[2] * offset, up, along), parent);
		}
		var motorPlate = new MillMotorPlate(motor.rating.bodyDiameter + 20, 8, 12);
		if (spec.id == "y") {
			var footZ = start[2] + near.connector("mount").frame.y;
			for (end in 0...2) {
				var unit = end == 0 ? near : far;
				var offset = end == 0 ? BallScrew.INPUT_JOURNAL_LENGTH - near.length / 2 : length - BallScrew.JOURNAL_LENGTH / 2;
				attach(id + 'Stand$end', new MillDriveBox(unit.width, unit.length, footZ - base.height),
					AssemblyFrames.translation(start[0], start[1] + offset, base.height), parent);
			}
			var plateBottom = start[2] - motorPlate.width / 2;
			var front = -base.depth / 2;
			attach(motorId + "Bridge", new MillDriveBox(motorPlate.width, front - motorPose.y, plateBottom - base.height + 10),
				AssemblyFrames.translation(0, (front + motorPose.y) / 2, base.height - 10), parent);
		} else if (spec.id == "x") {
			var footZ = start[2] + near.connector("mount").frame.y;
			for (side in [-1, 1]) {
				var reach = Math.abs(motorPose.x) + motorPlate.thickness + 4 - saddle.width / 2;
				var slot = side < 0 ? motorPose.x + saddle.width / 2 + reach / 2 + motorPlate.thickness / 2 : null;
				attach(id + (side < 0 ? "NearBridge" : "FarBridge"), new MillDriveBox(reach, 80, 10, slot, motorPlate.thickness),
					AssemblyFrames.translation(side * (saddle.width / 2 + reach / 2), start[1], footZ - 10), parent);
			}
		} else {
			for (end in 0...2) {
				var unit = end == 0 ? near : far;
				var offset = end == 0 ? BallScrew.INPUT_JOURNAL_LENGTH - near.length / 2 : length - BallScrew.JOURNAL_LENGTH / 2;
				var footX = start[0] + unit.connector("mount").frame.y;
				attach(id + 'Stand$end', new MillDriveBox(footX - column.width / 2, unit.width, unit.length),
					AssemblyFrames.translation((footX + column.width / 2) / 2, start[1], start[2] - offset - unit.length / 2), parent);
			}
		}
		if (spec.id == "z") {
			var top = zeroPose(parent).z + column.height;
			attach(motorId + "Post", new MillDriveBox(20, 20, motorPose.z - motorPlate.thickness - top),
				AssemblyFrames.translation(column.width / 2 - 10, start[1] + motorPlate.width / 2 - 10, top), parent);
		}
		attach(motorId + "Plate", motorPlate, motorPose, parent);
		var ratio = addTransmission(id + "-lead", spec.id, id + "-turn", Transmission.LeadScrew(id, id + "Nut"), Opposite);
		// Motor zero agrees with the table/head's initial displacement.
		addMateOnAxis(id + "-turn", "continuous", motorId, 'to-$id', id, 'attach-$id',
			{x: along[0], y: along[1], z: along[2]}, ratio * spec.initial);
		supportScrew(id + "-lead", near.support, far.support);
		var driver = motorId + "Driver";
		var index = spec.id == "x" ? 1 : spec.id == "y" ? 2 : 3;
		attach(driver, new MotorDriver("GENERIC-SERVO-AMP", 10),
			AssemblyFrames.translation(-190 + index * 65, base.depth / 2 - 30, 0), "base");
		connectPorts(driver + "-power", "powerSupply", 'power$index', driver, "power");
		addMotor(motorId, id + "-turn", motorId, driver);
		var encoderId = motorId + "Encoder";
		attach(encoderId, new ShaftEncoder(motor.rating.encoderCounts, true),
			orient(motorPose.x - along[0] * motor.rating.bodyLength, motorPose.y - along[1] * motor.rating.bodyLength,
				motorPose.z - along[2] * motor.rating.bodyLength, up, [-along[0], -along[1], -along[2]]), motorId);
		addEncoder(encoderId, id + "-turn", encoderId, motorId);
	}

	public function workOffset():Array<Float> {
		if (vise != null) {
			var datum = AssemblyFrames.compose(zeroPose("vise/body"), vise.datum);
			return [datum.x - gaugeZero.x, datum.y - gaugeZero.y, datum.z + STOCK_HEIGHT - gaugeZero.z];
		}
		var stock = zeroPose("stock");
		return [stock.x - STOCK_WIDTH / 2 - gaugeZero.x, stock.y - STOCK_DEPTH / 2 - gaugeZero.y,
			stock.z + STOCK_HEIGHT - gaugeZero.z];
	}
	/** CAM forbidden boxes come from the installed clamps in stock work coordinates. */
	public function fixtures():Array<Fixture> {
		var stock = zeroPose("stock");
		var origin = AssemblyFrames.translation(stock.x - STOCK_WIDTH / 2, stock.y - STOCK_DEPTH / 2, stock.z + STOCK_HEIGHT);
		var inverse = AssemblyFrames.inverse(origin);
		var result:Array<Fixture> = [];
		var model = new cadkit.modeling.AssemblyModel("mm");
		addTo(model, "");
		var state = new cadkit.modeling.AssemblyState(model.definition("mill-fixtures"));
		for (spec in specs) state.setJoint(spec.id, 0);
		for (entry in components()) if (entry.id == "clampLeft" || entry.id == "clampRight" ||
			entry.id == "vise/fixedJaw" || entry.id == "vise/movingJaw" || entry.id == "vise/endStop") {
			var part = entry.component.geometry();
			var bounds = part.shape.bounds(), minimum = bounds.get_min(), maximum = bounds.get_max();
			var pose = AssemblyFrames.compose(inverse, state.worldPose(entry.id));
			var low = [Math.POSITIVE_INFINITY, Math.POSITIVE_INFINITY, Math.POSITIVE_INFINITY];
			var high = [Math.NEGATIVE_INFINITY, Math.NEGATIVE_INFINITY, Math.NEGATIVE_INFINITY];
			for (x in [minimum.get_x(), maximum.get_x()]) for (y in [minimum.get_y(), maximum.get_y()])
				for (z in [minimum.get_z(), maximum.get_z()]) {
					var point = AssemblyFrames.transformPoint(pose, x, y, z);
					var values = [point.x, point.y, point.z];
					for (axis in 0...3) { low[axis] = Math.min(low[axis], values[axis]); high[axis] = Math.max(high[axis], values[axis]); }
				}
			part.close();
			result.push(new Fixture(entry.id, low[0] / 1000, high[0] / 1000, low[1] / 1000, high[1] / 1000, low[2] / 1000, high[2] / 1000));
		}
		return result;
	}

	public function tools():Array<Tool> return [Tool.shaped(1, tool.stickout / 1000,
		tool.cutter().withHolder(Er20Holder.DIAMETER / 1000, Er20Holder.LENGTH / 1000))];
	public function axisOvertravel(id:String):Float {
		var room = overtravel.get(id);
		if (room == null) throw 'Bench mill has no axis "$id"';
		return room;
	}

	function slide(spec:MillAxisSpec, parent:String, id:String, block:LinearRailBlock, pose:AssemblyFrame,
			axis:Array<Float>):Void {
		var guide:LinearRail = cast component(parent);
		var inverse = AssemblyFrames.inverse(zeroPose(parent));
		function along(coordinate:Float):Float return AssemblyFrames.transformPoint(inverse,
			pose.x + axis[0] * coordinate, pose.y + axis[1] * coordinate, pose.z + axis[2] * coordinate).z;
		var reach = guide.spec.railEndMargin + guide.spec.blockLength / 2;
		var first = along(spec.lower), last = along(spec.upper);
		var room = Math.min(Math.min(first, last) - reach, guide.length - reach - Math.max(first, last));
		if (!(room > 0)) throw 'Bench mill ${spec.id} needs room beyond travel on its rail';
		overtravel.set(spec.id, room);
		addComponent(id, block);
		zeroPoses.set(id, pose);
		connect(parent, id);
		addMateOnAxis(spec.id, "prismatic", parent, 'to-$id', id, 'attach-$id',
			{x: axis[0], y: axis[1], z: axis[2]}, spec.initial,
			{lower: spec.lower, upper: spec.upper, velocity: null, effort: null, overtravel: room});
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

	function component(id:String):MachineComponent {
		for (entry in components()) if (entry.id == id) return entry.component;
		throw 'Bench mill has no member "$id" yet';
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
		if (pose == null) throw 'Bench mill has no member "$id" yet';
		return pose;
	}
}

/** Assumed exposed servo shaft; its body remains in the reusable servo part. */
private class MillDriveShaft extends MachineComponent {
	final diameter:Float;
	final length:Float;
	public function new(diameter:Float, length:Float) {
		super('MILL-SHAFT-$diameter-$length', "Assumed servo shaft extension", "steel", true);
		this.diameter = diameter; this.length = length;
	}
	override public function hasGeometry():Bool return true;
	override public function geometry(detail:ComponentDetail = Preview):Part
		return Solids.named(Part.cylinderSpan(diameter / 2, 0, length), "shaft");
}

/** Designed flat bracket bodies; source placement derives from the supported shaft and casting. */
private class MillDriveBox extends MachineComponent {
	final width:Float; final depth:Float; final height:Float;
	final slot:Null<Float>; final slotWidth:Float;
	public function new(width:Float, depth:Float, height:Float, ?slot:Float, slotWidth:Float = 0) {
		super('MILL-BRACKET-$width-$depth-$height-$slot-$slotWidth', "Mill drive support bracket", "steel", true);
		this.width = width; this.depth = depth; this.height = height;
		this.slot = slot; this.slotWidth = slotWidth;
	}
	override public function hasGeometry():Bool return true;
	override public function geometry(detail:ComponentDetail = Preview):Part {
		var body = Solids.named(Part.box(width, depth, height), "bracket");
		if (slot == null) return body;
		return Solids.cut(body, [Solids.named(Part.box(slotWidth, depth + 2, height + 2)
			.translated(new Vector(slot, 0, -1)), "motor-plate-seat")]);
	}
}

private class MillMotorPlate extends MachineComponent {
	public final width:Float; public final thickness:Float; final bore:Float;
	public function new(width:Float, thickness:Float, bore:Float) {
		super('MILL-MOTOR-PLATE-$width-$thickness-$bore', "Mill motor mounting plate", "steel", true);
		this.width = width; this.thickness = thickness; this.bore = bore;
	}
	override public function hasGeometry():Bool return true;
	override public function geometry(detail:ComponentDetail = Preview):Part {
		var body = Part.box(width, width, thickness).translated(new Vector(0, 0, 0));
		var holes = [Part.cylinderSpan(bore / 2 + 1, -1, thickness + 1)];
		for (x in [-1, 1]) for (y in [-1, 1]) holes.push(Part.cylinderSpan(3.3, -1, thickness + 1,
			x * (width / 2 - 10), y * (width / 2 - 10)));
		return Solids.cut(Solids.named(body, "plate"), holes);
	}
}

/** Nut flange adapter; the vertical-axis tongue reaches the head's side face. */
private class MillNutBracket extends MachineComponent {
	final nut:BallNut;
	final tongue:Bool;
	public function new(nut:BallNut, tongue:Bool) {
		super('MILL-NUT-BRACKET-${nut.designation}-$tongue', "Ball-nut flange adapter", "steel", true);
		this.nut = nut; this.tongue = tongue;
	}
	override public function hasGeometry():Bool return true;
	override public function geometry(detail:ComponentDetail = Preview):Part {
		var radius = nut.flangeDiameter / 2 + 1;
		var plate = Solids.named(Part.cylinderSpan(radius, 0, 8), "flange-seat");
		if (tongue) plate = Solids.union([plate, Solids.named(Part.box(10, 10, 8)
			.translated(new Vector(-radius + 6, -radius + 4, 0)), "head-seat")]);
		var tools = [Solids.named(Part.cylinderSpan(nut.screwDiameter / 2 + 1, -1, 9), "screw-bore")];
		for (point in nut.boltPattern()) tools.push(Part.cylinderSpan(2.75, -1, 9, point.x, point.y));
		return Solids.cut(plate, tools);
	}
}
