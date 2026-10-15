package machinekit.assembly;

import machinekit.component.MachineComponent;
import machinekit.component.Dimension;
import machinekit.motion.LinearRail;
import machinekit.motion.LinearRailBlock;
import machinekit.motion.LeadScrew;
import machinekit.motion.LeadScrewNut;
import machinekit.motion.NemaStepper;
import machinekit.motion.ShaftCoupling;
import machinekit.motion.ScrewSupport;
import machinekit.transmission.TimingBelt;
import machinekit.transmission.TimingBelt.BeltWrap;
import machinekit.transmission.TimingPulley;
import machinekit.transmission.Rack;
import machinekit.transmission.SpurGear;
import machinekit.assembly.Sense.SenseTools;
import materia.assembly.AssemblyFrames;
import materia.assembly.AssemblyRecord.AssemblyFrame;

/** Travel and initial coordinate of a linear joint, in assembly units. */
typedef AxisSpec = {id:String, lower:Float, upper:Float, initial:Float, @:optional rackingTolerance:Null<Float>};

/** One linear leader contributing to a belt pulley's shaft angle. */
typedef BeltAxisMotion = {id:String, leader:String, initial:Float, rotation:Int, idler:Bool};

/** A belt drive's mechanical layout. The machine supplies its idler mounting hardware. */
typedef TwoPulleyAxis = {
	axis:AxisSpec, beltId:String, belt:TimingBelt, beltPose:AssemblyFrame, beltParent:Null<String>,
	motorId:String, pulleyId:String, pulley:TimingPulley, idlerId:String, idler:TimingPulley,
	idlerPose:AssemblyFrame, about:Array<Float>, strand:Int, travelX:Float, travelY:Float,
	mountIdler:Void->String, driverForMotor:String->String
};

/** Shared member placement and rail travel for assemblies built from linear axes. */
class AxisBuilder extends MachineAssembly {
	/** Poses with all axes at zero; these determine the mate connectors. */
	public final zeroPoses = new Map<String, AssemblyFrame>();
	/** Room beyond travel before each guide block reaches its rail's end. */
	public final overtravel = new Map<String, Float>();
	/** The world direction each axis made by `slide` moves its block, a unit vector. */
	final axisDirections = new Map<String, Array<Float>>();

	public function new() { super(); }

	/** The world direction axis `id` moves its block as its coordinate grows (see `slide`). */
	public function axisDirection(id:String):Array<Float> {
		var direction = axisDirections.get(id);
		if (direction == null) throw 'Axis "$id" does not slide on a rail of this assembly';
		return direction.copy();
	}

	/** Room past the travel of axis `id` before its rail blocks reach the rail ends, in millimetres. */
	public function axisOvertravel(id:String):Float {
		var room = overtravel.get(id);
		if (room == null) throw 'Axis assembly has no axis "$id"';
		return room;
	}

	/** Frame whose local +Y points along `up` and local +Z along `along`: perpendicular unit vectors (a parallel pair is refused). */
	public static function orient(x:Float, y:Float, z:Float, up:Array<Float>, along:Array<Float>):AssemblyFrame {
		var xx = up[1] * along[2] - up[2] * along[1];
		var xy = up[2] * along[0] - up[0] * along[2];
		var xz = up[0] * along[1] - up[1] * along[0];
		return AssemblyFrames.fromRotationMatrix(x, y, z, [xx, up[0], along[0], xy, up[1], along[1], xz, up[2], along[2]]);
	}

	/** A fixed root member at its world pose. */
	public function place(id:String, component:MachineComponent, pose:AssemblyFrame):Void {
		addComponent(id, component, pose);
		zeroPoses.set(id, pose);
	}

	/** A member fixed to `parent`, at its world pose with every axis at zero. */
	public function attach(id:String, component:MachineComponent, pose:AssemblyFrame, parent:String):Void {
		addComponent(id, component);
		zeroPoses.set(id, pose);
		connect(parent, id);
		addMate('$id-mount', "fixed", parent, 'to-$id', id, 'attach-$id');
	}

	/** Adds `component` at `pose` as a child of `parent` without a mate: the caller adds the joint. */
	public function hang(id:String, component:MachineComponent, pose:AssemblyFrame, parent:String):Void {
		addComponent(id, component);
		zeroPoses.set(id, pose);
		connect(parent, id);
	}

	/**
	 * Declares seat `name` on member `parent` at `frame`, in that member's own frame: a place the design puts a part on it,
	 * for a part that does not declare its own seats (a rail, a tube). Plates declare theirs (`SeatedPlate`).
	 */
	public function seat(parent:String, name:String, frame:AssemblyFrame):Void addMemberConnector(parent, name, frame);

	/** Declares seat `name` on member `parent` at `world`, a frame in assembly coordinates with every axis at zero. */
	public function seatAt(parent:String, name:String, world:AssemblyFrame):Void seat(parent, name, localFrame(zeroPose(parent), world));

	/**
	 * Mounts `component` as `id` by its connector `connector` on a seat of `parent` that the design puts where that connector
	 * lies with the component at `pose`: the part sits on its real interface. The seat, `seat-<id>`, carries the mounted
	 * member's id, so members that share one definition (a left and a right column) keep their seats apart.
	 */
	public function mountAt(id:String, component:MachineComponent, connector:String, parent:String, pose:AssemblyFrame):Void {
		var seat = 'seat-$id';
		seatAt(parent, seat, AssemblyFrames.compose(pose, component.connector(connector).frame));
		mount(id, component, connector, parent, seat);
	}

	/** `world` (a frame in assembly coordinates) in the frame of something at `pose`: how a part's seat is written in its own coordinates. */
	public static function localFrame(pose:AssemblyFrame, world:AssemblyFrame):AssemblyFrame
		return AssemblyFrames.compose(AssemblyFrames.inverse(pose), world);

	/** A frame at (x, y, z) whose local +X and +Z point along `xAxis` and `zAxis` (square to each other); +Y is Z x X. */
	public static function axes(x:Float, y:Float, z:Float, xAxis:Array<Float>, zAxis:Array<Float>):AssemblyFrame {
		var yAxis = cross(zAxis, xAxis);
		return AssemblyFrames.fromRotationMatrix(x, y, z, [xAxis[0], yAxis[0], zAxis[0], xAxis[1], yAxis[1], zAxis[1], xAxis[2], yAxis[2], zAxis[2]]);
	}

	/**
	 * Mounts `component` as `id` on `parent`'s seat `seat` by the component's own connector `connector`: the member sits
	 * where the seat puts it, its connector's frame on the seat's. Seats belong to the carrying part (or were declared with
	 * `seat`), so a member is never placed first and its carrier's connector invented where it landed.
	 */
	public function mount(id:String, component:MachineComponent, connector:String, parent:String, seat:String):Void {
		var seatFrame = connectorFrame(parent, seat);
		if (seatFrame == null) throw '$parent has no seat "$seat" for $id';
		addComponent(id, component);
		zeroPoses.set(id, AssemblyFrames.compose(AssemblyFrames.compose(zeroPose(parent), seatFrame),
			AssemblyFrames.inverse(component.connector(connector).frame)));
		addMate('$id-mount', "fixed", parent, seat, id, connector);
	}

	/**
	 * Block `id` slides on rail `rail`: at coordinate zero it sits `at` millimetres along the rail from its start, and a
	 * growing coordinate moves it towards the rail's far end (`sense` 1) or its start (-1). Its pose, the joint's axis and
	 * the axis's travel direction all come from the rail; the axis's overtravel is the room its block has left on the rail
	 * at either end of travel, where the rail's end stops are.
	 */
	public function slideOn(spec:AxisSpec, rail:String, id:String, block:LinearRailBlock, at:Float, sense:Int = 1):Void {
		var guide = component(rail);
		if (!Std.isOfType(guide, LinearRail)) throw 'Axis ${spec.id} must slide on a rail';
		if (sense != 1 && sense != -1) throw 'Axis ${spec.id} slides towards one end of its rail or the other';
		var length = cast(guide, LinearRail).length, railSpec = cast(guide, LinearRail).spec;
		var reach = railSpec.railEndMargin + railSpec.blockLength / 2;
		var first = at + sense * spec.lower, last = at + sense * spec.upper;
		var room = Math.min(Math.min(first, last) - reach, length - reach - Math.max(first, last));
		if (room < 0) throw 'Axis ${spec.id} runs its block off its rail by ${Dimension.format(-room)} mm';
		overtravel.set(spec.id, room);
		var direction = AssemblyFrames.transformVector(zeroPose(rail), 0, 0, sense);
		axisDirections.set(spec.id, [direction.x, direction.y, direction.z]);
		// The seat is the rail's axis connector moved `at` along the rail; a connector's +Y is its axis, here the rail's length.
		var seatName = 'block-$id';
		var seatFrame = AssemblyFrames.compose(AssemblyFrames.translation(0, 0, at), guide.connector("axis").frame);
		seat(rail, seatName, seatFrame);
		addComponent(id, block);
		zeroPoses.set(id, AssemblyFrames.compose(AssemblyFrames.compose(zeroPose(rail), seatFrame),
			AssemblyFrames.inverse(block.connector("rail").frame)));
		addMateOnAxis(spec.id, "prismatic", rail, seatName, id, "rail", {x: 0, y: sense, z: 0}, spec.initial,
			{lower: spec.lower, upper: spec.upper, velocity: null, effort: null, overtravel: room, rackingTolerance: spec.rackingTolerance,
				assumptions: spec.rackingTolerance == null ? null : [{quantity: "racking tolerance", label: "stated axis design assumption"}]});
	}

	public function component(id:String):MachineComponent {
		for (entry in components()) if (entry.id == id) return entry.component;
		throw 'Axis assembly has no member "$id" yet';
	}

	/**
	 * A rail block sliding on its rail (`parent`) along a world axis; `pose` is where it sits at
	 * coordinate zero. The axis's overtravel is the room its block has left on the rail at either end
	 * of travel, where the rail's end stops are.
	 */
	public function slide(spec:AxisSpec, parent:String, id:String, component:MachineComponent, pose:AssemblyFrame,
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
		var length = Math.sqrt(axis.x * axis.x + axis.y * axis.y + axis.z * axis.z);
		if (!(length > 0)) throw 'Axis ${spec.id} needs a direction';
		axisDirections.set(spec.id, [axis.x / length, axis.y / length, axis.z / length]);
		addComponent(id, component);
		zeroPoses.set(id, pose);
		connect(parent, id);
		addMateOnAxis(spec.id, "prismatic", parent, 'to-$id', id, 'attach-$id', axis, spec.initial,
			{lower: spec.lower, upper: spec.upper, velocity: null, effort: null, overtravel: room, rackingTolerance: spec.rackingTolerance,
				assumptions: spec.rackingTolerance == null ? null : [{quantity: "racking tolerance", label: "stated axis design assumption"}]});
	}

	/**
	 * Connectors meeting at the child's origin with world-aligned axes, so joint axes are world
	 * directions. Names carry the member ids, so members that share geometry can share one
	 * definition holding all of their connectors.
	 */
	public function connect(parent:String, child:String):Void {
		var childPose = zeroPose(child);
		var meeting = AssemblyFrames.translation(childPose.x, childPose.y, childPose.z);
		addMemberConnector(parent, 'to-$child', AssemblyFrames.compose(AssemblyFrames.inverse(zeroPose(parent)), meeting));
		addMemberConnector(child, 'attach-$child', AssemblyFrames.compose(AssemblyFrames.inverse(childPose), meeting));
	}

	public function zeroPose(id:String):AssemblyFrame {
		var pose = zeroPoses.get(id);
		if (pose == null) throw 'Axis assembly has no member "$id" yet';
		return pose;
	}

	/**
	 * Motor `motor` turns lead screw `id` (at `pose`, its input end on the shaft tip, pointing
	 * along world direction `along`) through a shaft coupling. Coupling and screw turn together on
	 * a continuous joint `id-turn`, which coupling `id-lead` ties to `axis` by the screw's lead; the
	 * sense follows from the thread and from the direction the axis slides (`slide` comes first).
	 * The coupling is centred on the shaft tip and turns inside the mount's pilot bore.
	 */
	public function driveScrew(axis:AxisSpec, id:String, motor:String, screw:LeadScrew, pose:AssemblyFrame,
			along:Array<Float>, flangedNut:Bool, driverForMotor:String->String, ?coupling:ShaftCoupling):Void {
		var shaft = cast(component(motor), NemaStepper).variant.shaftDiameter;
		if (coupling == null) coupling = new ShaftCoupling(shaft, screw.thread.screwDiameter);
		if (coupling.boreA != shaft || coupling.boreB != screw.thread.screwDiameter)
			throw "Screw coupling bores must fit the motor and screw shaft";
		var grip = coupling.length / 2;
		var couplingId = id + "Coupling";
		addComponent(couplingId, coupling);
		zeroPoses.set(couplingId, {x: pose.x - along[0] * grip, y: pose.y - along[1] * grip, z: pose.z - along[2] * grip,
			qx: pose.qx, qy: pose.qy, qz: pose.qz, qw: pose.qw});
		connect(motor, couplingId);
		attach(id, screw, pose, couplingId);
		// The screw's thread sets the ratio; the joint starts where the axis puts it.
		var nut = new LeadScrewNut(screw.thread, 4, flangedNut,
			screw.thread.family == machinekit.motion.LeadScrewThread.LeadScrewThreadFamily.Ball ?
				machinekit.motion.LeadScrewNutKind.BallNut : machinekit.motion.LeadScrewNutKind.AntiBacklash);
		if (screw.thread.family == machinekit.motion.LeadScrewThread.LeadScrewThreadFamily.Ball) nut.setMaterial("steel");
		addComponent(id + "Nut", nut);
		var ratio = addTransmission('$id-lead', axis.id, '$id-turn', Transmission.LeadScrew(id, id + "Nut"),
			screwSense(axis.id, along));
		addMateOnAxis('$id-turn', "continuous", motor, 'to-$couplingId', couplingId, 'attach-$couplingId',
			{x: along[0], y: along[1], z: along[2]}, ratio * axis.initial);
		// The input is fixed and the far end free; the moving nut is no bending support. Use the whole
		// shaft as a conservative unsupported length. Ball-screw layouts must also place the physical
		// fixed bearing whose axial load path is resolved by ScrewStretch.
		supportScrew('$id-lead', Fixed, Free);
		addMotor(motor, '$id-turn', motor, driverForMotor(motor));
	}

	/** Attach the source nut to its carriage once the bracket exists. */
	public function mountNut(screw:String, parent:String, face:AssemblyFrame):Void {
		var id = screw + "Nut";
		var nut:LeadScrewNut = cast component(id);
		zeroPoses.set(id, AssemblyFrames.compose(face, AssemblyFrames.translation(0, 0, -nut.bodyLength - nut.flangeThickness)));
		connect(parent, id);
		addMate('$id-mount', "fixed", parent, 'to-$id', id, 'attach-$id');
	}

	/** A pulley follows one linear axis, or several for a CoreXY belt. */
	public function turnWithBelt(id:String, parent:String, about:Array<Float>, beltId:String,
			motions:Array<BeltAxisMotion>, defaultAngle:Null<Float>):Void {
		if (motions.length == 0) throw "A driven belt pulley needs a linear leader";
		var angle = 0.0;
		for (index in 0...motions.length) {
			var motion = motions[index];
			var ratio = addTransmission(motion.id, motion.leader, '$id-turn',
				motion.idler ? Transmission.BeltIdler(beltId, id) : Transmission.TimingBelt(beltId, id),
				SenseTools.fromAlignment(motion.rotation));
			var term = ratio * motion.initial;
			angle = index == 0 ? term : angle + term;
		}
		addMateOnAxis('$id-turn', "continuous", parent, 'to-$id', id, 'attach-$id',
			{x: about[0], y: about[1], z: about[2]}, defaultAngle == null ? angle : defaultAngle);
	}

	/**
	 * A pulley on a CoreXY belt. Its signed contribution from each leader comes from the belt's strand geometry
	 * (`TimingBelt.strands()`) and the side of the belt it wraps (`BeltWrap.side`): the gantry strand, which the
	 * carriage clamps, moves with `x`, and the strand between the frame pulleys moves with `y`. A pulley on the
	 * gantry sees only the carriage moving, so follows `x` alone; a pulley on the frame follows both. `driving` is
	 * the motor's own pulley, which carries torque; every other pulley is an idler.
	 */
	public function turnCoreXyPulley(id:String, parent:String, beltId:String, belt:TimingBelt, wrap:BeltWrap,
			onFrame:Bool, driving:Bool, ?initialX:Float = 0.0, ?initialY:Float = 0.0):Void {
		var strands = belt.strands();
		var signs = [coreXySign(strands[0].dx)];
		if (onFrame) signs.push(coreXySign(strands[1].dy));
		var axes = ["x", "y"], initials = [initialX, initialY];
		var motions:Array<BeltAxisMotion> = [];
		for (index in 0...signs.length) if (signs[index] != 0)
			motions.push({id: '$id-${axes[index]}', leader: axes[index], initial: initials[index],
				rotation: wrap.side * signs[index], idler: !driving});
		turnWithBelt(id, parent, [0.0, 0, 1], beltId, motions, null);
	}

	static function coreXySign(value:Float):Int return value > 1e-9 ? 1 : value < -1e-9 ? -1 : 0;

	/** Build the belt, driving pulley, idler and motor relationship from their actual parts. */
	public function twoPulleyAxis(layout:TwoPulleyAxis):Void {
		if (layout.beltParent == null) place(layout.beltId, layout.belt, layout.beltPose);
		else attach(layout.beltId, layout.belt, layout.beltPose, layout.beltParent);
		hang(layout.pulleyId, layout.pulley, layout.beltPose, layout.motorId);
		turnWithBelt(layout.pulleyId, layout.motorId, layout.about, layout.beltId,
			[{id: layout.pulleyId + "-belt", leader: layout.axis.id, initial: layout.axis.initial,
				rotation: layout.belt.rotation(0, layout.strand, layout.travelX, layout.travelY), idler: false}], null);
		var parent = layout.mountIdler();
		hang(layout.idlerId, layout.idler, layout.idlerPose, parent);
		turnWithBelt(layout.idlerId, parent, layout.about, layout.beltId,
			[{id: layout.idlerId + "-belt", leader: layout.axis.id, initial: layout.axis.initial,
				rotation: layout.belt.rotation(1, layout.strand, layout.travelX, layout.travelY), idler: true}], null);
		addMotor(layout.motorId, layout.pulleyId + "-turn", layout.motorId, layout.driverForMotor(layout.motorId));
	}

	/**
	 * A moving motor's pinion rolls along a rack fixed to `rackParent`. The pinion sits at `pose` with the axis at
	 * coordinate zero (its axis along the pose's +Z, its face from z = 0 to its face width), and the rack lies on
	 * `rackSide` of it. The rack's pose comes from `rackPlacement`, and the coupling's sense from the pinion rolling
	 * on it as the axis slides (`slide` comes first).
	 */
	public function driveRack(axis:AxisSpec, id:String, motor:String, pinion:SpurGear, pose:AssemblyFrame, rackSide:Array<Float>,
			rackId:String, rack:Rack, rackParent:Null<String>, driverForMotor:String->String):Void {
		var placement = rackPlacement(axis, pinion, pose, rackSide, rack);
		if (rackParent == null) place(rackId, rack, placement.pose);
		else attach(rackId, rack, placement.pose, rackParent);
		hang(id, pinion, pose, motor);
		var ratio = addTransmission('$id-rack', axis.id, '$id-turn', Transmission.RackAndPinion(id, rackId), placement.sense);
		var about = AssemblyFrames.transformVector(pose, 0, 0, 1);
		addMateOnAxis('$id-turn', "continuous", motor, 'to-$id', id, 'attach-$id', about, ratio * axis.initial);
		addMotor(motor, '$id-turn', motor, driverForMotor(motor));
	}

	/**
	 * Where a rack meshes with `pinion` (at `pose` with the axis at zero, see `driveRack`) on its `rackSide` (a direction
	 * from the pinion's axis, square to it and to the axis's travel): the rack's pitch line touches the pinion's pitch
	 * circle, its face is centred on the pinion's, a rack gap faces the pinion tooth nearest the pitch point, and the rack
	 * covers the pitch point over the whole travel with two teeth to spare at each end (or the rack is too short and
	 * this throws). `sense` is the coupling sense that makes the pinion roll on it. Machines size the rack's support from
	 * the pose before calling `driveRack`.
	 */
	public function rackPlacement(axis:AxisSpec, pinion:SpurGear, pose:AssemblyFrame, rackSide:Array<Float>, rack:Rack)
			:{pose:AssemblyFrame, sense:Sense} {
		var a = AssemblyFrames.transformVector(pose, 0, 0, 1);
		var t = axisDirection(axis.id);
		var length = Math.sqrt(rackSide[0] * rackSide[0] + rackSide[1] * rackSide[1] + rackSide[2] * rackSide[2]);
		if (!(length > 0)) throw 'Rack of axis ${axis.id} needs a side';
		var n = [rackSide[0] / length, rackSide[1] / length, rackSide[2] / length];
		if (Math.abs(dot(n, [a.x, a.y, a.z])) > 1e-9 || Math.abs(dot(n, t)) > 1e-9)
			throw 'Rack of axis ${axis.id} must lie square to its pinion\'s axis and to the travel';
		// Rack frame: face along +X (the pinion's axis), teeth along +Y (towards the pinion), length along +Z = X x Y.
		var x = [a.x, a.y, a.z], y = [-n[0], -n[1], -n[2]];
		var z = cross(x, y);
		var radius = pinion.pitchDiameter / 2, pitch = Math.PI * pinion.moduleSize;
		// Pitch point at coordinate zero, on the pinion's face start; the face of the rack centred on the pinion's.
		var faceShift = (pinion.faceWidth - rack.faceWidth) / 2;
		var contact = [pose.x + radius * n[0] + faceShift * x[0], pose.y + radius * n[1] + faceShift * x[1], pose.z + radius * n[2] + faceShift * x[2]];
		// The pinion's tooth nearest the pitch point: teeth are centred on multiples of 2 pi / teeth from its local +X.
		var local = AssemblyFrames.transformVector(AssemblyFrames.inverse(pose), n[0], n[1], n[2]);
		var angle = Math.atan2(local.y, local.x), step = 2 * Math.PI / pinion.teeth;
		var offset = radius * (angle - Math.round(angle / step) * step);
		// That tooth's centre lies `offset` behind the pitch point along the direction the pitch circle turns (a x n).
		var tangent = cross(x, n);
		var tooth = -offset * dot(tangent, z);
		// Rack gaps are centred on multiples of the pitch from its start: start the rack a whole number of pitches before
		// the tooth, as near as that comes to centring the rack on the pitch point's travel.
		var travel = dot(t, z);
		var low = Math.min(axis.lower * travel, axis.upper * travel), high = Math.max(axis.lower * travel, axis.upper * travel);
		var centred = (low + high) / 2 - rack.length / 2;
		var start = tooth - Math.round((tooth - centred) / pitch) * pitch;
		if (start > low - 2 * pitch || start + rack.length < high + 2 * pitch)
			throw 'Rack of axis ${axis.id} is too short: it needs ${Math.ceil((high - low) / pitch) + 5} teeth, has ${rack.teethCount}';
		var origin = [contact[0] + start * z[0], contact[1] + start * z[1], contact[2] + start * z[2]];
		// Rolling: the pinion's centre moves along a x n per unit turn of radius; it turns by (t . z) / radius per unit
		// travel, and Rack.relation gives alignment * 2 / pitch diameter.
		var sense = SenseTools.fromAlignment(travel > 0 ? 1 : -1);
		if (Math.abs(Math.abs(travel) - 1) > 1e-9) throw 'Rack of axis ${axis.id} must run along the travel';
		return {pose: AssemblyFrames.fromRotationMatrix(origin[0], origin[1], origin[2], [x[0], y[0], z[0], x[1], y[1], z[1], x[2], y[2], z[2]]),
			sense: sense};
	}

	/**
	 * The coupling sense of a lead screw pointing along world `along` (its +Z) that drives axis `axisId`: a screw turning by
	 * an angle against its nut moves the nut by its signed lead per turn along +Z (`LeadScrewThread.signedLead`), and
	 * `LeadScrew.relation` gives the turn per unit of travel as alignment * 2 pi / signed lead.
	 */
	public function screwSense(axisId:String, along:Array<Float>):Sense {
		var alignment = dot(axisDirection(axisId), along);
		if (Math.abs(Math.abs(alignment) - 1) > 1e-9) throw 'Screw of axis $axisId must run along the travel';
		return SenseTools.fromAlignment(alignment > 0 ? 1 : -1);
	}

	/**
	 * A home switch for axis `axis` at the end `side` (-1 lower, 1 upper) of its travel: a flag on `moving` (`size` big, its
	 * base centre at `point` with the axis at zero) presses a switch held on `fixed` as the axis comes into that end. The
	 * switch trips a quarter of the axis's overtravel beyond its travel, where its uncertainty still fits before the end
	 * stop; `direction` is the axis's travel direction (along a world axis) and `transverse` the switch's up. The switch is
	 * held by a bracket (`switchBracket`): a plate on its back and, when `fixed` is not right behind it, one leg beside it to
	 * the nearest face of `fixed` (that member's geometry box). An inductive switch senses the flag across its sensing
	 * distance, so its flag is steel; a roller switch
	 * trips at its roller. `seed` numbers switches that share an axis. `host` is the box (assembly coordinates, axes at
	 * zero) of the part of `fixed` the mount is bolted to, when that is not the whole member (one tube of a welded frame).
	 * Returns the switch's member id, `home<suffix>`.
	 */
	public function addHomeSwitch(axis:AxisSpec, suffix:String, fixed:String, moving:String, point:Array<Float>, size:Array<Float>,
			direction:Array<Float>, side:Int, transverse:Array<Float>, inductive:Bool = false, ?shaft:String, seed:Int = 1,
			?host:{min:Array<Float>, max:Array<Float>}):String {
		var room = axisOvertravel(axis.id);
		var sensor:MachineComponent = inductive ? new machinekit.motion.ProximitySwitch() : new machinekit.motion.LimitSwitch();
		var contact:machinekit.motion.SwitchPart = cast sensor;
		if (!Math.isFinite(room) || room <= 4 * contact.switchRepeatability())
			throw 'Home switch of axis ${axis.id}: its uncertainty does not fit the guide overtravel';
		var trigger = "homeTrigger" + suffix;
		var flag = new machinekit.gantry.GantryParts.GantryPlate("Home flag " + suffix, size[0], size[1], size[2]);
		if (inductive) flag.setMaterial("steel");
		attach(trigger, flag, AssemblyFrames.translation(point[0], point[1], point[2] - size[2] / 2), moving);
		var half = 0.0;
		for (i in 0...3) half += Math.abs(direction[i]) * size[i] / 2;
		addMemberConnector(trigger, "trip", AssemblyFrames.translation(direction[0] * side * half,
			direction[1] * side * half, size[2] / 2 + direction[2] * side * half));
		var travel = (side < 0 ? axis.lower : axis.upper) + side * room * 0.25;
		var normal = [for (value in direction) -side * value];
		var face = [for (i in 0...3) point[i] + direction[i] * (travel + side * half)];
		var tripLocal = if (inductive) {
			var spec = cast(sensor, machinekit.motion.ProximitySwitch).spec;
			{x: 0.0, z: spec.length + spec.sensingDistance};
		} else {
			var spec = cast(sensor, machinekit.motion.LimitSwitch).spec;
			{x: spec.width * 0.4, z: spec.depth + spec.rollerDiameter - spec.pretravel};
		}
		// orient(up = transverse, along = normal): local x is up x along.
		var localX = cross(transverse, normal);
		var origin = [for (i in 0...3) face[i] - localX[i] * tripLocal.x - normal[i] * tripLocal.z];
		var pose = orient(origin[0], origin[1], origin[2], transverse, normal);
		if (host == null) host = memberBounds(fixed);
		var switchBox = posedBounds(sensor, pose);
		var blocks = switchBracket(switchBox, normal, host);
		var corner = blocks[0].min;
		var id = "home" + suffix, mount = id + "Mount";
		attach(mount, new machinekit.structural.BlockBracket("Home switch bracket", [for (block in blocks)
			{min: [for (i in 0...3) block.min[i] - corner[i]], max: [for (i in 0...3) block.max[i] - corner[i]]}]),
			AssemblyFrames.translation(corner[0], corner[1], corner[2]), fixed);
		attach(id, sensor, pose, mount);
		addTripSwitch(id, axis.id, id, {instanceId: trigger, connectorName: "trip"}, side, "home", seed, shaft);
		return id;
	}

	/**
	 * The blocks (assembly coordinates) of a bracket that holds a part whose box is `part`, sensing along `normal` (a world
	 * axis), on a member whose box is `host`: a plate on the part's back (the side away from `normal`) and, unless the plate
	 * already meets the host, one leg to the host. The leg runs straight back from the plate when the host is right behind
	 * it; otherwise it runs beside the part (beyond its face in a sideways axis) to the host's face, so it never passes
	 * through the part. Across the leg's other axes it overlaps the host's face by up to `contact`, a face contact rather
	 * than an edge, while stopping at the face in its own axis, so it never enters the host.
	 */
	public static function switchBracket(part:{min:Array<Float>, max:Array<Float>}, normal:Array<Float>,
			host:{min:Array<Float>, max:Array<Float>}, thickness:Float = 5, contact:Float = 20):Array<{min:Array<Float>, max:Array<Float>}> {
		var axis = 0;
		for (i in 1...3) if (Math.abs(normal[i]) > Math.abs(normal[axis])) axis = i;
		if (Math.abs(Math.abs(normal[axis]) - 1) > 1e-9) throw "A switch bracket needs the switch to sense along a world axis";
		var plate = {min: part.min.copy(), max: part.max.copy()};
		if (normal[axis] > 0) {
			plate.max[axis] = part.min[axis];
			plate.min[axis] = part.min[axis] - thickness;
		} else {
			plate.min[axis] = part.max[axis];
			plate.max[axis] = part.max[axis] + thickness;
		}
		function above(i:Int):Bool return host.min[i] >= plate.max[i];
		function below(i:Int):Bool return host.max[i] <= plate.min[i];
		function gap(i:Int):Float return above(i) ? host.min[i] - plate.max[i] : below(i) ? plate.min[i] - host.max[i] : 0;
		var sideways = [for (i in 0...3) if (i != axis) i];
		var behind = normal[axis] > 0 ? below(axis) : above(axis);
		// Right behind the plate (no sideways gap): a straight standoff; otherwise a leg beside the part.
		var leg = behind && gap(sideways[0]) == 0 && gap(sideways[1]) == 0 ? axis : gap(sideways[1]) > gap(sideways[0]) ? sideways[1] : sideways[0];
		if (gap(0) == 0 && gap(1) == 0 && gap(2) == 0) {
			for (i in 0...3) if (above(i) || below(i)) return [plate];
			throw "The switch's bracket would sit inside the member it is mounted on";
		}
		var block = {min: plate.min.copy(), max: plate.max.copy()};
		// The leg overlaps the host's face across its other axes only when it stops at a face of the host in its own axis;
		// a side plate (the host spans the leg's axis) stops at the host's face in the axis that parts them instead.
		var overlap = gap(leg) > 0 ? contact : 0.0;
		for (i in 0...3) {
			if (i == leg) {
				if (above(i) && host.min[i] > plate.max[i]) {
					block.min[i] = plate.max[i];
					block.max[i] = host.min[i];
				} else if (below(i) && host.max[i] < plate.min[i]) {
					block.min[i] = host.max[i];
					block.max[i] = plate.min[i];
				} else {
					// The host spans this axis: a side plate just beyond the part's face.
					block.min[i] = plate.max[i];
					block.max[i] = plate.max[i] + thickness;
				}
			} else if (above(i)) block.max[i] = host.min[i] + Math.min(overlap, host.max[i] - host.min[i]);
			else if (below(i)) block.min[i] = host.max[i] - Math.min(overlap, host.max[i] - host.min[i]);
		}
		return [plate, block];
	}

	/** World bounds of `component` at `pose`, from its geometry. */
	static function posedBounds(component:MachineComponent, pose:AssemblyFrame):{min:Array<Float>, max:Array<Float>} {
		var posed = new PosedParts();
		try {
			var part = posed.posed(component, pose);
			var box = PosedParts.boxOf(part);
			part.close();
			posed.close();
			return {min: [box.minX, box.minY, box.minZ], max: [box.maxX, box.maxY, box.maxZ]};
		} catch (error:Dynamic) {
			posed.close();
			throw error;
		}
	}

	/** World bounds of member `id` at the zero pose, from its geometry. */
	function memberBounds(id:String):{min:Array<Float>, max:Array<Float>} {
		var posed = new PosedParts();
		try {
			var part = posed.posed(component(id), zeroPose(id));
			var box = PosedParts.boxOf(part);
			part.close();
			posed.close();
			return {min: [box.minX, box.minY, box.minZ], max: [box.maxX, box.maxY, box.maxZ]};
		} catch (error:Dynamic) {
			posed.close();
			throw error;
		}
	}

	static function dot(a:Array<Float>, b:Array<Float>):Float return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];

	static function cross(a:Array<Float>, b:Array<Float>):Array<Float>
		return [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];
}
