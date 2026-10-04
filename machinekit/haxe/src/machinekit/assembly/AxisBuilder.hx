package machinekit.assembly;

import machinekit.component.MachineComponent;
import machinekit.component.Dimension;
import machinekit.motion.LinearRail;
import machinekit.motion.LeadScrew;
import machinekit.motion.LeadScrewNut;
import machinekit.motion.NemaStepper;
import machinekit.motion.ShaftCoupling;
import machinekit.motion.ScrewSupport;
import machinekit.transmission.TimingBelt;
import machinekit.transmission.TimingPulley;
import machinekit.transmission.Rack;
import machinekit.transmission.SpurGear;
import machinekit.assembly.Sense.SenseTools;
import materia.assembly.AssemblyFrames;
import materia.assembly.AssemblyRecord.AssemblyFrame;

/** Travel and initial coordinate of a linear joint, in assembly units. */
typedef AxisSpec = {id:String, lower:Float, upper:Float, initial:Float};

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

	public function new() { super(); }

	/** Room past the travel of axis `id` before its rail blocks reach the rail ends, in millimetres. */
	public function axisOvertravel(id:String):Float {
		var room = overtravel.get(id);
		if (room == null) throw 'Axis assembly has no axis "$id"';
		return room;
	}

	/** Frame whose local +Y points along `up` and local +Z along `along`. */
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
	 * a continuous joint `id-turn`, which coupling `id-lead` ties to `axis` (moving along world
	 * `axisDirection`) by the screw's lead. The coupling is centred on the shaft tip and turns
	 * inside the mount's pilot bore.
	 */
	public function driveScrew(axis:AxisSpec, id:String, motor:String, screw:LeadScrew, pose:AssemblyFrame,
			along:Array<Float>, axisDirection:Array<Float>, flangedNut:Bool, driverForMotor:String->String):Void {
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
		addComponent(id + "Nut", new LeadScrewNut(screw.thread, 4, flangedNut));
		var ratio = addTransmission('$id-lead', axis.id, '$id-turn', Transmission.LeadScrew(id, id + "Nut"),
			SenseTools.fromAlignment(alongAxis));
		addMateOnAxis('$id-turn', "continuous", motor, 'to-$couplingId', couplingId, 'attach-$couplingId',
			{x: along[0], y: along[1], z: along[2]}, ratio * axis.initial);
		// The motor holds the screw's input end through the coupling. Nothing holds the far end, and the
		// nut floats on the carriage, so it is no support: fixed at the motor, free at the far end, over
		// the whole screw. That is what sets the screw's top speed.
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

	/** A moving motor's pinion rolls along a rack fixed to the frame. */
	public function driveRack(axis:AxisSpec, id:String, motor:String, pinion:SpurGear, pose:AssemblyFrame,
			about:Array<Float>, rackId:String, rack:Rack, rackPose:AssemblyFrame, rackParent:Null<String>,
			alignment:Float, driverForMotor:String->String):Void {
		if (rackParent == null) place(rackId, rack, rackPose);
		else attach(rackId, rack, rackPose, rackParent);
		hang(id, pinion, pose, motor);
		var ratio = addTransmission('$id-rack', axis.id, '$id-turn', Transmission.RackAndPinion(id, rackId),
			SenseTools.fromAlignment(alignment));
		addMateOnAxis('$id-turn', "continuous", motor, 'to-$id', id, 'attach-$id',
			{x: about[0], y: about[1], z: about[2]}, ratio * axis.initial);
		addMotor(motor, '$id-turn', motor, driverForMotor(motor));
	}
}
