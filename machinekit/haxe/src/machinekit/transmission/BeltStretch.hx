package machinekit.transmission;

import cadkit.modeling.AssemblyState;
import haxeon.wire.JsonWire;
import materia.assembly.AssemblyDefinition;
import materia.assembly.AssemblyDefinition.AssemblyJointType;
import materia.assembly.AssemblyFrames;
import machinekit.assembly.MachineAssemblyDescription.BeltPathRecord;
import machinekit.transmission.TimingBelt.BeltWrap;

/** Elastic energy in the two paths from an attached clamp to the held driving pulley. */
class BeltStretch {
	public static function stiffness(belt:TimingBelt, path:BeltPathRecord, leader:String,
			pulley:String, mechanical:AssemblyDefinition):Float {
		var definition:AssemblyDefinition = JsonWire.decode(JsonWire.encode(mechanical));
		definition.couplings = [];
		definition.actuators = [];
		definition.encoders = [];
		var axis:Null<materia.assembly.AssemblyDefinition.KinematicJoint> = null;
		for (joint in definition.joints) if (joint.id == leader) axis = joint;
		if (axis == null || axis.type != AssemblyJointType.Prismatic)
			throw new TransmissionDesignError('Belt "$path.belt" needs a sliding leader; attach its clamp to the axis');
		var lower = axis.limits.lower, upper = axis.limits.upper;
		var positions = [axis.defaultValue];
		if (axis.limits.lower != null) positions.push(axis.limits.lower);
		if (axis.limits.upper != null) positions.push(axis.limits.upper);
		for (joint in definition.joints) {
			joint.limits.lower = null;
			joint.limits.upper = null;
		}
		var anchor = -1;
		for (index in 0...path.wraps.length) if (path.wraps[index].instanceId == pulley) anchor = index;
		if (anchor < 0 || path.wraps.length != belt.wraps().length)
			throw new TransmissionDesignError('Belt "${path.belt}" needs one attachment per wrap, including driving pulley "$pulley"');
		var state = new AssemblyState(definition);
		var neutral = posedBelt(belt, path, state);
		if (Math.abs(neutral.length - belt.length) > 1e-4) throw new TransmissionDesignError(
			'Belt "${path.belt}" length does not match its wraps; update the belt path or pulley attachments');
		var weakest = Math.POSITIVE_INFINITY;
		var epsilon = 0.001;
		var heldPhase = neutral.anchorPhase(anchor);
		var consideredInterior = false;
		for (position in positions) {
			state.setJoint(leader, position);
			var posed = posedBelt(belt, path, state);
			var phase = heldPhase;
			var current = lengths(posed, path, state, anchor, phase);
			state.setJoint(leader, position + epsilon);
			var plus = lengths(posedBelt(belt, path, state), path, state, anchor, phase);
			state.setJoint(leader, position - epsilon);
			var minus = lengths(posedBelt(belt, path, state), path, state, anchor, phase);
			var da = (plus[0] - minus[0]) / (2 * epsilon);
			var db = (plus[1] - minus[1]) / (2 * epsilon);
			// A constant-length loop is weakest when its two elastic paths have equal lengths.
			if (!consideredInterior && Math.abs(da + db) < 1e-5 && Math.abs(da - db) > 1e-6) {
				consideredInterior = true;
				var balanced = position + (current[1] - current[0]) / (da - db);
				if (lower != null && upper != null && balanced > lower && balanced < upper)
					positions.push(balanced);
			}
			var ea = TimingBelt.cordStiffnessPerMm(belt.beltProfile) * belt.width;
			var stiffness = ea * (da * da / current[0] + db * db / current[1]);
			weakest = Math.min(weakest, stiffness);
		}
		if (!(weakest > 0) || !Math.isFinite(weakest))
			throw new TransmissionDesignError('Belt "${path.belt}" clamp does not resist "$leader"; update its clamp and wrap attachments');
		return weakest;
	}

	static function posedBelt(belt:TimingBelt, path:BeltPathRecord, state:AssemblyState):TimingBelt {
		var inverse = AssemblyFrames.inverse(state.worldPose(path.belt));
		var original = belt.wraps();
		var wraps:Array<BeltWrap> = [];
		for (index in 0...path.wraps.length) {
			var reference = path.wraps[index];
			var world = state.worldConnector(reference.instanceId, reference.connectorName);
			var point = AssemblyFrames.transformPoint(inverse, world.x, world.y, world.z);
			if (Math.abs(point.z) > 1e-5) throw new TransmissionDesignError("Belt wrap leaves its plane; align its pulley attachment");
			wraps.push(new BeltWrap(point.x, point.y, original[index].radius, original[index].side));
		}
		try return new TimingBelt(belt.beltProfile, belt.width, wraps) catch (error:Dynamic)
			throw new TransmissionDesignError('Belt "${path.belt}" wrap geometry is incompatible; update its pulley attachments: $error');
	}

	static function lengths(belt:TimingBelt, path:BeltPathRecord, state:AssemblyState,
			anchor:Int, phase:Float):Array<Float> {
		var world = state.worldConnector(path.clamp.instanceId, path.clamp.connectorName);
		var point = AssemblyFrames.transformPoint(AssemblyFrames.inverse(state.worldPose(path.belt)), world.x, world.y, world.z);
		if (Math.abs(point.z) > 1e-5) throw new TransmissionDesignError("Belt clamp leaves its plane; align its attachment with axis travel");
		var difference = belt.clampDistance(path.strand, point.x, point.y) - belt.anchorDistance(anchor, phase);
		var a = difference - belt.length * Math.floor(difference / belt.length);
		var b = belt.length - a;
		if (!(a > 1e-6 && b > 1e-6))
			throw new TransmissionDesignError('Belt "${path.belt}" clamp meets its drive anchor; give both elastic paths a free length');
		return [a, b];
	}
}
