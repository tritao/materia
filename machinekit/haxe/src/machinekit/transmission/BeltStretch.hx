package machinekit.transmission;

import cadkit.modeling.AssemblyState;
import materia.assembly.AssemblyDefinition;
import materia.assembly.AssemblyDefinition.AssemblyJointType;
import materia.assembly.AssemblyFrames;
import machinekit.assembly.MachineAssemblyDescription.BeltPathRecord;
import machinekit.transmission.TimingBelt.BeltWrap;

/** Elastic energy in the two paths from an attached clamp to the held driving pulley. */
class BeltStretch {
	/** Two loaded pulley attachments, with every intermediate wrap free to turn. */
	public static function reduction(belt:TimingBelt, path:BeltPathRecord, leader:String,
			follower:String, driver:String, driven:String, mechanical:AssemblyDefinition, relation:TransmissionRelation,
			?context:BeltPoseContext):Float {
		if (context == null) context = new BeltPoseContext(mechanical);
		context.reset();
		var definition = context.definition, state = context.state;
		var posed = posedBelt(belt, path, state);
		if (Math.abs(posed.length - belt.length) > 1e-4)
			throw new TransmissionDesignError("Belt length does not match its wrap attachments");
		var first = wrapIndex(path, driver), second = wrapIndex(path, driven);
		var contact = contactDirection(belt, first, second);
		if (relation.ratio * contact < 0)
			throw new TransmissionDesignError('Belt "${path.belt}" Sense disagrees with its pulley contact sides');
		var driverSign = axisSign(definition, state, path.belt, leader, driver);
		var drivenSign = axisSign(definition, state, path.belt, follower, driven);
		relation.ratio = Math.abs(relation.ratio) * contact * driverSign * drivenSign;
		var paths = posed.freePaths(first, second);
		var radius = posed.wraps()[first].radius;
		return energy(TimingBelt.cordStiffnessPerMm(belt.beltProfile) * belt.width,
			paths, radius, -radius) / 1000;
	}

	/** Relative pulley direction about the belt normal, from its physical contact sides. */
	public static function contactDirection(belt:TimingBelt, first:Int, second:Int):Int
		return belt.wraps()[first].side * belt.wraps()[second].side;

	public static function energy(ea:Float, lengths:Array<Float>, da:Float, db:Float):Float
		return spanStiffness(ea, lengths[0]) * da * da + spanStiffness(ea, lengths[1]) * db * db;

	/** Axial spring of one free belt span, shared by clamp and rotary drives. */
	public static function spanStiffness(ea:Float, length:Float):Float {
		if (!(ea > 0) || !(length > 0) || !Math.isFinite(length))
			throw new TransmissionDesignError("Belt span needs positive cord rigidity and free length");
		return ea / length;
	}

	public static function wrapIndex(path:BeltPathRecord, member:String):Int {
		var found = -1;
		for (index in 0...path.wraps.length) if (path.wraps[index].instanceId == member) {
			if (found >= 0) throw new TransmissionDesignError('Pulley "$member" appears twice on its belt');
			found = index;
		}
		if (found < 0) throw new TransmissionDesignError('Belt "${path.belt}" has no attachment for pulley "$member"');
		return found;
	}

	/** Positive joint rotation as seen about the belt-plane normal. */
	public static function axisSign(definition:AssemblyDefinition, state:AssemblyState, belt:String,
			jointId:String, pulley:String):Float {
		var axis:Null<materia.assembly.AssemblyDefinition.KinematicJoint> = null;
		for (joint in definition.joints) if (joint.id == jointId) axis = joint;
		if (axis == null || (axis.type != AssemblyJointType.Revolute && axis.type != AssemblyJointType.Continuous))
			throw new TransmissionDesignError('Belt pulley "$pulley" needs a rotary joint "$jointId"');
		var member = pulley, found = member == axis.child;
		for (_ in 0...definition.joints.length) {
			if (found) break;
			var parent:Null<String> = null;
			for (joint in definition.joints) if (joint.child == member && joint.type == AssemblyJointType.Fixed) parent = joint.parent;
			if (parent == null) break;
			member = parent; found = member == axis.child;
		}
		if (!found) throw new TransmissionDesignError('Pulley "$pulley" does not turn with joint "$jointId"');
		var world = AssemblyFrames.transformVector(state.worldConnector(axis.parent, axis.parentConnector), axis.axis.x, axis.axis.y, axis.axis.z);
		var local = AssemblyFrames.transformVector(AssemblyFrames.inverse(state.worldPose(belt)), world.x, world.y, world.z);
		var centre = state.worldPose(pulley), origin = state.worldConnector(axis.parent, axis.parentConnector);
		var offset = AssemblyFrames.transformVector(AssemblyFrames.inverse(state.worldPose(belt)), centre.x - origin.x, centre.y - origin.y, centre.z - origin.z);
		if (Math.abs(offset.x) > 1e-5 || Math.abs(offset.y) > 1e-5)
			throw new TransmissionDesignError('Pulley "$pulley" is eccentric to joint "$jointId"; align its shaft centre');
		if (Math.abs(local.x) > 1e-5 || Math.abs(local.y) > 1e-5 || Math.abs(local.z) < 1e-5)
			throw new TransmissionDesignError('Pulley "$pulley" joint axis is not normal to its belt');
		return local.z > 0 ? 1.0 : -1.0;
	}

	public static function stiffness(belt:TimingBelt, path:BeltPathRecord, leader:String,
			pulley:String, mechanical:AssemblyDefinition, ?context:BeltPoseContext):Float {
		if (context == null) context = new BeltPoseContext(mechanical);
		context.reset();
		var definition = context.definition;
		var axis:Null<materia.assembly.AssemblyDefinition.KinematicJoint> = null;
		for (joint in definition.joints) if (joint.id == leader) axis = joint;
		if (axis == null || axis.type != AssemblyJointType.Prismatic)
			throw new TransmissionDesignError('Belt "$path.belt" needs a sliding leader; attach its clamp to the axis');
		var anchor = -1;
		for (index in 0...path.wraps.length) if (path.wraps[index].instanceId == pulley) anchor = index;
		if (anchor < 0 || path.wraps.length != belt.wraps().length)
			throw new TransmissionDesignError('Belt "${path.belt}" needs one attachment per wrap, including driving pulley "$pulley"');
		var state = context.state;
		var neutral = posedBelt(belt, path, state);
		if (Math.abs(neutral.length - belt.length) > 1e-4) throw new TransmissionDesignError(
			'Belt "${path.belt}" length does not match its wraps; update the belt path or pulley attachments');
		var heldPhase = neutral.anchorPhase(anchor);
		var baseline = lengths(neutral, path, state, anchor, heldPhase);
		var sampled:Array<{id:String, lower:Float, upper:Float, zero:Float}> = [];
		for (joint in definition.joints) if (joint.role == materia.assembly.AssemblyDefinition.AssemblyJointRole.Tree &&
				joint.type == AssemblyJointType.Prismatic) {
			var source:Null<materia.assembly.AssemblyDefinition.KinematicJoint> = null;
			for (candidate in mechanical.joints) if (candidate.id == joint.id) source = candidate;
			var lower = source == null ? null : source.limits.lower;
			var upper = source == null ? null : source.limits.upper;
			var moves = joint.id == leader;
			for (position in [lower == null ? joint.defaultValue - 1 : lower,
					upper == null ? joint.defaultValue + 1 : upper]) {
				context.reset();
				state.setJoint(joint.id, position);
				var moved = lengths(posedBelt(belt, path, state), path, state, anchor, heldPhase);
				if (Math.abs(moved[0] - baseline[0]) > 1e-5 || Math.abs(moved[1] - baseline[1]) > 1e-5) moves = true;
			}
			if (moves) {
				if (lower == null || upper == null)
					throw new TransmissionDesignError('Belt "${path.belt}" has no known weakest stiffness without limits for joint "${joint.id}"');
				sampled.push({id: joint.id, lower: lower, upper: upper, zero: joint.defaultValue});
			}
		}
		if (sampled.length > 5)
			throw new TransmissionDesignError('Belt "${path.belt}" spans too many moving axes for the sampled stiffness envelope');
		var others = [for (joint in sampled) if (joint.id != leader) joint];
		var poses:Array<Array<Float>> = [[]];
		for (joint in others) {
			var next:Array<Array<Float>> = [];
			for (pose in poses) for (position in [joint.lower, joint.zero, joint.upper])
				next.push(pose.concat([position]));
			poses = next;
		}
		var lead = [for (joint in sampled) if (joint.id == leader) joint][0];
		var weakest = Math.POSITIVE_INFINITY;
		var epsilon = 0.001;
		for (pose in poses) {
			context.reset();
			for (index in 0...others.length) state.setJoint(others[index].id, pose[index]);
			var positions = [lead.lower, lead.zero, lead.upper];
			var at = 0;
			while (at < positions.length) {
				var position = positions[at++];
				state.setJoint(leader, position);
				var current = lengths(posedBelt(belt, path, state), path, state, anchor, heldPhase);
				state.setJoint(leader, position + epsilon);
				var plus = lengths(posedBelt(belt, path, state), path, state, anchor, heldPhase);
				state.setJoint(leader, position - epsilon);
				var minus = lengths(posedBelt(belt, path, state), path, state, anchor, heldPhase);
				var da = (plus[0] - minus[0]) / (2 * epsilon);
				var db = (plus[1] - minus[1]) / (2 * epsilon);
				if (at == 2 && Math.abs(da + db) < 1e-5 && Math.abs(da - db) > 1e-6) {
					var balanced = position + (current[1] - current[0]) / (da - db);
					if (balanced > lead.lower && balanced < lead.upper) positions.push(balanced);
				}
				weakest = Math.min(weakest,
					energy(TimingBelt.cordStiffnessPerMm(belt.beltProfile) * belt.width, current, da, db));
			}
		}
		if (!(weakest > 0) || !Math.isFinite(weakest))
			throw new TransmissionDesignError('Belt "${path.belt}" clamp does not resist "$leader"; update its clamp and wrap attachments');
		return weakest;
	}

	public static function posedBelt(belt:TimingBelt, path:BeltPathRecord, state:AssemblyState):TimingBelt {
		var inverse = AssemblyFrames.inverse(state.worldPose(path.belt));
		var original = belt.wraps();
		if (path.wraps == null || path.wraps.length != original.length)
			throw new TransmissionDesignError('Belt "${path.belt}" path has ${path.wraps == null ? 0 : path.wraps.length} wrap attachments for ${original.length} wraps');
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
		var clamp = path.clamp;
		if (clamp == null) throw new TransmissionDesignError("Carriage belt needs a clamp attachment");
		var world = state.worldConnector(clamp.instanceId, clamp.connectorName);
		var point = AssemblyFrames.transformPoint(AssemblyFrames.inverse(state.worldPose(path.belt)), world.x, world.y, world.z);
		if (Math.abs(point.z) > 1e-5) throw new TransmissionDesignError("Belt clamp leaves its plane; align its attachment with axis travel");
		var difference = belt.clampDistanceAt(point.x, point.y) - belt.anchorDistance(anchor, phase);
		var a = difference - belt.length * Math.floor(difference / belt.length);
		var b = belt.length - a;
		if (!(a > 1e-6 && b > 1e-6))
			throw new TransmissionDesignError('Belt "${path.belt}" clamp meets its drive anchor; give both elastic paths a free length');
		return [a, b];
	}
}
