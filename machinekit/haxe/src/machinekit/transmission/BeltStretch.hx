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
		return energy(belt.cordRigidityPerWidth() * belt.width,
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

	/** Exact affine span geometry for straight Cartesian clamp travel. Unsupported changing wrap geometry is refused. */
	public static function law(belt:TimingBelt, path:BeltPathRecord, leader:String, pulley:String,
			mechanical:AssemblyDefinition, ?context:BeltPoseContext):Array<materia.assembly.AssemblyDefinition.AssemblyComplianceSpring> {
		if (context == null) context = new BeltPoseContext(mechanical);
		context.reset();
		var state = context.state, anchor = wrapIndex(path,pulley), neutral = posedBelt(belt,path,state);
		if (Math.abs(neutral.length - belt.length) > 1e-4)
			throw new TransmissionDesignError("Belt length does not match its wrap attachments");
		var phase = neutral.anchorPhase(anchor), baseline = lengths(neutral,path,state,anchor,phase);
		var gradients:Array<{joint:String,zero:Float,lower:Float,upper:Float,a:Float,b:Float}> = [];
		for (joint in mechanical.joints) if (joint.role == materia.assembly.AssemblyDefinition.AssemblyJointRole.Tree &&
				joint.type == AssemblyJointType.Prismatic) {
			context.reset(); state.setJoint(joint.id,joint.defaultValue+1.0);
			var plus = lengths(posedBelt(belt,path,state),path,state,anchor,phase);
			context.reset(); state.setJoint(joint.id,joint.defaultValue-1.0);
			var minus = lengths(posedBelt(belt,path,state),path,state,anchor,phase);
			var a = (plus[0]-minus[0])*0.5, b = (plus[1]-minus[1])*0.5;
			if (Math.abs(a)<1e-7 && Math.abs(b)<1e-7) continue;
			if (Math.abs(a-Math.round(a))>1e-7 || Math.abs(b-Math.round(b))>1e-7)
				throw new TransmissionDesignError('Moving belt spans need affine Cartesian geometry for joint "${joint.id}"');
			if (joint.limits.lower == null || joint.limits.upper == null)
				throw new TransmissionDesignError('Cannot certify belt stiffness without limits for joint "${joint.id}"');
			var overtravel = joint.limits.overtravel == null ? 0.0 : joint.limits.overtravel;
			gradients.push({joint:joint.id,zero:joint.defaultValue,lower:joint.limits.lower-overtravel,upper:joint.limits.upper+overtravel,
				a:Math.round(a),b:Math.round(b)});
		}
		if (gradients.length>5) throw new TransmissionDesignError("Belt span law has too many moving coordinates");
		var poses:Array<Array<Float>> = [[]];
		for (coordinate in gradients) {
			var next:Array<Array<Float>> = [];
			for (pose in poses) for (q in [coordinate.lower,(coordinate.lower+coordinate.upper)*0.5,coordinate.upper]) next.push(pose.concat([q]));
			poses = next;
		}
		for (pose in poses) {
			context.reset(); var expected = baseline.copy();
			for (index in 0...gradients.length) {
				var coordinate = gradients[index]; state.setJoint(coordinate.joint,pose[index]);
				expected[0] += coordinate.a*(pose[index]-coordinate.zero); expected[1] += coordinate.b*(pose[index]-coordinate.zero);
			}
			var actual = lengths(posedBelt(belt,path,state),path,state,anchor,phase);
			for (index in 0...2) if (Math.abs(actual[index]-expected[index])>1e-6)
				throw new TransmissionDesignError("Belt free paths are not affine over their declared travel");
		}
		var derivative = [0.0,0.0];
		for (coordinate in gradients) if (coordinate.joint == leader) derivative = [coordinate.a,coordinate.b];
		var ea = belt.cordRigidityPerWidth()*belt.width;
		var result:Array<materia.assembly.AssemblyDefinition.AssemblyComplianceSpring> = [];
		for (index in 0...2) if (derivative[index] != 0.0) {
			var scale = ea*derivative[index]*derivative[index], constant = baseline[index];
			var terms:Array<materia.assembly.AssemblyDefinition.AssemblyComplianceCoordinate> = [];
			for (coordinate in gradients) {
				var gradient = index == 0 ? coordinate.a : coordinate.b;
				constant -= gradient*coordinate.zero;
				if (gradient != 0.0) terms.push({joint:coordinate.joint,coefficient:gradient/scale});
			}
			result.push({compliance:constant/scale,coordinates:terms});
		}
		context.reset();
		if (result.length == 0) throw new TransmissionDesignError("The belt's spans do not resist its carriage");
		return result;
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

	/** Certified weakest stiffness over the declared travel, from the same law used by physics. */
	public static function stiffness(belt:TimingBelt, path:BeltPathRecord, leader:String,
			pulley:String, mechanical:AssemblyDefinition, ?context:BeltPoseContext):Float {
		var response = new materia.assembly.PositionStiffness(law(belt,path,leader,pulley,mechanical,context));
		return response.range(id -> {
			for (joint in mechanical.joints) if (joint.id == id) {
				var lower = joint.limits.lower, upper = joint.limits.upper;
				if (lower == null || upper == null) throw new TransmissionDesignError("Belt stiffness needs bounded physical travel");
				var overtravel = joint.limits.overtravel == null ? 0.0 : joint.limits.overtravel;
				return {lower:lower-overtravel,upper:upper+overtravel};
			}
			throw new TransmissionDesignError('Missing belt stiffness coordinate "$id"');
		}).minimum;
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
		try return new TimingBelt(belt.beltProfile, belt.width, wraps, null, belt.statedCordRigidity, belt.statedDampingRatio) catch (error:Dynamic)
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
