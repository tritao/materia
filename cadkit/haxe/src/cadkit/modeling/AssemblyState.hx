package cadkit.modeling;

import materia.project.AssemblyCodec;
import materia.project.AssemblyDefinition;
import materia.project.AssemblyDefinition.AssemblyComponentDefinition;
import materia.project.AssemblyDefinition.AssemblyJointCoordinate;
import materia.project.AssemblyDefinition.AssemblyJointRole;
import materia.project.AssemblyDefinition.AssemblyJointType;
import materia.project.AssemblyDefinition.AssemblyRootPose;
import materia.project.AssemblyDefinition.AssemblyStateRecord;
import materia.project.AssemblyDefinition.AssemblyComponentOccurrence;
import materia.project.AssemblyDefinition.KinematicJoint;
import materia.project.AssemblyDefinitionCodec;
import materia.project.AssemblyFrames;
import materia.project.AssemblyRecord.AssemblyConnector;
import materia.project.AssemblyRecord.AssemblyFrame;

typedef AssemblyClosureResidual = {
	var joint:String;
	var position:Float;
	var axis:Float;
	var rotation:Float;
}

/** Mutable configuration and derived world poses for an immutable assembly definition. */
class AssemblyState {
	public final definition:AssemblyDefinition;
	final coordinates:Map<String, Float> = [];
	final rootPoses:Map<String, AssemblyFrame> = [];
	final occurrences:Map<String, AssemblyComponentOccurrence> = [];
	final components:Map<String, AssemblyComponentDefinition> = [];
	final incoming:Map<String, KinematicJoint> = [];
	final joints:Map<String, KinematicJoint> = [];
	final poses:Map<String, AssemblyFrame> = [];
	var dirty:Bool = true;

	public function new(definition:AssemblyDefinition, ?state:AssemblyStateRecord) {
		AssemblyDefinitionCodec.validate(definition);
		this.definition = definition;
		for (component in definition.definitions) components.set(component.id, component);
		for (occurrence in definition.occurrences) occurrences.set(occurrence.id, occurrence);
		for (joint in definition.joints) {
			joints.set(joint.id, joint);
			if (joint.role == AssemblyJointRole.Tree) {
				incoming.set(joint.child, joint);
				if (AssemblyDefinitionCodec.hasCoordinate(joint.type))
					coordinates.set(joint.id, joint.defaultValue);
			}
		}
		if (state != null) {
			AssemblyDefinitionCodec.validateState(definition, state);
			for (coordinate in state.jointCoordinates) coordinates.set(coordinate.joint, coordinate.value);
			for (root in state.rootPoses) rootPoses.set(root.occurrence, root.pose);
		}
	}

	public function setJoint(id:String, value:Float):Void {
		var joint = joints.get(id);
		if (joint == null || joint.role != AssemblyJointRole.Tree ||
			!AssemblyDefinitionCodec.hasCoordinate(joint.type))
			throw 'Joint "$id" is not a driving tree joint';
		if (!Math.isFinite(value) || (joint.limits.lower != null && value < joint.limits.lower) ||
			(joint.limits.upper != null && value > joint.limits.upper))
			throw 'Joint "$id" coordinate is outside its limits';
		coordinates.set(id, value);
		dirty = true;
	}

	public function joint(id:String):Float {
		var joint = joints.get(id);
		if (joint == null || joint.role != AssemblyJointRole.Tree ||
			!AssemblyDefinitionCodec.hasCoordinate(joint.type))
			throw 'Joint "$id" is not a driving tree joint';
		var value = coordinates.get(id);
		if (value == null) throw 'Missing coordinate for joint "$id"';
		return value;
	}

	public function setRootPose(id:String, pose:AssemblyFrame):Void {
		if (!AssemblyDefinitionCodec.rootOccurrences(definition).exists(id))
			throw 'Occurrence "$id" is not a kinematic root';
		AssemblyCodec.validateFrame(pose);
		rootPoses.set(id, pose);
		dirty = true;
	}

	/** Recomputes all occurrence poses from root placements and tree-joint coordinates. */
	public function forwardKinematics():Void {
		poses.clear();
		var visiting = new Map<String, Bool>();
		for (occurrence in definition.occurrences) solveOccurrence(occurrence.id, visiting);
		dirty = false;
	}

	public function worldPose(id:String):AssemblyFrame {
		if (!occurrences.exists(id)) throw 'Missing assembly occurrence "$id"';
		if (dirty) forwardKinematics();
		return poses.get(id);
	}

	public function worldConnector(id:String, name:String):AssemblyFrame {
		var occurrence = occurrences.get(id);
		if (occurrence == null) throw 'Missing assembly occurrence "$id"';
		var component = components.get(occurrence.definition);
		if (component == null) throw 'Missing assembly component "${occurrence.definition}"';
		for (connector in component.connectors) if (connector.name == name)
			return AssemblyFrames.compose(worldPose(id), connector.frame);
		throw 'Missing assembly connector "$id/$name"';
	}

	/** Closure checks are diagnostic only; dependent coordinates need a loop solver. */
	public function closureResiduals():Array<AssemblyClosureResidual> {
		if (dirty) forwardKinematics();
		var result:Array<AssemblyClosureResidual> = [];
		for (joint in definition.joints) if (joint.role == AssemblyJointRole.Closure) {
			var first = worldConnector(joint.parent, joint.parentConnector);
			var second = worldConnector(joint.child, joint.childConnector);
			var dx = second.x - first.x, dy = second.y - first.y, dz = second.z - first.z;
			var axisFirst = AssemblyFrames.transformVector(first, joint.axis.x, joint.axis.y, joint.axis.z);
			var axisSecond = AssemblyFrames.transformVector(second, joint.axis.x, joint.axis.y, joint.axis.z);
			var axisDot = Math.abs(axisFirst.x * axisSecond.x + axisFirst.y * axisSecond.y + axisFirst.z * axisSecond.z);
			var position:Float;
			if (joint.type == AssemblyJointType.Prismatic) {
				var along = dx * axisFirst.x + dy * axisFirst.y + dz * axisFirst.z;
				var px = dx - along * axisFirst.x, py = dy - along * axisFirst.y, pz = dz - along * axisFirst.z;
				position = Math.sqrt(px * px + py * py + pz * pz);
			} else position = Math.sqrt(dx * dx + dy * dy + dz * dz);
			var rotation = 0.0;
			if (joint.type == AssemblyJointType.Fixed) {
				var dot = first.qx * second.qx + first.qy * second.qy + first.qz * second.qz + first.qw * second.qw;
				rotation = 1 - Math.abs(dot);
			}
			result.push({joint: joint.id, position: position, axis: 1 - axisDot, rotation: rotation});
		}
		return result;
	}

	public function record():AssemblyStateRecord {
		var values:Array<AssemblyJointCoordinate> = [];
		for (joint in definition.joints) if (joint.role == AssemblyJointRole.Tree &&
			AssemblyDefinitionCodec.hasCoordinate(joint.type)) {
			var value = coordinates.get(joint.id);
			if (value == null) throw 'Missing coordinate for joint "${joint.id}"';
			values.push({joint: joint.id, value: value});
		}
		var roots:Array<AssemblyRootPose> = [];
		for (occurrence in definition.occurrences) if (rootPoses.exists(occurrence.id))
			roots.push({occurrence: occurrence.id, pose: rootPoses.get(occurrence.id)});
		var result:AssemblyStateRecord = {schemaVersion: AssemblyDefinitionCodec.VERSION,
			definition: definition.id, jointCoordinates: values, rootPoses: roots};
		AssemblyDefinitionCodec.validateState(definition, result);
		return result;
	}

	function solveOccurrence(id:String, visiting:Map<String, Bool>):AssemblyFrame {
		var solved = poses.get(id);
		if (solved != null) return solved;
		if (visiting.exists(id)) throw 'Assembly kinematic tree contains a cycle at "$id"';
		visiting.set(id, true);
		var joint = incoming.get(id);
		var pose:AssemblyFrame;
		if (joint == null) {
			var occurrence = occurrences.get(id);
			if (occurrence == null) throw 'Missing assembly occurrence "$id"';
			pose = rootPoses.exists(id) ? rootPoses.get(id) : occurrence.initialPose;
		} else {
			var parentPose = solveOccurrence(joint.parent, visiting);
			var parentConnector = connector(joint.parent, joint.parentConnector);
			var childConnector = connector(joint.child, joint.childConnector);
			var parentFrame = AssemblyFrames.compose(parentPose, parentConnector.frame);
			var value = coordinates.get(joint.id);
			if (value == null) value = joint.defaultValue;
			var motion = AssemblyFrames.axisMotion(joint.type, joint.axis, value);
			pose = AssemblyFrames.compose(AssemblyFrames.compose(parentFrame, motion), AssemblyFrames.inverse(childConnector.frame));
		}
		visiting.remove(id);
		poses.set(id, pose);
		return pose;
	}

	function connector(occurrenceId:String, name:String):AssemblyConnector {
		var occurrence = occurrences.get(occurrenceId);
		if (occurrence == null) throw 'Missing assembly occurrence "$occurrenceId"';
		var component = components.get(occurrence.definition);
		if (component == null) throw 'Missing assembly component "${occurrence.definition}"';
		for (connector in component.connectors) if (connector.name == name) return connector;
		throw 'Missing assembly connector "$occurrenceId/$name"';
	}
}
