package cadkit.modeling;

import materia.project.AssemblyRecord;
import materia.project.AssemblyCodec;
import materia.project.AssemblyFrames;
import materia.project.AssemblyRecord.AssemblyFrame;
import materia.project.AssemblyRecord.AssemblyConnector;
import materia.project.AssemblyRecord.AssemblyInstance;
import materia.project.AssemblyRecord.AssemblyJoint;
import materia.project.AssemblyDefinition;
import materia.project.AssemblyDefinition.AssemblyJointLimits;
import materia.project.AssemblyDefinition.AssemblyJointRole;
import materia.project.AssemblyDefinition.AssemblyVector;
import materia.project.AssemblyDefinitionCodec;

/** Builds a posed assembly from part instances, connector frames, and tree joints. */
class AssemblyModel {
	final instances:Array<AssemblyInstance> = [];
	final joints:Array<AssemblyJoint> = [];
	final byId:Map<String, AssemblyInstance> = [];
	final attached:Map<String, Bool> = [];
	final jointRoles:Map<String, AssemblyJointRole> = [];
	final jointAxes:Map<String, AssemblyVector> = [];
	final jointLimits:Map<String, AssemblyJointLimits> = [];

	public function new() {}

	public function add(id:String, ?pose:AssemblyFrame):Void {
		if (id == null || id.length == 0 || byId.exists(id)) throw 'Duplicate assembly instance "$id"';
		var instance:AssemblyInstance = {id: id, pose: pose == null ? AssemblyFrames.identity() : pose,
			connectors: []};
		AssemblyCodec.validateFrame(instance.pose);
		instances.push(instance);
		byId.set(id, instance);
	}

	public function connector(id:String, name:String, frame:AssemblyFrame):Void {
		var instance = require(id);
		if (name == null || name.length == 0) throw "Assembly connector needs a name";
		for (existing in instance.connectors) if (existing.name == name)
			throw 'Duplicate connector "$id/$name"';
		AssemblyCodec.validateFrame(frame);
		instance.connectors.push({name: name, frame: frame});
	}

	public function pose(id:String):AssemblyFrame return require(id).pose;

	public function worldPoint(id:String, connectorName:String):{x:Float, y:Float, z:Float} {
		var frame = requireConnector(id, connectorName);
		return AssemblyFrames.transformPoint(require(id).pose, frame.x, frame.y, frame.z);
	}

	public function place(id:String, pose:AssemblyFrame):Void {
		if (attached.exists(id)) throw 'Joint-owned instance "$id" cannot be placed independently';
		AssemblyCodec.validateFrame(pose);
		require(id).pose = pose;
	}

	/** The child pose makes its named connector coincide with the parent joint frame. */
	public function mate(id:String, kind:String, parent:String, parentConnector:String,
			child:String, childConnector:String, value:Float = 0):Void {
		mateOnAxis(id, kind, parent, parentConnector, child, childConnector,
			{x: 0, y: 1, z: 0}, value);
	}

	/** Creates a tree joint with an explicit unit axis in the parent connector frame. */
	public function mateOnAxis(id:String, kind:String, parent:String, parentConnector:String,
			child:String, childConnector:String, axis:AssemblyVector, value:Float = 0,
			?limits:AssemblyJointLimits):Void {
		if (attached.exists(child)) throw 'Instance "$child" already has a parent joint';
		if (!validJointKind(kind))
			throw 'Unsupported assembly joint "$kind"';
		if (!Math.isFinite(value)) throw "Assembly joint value must be finite";
		if (kind == "fixed" && value != 0) throw "Fixed assembly joints do not have a coordinate";
		validateAxis(axis);
		validateLimits(limits, value);
		var parentPose = require(parent).pose;
		var parentFrame = requireConnector(parent, parentConnector);
		var childFrame = requireConnector(child, childConnector);
		var motion = AssemblyFrames.axisMotion(cast kind, axis, value);
		var worldJoint = AssemblyFrames.compose(AssemblyFrames.compose(parentPose, parentFrame), motion);
		var childPose = AssemblyFrames.compose(worldJoint, AssemblyFrames.inverse(childFrame));
		AssemblyCodec.validateFrame(childPose);
		require(child).pose = childPose;
		attached.set(child, true);
		joints.push({id: id, kind: kind, parent: parent, parentConnector: parentConnector,
			child: child, childConnector: childConnector, value: value});
		jointRoles.set(id, AssemblyJointRole.Tree);
		jointAxes.set(id, axis);
		jointLimits.set(id, limits == null ? noLimits() : limits);
	}

	/** Records a closure between already placed instances after checking its pin centers. */
	public function constrain(id:String, kind:String, parent:String, parentConnector:String,
			child:String, childConnector:String, tolerance:Float = 0.001):Void {
		constrainOnAxis(id, kind, parent, parentConnector, child, childConnector,
			{x: 0, y: 1, z: 0}, tolerance);
	}

	/** Records a closure using its explicit unit axis in the parent connector frame. */
	public function constrainOnAxis(id:String, kind:String, parent:String, parentConnector:String,
			child:String, childConnector:String, axis:AssemblyVector, tolerance:Float = 0.001,
			?limits:AssemblyJointLimits):Void {
		if (!validJointKind(kind))
			throw 'Unsupported assembly joint "$kind"';
		validateAxis(axis);
		if (!Math.isFinite(tolerance) || tolerance < 0) throw "Assembly tolerance must be finite and nonnegative";
		validateLimits(limits, 0);
		var first = worldPoint(parent, parentConnector), second = worldPoint(child, childConnector);
		var dx = first.x - second.x, dy = first.y - second.y, dz = first.z - second.z;
		var firstFrame = AssemblyFrames.compose(require(parent).pose, requireConnector(parent, parentConnector));
		var secondFrame = AssemblyFrames.compose(require(child).pose, requireConnector(child, childConnector));
		var positionError = Math.sqrt(dx * dx + dy * dy + dz * dz);
		if (kind == "prismatic") {
			var worldAxis = AssemblyFrames.transformVector(firstFrame, axis.x, axis.y, axis.z);
			var along = dx * worldAxis.x + dy * worldAxis.y + dz * worldAxis.z;
			var px = dx - along * worldAxis.x, py = dy - along * worldAxis.y, pz = dz - along * worldAxis.z;
			positionError = Math.sqrt(px * px + py * py + pz * pz);
		}
		if (positionError > tolerance)
			throw 'Assembly joint "$id" has separated connectors';
		if (kind == "fixed") {
			var dot = firstFrame.qx * secondFrame.qx + firstFrame.qy * secondFrame.qy +
				firstFrame.qz * secondFrame.qz + firstFrame.qw * secondFrame.qw;
			if (1 - Math.abs(dot) > 1e-5) throw 'Assembly fixed joint "$id" has misaligned frames';
		} else {
			var firstAxis = AssemblyFrames.transformVector(firstFrame, axis.x, axis.y, axis.z);
			var secondAxis = AssemblyFrames.transformVector(secondFrame, axis.x, axis.y, axis.z);
			var ax = firstAxis.x, ay = firstAxis.y, az = firstAxis.z;
			var bx = secondAxis.x, by = secondAxis.y, bz = secondAxis.z;
			if (1 - Math.abs(ax * bx + ay * by + az * bz) > 1e-5)
				throw 'Assembly joint "$id" has misaligned axes';
		}
		joints.push({id: id, kind: kind, parent: parent, parentConnector: parentConnector,
			child: child, childConnector: childConnector, value: 0});
		jointRoles.set(id, AssemblyJointRole.Closure);
		jointAxes.set(id, axis);
		jointLimits.set(id, limits == null ? noLimits() : limits);
	}

	public function record():AssemblyRecord {
		var result:AssemblyRecord = {instances: instances, joints: joints};
		AssemblyCodec.validate(result);
		return result;
	}

	/** Exports reusable definitions and explicit tree/closure semantics. */
	public function definition(id:String = "assembly"):AssemblyDefinition {
		var legacy = AssemblyDefinitionCodec.fromLegacy(record(), id);
		for (joint in legacy.joints) {
			var role = jointRoles.get(joint.id), axis = jointAxes.get(joint.id), limits = jointLimits.get(joint.id);
			if (role != null) joint.role = role;
			if (axis != null) joint.axis = axis;
			if (limits != null) joint.limits = limits;
		}
		AssemblyDefinitionCodec.validate(legacy);
		return legacy;
	}

	public function initialState(?definitionId:String):AssemblyState
		return new AssemblyState(definition(definitionId == null ? "assembly" : definitionId));

	function require(id:String):AssemblyInstance {
		var instance = byId.get(id);
		if (instance == null) throw 'Missing assembly instance "$id"';
		return instance;
	}
	function requireConnector(id:String, name:String):AssemblyFrame {
		for (connector in require(id).connectors) if (connector.name == name) return connector.frame;
		throw 'Missing assembly connector "$id/$name"';
	}

	static function validJointKind(kind:String):Bool
		return kind == "fixed" || kind == "revolute" || kind == "continuous" || kind == "prismatic";

	static function validateAxis(axis:AssemblyVector):Void {
		if (axis == null || !Math.isFinite(axis.x) || !Math.isFinite(axis.y) || !Math.isFinite(axis.z) ||
			Math.abs(Math.sqrt(axis.x * axis.x + axis.y * axis.y + axis.z * axis.z) - 1) > 1e-5)
			throw "Assembly joint axis must be a finite unit vector";
	}

	static function validateLimits(limits:Null<AssemblyJointLimits>, value:Float):Void {
		if (limits == null) return;
		if ((limits.lower != null && !Math.isFinite(limits.lower)) ||
			(limits.upper != null && !Math.isFinite(limits.upper)) ||
			(limits.velocity != null && (!Math.isFinite(limits.velocity) || limits.velocity < 0)) ||
			(limits.effort != null && (!Math.isFinite(limits.effort) || limits.effort < 0)) ||
			(limits.lower != null && limits.upper != null && limits.lower > limits.upper) ||
			(limits.lower != null && value < limits.lower) || (limits.upper != null && value > limits.upper))
			throw "Assembly joint limits or default value are invalid";
	}

	static function noLimits():AssemblyJointLimits
		return {lower: null, upper: null, velocity: null, effort: null};
}
