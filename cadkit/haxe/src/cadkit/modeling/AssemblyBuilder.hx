package cadkit.modeling;

import materia.project.AssemblyDefinition;
import materia.project.AssemblyDefinition.AssemblyComponentDefinition;
import materia.project.AssemblyDefinition.AssemblyComponentOccurrence;
import materia.project.AssemblyDefinition.AssemblyJointLimits;
import materia.project.AssemblyDefinition.AssemblyJointRole;
import materia.project.AssemblyDefinition.AssemblyJointType;
import materia.project.AssemblyDefinition.AssemblyVector;
import materia.project.AssemblyDefinition.KinematicJoint;
import materia.project.AssemblyCodec;
import materia.project.AssemblyDefinitionCodec;
import materia.project.AssemblyFrames;
import materia.project.AssemblyRecord.AssemblyConnector;
import materia.project.AssemblyRecord.AssemblyFrame;

/** Authors reusable component definitions, occurrences, and semantic joints. */
class AssemblyBuilder {
	final id:String;
	final definitions:Array<AssemblyComponentDefinition> = [];
	final occurrences:Array<AssemblyComponentOccurrence> = [];
	final joints:Array<KinematicJoint> = [];
	final byDefinition:Map<String, AssemblyComponentDefinition> = [];
	final byOccurrence:Map<String, AssemblyComponentOccurrence> = [];
	final byJoint:Map<String, Bool> = [];

	public function new(id:String) {
		if (id == null || StringTools.trim(id).length == 0) throw "Assembly needs an ID";
		this.id = id;
	}

	public function component(id:String):Void {
		if (id == null || StringTools.trim(id).length == 0 || byDefinition.exists(id))
			throw 'Duplicate or empty component definition "$id"';
		var definition:AssemblyComponentDefinition = {id: id, connectors: []};
		definitions.push(definition);
		byDefinition.set(id, definition);
	}

	public function connector(definition:String, name:String, frame:AssemblyFrame):Void {
		var component = requireDefinition(definition);
		if (name == null || StringTools.trim(name).length == 0) throw "Assembly connector needs a name";
		for (existing in component.connectors) if (existing.name == name)
			throw 'Duplicate connector "$definition/$name"';
		AssemblyCodec.validateFrame(frame);
		component.connectors.push({name: name, frame: frame});
	}

	public function instance(id:String, definition:String, ?initialPose:AssemblyFrame):Void {
		if (id == null || StringTools.trim(id).length == 0 || byOccurrence.exists(id))
			throw 'Duplicate or empty assembly occurrence "$id"';
		requireDefinition(definition);
		var occurrence:AssemblyComponentOccurrence = {id: id, definition: definition,
			initialPose: initialPose == null ? AssemblyFrames.identity() : initialPose};
		AssemblyCodec.validateFrame(occurrence.initialPose);
		occurrences.push(occurrence);
		byOccurrence.set(id, occurrence);
	}

	public function treeJoint(id:String, type:AssemblyJointType, parent:String, parentConnector:String,
			child:String, childConnector:String, axis:AssemblyVector, defaultValue:Float = 0,
			?limits:AssemblyJointLimits):Void {
		addJoint(id, type, AssemblyJointRole.Tree, parent, parentConnector, child, childConnector,
			axis, defaultValue, limits);
	}

	public function closure(id:String, type:AssemblyJointType, parent:String, parentConnector:String,
			child:String, childConnector:String, axis:AssemblyVector,
			?limits:AssemblyJointLimits):Void {
		addJoint(id, type, AssemblyJointRole.Closure, parent, parentConnector, child, childConnector,
			axis, 0, limits);
	}

	public function finish():AssemblyDefinition {
		var result:AssemblyDefinition = {schemaVersion: AssemblyDefinitionCodec.VERSION, id: id,
			definitions: definitions.copy(), occurrences: occurrences.copy(), joints: joints.copy()};
		AssemblyDefinitionCodec.validate(result);
		return result;
	}

	function addJoint(id:String, type:AssemblyJointType, role:AssemblyJointRole,
			parent:String, parentConnector:String, child:String, childConnector:String,
			axis:AssemblyVector, defaultValue:Float, limits:Null<AssemblyJointLimits>):Void {
		if (id == null || StringTools.trim(id).length == 0 || byJoint.exists(id))
			throw 'Duplicate or empty assembly joint "$id"';
		var parentOccurrence = requireOccurrence(parent), childOccurrence = requireOccurrence(child);
		if (!hasConnector(requireDefinition(parentOccurrence.definition), parentConnector) ||
			!hasConnector(requireDefinition(childOccurrence.definition), childConnector))
			throw 'Assembly joint "$id" references a missing connector';
		var joint:KinematicJoint = {id: id, type: type, role: role, parent: parent,
			parentConnector: parentConnector, child: child, childConnector: childConnector,
			axis: axis, limits: limits == null ? noLimits() : limits, defaultValue: defaultValue};
		// Validate the axis and limit values before the joint enters the builder.
		// The complete graph can be temporarily disconnected while authoring, so validate its
		// local fields here and the graph once finish() is called.
		validateJointFields(joint);
		joints.push(joint);
		byJoint.set(id, true);
	}

	function requireDefinition(id:String):AssemblyComponentDefinition {
		var result = byDefinition.get(id);
		if (result == null) throw 'Missing component definition "$id"';
		return result;
	}

	function requireOccurrence(id:String):AssemblyComponentOccurrence {
		var result = byOccurrence.get(id);
		if (result == null) throw 'Missing assembly occurrence "$id"';
		return result;
	}

	static function hasConnector(component:AssemblyComponentDefinition, name:String):Bool {
		for (connector in component.connectors) if (connector.name == name) return true;
		return false;
	}

	static function noLimits():AssemblyJointLimits
		return {lower: null, upper: null, velocity: null, effort: null};

	static function validateJointFields(joint:KinematicJoint):Void {
		if (joint.axis == null || !Math.isFinite(joint.axis.x) || !Math.isFinite(joint.axis.y) ||
			!Math.isFinite(joint.axis.z) || Math.abs(Math.sqrt(joint.axis.x * joint.axis.x +
			joint.axis.y * joint.axis.y + joint.axis.z * joint.axis.z) - 1) > 1e-5)
			throw 'Assembly joint "${joint.id}" axis must be a finite unit vector';
		if (!Math.isFinite(joint.defaultValue) ||
			(joint.type == AssemblyJointType.Fixed && joint.defaultValue != 0) ||
			(joint.limits.lower != null &&
			joint.defaultValue < joint.limits.lower) || (joint.limits.upper != null &&
			joint.defaultValue > joint.limits.upper))
			throw 'Assembly joint "${joint.id}" has an invalid default coordinate';
	}
}
