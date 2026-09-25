package materia.project;

import haxe.Json;
import materia.project.AssemblyDefinition;
import materia.project.AssemblyDefinition.AssemblyComponentDefinition;
import materia.project.AssemblyDefinition.AssemblyComponentOccurrence;
import materia.project.AssemblyDefinition.AssemblyJointCoordinate;
import materia.project.AssemblyDefinition.AssemblyJointLimits;
import materia.project.AssemblyDefinition.AssemblyJointRole;
import materia.project.AssemblyDefinition.AssemblyJointType;
import materia.project.AssemblyDefinition.AssemblyRootPose;
import materia.project.AssemblyDefinition.AssemblyStateRecord;
import materia.project.AssemblyDefinition.AssemblyVector;
import materia.project.AssemblyDefinition.KinematicJoint;
import materia.project.AssemblyRecord;
import materia.project.AssemblyRecord.AssemblyConnector;
import materia.project.AssemblyRecord.AssemblyFrame;

/** Versioned transport and validation for reusable assembly definitions and states. */
class AssemblyDefinitionCodec {
	public static inline var VERSION:Int = 1;

	public static function encode(definition:AssemblyDefinition):String {
		validate(definition);
		return Json.stringify(definition);
	}

	public static function decode(text:String):AssemblyDefinition {
		var raw:Dynamic = Json.parse(text);
		var definitions:Array<AssemblyComponentDefinition> = [];
		for (item in arrayField(raw, "definitions")) {
			var connectors:Array<AssemblyConnector> = [];
			for (connector in arrayField(item, "connectors"))
				connectors.push({name: textField(connector, "name"), frame: readFrame(field(connector, "frame"))});
			definitions.push({id: textField(item, "id"), connectors: connectors});
		}
		var occurrences:Array<AssemblyComponentOccurrence> = [];
		for (item in arrayField(raw, "occurrences"))
			occurrences.push({id: textField(item, "id"), definition: textField(item, "definition"),
				initialPose: readFrame(field(item, "initialPose"))});
		var joints:Array<KinematicJoint> = [];
		for (item in arrayField(raw, "joints")) {
			var limits = field(item, "limits");
			joints.push({id: textField(item, "id"), type: cast textField(item, "type"),
				role: cast textField(item, "role"), parent: textField(item, "parent"),
				parentConnector: textField(item, "parentConnector"), child: textField(item, "child"),
				childConnector: textField(item, "childConnector"), axis: readVector(field(item, "axis")),
				limits: {lower: optionalNumberField(limits, "lower"),
					upper: optionalNumberField(limits, "upper"), velocity: optionalNumberField(limits, "velocity"),
					effort: optionalNumberField(limits, "effort")},
				defaultValue: numberField(item, "defaultValue")});
		}
		var result:AssemblyDefinition = {schemaVersion: integerField(raw, "schemaVersion"),
			id: textField(raw, "id"), definitions: definitions, occurrences: occurrences, joints: joints};
		validate(result);
		return result;
	}

	public static function encodeState(definition:AssemblyDefinition, state:AssemblyStateRecord):String {
		validateState(definition, state);
		return Json.stringify(state);
	}

	public static function decodeState(definition:AssemblyDefinition, text:String):AssemblyStateRecord {
		var raw:Dynamic = Json.parse(text);
		var coordinates:Array<AssemblyJointCoordinate> = [];
		for (item in arrayField(raw, "jointCoordinates"))
			coordinates.push({joint: textField(item, "joint"), value: numberField(item, "value")});
		var roots:Array<AssemblyRootPose> = [];
		for (item in arrayField(raw, "rootPoses"))
			roots.push({occurrence: textField(item, "occurrence"), pose: readFrame(field(item, "pose"))});
		var state:AssemblyStateRecord = {schemaVersion: integerField(raw, "schemaVersion"),
			definition: textField(raw, "definition"), jointCoordinates: coordinates, rootPoses: roots};
		validateState(definition, state);
		return state;
	}

	public static function validate(definition:AssemblyDefinition):Void {
		if (definition == null || definition.schemaVersion != VERSION || !validText(definition.id) ||
			definition.definitions == null || definition.definitions.length == 0 ||
			definition.definitions.length > 1000 || definition.occurrences == null ||
			definition.occurrences.length == 0 || definition.occurrences.length > 1000 ||
			definition.joints == null || definition.joints.length > 4000)
			throw "Assembly definition has an invalid version, ID, or item count";

		var definitions = new Map<String, AssemblyComponentDefinition>();
		for (component in definition.definitions) {
			if (component == null || !validText(component.id) || definitions.exists(component.id) ||
				component.connectors == null || component.connectors.length > 100)
				throw "Assembly definition has an invalid or duplicate component definition";
			var names = new Map<String, Bool>();
			for (connector in component.connectors) {
				if (connector == null || !validText(connector.name) || names.exists(connector.name))
					throw 'Component definition "${component.id}" has an invalid or duplicate connector';
				names.set(connector.name, true);
				AssemblyCodec.validateFrame(connector.frame);
			}
			definitions.set(component.id, component);
		}

		var occurrences = new Map<String, AssemblyComponentOccurrence>();
		for (occurrence in definition.occurrences) {
			if (occurrence == null || !validText(occurrence.id) || occurrences.exists(occurrence.id) ||
				!definitions.exists(occurrence.definition))
				throw "Assembly definition has an invalid or duplicate occurrence";
			AssemblyCodec.validateFrame(occurrence.initialPose);
			occurrences.set(occurrence.id, occurrence);
		}

		var joints = new Map<String, Bool>(), incoming = new Map<String, String>();
		for (joint in definition.joints) {
			if (joint == null || !validText(joint.id) || joints.exists(joint.id) ||
				!validJointType(joint.type) || (joint.role != AssemblyJointRole.Tree &&
				joint.role != AssemblyJointRole.Closure) || joint.parent == joint.child ||
				!Math.isFinite(joint.defaultValue) || !validAxis(joint.axis) ||
				(joint.type == AssemblyJointType.Fixed && joint.defaultValue != 0) ||
				!validLimits(joint.limits, joint.defaultValue))
				throw "Assembly definition has an invalid joint";
			joints.set(joint.id, true);
			var parent = occurrences.get(joint.parent), child = occurrences.get(joint.child);
			if (parent == null || child == null ||
				!hasConnector(definitions.get(parent.definition), joint.parentConnector) ||
				!hasConnector(definitions.get(child.definition), joint.childConnector))
				throw 'Assembly joint "${joint.id}" has a missing endpoint';
			if (joint.role == AssemblyJointRole.Tree) {
				if (incoming.exists(joint.child))
					throw 'Assembly occurrence "${joint.child}" has multiple tree parents';
				incoming.set(joint.child, joint.parent);
			}
		}
		validateTree(occurrences, incoming);
	}

	public static function validateState(definition:AssemblyDefinition, state:AssemblyStateRecord):Void {
		validate(definition);
		if (state == null || state.schemaVersion != VERSION || state.definition != definition.id ||
			state.jointCoordinates == null || state.rootPoses == null ||
			state.jointCoordinates.length > definition.joints.length ||
			state.rootPoses.length > definition.occurrences.length)
			throw "Assembly state has an invalid version, definition, or item count";

		var joints = new Map<String, KinematicJoint>();
		for (joint in definition.joints) if (joint.role == AssemblyJointRole.Tree && hasCoordinate(joint.type))
			joints.set(joint.id, joint);
		var values = new Map<String, Bool>();
		for (coordinate in state.jointCoordinates) {
			var joint = coordinate == null ? null : joints.get(coordinate.joint);
			if (joint == null || values.exists(coordinate.joint) || !Math.isFinite(coordinate.value) ||
				!withinLimits(joint.limits, coordinate.value))
				throw "Assembly state has an invalid joint coordinate";
			values.set(coordinate.joint, true);
		}

		var roots = rootOccurrences(definition);
		var seenRoots = new Map<String, Bool>();
		for (root in state.rootPoses) {
			if (root == null || !roots.exists(root.occurrence) || seenRoots.exists(root.occurrence))
				throw "Assembly state has an invalid root pose";
			AssemblyCodec.validateFrame(root.pose);
			seenRoots.set(root.occurrence, true);
		}
	}

	public static function fromLegacy(record:AssemblyRecord, id:String = "assembly"):AssemblyDefinition {
		AssemblyCodec.validate(record);
		if (!validText(id)) throw "Assembly definition needs an ID";
		var definitions:Array<AssemblyComponentDefinition> = [];
		var occurrences:Array<AssemblyComponentOccurrence> = [];
		for (instance in record.instances) {
			var componentId = instance.id;
			definitions.push({id: componentId, connectors: instance.connectors.copy()});
			occurrences.push({id: instance.id, definition: componentId, initialPose: instance.pose});
		}
		var incoming = new Map<String, Bool>(), joints:Array<KinematicJoint> = [];
		for (joint in record.joints) {
			var role = incoming.exists(joint.child) ? AssemblyJointRole.Closure : AssemblyJointRole.Tree;
			if (role == AssemblyJointRole.Tree) incoming.set(joint.child, true);
			joints.push({id: joint.id, type: cast joint.kind, role: role, parent: joint.parent,
				parentConnector: joint.parentConnector, child: joint.child,
				childConnector: joint.childConnector, axis: {x: 0, y: 1, z: 0},
				limits: {lower: null, upper: null, velocity: null, effort: null},
				defaultValue: joint.kind == "fixed" ? 0 : joint.value});
		}
		var result:AssemblyDefinition = {schemaVersion: VERSION, id: id, definitions: definitions,
			occurrences: occurrences, joints: joints};
		validate(result);
		return result;
	}

	public static function rootOccurrences(definition:AssemblyDefinition):Map<String, Bool> {
		var hasParent = new Map<String, Bool>();
		for (joint in definition.joints) if (joint.role == AssemblyJointRole.Tree) hasParent.set(joint.child, true);
		var result = new Map<String, Bool>();
		for (occurrence in definition.occurrences) if (!hasParent.exists(occurrence.id)) result.set(occurrence.id, true);
		return result;
	}

	public static function hasCoordinate(type:AssemblyJointType):Bool
		return type == AssemblyJointType.Revolute || type == AssemblyJointType.Continuous ||
			type == AssemblyJointType.Prismatic;

	static function validateTree(occurrences:Map<String, AssemblyComponentOccurrence>, incoming:Map<String, String>):Void {
		for (id in occurrences.keys()) {
			var path = new Map<String, Bool>(), current:Null<String> = id;
			while (current != null) {
				if (path.exists(current)) throw 'Assembly tree contains a cycle at "$current"';
				path.set(current, true);
				current = incoming.get(current);
			}
		}
	}

	static function hasConnector(definition:Null<AssemblyComponentDefinition>, name:String):Bool {
		if (definition == null) return false;
		for (connector in definition.connectors) if (connector.name == name) return true;
		return false;
	}

	static function validJointType(type:AssemblyJointType):Bool
		return type == AssemblyJointType.Fixed || type == AssemblyJointType.Revolute ||
			type == AssemblyJointType.Continuous || type == AssemblyJointType.Prismatic;

	static function validAxis(axis:AssemblyVector):Bool {
		if (axis == null || !Math.isFinite(axis.x) || !Math.isFinite(axis.y) || !Math.isFinite(axis.z)) return false;
		var length = Math.sqrt(axis.x * axis.x + axis.y * axis.y + axis.z * axis.z);
		return Math.isFinite(length) && Math.abs(length - 1) <= 1e-5;
	}

	static function validLimits(limits:AssemblyJointLimits, value:Float):Bool {
		if (limits == null) return false;
		for (limit in [limits.lower, limits.upper, limits.velocity, limits.effort])
			if (limit != null && !Math.isFinite(limit)) return false;
		if (limits.lower != null && limits.upper != null && limits.lower > limits.upper) return false;
		if ((limits.velocity != null && limits.velocity < 0) || (limits.effort != null && limits.effort < 0)) return false;
		return withinLimits(limits, value);
	}

	static function withinLimits(limits:AssemblyJointLimits, value:Float):Bool
		return (limits.lower == null || value >= limits.lower) && (limits.upper == null || value <= limits.upper);

	static function readFrame(value:Dynamic):AssemblyFrame
		return {x: numberField(value, "x"), y: numberField(value, "y"), z: numberField(value, "z"),
			qx: numberField(value, "qx"), qy: numberField(value, "qy"),
			qz: numberField(value, "qz"), qw: numberField(value, "qw")};

	static function readVector(value:Dynamic):AssemblyVector
		return {x: numberField(value, "x"), y: numberField(value, "y"), z: numberField(value, "z")};

	static function field(value:Dynamic, name:String):Dynamic {
		if (value == null || !Reflect.hasField(value, name)) throw 'Assembly field "$name" is missing';
		return Reflect.field(value, name);
	}
	static function arrayField(value:Dynamic, name:String):Array<Dynamic> {
		var result = field(value, name);
		if (!Std.isOfType(result, Array)) throw 'Assembly field "$name" must be an array';
		return cast result;
	}
	static function textField(value:Dynamic, name:String):String {
		var result = field(value, name);
		if (!Std.isOfType(result, String) || !validText(cast result))
			throw 'Assembly field "$name" must be nonempty text';
		return cast result;
	}
	static function numberField(value:Dynamic, name:String):Float {
		var result = field(value, name);
		if ((!Std.isOfType(result, Float) && !Std.isOfType(result, Int)) || !Math.isFinite(cast result))
			throw 'Assembly field "$name" must be finite';
		return cast result;
	}
	static function integerField(value:Dynamic, name:String):Int {
		var result = field(value, name);
		if (!Std.isOfType(result, Int)) throw 'Assembly field "$name" must be an integer';
		return cast result;
	}
	static function optionalNumberField(value:Dynamic, name:String):Null<Float> {
		if (value == null || !Reflect.hasField(value, name)) return null;
		var result = Reflect.field(value, name);
		if (result == null) return null;
		if ((!Std.isOfType(result, Float) && !Std.isOfType(result, Int)) || !Math.isFinite(cast result))
			throw 'Assembly field "$name" must be finite or null';
		return cast result;
	}
	static function validText(value:Null<String>):Bool
		return value != null && value.length > 0 && value.length <= 4096 &&
			StringTools.trim(value).length > 0 && value.indexOf("\x00") < 0;
}
