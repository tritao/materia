package materia.assembly;

import haxe.Json;
import haxeon.wire.JsonWire;
import materia.assembly.AssemblyDefinition;
import materia.assembly.AssemblyDefinition.AssemblyMateKind;
import materia.assembly.AssemblyDefinition.AssemblyComponentDefinition;
import materia.assembly.AssemblyDefinition.AssemblyComponentOccurrence;
import materia.assembly.AssemblyDefinition.AssemblyJointLimits;
import materia.assembly.AssemblyDefinition.AssemblyJointRole;
import materia.assembly.AssemblyDefinition.AssemblyJointType;
import materia.assembly.AssemblyDefinition.AssemblyStateRecord;
import materia.assembly.AssemblyDefinition.AssemblyVector;
import materia.assembly.AssemblyDefinition.KinematicJoint;
import materia.units.LengthUnit;

/** Versioned transport and validation for reusable assembly definitions and states. */
class AssemblyDefinitionCodec {
	public static inline var VERSION:Int = 2;

	public static function encode(definition:AssemblyDefinition):String {
		validate(definition);
		return JsonWire.encode(definition);
	}

	public static function decode(text:String):AssemblyDefinition {
		if (Reflect.hasField(Json.parse(text), "schemaVersion"))
			throw "Assembly definitions in the old JSON format are no longer supported";
		var result:AssemblyDefinition = JsonWire.decode(text);
		validate(result);
		return result;
	}

	public static function encodeState(definition:AssemblyDefinition, state:AssemblyStateRecord):String {
		validateState(definition, state);
		return JsonWire.encode(state);
	}

	public static function decodeState(definition:AssemblyDefinition, text:String):AssemblyStateRecord {
		if (Reflect.hasField(Json.parse(text), "schemaVersion"))
			throw "Assembly states in the old JSON format are no longer supported";
		var decoded:AssemblyStateRecord = JsonWire.decode(text);
		validateState(definition, decoded);
		return decoded;
	}

	public static function validate(definition:AssemblyDefinition):Void {
		if (definition != null && definition.assemblies != null && definition.assemblies.length > 0) {
			validateFlat(AssemblyDefinitionFlattener.flatten(definition));
			return;
		}
		validateFlat(definition);
	}

	static function validateFlat(definition:AssemblyDefinition):Void {
		if (definition == null || definition.schemaVersion != VERSION || !validText(definition.id) ||
			definition.definitions == null || definition.definitions.length == 0 ||
			definition.definitions.length > 1000 || definition.occurrences == null ||
			definition.occurrences.length == 0 || definition.occurrences.length > 1000 ||
			definition.joints == null || definition.joints.length > 4000)
			throw "Assembly definition has an invalid version, ID, or item count";
		LengthUnit.metresPerUnit(definition.lengthUnit == null ? "mm" : definition.lengthUnit);

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
				var reference = connector.reference;
				if (reference != null && (reference.length == 0 || reference.length > 4096))
					throw 'Connector "${connector.name}" of "${component.id}" has an invalid reference';
			}
			definitions.set(component.id, component);
		}

		var occurrences = new Map<String, AssemblyComponentOccurrence>();
		for (occurrence in definition.occurrences) {
			if (occurrence == null || !validText(occurrence.id) || occurrences.exists(occurrence.id) ||
				occurrence.assembly != null || !definitions.exists(occurrence.definition))
				throw "Assembly definition has an invalid or duplicate occurrence";
			AssemblyCodec.validateFrame(occurrence.initialPose);
			occurrences.set(occurrence.id, occurrence);
		}
		var exposedNames = new Map<String, Bool>();
		if (definition.exposedConnectors != null) for (exposed in definition.exposedConnectors) {
			var member = exposed == null ? null : occurrences.get(exposed.occurrence);
			if (exposed == null || !validText(exposed.name) || exposedNames.exists(exposed.name) ||
				member == null || !hasConnector(definitions.get(member.definition), exposed.connector))
				throw "Assembly has an invalid or duplicate exposed connector";
			exposedNames.set(exposed.name, true);
		}

		var joints = new Map<String, Bool>(), incoming = new Map<String, String>();
		var movable = new Map<String, KinematicJoint>();
		for (joint in definition.joints) {
			if (joint == null || !validText(joint.id) || joints.exists(joint.id) ||
				!validJointType(joint.type) || (joint.role != AssemblyJointRole.Tree &&
				joint.role != AssemblyJointRole.Closure) || joint.parent == joint.child ||
				!Math.isFinite(joint.defaultValue) || !validAxis(joint.axis) ||
				(joint.type == AssemblyJointType.Fixed && joint.defaultValue != 0) ||
				(joint.closureTolerance != null && (joint.role != AssemblyJointRole.Closure ||
					!Math.isFinite(joint.closureTolerance) || joint.closureTolerance < 0)) ||
				(joint.driven == true && (joint.role != AssemblyJointRole.Tree || !hasCoordinate(joint.type))) ||
				(closureOnlyType(joint.type) && joint.role != AssemblyJointRole.Closure) ||
				!validLimits(joint.limits, joint.defaultValue))
				throw "Assembly definition has an invalid joint";
			joints.set(joint.id, true);
			if (joint.role == AssemblyJointRole.Tree && hasCoordinate(joint.type)) movable.set(joint.id, joint);
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
		var mates = definition.mates == null ? [] : definition.mates;
		var mateIds = new Map<String, Bool>();
		for (mate in mates) {
			var first = mate == null ? null : occurrences.get(mate.first), second = mate == null ? null : occurrences.get(mate.second);
			var value:Null<Float> = mate == null ? null : mate.value;
			if (mate == null || !validText(mate.id) || mateIds.exists(mate.id) || joints.exists(mate.id) || !validMateKind(mate.kind) ||
				first == null || second == null || mate.first == mate.second || !validAxis(mate.axis) ||
				!hasConnector(definitions.get(first.definition), mate.firstConnector) ||
				!hasConnector(definitions.get(second.definition), mate.secondConnector) ||
				(value != null && !Math.isFinite(value)) ||
				((mate.kind == AssemblyMateKind.Distance || mate.kind == AssemblyMateKind.Angle) && value == null))
				throw 'Assembly mate "${mate == null ? "" : mate.id}" is invalid';
			// A distance's direction is undefined where the origins meet; an angle between axes lies in [0, π].
			if (mate.kind == AssemblyMateKind.Distance && value != null && !(value > 0))
				throw 'Assembly mate "${mate.id}" needs a positive distance (use a coincident mate for zero)';
			if (mate.kind == AssemblyMateKind.Angle && value != null && !(value >= 0 && value <= Math.PI))
				throw 'Assembly mate "${mate.id}" needs an angle between 0 and π';
			mateIds.set(mate.id, true);
		}
		var couplings = definition.couplings == null ? [] : definition.couplings;
		if (couplings.length > 4000) throw "Assembly has too many coupled joints";
		var targets = new Map<String, Bool>(), names = new Map<String, Bool>();
		var sourceByTarget = new Map<String, String>();
		for (coupling in couplings) {
			if (coupling == null || !validText(coupling.id) || names.exists(coupling.id) ||
				coupling.source == coupling.target || movable.get(coupling.source) == null ||
				movable.get(coupling.target) == null || targets.exists(coupling.target) ||
				!Math.isFinite(coupling.ratio) || coupling.ratio == 0 || !Math.isFinite(coupling.offset))
				throw "Assembly has an invalid coupled joint";
			if (movable.get(coupling.target).driven == true)
				throw 'Assembly joint "${coupling.target}" is driven by a coupling, so it cannot also be an input';
			names.set(coupling.id, true);
			targets.set(coupling.target, true);
			sourceByTarget.set(coupling.target, coupling.source);
		}
		for (coupling in couplings) {
			var seen = new Map<String, Bool>();
			var current = coupling.target;
			while (true) {
				if (seen.exists(current)) throw "Assembly coupled joints contain a cycle";
				seen.set(current, true);
				var next = sourceByTarget.get(current);
				if (next == null) break;
				current = next;
			}
		}
	}

	public static function validateState(definition:AssemblyDefinition, state:AssemblyStateRecord):Void {
		validate(definition);
		state = AssemblyDefinitionFlattener.flattenState(definition, state);
		definition = AssemblyDefinitionFlattener.flatten(definition);
		if (state == null || state.schemaVersion != VERSION || state.definition != definition.id ||
			state.jointCoordinates == null || state.rootPoses == null ||
			state.jointCoordinates.length > definition.joints.length ||
			state.rootPoses.length > definition.occurrences.length)
			throw "Assembly state has an invalid version, definition, or item count";

		var joints = new Map<String, KinematicJoint>();
		for (joint in definition.joints) if (joint.role == AssemblyJointRole.Tree && hasCoordinate(joint.type))
			joints.set(joint.id, joint);
		var values = new Map<String, Bool>();
		var coordinateValues = new Map<String, Float>();
		for (coordinate in state.jointCoordinates) {
			var joint = coordinate == null ? null : joints.get(coordinate.joint);
			if (joint == null || values.exists(coordinate.joint) || !Math.isFinite(coordinate.value) ||
				!withinLimits(joint.limits, coordinate.value))
				throw "Assembly state has an invalid joint coordinate";
			values.set(coordinate.joint, true);
			coordinateValues.set(coordinate.joint, coordinate.value);
		}
		if (definition.couplings != null) for (coupling in definition.couplings) {
			var source = coordinateValues.get(coupling.source);
			var target = coordinateValues.get(coupling.target);
			var sourceJoint = joints.get(coupling.source), targetJoint = joints.get(coupling.target);
			if (sourceJoint == null || targetJoint == null) throw "Assembly coupling has a missing joint";
			if (source == null) source = sourceJoint.defaultValue;
			if (target == null) target = targetJoint.defaultValue;
			if (Math.abs(target - (source * coupling.ratio + coupling.offset)) > 1e-7)
				throw 'Assembly coupling "${coupling.id}" has inconsistent state';
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

	public static function rootOccurrences(definition:AssemblyDefinition):Map<String, Bool> {
		definition = AssemblyDefinitionFlattener.flatten(definition);
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
			type == AssemblyJointType.Continuous || type == AssemblyJointType.Prismatic || closureOnlyType(type);

	/** Joint types a closure can have but a tree joint cannot (they have more than one coordinate). */
	public static function closureOnlyType(type:AssemblyJointType):Bool
		return type == AssemblyJointType.Spherical || type == AssemblyJointType.Cylindrical || type == AssemblyJointType.Planar;

	static function validMateKind(kind:AssemblyMateKind):Bool
		return kind == AssemblyMateKind.Coincident || kind == AssemblyMateKind.Coaxial || kind == AssemblyMateKind.Planar ||
			kind == AssemblyMateKind.Parallel || kind == AssemblyMateKind.Perpendicular || kind == AssemblyMateKind.Distance ||
			kind == AssemblyMateKind.Angle || kind == AssemblyMateKind.Lock;

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
		if (limits.overtravel != null && (!Math.isFinite(limits.overtravel) || limits.overtravel < 0)) return false;
		if (limits.acceleration != null && (!Math.isFinite(limits.acceleration) || limits.acceleration < 0)) return false;
		return withinLimits(limits, value);
	}

	static function withinLimits(limits:AssemblyJointLimits, value:Float):Bool
		return (limits.lower == null || value >= limits.lower) && (limits.upper == null || value <= limits.upper);

	static function validText(value:Null<String>):Bool
		return value != null && value.length > 0 && value.length <= 4096 &&
			StringTools.trim(value).length > 0 && !AssemblyCodec.containsNul(value);
}
