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
	public static inline var VERSION:Int = 3;
	/** Pneumatic process drives require v4; assemblies without them retain v3 bytes. */
	public static inline var PROCESS_VERSION:Int = 4;
	/** Optional native sensor bindings require v5; older assemblies retain their bytes. */
	public static inline var SENSOR_VERSION:Int = 5;
	/** Analog process-velocity bindings require v6; earlier assembly bytes remain stable. */
	public static inline var VELOCITY_VERSION:Int = 6;

	static function supportedVersion(version:Int):Bool
		return version >= VERSION && version <= VELOCITY_VERSION;

	public static function encode(definition:AssemblyDefinition):String {
		validate(definition);
		return JsonWire.encode(definition);
	}

	public static function decode(text:String):AssemblyDefinition {
		var result:AssemblyDefinition = JsonWire.decode(text);
		validate(result);
		return result;
	}

	public static function encodeState(definition:AssemblyDefinition, state:AssemblyStateRecord):String {
		validateState(definition, state);
		return JsonWire.encode(state);
	}

	public static function decodeState(definition:AssemblyDefinition, text:String):AssemblyStateRecord {
		var decoded:AssemblyStateRecord = JsonWire.decode(text);
		validateState(definition, decoded);
		return decoded;
	}

	public static function validate(definition:AssemblyDefinition):Void {
		if (definition != null && !supportedVersion(definition.schemaVersion))
			throw 'schema v${definition.schemaVersion} is unsupported; supported v$VERSION through v$VELOCITY_VERSION';
		if (definition != null && definition.assemblies != null && definition.assemblies.length > 0) {
			validateFlat(AssemblyDefinitionFlattener.flattenView(definition));
			return;
		}
		validateFlat(definition);
	}

	static function validateFlat(definition:AssemblyDefinition):Void {
		if (definition == null || !supportedVersion(definition.schemaVersion) || !validText(definition.id) ||
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
			if (component.robotFlangeConnector != null && !names.exists(component.robotFlangeConnector))
				throw 'Robot flange "${component.id}" references a missing face connector';
			if (component.collisionHulls != null) {
				if (component.collisionHulls.length == 0) throw 'Component "${component.id}" has no collision pieces';
				for (hull in component.collisionHulls) {
					if (hull == null || hull.length < 12 || hull.length > 192 || hull.length % 3 != 0)
						throw 'Component "${component.id}" has an invalid collision hull';
					for (value in hull) if (!Math.isFinite(value)) throw 'Component "${component.id}" has a non-finite collision hull';
				}
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
		var sourcesByTarget = new Map<String, Array<String>>();
		for (coupling in couplings) {
			if (coupling == null || !validText(coupling.id) || names.exists(coupling.id) ||
				coupling.source == coupling.target || movable.get(coupling.source) == null ||
				movable.get(coupling.target) == null ||
				(sourcesByTarget.exists(coupling.target) && sourcesByTarget.get(coupling.target).indexOf(coupling.source) >= 0) ||
				!Math.isFinite(coupling.ratio) || coupling.ratio == 0 || !Math.isFinite(coupling.offset) ||
				(coupling.efficiency != null && !(coupling.efficiency > 0 && coupling.efficiency <= 1)) ||
				(coupling.stiffness != null && !(coupling.stiffness > 0 && Math.isFinite(coupling.stiffness))) ||
				(coupling.backlash != null && !(coupling.backlash >= 0 && Math.isFinite(coupling.backlash))) ||
				(coupling.drag != null && !(coupling.drag >= 0 && Math.isFinite(coupling.drag))))
				throw "Assembly has an invalid coupled joint";
			if (movable.get(coupling.target).driven == true)
				throw 'Assembly joint "${coupling.target}" is driven by a coupling, so it cannot also be an input';
			names.set(coupling.id, true);
			targets.set(coupling.target, true);
			// A target with several couplings is the sum of their terms (see AssemblyJointCoupling).
			if (!sourcesByTarget.exists(coupling.target)) sourcesByTarget.set(coupling.target, []);
			sourcesByTarget.get(coupling.target).push(coupling.source);
		}
		var networkIds = new Map<String, Bool>(), owned = new Map<String, Bool>();
		if (definition.elasticNetworks != null) for (network in definition.elasticNetworks) {
			if (!validText(network.id) || networkIds.exists(network.id) || network.spans.length == 0)
				throw "Assembly has an invalid elastic network";
			networkIds.set(network.id, true);
			for (id in network.couplings) {
				if (!names.exists(id) || owned.exists(id)) throw "Elastic networks require distinct, existing motion couplings";
				owned.set(id, true);
			}
			var clearanceJoints = new Map<String, Bool>();
			if (network.clearances != null) for (clearance in network.clearances) {
				if (!movable.exists(clearance.joint) || clearanceJoints.exists(clearance.joint) || !Math.isFinite(clearance.allowance) || clearance.allowance < 0)
					throw "Elastic network has an invalid tooth-clearance coordinate";
				clearanceJoints.set(clearance.joint, true);
			}
			for (span in network.spans) {
				if (!(span.stiffness > 0) || !Math.isFinite(span.stiffness) || span.terms.length == 0)
					throw "Elastic span needs positive finite stiffness and coordinate terms";
				var terms = new Map<String, Bool>();
				for (term in span.terms) {
					if (!movable.exists(term.joint) || terms.exists(term.joint) || !Math.isFinite(term.coefficient) || term.coefficient == 0)
						throw "Elastic span has an invalid or duplicate coordinate";
					terms.set(term.joint, true);
				}
			}
		}

		var actuators = definition.actuators == null ? [] : definition.actuators;
		if (actuators.length > 4000) throw "Assembly has too many actuators";
		var actuatorIds = new Map<String, Bool>();
		for (actuator in actuators) {
			if (actuator == null || !validText(actuator.id) || actuatorIds.exists(actuator.id) ||
				movable.get(actuator.joint) == null || !Math.isFinite(actuator.maxEffort) || actuator.maxEffort < 0 ||
				!Math.isFinite(actuator.maxRate) || actuator.maxRate < 0 ||
				(actuator.rotorInertia != null && !(actuator.rotorInertia >= 0 && Math.isFinite(actuator.rotorInertia))) ||
				(actuator.fullStepsPerRevolution != null && !(actuator.fullStepsPerRevolution > 0 && Math.isFinite(actuator.fullStepsPerRevolution))) ||
				!validDrive(actuator))
				throw 'Assembly has an invalid actuator "${actuator == null ? "" : actuator.id}"';
			actuatorIds.set(actuator.id, true);
			if (actuator.drive == "pneumatic" && definition.schemaVersion == VERSION)
				throw "Pneumatic assembly drives require schema v$PROCESS_VERSION";
			if (actuator.processVelocity != null && definition.schemaVersion != VELOCITY_VERSION)
				throw 'Process-velocity assembly drives require schema v$VELOCITY_VERSION';
			if (actuator.processVelocity != null) {
				var process = actuator.processVelocity;
				var joint = movable.get(actuator.joint);
				if (actuator.drive != "servo" || actuator.pneumatic != null || joint == null ||
					joint.type != AssemblyJointType.Continuous || !validText(process.speedChannel) ||
					!validText(process.directionChannel) || process.speedChannel == process.directionChannel ||
					!(process.radiansPerSpeedUnit > 0) || !Math.isFinite(process.radiansPerSpeedUnit))
					throw 'Assembly actuator "${actuator.id}" has an invalid process-velocity binding';
			}
		}
		var sensorIds = new Map<String, Bool>();
		var sensors = definition.sensors == null ? [] : definition.sensors;
		if (sensors.length > 4000) throw "Assembly has too many sensors";
		for (sensor in sensors) {
			if (sensor == null || !validText(sensor.id) || sensorIds.exists(sensor.id) ||
				(sensor.kind != "joint_switch" && sensor.kind != "at_speed" && sensor.kind != "presence"))
				throw "Assembly has an invalid or duplicate native sensor";
			sensorIds.set(sensor.id, true);
			if (sensor.kind == "joint_switch" || sensor.kind == "at_speed") {
				var joint = sensor.joint == null ? null : movable.get(sensor.joint);
				if (joint == null || sensor.occurrence != null || sensor.connector != null ||
					sensor.windowLower == null || !Math.isFinite(sensor.windowLower) ||
					sensor.hysteresis == null || !Math.isFinite(sensor.hysteresis) || sensor.hysteresis < 0 ||
					(sensor.kind == "joint_switch" && (sensor.windowUpper == null ||
						!Math.isFinite(sensor.windowUpper) || sensor.windowLower > sensor.windowUpper)) ||
					(sensor.kind == "at_speed" && (sensor.windowLower <= 0 || sensor.windowUpper != null)))
					throw 'Assembly sensor "${sensor.id}" has an invalid joint window';
			} else {
				var member = sensor.occurrence == null ? null : occurrences.get(sensor.occurrence);
				if (sensor.joint != null || sensor.windowLower != null || sensor.windowUpper != null ||
					sensor.hysteresis != null || member == null || !validText(sensor.connector) ||
					!hasConnector(definitions.get(member.definition), sensor.connector) || sensor.range == null ||
					!(sensor.range > 0) || !Math.isFinite(sensor.range))
					throw 'Assembly sensor "${sensor.id}" has an invalid presence connector or range';
			}
		}
		if (sensors.length > 0 && definition.schemaVersion < SENSOR_VERSION)
			throw "Native assembly sensors require schema v$SENSOR_VERSION";
		var switches = definition.switches == null ? [] : definition.switches;
		if (switches.length > 4000) throw "Assembly has too many switches";
		var switchIds = new Map<String, Bool>();
		for (contact in switches) {
			if (contact == null) throw "Assembly has a null switch";
			var part = occurrences.get(contact.part), trigger = occurrences.get(contact.trigger);
			if (!validText(contact.id) || switchIds.exists(contact.id) || movable.get(contact.joint) == null ||
				(contact.driveJoint != null && (!validText(contact.driveJoint) || movable.get(contact.driveJoint) == null)) ||
				part == null || trigger == null || part.id == trigger.id ||
				!hasConnector(definitions.get(part.definition), contact.connector) ||
				!hasConnector(definitions.get(trigger.definition), contact.triggerConnector) ||
				(contact.role != "home" && contact.role != "limit") || (contact.side != -1 && contact.side != 1) ||
				!Math.isFinite(contact.trip) || !Math.isFinite(contact.hysteresis) || contact.hysteresis < 0 ||
				!Math.isFinite(contact.repeatability) || contact.repeatability < 0)
				throw 'Assembly has an invalid switch "${contact.id}"';
			switchIds.set(contact.id, true);
		}
		var homeDependencies = new Map<String, Array<String>>();
		for (contact in switches) {
			var dependencies = contact.homeAfter == null ? [] : contact.homeAfter;
			if (contact.role != "home" && dependencies.length > 0) throw "Only home switches may declare homing dependencies";
			if (contact.role != "home") continue;
			var seenDependencies = new Map<String, Bool>();
			for (id in dependencies) {
				if (!validText(id) || id == contact.joint || seenDependencies.exists(id)) throw "Invalid home dependency";
				seenDependencies.set(id, true);
			}
			var sorted = dependencies.copy(); sorted.sort(Reflect.compare);
			if (homeDependencies.exists(contact.joint) && homeDependencies.get(contact.joint).join("\n") != sorted.join("\n"))
				throw "Home switches on one coordinate must agree on dependencies";
			homeDependencies.set(contact.joint, sorted);
		}
		var homeVisiting = new Map<String, Bool>(), homeVisited = new Map<String, Bool>();
		function visitHome(id:String):Void {
			if (!homeDependencies.exists(id)) throw 'Home dependency "$id" has no home switch';
			if (homeVisiting.exists(id)) throw "Homing dependencies contain a cycle";
			if (homeVisited.exists(id)) return;
			homeVisiting.set(id, true);
			for (dependency in homeDependencies.get(id)) visitHome(dependency);
			homeVisiting.remove(id); homeVisited.set(id, true);
		}
		for (id in homeDependencies.keys()) visitHome(id);
		var encoders = definition.encoders == null ? [] : definition.encoders;
		if (encoders.length > 4000) throw "Assembly has too many encoders";
		var encoderIds = new Map<String, Bool>();
		for (encoder in encoders) {
			if (encoder == null || !validText(encoder.id) || encoderIds.exists(encoder.id) || movable.get(encoder.joint) == null ||
				(encoder.kind != "incremental" && encoder.kind != "absolute") ||
				!(encoder.counts > 0 && Math.isFinite(encoder.counts)))
				throw 'Assembly has an invalid encoder "${encoder == null ? "" : encoder.id}"';
			encoderIds.set(encoder.id, true);
		}
		for (actuator in actuators)
			if (actuator.encoder != null && !encoderIds.exists(actuator.encoder))
				throw 'Assembly actuator "${actuator.id}" names an unknown encoder "${actuator.encoder}"';
		// No joint may depend on itself through any chain of terms: depth-first search, 1 on the path, 2 done.
		var visiting = new Map<String, Int>();
		function visit(joint:String):Void {
			var mark = visiting.get(joint);
			if (mark == 2) return;
			if (mark == 1) throw "Assembly coupled joints contain a cycle";
			visiting.set(joint, 1);
			var sources = sourcesByTarget.get(joint);
			if (sources != null) for (source in sources) visit(source);
			visiting.set(joint, 2);
		}
		for (coupling in couplings) visit(coupling.target);
	}

	public static function validateState(definition:AssemblyDefinition, state:AssemblyStateRecord):Void {
		validate(definition);
		state = AssemblyDefinitionFlattener.flattenState(definition, state);
		definition = AssemblyDefinitionFlattener.flattenView(definition);
		if (state == null || !supportedVersion(state.schemaVersion) || state.definition != definition.id ||
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
		if (definition.couplings != null) {
			var expected = new Map<String, Float>();
			for (coupling in definition.couplings) {
				var source = coordinateValues.get(coupling.source);
				var sourceJoint = joints.get(coupling.source), targetJoint = joints.get(coupling.target);
				if (sourceJoint == null || targetJoint == null) throw "Assembly coupling has a missing joint";
				if (source == null) source = sourceJoint.defaultValue;
				var sum = expected.get(coupling.target);
				expected.set(coupling.target, (sum == null ? 0.0 : sum) + source * coupling.ratio + coupling.offset);
			}
			for (target => sum in expected) {
				var value = coordinateValues.get(target);
				var targetJoint = joints.get(target);
				if (targetJoint == null) throw "Assembly coupling has a missing joint";
				if (value == null) value = targetJoint.defaultValue;
				if (Math.abs(value - sum) > 1e-7) throw 'Assembly coupling on "$target" has inconsistent state';
			}
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
		definition = AssemblyDefinitionFlattener.flattenView(definition);
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
		if (limits.rackingTolerance != null && (!Math.isFinite(limits.rackingTolerance) || limits.rackingTolerance <= 0)) return false;
		if (limits.overtravel != null && (!Math.isFinite(limits.overtravel) || limits.overtravel < 0)) return false;
		if (limits.acceleration != null && (!Math.isFinite(limits.acceleration) || limits.acceleration < 0)) return false;
		return withinLimits(limits, value);
	}

	static function withinLimits(limits:AssemblyJointLimits, value:Float):Bool
		return (limits.lower == null || value >= limits.lower) && (limits.upper == null || value <= limits.upper);

	/** An actuator's drive fields: each present number finite and not negative, and a drive kind that needs its numbers has them. */
	static function validDrive(actuator:AssemblyActuator):Bool {
		for (value in [actuator.holdingTorque, actuator.ratedTorque, actuator.peakTorque, actuator.ratedSpeed, actuator.maxSpeed,
				actuator.encoderCounts, actuator.servoStiffness, actuator.servoDamping])
			if (value != null && !(value >= 0 && Math.isFinite(value))) return false;
		if (actuator.microsteps != null && (actuator.microsteps < 1 || actuator.microsteps > 1024)) return false;
		if (actuator.positionLoopRate != null && (!(actuator.positionLoopRate > 0) || !Math.isFinite(actuator.positionLoopRate))) return false;
		if (actuator.maxStepRate != null && (!(actuator.maxStepRate > 0) || !Math.isFinite(actuator.maxStepRate))) return false;
		if (actuator.gearRatio != null && !(actuator.gearRatio > 0 && Math.isFinite(actuator.gearRatio))) return false;
		if (actuator.gearEfficiency != null && !(actuator.gearEfficiency > 0 && actuator.gearEfficiency <= 1)) return false;
		var curve = actuator.torqueSpeed;
		if (curve != null) {
			if (curve.length == 0 || curve.length % 2 != 0) return false;
			for (index in 0...curve.length) {
				if (!(curve[index] >= 0 && Math.isFinite(curve[index]))) return false;
				if (index >= 2 && index % 2 == 0 && !(curve[index] > curve[index - 2])) return false;
			}
		}
		if (actuator.fullStepsPerRevolution != null && actuator.fullStepsPerRevolution > 0
			&& (actuator.microsteps == null || actuator.maxStepRate == null)) return false;
		var kind = actuator.drive;
		if (kind == null) return true;
		if (kind == "pneumatic") {
			var p = actuator.pneumatic;
			return p != null && p.bore > p.rod && p.rod > 0 && p.stroke > 0 && p.ratedSpeed > 0 &&
				p.pressurePa >= 0 && Math.isFinite(p.bore + p.rod + p.stroke + p.ratedSpeed + p.pressurePa) &&
				validText(p.channelA) && (p.channelB == null || validText(p.channelB)) && Math.abs(p.extendSign) == 1;
		}
		if (actuator.pneumatic != null) return false;
		if (kind == "stepper") return actuator.fullStepsPerRevolution != null && actuator.holdingTorque != null && curve != null
			&& actuator.microsteps != null && actuator.maxStepRate != null;
		if (kind == "servo")
			return actuator.ratedTorque != null && actuator.peakTorque != null && actuator.ratedSpeed != null &&
				actuator.maxSpeed != null && actuator.ratedTorque > 0 && actuator.peakTorque >= actuator.ratedTorque &&
				actuator.ratedSpeed > 0 && actuator.maxSpeed >= actuator.ratedSpeed;
		return false;
	}

	static function validText(value:Null<String>):Bool
		return value != null && value.length > 0 && value.length <= 4096 &&
			StringTools.trim(value).length > 0 && !AssemblyCodec.containsNul(value);
}
