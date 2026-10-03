package materia.assembly;

import materia.assembly.AssemblyDefinition;
import materia.assembly.AssemblyDefinition.AssemblyComponentDefinition;
import materia.assembly.AssemblyDefinition.AssemblyComponentOccurrence;
import materia.assembly.AssemblyDefinition.AssemblyExposedConnector;
import materia.assembly.AssemblyDefinition.AssemblyJointCoupling;
import materia.assembly.AssemblyDefinition.AssemblyActuator;
import materia.assembly.AssemblyDefinition.AssemblyEncoder;
import materia.assembly.AssemblyDefinition.AssemblyMate;
import materia.assembly.AssemblyDefinition.AssemblySubdefinition;
import materia.assembly.AssemblyDefinition.AssemblyStateRecord;
import materia.assembly.AssemblyDefinition.AssemblyRootPose;
import materia.assembly.AssemblyDefinition.KinematicJoint;
import materia.assembly.AssemblyRecord.AssemblyFrame;
import haxeon.wire.JsonWire;

private typedef FlatEndpoint = {var occurrence:String; var connector:String;}
private typedef FlatMember = {var connectors:Map<String, FlatEndpoint>;}

/** Expands reusable assembly occurrences into the flat mechanical solver schema. */
class AssemblyDefinitionFlattener {
	/** Maps a saved subassembly root pose to the roots of its flattened members. */
	public static function flattenState(source:AssemblyDefinition, state:AssemblyStateRecord):AssemblyStateRecord {
		if (source.assemblies == null || source.assemblies.length == 0 || state == null || state.rootPoses == null) return state;
		var flat = flatten(source);
		var incoming = new Map<String, Bool>();
		for (joint in flat.joints) if (joint.role == materia.assembly.AssemblyDefinition.AssemblyJointRole.Tree)
			incoming.set(joint.child, true);
		var roots:Array<AssemblyRootPose> = [];
		for (root in state.rootPoses) {
			if (root == null || root.occurrence == null) {
				roots.push(root);
				continue;
			}
			var authored = assemblyPose(source, root.occurrence);
			if (authored == null) {
				roots.push(root);
				continue;
			}
			var prefix = root.occurrence + "/", count = 0;
			var inverse = AssemblyFrames.inverse(authored);
			for (member in flat.occurrences) if (StringTools.startsWith(member.id, prefix) && !incoming.exists(member.id)) {
				roots.push({occurrence: member.id,
					pose: AssemblyFrames.compose(root.pose, AssemblyFrames.compose(inverse, member.initialPose))});
				count++;
			}
			if (count == 0) throw 'Assembly root pose "${root.occurrence}" has no flattened root';
		}
		return {schemaVersion: state.schemaVersion, definition: state.definition,
			jointCoordinates: state.jointCoordinates, rootPoses: roots};
	}

	static function assemblyPose(source:AssemblyDefinition, path:String):Null<AssemblyFrame> {
		var library = new Map<String, AssemblySubdefinition>();
		for (entry in source.assemblies) library.set(entry.id, entry);
		var members = source.occurrences, pose = AssemblyFrames.identity();
		var segments = path.split("/");
		for (index in 0...segments.length) {
			var found:Null<AssemblyComponentOccurrence> = null;
			for (member in members) if (member.id == segments[index]) found = member;
			if (found == null || found.assembly == null) return null;
			pose = AssemblyFrames.compose(pose, found.initialPose);
			if (index == segments.length - 1) return pose;
			var next = library.get(found.assembly);
			if (next == null) return null;
			members = next.occurrences;
		}
		return null;
	}

	public static function flatten(source:AssemblyDefinition):AssemblyDefinition {
		if (source == null) throw "Assembly definition is null";
		if (source.assemblies == null || source.assemblies.length == 0)
			return JsonWire.decode(JsonWire.encode(source));
		var library = new Map<String, AssemblySubdefinition>();
		for (assembly in source.assemblies) {
			if (assembly == null || !validName(assembly.id) || library.exists(assembly.id))
				throw "Assembly has an invalid or duplicate nested definition";
			library.set(assembly.id, assembly);
		}
		var flat:AssemblyDefinition = {schemaVersion: source.schemaVersion, id: source.id,
			definitions: [], occurrences: [], joints: []};
		flat.lengthUnit = source.lengthUnit;
		flat.couplings = [];
		if (source.actuators != null) flat.actuators = [];
		if (source.encoders != null) flat.encoders = [];
		var active = new Map<String, Bool>();
		var rootMembers = expand("", AssemblyFrames.identity(), source.definitions, source.occurrences, source.joints,
			source.couplings, source.mates, library, flat, active, source.actuators, source.encoders);
		exposed(source.exposedConnectors, rootMembers, source.id);
		for (entry in source.assemblies) {
			var unused:AssemblyDefinition = {schemaVersion: source.schemaVersion, id: entry.id,
				definitions: [], occurrences: [], joints: [], couplings: []};
			active.set(entry.id, true);
			var members = expand("", AssemblyFrames.identity(), entry.definitions, entry.occurrences, entry.joints,
				entry.couplings, entry.mates, library, unused, active, entry.actuators, entry.encoders);
			active.remove(entry.id);
			exposed(entry.exposedConnectors, members, entry.id);
			AssemblyDefinitionCodec.validate(unused);
		}
		return JsonWire.decode(JsonWire.encode(flat));
	}

	/** A copy of `actuator` under a new id, on a new joint, with every drive field. */
	public static function copyActuator(actuator:AssemblyActuator, id:String, joint:String):AssemblyActuator {
		var copy:AssemblyActuator = {id: id, joint: joint, maxEffort: actuator.maxEffort, maxRate: actuator.maxRate};
		if (actuator.rotorInertia != null) copy.rotorInertia = actuator.rotorInertia;
		if (actuator.fullStepsPerRevolution != null) copy.fullStepsPerRevolution = actuator.fullStepsPerRevolution;
		if (actuator.drive != null) copy.drive = actuator.drive;
		if (actuator.torqueSpeed != null) copy.torqueSpeed = actuator.torqueSpeed.copy();
		if (actuator.holdingTorque != null) copy.holdingTorque = actuator.holdingTorque;
		if (actuator.ratedTorque != null) copy.ratedTorque = actuator.ratedTorque;
		if (actuator.peakTorque != null) copy.peakTorque = actuator.peakTorque;
		if (actuator.ratedSpeed != null) copy.ratedSpeed = actuator.ratedSpeed;
		if (actuator.maxSpeed != null) copy.maxSpeed = actuator.maxSpeed;
		if (actuator.encoderCounts != null) copy.encoderCounts = actuator.encoderCounts;
		if (actuator.servoStiffness != null) copy.servoStiffness = actuator.servoStiffness;
		if (actuator.servoDamping != null) copy.servoDamping = actuator.servoDamping;
		if (actuator.encoder != null) copy.encoder = actuator.encoder;
		if (actuator.gearRatio != null) copy.gearRatio = actuator.gearRatio;
		if (actuator.gearEfficiency != null) copy.gearEfficiency = actuator.gearEfficiency;
		if (actuator.assumed != null && actuator.assumed.length > 0) copy.assumed = [for (label in actuator.assumed) label];
		return copy;
	}

	/** A copy of `encoder` under a new id, on a new joint. */
	public static function copyEncoder(encoder:AssemblyEncoder, id:String, joint:String):AssemblyEncoder {
		var copy:AssemblyEncoder = {id: id, joint: joint, kind: encoder.kind, counts: encoder.counts};
		if (encoder.index != null) copy.index = encoder.index;
		return copy;
	}

	static function expand(prefix:String, pose:AssemblyFrame, definitions:Array<AssemblyComponentDefinition>,
			occurrences:Array<AssemblyComponentOccurrence>, joints:Array<KinematicJoint>, couplings:Array<AssemblyJointCoupling>,
			mates:Null<Array<AssemblyMate>>, library:Map<String, AssemblySubdefinition>, flat:AssemblyDefinition, active:Map<String, Bool>,
			?actuators:Array<AssemblyActuator>, ?encoders:Array<AssemblyEncoder>):Map<String, FlatMember> {
		if (definitions == null || occurrences == null || joints == null) throw "Nested assembly has missing members or joints";
		var localDefinitions = new Map<String, AssemblyComponentDefinition>();
		var emittedDefinitions = new Map<String, Bool>();
		for (definition in definitions) {
			if (definition == null || !validName(definition.id) || localDefinitions.exists(definition.id))
				throw "Nested assembly has an invalid or duplicate component definition";
			if (definition.connectors == null || definition.connectors.length > 100)
				throw 'Nested component "${definition.id}" has invalid connectors';
			var connectorNames = new Map<String, Bool>();
			for (connector in definition.connectors) {
				if (connector == null || connector.name == null || StringTools.trim(connector.name).length == 0 ||
					connectorNames.exists(connector.name))
					throw 'Nested component "${definition.id}" has an invalid or duplicate connector';
				connectorNames.set(connector.name, true);
				AssemblyCodec.validateFrame(connector.frame);
			}
			localDefinitions.set(definition.id, definition);
		}
		var members = new Map<String, FlatMember>();
		for (occurrence in occurrences) {
			if (occurrence == null || !validName(occurrence.id) || members.exists(occurrence.id))
				throw "Nested assembly has an invalid or duplicate occurrence";
			var path = scoped(prefix, occurrence.id);
			var worldPose = AssemblyFrames.compose(pose, occurrence.initialPose);
			var connectors = new Map<String, FlatEndpoint>();
			if (occurrence.assembly != null) {
				if (occurrence.definition != occurrence.assembly || active.exists(occurrence.assembly))
					throw 'Assembly nesting contains a cycle or invalid reference at "$path"';
				var nested = library.get(occurrence.assembly);
				if (nested == null) throw 'Assembly "$path" references a missing nested definition';
				active.set(nested.id, true);
				var children = expand(path, worldPose, nested.definitions, nested.occurrences, nested.joints,
					nested.couplings, nested.mates, library, flat, active, nested.actuators, nested.encoders);
				active.remove(nested.id);
				connectors = exposed(nested.exposedConnectors, children, path);
			} else {
				var component = localDefinitions.get(occurrence.definition);
				if (component == null) throw 'Assembly "$path" references a missing component definition';
				if (!emittedDefinitions.exists(component.id)) {
					flat.definitions.push({id: scoped(prefix, component.id), connectors: component.connectors});
					emittedDefinitions.set(component.id, true);
				}
				var flatOccurrence:AssemblyComponentOccurrence = {id: path, definition: scoped(prefix, occurrence.definition), initialPose: worldPose};
				if (occurrence.grounded == true) flatOccurrence.grounded = true;
				flat.occurrences.push(flatOccurrence);
				for (connector in component.connectors)
					connectors.set(connector.name, {occurrence: path, connector: connector.name});
			}
			members.set(occurrence.id, {connectors: connectors});
		}
		for (joint in joints) {
			var parent = endpoint(members, joint.parent, joint.parentConnector, prefix);
			var child = endpoint(members, joint.child, joint.childConnector, prefix);
			var expanded:KinematicJoint = {id: scoped(prefix, joint.id), type: joint.type, role: joint.role,
				parent: parent.occurrence, parentConnector: parent.connector, child: child.occurrence,
				childConnector: child.connector, axis: joint.axis, limits: joint.limits,
				defaultValue: joint.defaultValue};
			if (joint.closureTolerance != null) expanded.closureTolerance = joint.closureTolerance;
			if (joint.driven == true) expanded.driven = true;
			flat.joints.push(expanded);
		}
		if (couplings != null) for (coupling in couplings) {
			var expanded:AssemblyJointCoupling = {id: scoped(prefix, coupling.id), source: scoped(prefix, coupling.source),
				target: scoped(prefix, coupling.target), ratio: coupling.ratio, offset: coupling.offset};
			if (coupling.efficiency != null) expanded.efficiency = coupling.efficiency;
			if (coupling.stiffness != null) expanded.stiffness = coupling.stiffness;
			if (coupling.backlash != null) expanded.backlash = coupling.backlash;
			if (coupling.drag != null) expanded.drag = coupling.drag;
			if (coupling.assumed != null && coupling.assumed.length > 0) expanded.assumed = [for (label in coupling.assumed) label];
			flat.couplings.push(expanded);
		}
		if (actuators != null) {
			if (flat.actuators == null) flat.actuators = [];
			for (actuator in actuators) {
				var copy = copyActuator(actuator, scoped(prefix, actuator.id), scoped(prefix, actuator.joint));
				if (actuator.encoder != null) copy.encoder = scoped(prefix, actuator.encoder);
				flat.actuators.push(copy);
			}
		}
		if (encoders != null) {
			if (flat.encoders == null) flat.encoders = [];
			for (encoder in encoders)
				flat.encoders.push(copyEncoder(encoder, scoped(prefix, encoder.id), scoped(prefix, encoder.joint)));
		}
		if (mates != null) for (mate in mates) {
			var first = endpoint(members, mate.first, mate.firstConnector, prefix);
			var second = endpoint(members, mate.second, mate.secondConnector, prefix);
			var expanded:AssemblyMate = {id: scoped(prefix, mate.id), kind: mate.kind, first: first.occurrence,
				firstConnector: first.connector, second: second.occurrence, secondConnector: second.connector, axis: mate.axis};
			if (mate.value != null) expanded.value = mate.value;
			if (flat.mates == null) flat.mates = [];
			flat.mates.push(expanded);
		}
		return members;
	}

	static function exposed(items:Array<AssemblyExposedConnector>, members:Map<String, FlatMember>, path:String):Map<String, FlatEndpoint> {
		var result = new Map<String, FlatEndpoint>();
		if (items != null) for (item in items) {
			if (item == null || !validExposedName(item.name) || result.exists(item.name))
				throw 'Assembly "$path" has an invalid or duplicate exposed connector';
			result.set(item.name, endpoint(members, item.occurrence, item.connector, path));
		}
		return result;
	}

	static function endpoint(members:Map<String, FlatMember>, occurrence:String, connector:String, path:String):FlatEndpoint {
		var member = members.get(occurrence);
		var result = member == null ? null : member.connectors.get(connector);
		if (result == null) throw 'Assembly "$path" has a missing connector "$occurrence.$connector"';
		return result;
	}

	static function scoped(prefix:String, name:String):String
		return prefix == "" ? name : prefix + "/" + name;

	static function validName(name:String):Bool
		return name != null && name.length > 0 && name.indexOf("/") < 0;

	/** Public connector labels may be paths; only occurrence and definition IDs are local segments. */
	static function validExposedName(name:String):Bool
		return name != null && StringTools.trim(name).length > 0 && !AssemblyCodec.containsNul(name);
}
