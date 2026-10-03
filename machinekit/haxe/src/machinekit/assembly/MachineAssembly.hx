package machinekit.assembly;

import haxe.ds.ReadOnlyArray;

import cadkit.modeling.AssemblyModel;
import cadkit.modeling.AssemblyState;
import cadkit.modeling.Vector;
import cadkit.InertiaTensor;
import machinekit.component.Bom;
import machinekit.component.BomItem;
import machinekit.component.MachineComponent;
import machinekit.component.ComponentPort;
import machinekit.component.PortInterface;
import machinekit.component.PortInterfaces;
import machinekit.component.PortRole;
import machinekit.component.ComponentValue;
import machinekit.component.ComponentValues;
import machinekit.component.MachineKitComponents;
import machinekit.assembly.MachineAssemblyDescription;
import machinekit.assembly.MachineAssemblyDescription.MemberSource;
import machinekit.assembly.MachineAssemblyDescription.SavedValue;
import machinekit.assembly.MachineAssemblyDescription.PortRecord;
import machinekit.assembly.MachineAssemblyDescription.ServiceLinkRecord;
import haxeon.wire.JsonWire;
import haxeon.Equality;
import materia.assembly.AssemblyDefinitionCodec;
import materia.assembly.AssemblyDefinitionFlattener;
import materia.assembly.AssemblyDefinition;
import materia.assembly.AssemblyDefinition.AssemblySubdefinition;
import materia.assembly.AssemblyFrames;
import materia.assembly.AssemblyDefinition.AssemblyJointType;
import materia.assembly.AssemblyDefinition.AssemblyJointRole;
import materia.assembly.AssemblyDefinition.KinematicJoint;
import materia.assembly.AssemblyDefinition.AssemblyJointLimits;
import materia.assembly.AssemblyDefinition.AssemblyVector;
import materia.assembly.AssemblyRecord.AssemblyFrame;

typedef MachineAssemblyComponent = { var id:String; var component:MachineComponent; }
typedef MachineAssemblyConnector = { var instanceId:String; var connectorName:String; }
typedef PortRef = { var instanceId:String; var portName:String; }
typedef UpstreamResult = { var port:PortRef; var external:Bool; }
private typedef MotorCompilation = {
	var actuators:Array<materia.assembly.AssemblyDefinition.AssemblyActuator>;
	var encoders:Array<materia.assembly.AssemblyDefinition.AssemblyEncoder>;
}
private typedef ServiceTrace = { var port:PortRef; var external:Bool; var supplied:Bool; var chain:Array<String>; }
typedef MachineSubassembly = { var id:String; var assembly:MachineAssembly; var pose:Null<AssemblyFrame>; }
/** BOM-only mass is a point-mass estimate, fixed or attached to a member. */
enum AssemblyBomMass {
	Unknown;
	Point(kg:Float, centreOfMass:Vector);
	Attached(kg:Float, instanceId:String, centreOfMass:Vector);
}
typedef MachineAssemblyMassProperties = {
	var mass:Float;
	var centreOfMass:Vector;
	/** Centroidal inertia in kg mm²; null if any component lacks inertia. */
	var inertia:Null<InertiaTensor>;
	var unaccounted:Array<String>;
	var unaccountedInertia:Array<String>;
}

private typedef AssemblyMember = {
	var id:String;
	var component:MachineComponent;
}
private typedef MutableIncludedRecord = {var id:String; var pose:AssemblyFrame; var mechanical:AssemblyDefinition;}

/** Reusable, prefixable assembly made from MachineComponents and named connector references. */
class MachineAssembly {
	public static inline var SCHEMA_VERSION:Int = 5;
	final members:Array<AssemblyMember> = [];
	final mechanical:AssemblyDefinition = {schemaVersion: AssemblyDefinitionCodec.VERSION,
		id: "assembly", lengthUnit: "mm", definitions: [], occurrences: [], joints: [], couplings: []};
	final included:Array<MachineSubassembly> = [];
	final externalConnectors:Array<{name:String, instanceId:String, connectorName:String}> = [];
	final externalPorts:Array<{name:String, instanceId:String, portName:String}> = [];
	final portConnections:Array<machinekit.assembly.MachineAssemblyDescription.PortConnectionRecord> = [];
	final portRecords:Array<PortRecord> = [];
	final portBridges:Array<ServiceLinkRecord> = [];
	final portConversions:Array<ServiceLinkRecord> = [];
	final bomItems:Array<{item:BomItem, quantity:Int, mass:AssemblyBomMass}> = [];
	final memberConnectorFrames:Array<{instanceId:String, name:String, frame:AssemblyFrame}> = [];
	final nestedEntries:Array<MutableIncludedRecord> = [];
	/** Couplings whose ratio their parts set, by coupling id. */
	final transmissions:Array<machinekit.assembly.MachineAssemblyDescription.TransmissionRecord> = [];
	/** Motors driving joints, by actuator id. */
	final motors:Array<machinekit.assembly.MachineAssemblyDescription.MotorRecord> = [];
	/** Encoders reading joints, by encoder id. */
	final encoders:Array<machinekit.assembly.MachineAssemblyDescription.EncoderRecord> = [];
	final massByDefinition:Map<String, machinekit.component.MassProperties> = [];

	public function new() {}

	/** Capture a portable description. Code-only members remain visible but cannot be saved. */
	public function describe():MachineAssemblyDescription {
		validateStructure();
		var mechanical = cloneDefinition(this.mechanical);
		var sources:Array<machinekit.assembly.MachineAssemblyDescription.MemberRecord> = [];
		var sourceById:Map<String, machinekit.assembly.MachineAssemblyDescription.MemberRecord> = [];
		for (member in members) {
			var component = member.component;
			var recipe = component.componentType();
			var source:MemberSource = if (recipe == null) MemberSource.Code(component.designation)
			else {
				var values = component.values();
				var names = values.names();
				names.sort(Reflect.compare);
				MemberSource.Typed(recipe.id, [for (name in names) {name: name, value: switch values.get(name) {
					case Number(value): SavedValue.Number(value);
					case Integer(value): SavedValue.Integer(value);
					case Boolean(value): SavedValue.Boolean(value);
					case Token(value): SavedValue.Token(value);
					case Unset: SavedValue.Unset;
					case null: throw 'Missing value "$name"';
				}}]);
			};
			var record = {occurrence: member.id, source: source, material: component.materialSpec()};
			sources.push(record);
			sourceById.set(member.id, record);
		}
		// The model initially gives every occurrence its own connector definition.
		// Intern definitions with the same recipe identity and connector geometry.
		mechanical.occurrences.sort((a, b) -> Reflect.compare(a.id, b.id));
		var shared:Map<String, Array<String>> = [];
		var kept:Array<materia.assembly.AssemblyDefinition.AssemblyComponentDefinition> = [];
		for (occurrence in mechanical.occurrences) {
			var source = sourceById.get(occurrence.id);
			if (source == null) throw 'Missing source for "${occurrence.id}"';
			var definition = null;
			for (candidate in mechanical.definitions) if (candidate.id == occurrence.definition) definition = candidate;
			if (definition == null) throw 'Missing mechanical definition for "${occurrence.id}"';
			var key = definitionKey(occurrence.id, requireMember(occurrence.id));
			var candidates = shared.get(key);
			var existing:Null<String> = null;
			if (candidates != null) for (id in candidates) for (item in kept)
				if (item.id == id && Equality.equals(item.connectors, definition.connectors)) existing = id;
			if (existing != null) occurrence.definition = existing;
			else {
				kept.push(definition);
				if (candidates == null) {candidates = []; shared.set(key, candidates);}
				candidates.push(definition.id);
			}
		}
		mechanical.definitions = kept;
		mechanical.exposedConnectors = [for (entry in externalConnectors) {name: entry.name,
			occurrence: entry.instanceId, connector: entry.connectorName}];
		if (nestedEntries.length > 0) mechanical = nestedMechanical(mechanical);
		canonicalizeDefinition(mechanical);
		sources.sort((a, b) -> Reflect.compare(a.occurrence, b.occurrence));
		var emptyTools:Array<machinekit.assembly.MachineAssemblyDescription.ToolRecord> = [];
		var savedPorts:Array<PortRecord> = [];
		for (port in portRecords) savedPorts.push({occurrence: port.occurrence, name: port.name,
			kind: port.kind, role: port.role, iface: port.iface, required: port.required,
			connector: port.connector});
		savedPorts.sort((a, b) -> Reflect.compare(a.occurrence + "/" + a.name, b.occurrence + "/" + b.name));
		var savedConnections = [for (connection in portConnections) {id: connection.id,
			fromInstance: connection.fromInstance, fromPort: connection.fromPort,
			toInstance: connection.toInstance, toPort: connection.toPort}];
		savedConnections.sort((a, b) -> Reflect.compare(a.id, b.id));
		return {schemaVersion: SCHEMA_VERSION, mechanical: FrozenAssemblyDefinitions.freeze(mechanical), machine: {
			members: sources,
			ports: savedPorts,
			included: [for (entry in nestedEntries) {id: entry.id, pose: copyFrame(entry.pose),
				mechanical: FrozenAssemblyDefinitions.freeze(entry.mechanical)}],
			portConnections: savedConnections,
			portExposures: [for (entry in externalPorts) {name: entry.name,
				instanceId: entry.instanceId, portName: entry.portName}],
			connectorExposures: [for (entry in externalConnectors) {name: entry.name,
				instanceId: entry.instanceId, connectorName: entry.connectorName}],
			memberConnectors: [for (entry in memberConnectorFrames) {instanceId: entry.instanceId,
				name: entry.name, frame: copyFrame(entry.frame)}],
			tools: emptyTools,
			transmissions: {
				var saved = [for (transmission in transmissions) copyTransmission(transmission, "")];
				saved.sort((a, b) -> Reflect.compare(a.coupling, b.coupling));
				// Left out when empty, so descriptions without transmissions keep their saved form.
				saved.length == 0 ? null : saved;
			},
			motors: {
				var saved = [for (motor in motors) copyMotor(motor, "")];
				saved.sort((a, b) -> Reflect.compare(a.actuator, b.actuator));
				saved.length == 0 ? null : saved;
			},
			encoders: {
				var saved = [for (encoder in encoders) copyEncoder(encoder, "")];
				saved.sort((a, b) -> Reflect.compare(a.encoder, b.encoder));
				saved.length == 0 ? null : saved;
			},
			bomExtras: [for (entry in bomItems) {item: copyBomItem(entry.item), quantity: entry.quantity,
				mass: switch entry.mass {
					case Unknown: machinekit.assembly.MachineAssemblyDescription.SavedBomMass.Unknown;
					case Point(kg, centre): machinekit.assembly.MachineAssemblyDescription.SavedBomMass.Point(kg, centre.x, centre.y, centre.z);
					case Attached(kg, id, centre): machinekit.assembly.MachineAssemblyDescription.SavedBomMass.Attached(kg, id, centre.x, centre.y, centre.z);
				}}]
		}};
	}

	static function canonicalizeDefinition(definition:AssemblyDefinition):Void {
		definition.definitions.sort((a, b) -> Reflect.compare(a.id, b.id));
		for (component in definition.definitions)
			component.connectors.sort((a, b) -> Reflect.compare(a.name, b.name));
		definition.occurrences.sort((a, b) -> Reflect.compare(a.id, b.id));
		definition.joints.sort((a, b) -> Reflect.compare(a.id, b.id));
		if (definition.couplings != null) definition.couplings.sort((a, b) -> Reflect.compare(a.id, b.id));
		if (definition.exposedConnectors != null)
			definition.exposedConnectors.sort((a, b) -> Reflect.compare(a.name, b.name));
		if (definition.assemblies != null) {
			definition.assemblies.sort((a, b) -> Reflect.compare(a.id, b.id));
			for (nested in definition.assemblies) {
				nested.definitions.sort((a, b) -> Reflect.compare(a.id, b.id));
				for (component in nested.definitions)
					component.connectors.sort((a, b) -> Reflect.compare(a.name, b.name));
				nested.occurrences.sort((a, b) -> Reflect.compare(a.id, b.id));
				nested.joints.sort((a, b) -> Reflect.compare(a.id, b.id));
				if (nested.couplings != null) nested.couplings.sort((a, b) -> Reflect.compare(a.id, b.id));
				if (nested.exposedConnectors != null)
					nested.exposedConnectors.sort((a, b) -> Reflect.compare(a.name, b.name));
			}
		}
	}

	/** Generated wire codec; reject every unbuildable member before emitting a file. */
	public function encode():String {
		var description = describe();
		var problems = new Diagnostics();
		checkSavableMembers(description.machine.members, "", problems);
		if (description.machine.tools != null) for (tool in description.machine.tools)
			checkSavableMembers(tool.machine.members, tool.id + "/", problems);
		problems.throwIfErrors();
		return JsonWire.encode(description);
	}

	static function checkSavableMembers(members:haxe.ds.ReadOnlyArray<machinekit.assembly.MachineAssemblyDescription.MemberRecord>,
			prefix:String, problems:Diagnostics):Void
		for (member in members) switch member.source {
			case Code(designation): problems.error("assembly.code-member", prefix + member.occurrence,
				'Code-only member "$designation" has no rebuild recipe');
			case _:
		}

	public static function decode(text:String):MachineAssembly {
		var version:machinekit.assembly.MachineAssemblyDescription.DescriptionVersion = JsonWire.decode(text);
		checkDescriptionVersion(version.schemaVersion);
		return fromDescription(JsonWire.decode(text));
	}

	static function checkDescriptionVersion(version:Null<Int>):Void {
		if (version != SCHEMA_VERSION) throw 'Machine assembly schema v$version is unsupported; expected v$SCHEMA_VERSION typed gearbox members';
	}

	/** Rebuild through registered recipes; no component object is stored in the description. */
	public static function fromDescription(description:MachineAssemblyDescription):MachineAssembly {
		if (description == null || description.machine == null) throw "Missing machine assembly description";
		checkDescriptionVersion(description.schemaVersion);
		var savedMechanical = FrozenAssemblyDefinitions.thaw(description.mechanical);
		AssemblyDefinitionCodec.validate(savedMechanical);
		var mechanical = AssemblyDefinitionFlattener.flatten(savedMechanical);
		var result = new MachineAssembly();
		var sources:Map<String, machinekit.assembly.MachineAssemblyDescription.MemberRecord> = [];
		for (member in description.machine.members) {
			if (sources.exists(member.occurrence)) throw 'Duplicate member source "${member.occurrence}"';
			sources.set(member.occurrence, member);
		}
		for (occurrence in mechanical.occurrences) {
			var member = sources.get(occurrence.id);
			if (member == null) throw 'Missing member source "${occurrence.id}"';
			var component = switch member.source {
				case Code(designation): throw 'Code-only member "$designation" has no rebuild recipe';
				case Typed(typeId, saved):
					var values = new ComponentValues();
					for (entry in saved) values.set(entry.name, switch entry.value {
						case SavedValue.Number(value): ComponentValue.Number(value);
						case SavedValue.Integer(value): ComponentValue.Integer(value);
						case SavedValue.Boolean(value): ComponentValue.Boolean(value);
						case SavedValue.Token(value): ComponentValue.Token(value);
						case SavedValue.Unset: ComponentValue.Unset;
					});
					MachineKitComponents.byId(typeId).create(values);
			};
			component.setMaterial(member.material);
			result.addComponentAt(InstancePath.of(occurrence.id), component, occurrence.initialPose);
		}
		// Port records are snapshots for readers. Recipes own their current interfaces and required flags.

		for (connector in description.machine.memberConnectors)
			result.addMemberConnector(connector.instanceId, connector.name, connector.frame);
		for (joint in mechanical.joints) {
			if (joint.role == materia.assembly.AssemblyDefinition.AssemblyJointRole.Tree)
				result.addMateOnAxis(joint.id, joint.type, joint.parent, joint.parentConnector,
					joint.child, joint.childConnector, joint.axis, joint.defaultValue, joint.limits);
			else result.addConstraintOnAxis(joint.id, joint.type, joint.parent, joint.parentConnector,
				joint.child, joint.childConnector, joint.axis, joint.closureTolerance, joint.limits);
		}
		if (mechanical.couplings != null) for (coupling in mechanical.couplings)
			result.addCoupling(coupling.id, coupling.source, coupling.target, coupling.ratio, coupling.offset,
				coupling.efficiency, coupling.stiffness, coupling.backlash, coupling.drag, coupling.assumed);
		// A driven coupling's ratio comes from its parts as they are now, not as they were saved.
		// The coupling decides whether there is one: a transmission whose coupling was removed goes too.
		if (description.machine.transmissions != null) for (transmission in description.machine.transmissions)
			if (result.applyTransmission(transmission)) result.transmissions.push(copyTransmission(transmission, ""));
		for (connection in description.machine.portConnections)
			result.connectPorts(connection.id, connection.fromInstance, connection.fromPort,
				connection.toInstance, connection.toPort);
		for (entry in description.machine.portExposures)
			result.exposePort(entry.name, entry.instanceId, entry.portName);
		// Motors too: their actuators follow the motor parts as they are now.
		if (description.machine.motors != null) for (motor in description.machine.motors)
			result.addMotorRecord(copyMotor(motor, ""));
		// Encoders after the motors they read, whose actuators they point at.
		if (description.machine.encoders != null) for (encoder in description.machine.encoders)
			result.addEncoderRecord(copyEncoder(encoder, ""));

		for (entry in description.machine.connectorExposures)
			result.exposeConnector(entry.name, entry.instanceId, entry.connectorName);
		for (entry in description.machine.bomExtras) {
			var mass:AssemblyBomMass = switch entry.mass {
				case machinekit.assembly.MachineAssemblyDescription.SavedBomMass.Unknown: AssemblyBomMass.Unknown;
				case machinekit.assembly.MachineAssemblyDescription.SavedBomMass.Point(kg, x, y, z): AssemblyBomMass.Point(kg, new Vector(x, y, z));
				case machinekit.assembly.MachineAssemblyDescription.SavedBomMass.Attached(kg, id, x, y, z): AssemblyBomMass.Attached(kg, id, new Vector(x, y, z));
			};
			result.addBomItem(entry.item, entry.quantity, mass);
		}
		if (description.machine.included != null) for (entry in description.machine.included)
			result.nestedEntries.push({id: entry.id, pose: copyFrame(entry.pose),
				mechanical: FrozenAssemblyDefinitions.thaw(entry.mechanical)});
		for (entry in result.nestedEntries)
			result.included.push({id: entry.id, assembly: result.rebuildIncluded(entry.id, entry.pose),
				pose: copyFrame(entry.pose)});
		return result;
	}

	function rebuildIncluded(id:String, pose:AssemblyFrame):MachineAssembly {
		var prefix = id + "/";
		var child = new MachineAssembly();
		var inverse = AssemblyFrames.inverse(pose);
		for (occurrence in mechanical.occurrences) if (StringTools.startsWith(occurrence.id, prefix))
			child.addComponentAt(InstancePath.of(occurrence.id.substr(prefix.length)),
				copyComponent(requireMember(occurrence.id)),
				AssemblyFrames.compose(inverse, occurrence.initialPose));
		for (connector in memberConnectorFrames) if (StringTools.startsWith(connector.instanceId, prefix))
			child.addMemberConnector(connector.instanceId.substr(prefix.length), connector.name, connector.frame);
		for (joint in mechanical.joints) if (StringTools.startsWith(joint.id, prefix) &&
			StringTools.startsWith(joint.parent, prefix) && StringTools.startsWith(joint.child, prefix)) {
			var parent = joint.parent.substr(prefix.length), member = joint.child.substr(prefix.length);
			if (joint.role == AssemblyJointRole.Tree)
				child.addMateOnAxis(joint.id.substr(prefix.length), joint.type, parent, joint.parentConnector,
					member, joint.childConnector, joint.axis, joint.defaultValue, joint.limits);
			else child.addConstraintOnAxis(joint.id.substr(prefix.length), joint.type, parent,
					joint.parentConnector, member, joint.childConnector, joint.axis,
					joint.closureTolerance, joint.limits);
		}
		if (mechanical.couplings != null) for (coupling in mechanical.couplings)
			if (StringTools.startsWith(coupling.id, prefix))
				child.addCoupling(coupling.id.substr(prefix.length), coupling.source.substr(prefix.length),
					coupling.target.substr(prefix.length), coupling.ratio, coupling.offset, coupling.efficiency,
					coupling.stiffness, coupling.backlash, coupling.drag, coupling.assumed);
		for (motor in motors) if (StringTools.startsWith(motor.actuator, prefix))
			child.addMotorRecord({actuator: motor.actuator.substr(prefix.length), joint: motor.joint.substr(prefix.length),
				motor: motor.motor.substr(prefix.length), driver: motor.driver.substr(prefix.length), margin: motor.margin,
				gearbox: motor.gearbox == null ? null : motor.gearbox.substr(prefix.length)});
		for (encoder in encoders) if (StringTools.startsWith(encoder.encoder, prefix))
			child.addEncoderRecord({encoder: encoder.encoder.substr(prefix.length), joint: encoder.joint.substr(prefix.length),
				part: encoder.part.substr(prefix.length),
				actuator: encoder.actuator == null ? null : encoder.actuator.substr(prefix.length)});
		for (transmission in transmissions) if (StringTools.startsWith(transmission.coupling, prefix))
			{
				var record:machinekit.assembly.MachineAssemblyDescription.TransmissionRecord = {coupling: transmission.coupling.substr(prefix.length),
					source: machinekit.transmission.TransmissionResolver.mapSource(transmission.source, member -> member.substr(prefix.length)),
					sense: transmission.sense, leaderZero: transmission.leaderZero};
				record.stiffness = transmission.stiffness;
				record.backlash = transmission.backlash;
				record.drag = transmission.drag;
				record.near = transmission.near;
				record.far = transmission.far;
				record.unsupported = transmission.unsupported;
				child.applyTransmission(record);
				child.transmissions.push(record);
			}
		for (connection in portConnections) if (StringTools.startsWith(connection.id, prefix) &&
			StringTools.startsWith(connection.fromInstance, prefix) &&
			StringTools.startsWith(connection.toInstance, prefix))
			child.connectPorts(connection.id.substr(prefix.length),
				connection.fromInstance.substr(prefix.length), connection.fromPort,
				connection.toInstance.substr(prefix.length), connection.toPort);
		for (connector in externalConnectors) if (StringTools.startsWith(connector.name, prefix) &&
			StringTools.startsWith(connector.instanceId, prefix))
			child.exposeConnector(connector.name.substr(prefix.length),
				connector.instanceId.substr(prefix.length), connector.connectorName);
		return child;
	}

	/** Copy builder state for a derived assembly or an owned tool snapshot. */
	public function copyInto(target:MachineAssembly):Void {
		for (member in members) target.members.push({id: member.id,
			component: copyComponent(member.component)});
		var copy = cloneDefinition(mechanical);
		target.mechanical.definitions = copy.definitions;
		target.mechanical.occurrences = copy.occurrences;
		target.mechanical.joints = copy.joints;
		target.mechanical.couplings = copy.couplings;
		for (transmission in transmissions) target.transmissions.push(copyTransmission(transmission, ""));
		for (motor in motors) target.motors.push(copyMotor(motor, ""));
		for (encoder in encoders) target.encoders.push(copyEncoder(encoder, ""));
		for (entry in included) target.included.push({id: entry.id,
			assembly: entry.assembly.snapshot(), pose: entry.pose == null ? null : copyFrame(entry.pose)});
		for (entry in nestedEntries) target.nestedEntries.push({id: entry.id,
			pose: copyFrame(entry.pose), mechanical: cloneDefinition(entry.mechanical)});
		for (entry in externalConnectors) target.externalConnectors.push({name: entry.name,
			instanceId: entry.instanceId, connectorName: entry.connectorName});
		for (entry in externalPorts) target.externalPorts.push({name: entry.name,
			instanceId: entry.instanceId, portName: entry.portName});
		for (entry in portConnections) target.portConnections.push({id: entry.id,
			fromInstance: entry.fromInstance, fromPort: entry.fromPort,
			toInstance: entry.toInstance, toPort: entry.toPort});
		for (entry in portRecords) target.portRecords.push({occurrence: entry.occurrence,
			name: entry.name, kind: entry.kind, role: entry.role, iface: entry.iface,
			required: entry.required, connector: entry.connector});
		for (entry in portBridges) target.portBridges.push({occurrence: entry.occurrence,
			fromPort: entry.fromPort, toPort: entry.toPort});
		for (entry in portConversions) target.portConversions.push({occurrence: entry.occurrence,
			fromPort: entry.fromPort, toPort: entry.toPort});
		for (entry in bomItems) target.bomItems.push({item: copyBomItem(entry.item), quantity: entry.quantity,
			mass: entry.mass});
		for (entry in memberConnectorFrames) target.memberConnectorFrames.push({instanceId: entry.instanceId,
			name: entry.name, frame: copyFrame(entry.frame)});
	}

	public function snapshot():MachineAssembly {
		var result = new MachineAssembly();
		copyInto(result);
		return result;
	}

	static function copyComponent(component:MachineComponent):MachineComponent {
		var recipe = component.componentType();
		if (recipe == null) return component;
		var result = recipe.create(component.values());
		result.setMaterial(component.materialSpec());
		return result;
	}

	static function copyBomItem(item:BomItem):BomItem return {partNumber: item.partNumber,
		description: item.description, quantity: item.quantity, material: item.material,
		typeId: item.typeId, valuesKey: item.valuesKey};

	static function cloneDefinition(value:AssemblyDefinition):AssemblyDefinition
		return AssemblyDefinitionCodec.decode(AssemblyDefinitionCodec.encode(value));

	/** Move included members into reusable subdefinitions while retaining flat side-record IDs. */
	function nestedMechanical(flat:AssemblyDefinition):AssemblyDefinition {
		flat.assemblies = [];
		for (index in 0...nestedEntries.length) {
			var entry = nestedEntries[index];
			var child = cloneDefinition(entry.mechanical);
			var prefix = entry.id + "/";
			syncNestedConnectors(child.definitions, child.occurrences, child.assemblies,
				prefix, flat);
			var subId = 'machinekit-sub-$index';
			var imported:Map<String, String> = [];
			if (child.assemblies != null) for (sub in child.assemblies)
				imported.set(sub.id, subId + "-" + sub.id);
			if (child.assemblies != null) for (sub in child.assemblies) {
				sub.id = imported.get(sub.id);
				for (occurrence in sub.occurrences) if (occurrence.assembly != null) {
					occurrence.assembly = imported.get(occurrence.assembly);
					occurrence.definition = occurrence.assembly;
				}
				flat.assemblies.push(sub);
			}
			for (occurrence in child.occurrences) if (occurrence.assembly != null) {
				occurrence.assembly = imported.get(occurrence.assembly);
				occurrence.definition = occurrence.assembly;
			}
			var sub:AssemblySubdefinition = {id: subId, definitions: child.definitions,
				occurrences: child.occurrences, joints: child.joints,
				couplings: child.couplings == null ? [] : child.couplings,
				exposedConnectors: child.exposedConnectors == null ? [] : child.exposedConnectors};
			flat.assemblies.push(sub);
			var childFlat = AssemblyDefinitionFlattener.flatten(entry.mechanical);
			var internalJoints:Map<String, Bool> = [];
			for (joint in childFlat.joints) internalJoints.set(prefix + joint.id, true);
			flat.joints = [for (joint in flat.joints) if (!internalJoints.exists(joint.id)) joint];
			var internalCouplings:Map<String, Bool> = [];
			if (childFlat.couplings != null) for (coupling in childFlat.couplings)
				internalCouplings.set(prefix + coupling.id, true);
			if (flat.couplings != null)
				flat.couplings = [for (coupling in flat.couplings) if (!internalCouplings.exists(coupling.id)) coupling];
			var keptOccurrences:Array<materia.assembly.AssemblyDefinition.AssemblyComponentOccurrence> = [];
			var insertAt = -1;
			for (occurrence in flat.occurrences) {
				if (StringTools.startsWith(occurrence.id, prefix)) {
					if (insertAt < 0) insertAt = keptOccurrences.length;
				} else keptOccurrences.push(occurrence);
			}
			if (insertAt < 0) throw 'Included assembly "${entry.id}" has no members';
			keptOccurrences.insert(insertAt, {id: entry.id, definition: subId,
				assembly: subId, initialPose: copyFrame(entry.pose)});
			flat.occurrences = keptOccurrences;
			for (joint in flat.joints) {
				if (StringTools.startsWith(joint.parent, prefix)) {
					joint.parentConnector = exposeNested(sub, flat.assemblies,
						joint.parent.substr(prefix.length), joint.parentConnector);
					joint.parent = entry.id;
				}
				if (StringTools.startsWith(joint.child, prefix)) {
					joint.childConnector = exposeNested(sub, flat.assemblies,
						joint.child.substr(prefix.length), joint.childConnector);
					joint.child = entry.id;
				}
			}
			if (flat.exposedConnectors != null) for (connector in flat.exposedConnectors)
				if (StringTools.startsWith(connector.occurrence, prefix)) {
					connector.connector = exposeNested(sub, flat.assemblies,
						connector.occurrence.substr(prefix.length), connector.connector);
					connector.occurrence = entry.id;
				}
		}
		var referenced:Map<String, Bool> = [];
		for (occurrence in flat.occurrences) if (occurrence.assembly == null)
			referenced.set(occurrence.definition, true);
		flat.definitions = [for (definition in flat.definitions) if (referenced.exists(definition.id)) definition];
		AssemblyDefinitionCodec.validate(flat);
		return flat;
	}

	static function syncNestedConnectors(
			definitions:Array<materia.assembly.AssemblyDefinition.AssemblyComponentDefinition>,
			occurrences:Array<materia.assembly.AssemblyDefinition.AssemblyComponentOccurrence>,
			library:Array<AssemblySubdefinition>, prefix:String, current:AssemblyDefinition):Void {
		for (occurrence in occurrences) {
			if (occurrence.assembly != null) {
				var nested = requireSubdefinition(library, occurrence.assembly);
				syncNestedConnectors(nested.definitions, nested.occurrences, library,
					prefix + occurrence.id + "/", current);
				continue;
			}
			var currentId = prefix + occurrence.id;
			var currentOccurrence:Null<materia.assembly.AssemblyDefinition.AssemblyComponentOccurrence> = null;
			for (candidate in current.occurrences) if (candidate.id == currentId) currentOccurrence = candidate;
			if (currentOccurrence == null) throw 'Missing included member "$currentId"';
			var live:Null<materia.assembly.AssemblyDefinition.AssemblyComponentDefinition> = null;
			var destination:Null<materia.assembly.AssemblyDefinition.AssemblyComponentDefinition> = null;
			for (candidate in current.definitions) if (candidate.id == currentOccurrence.definition) live = candidate;
			for (candidate in definitions) if (candidate.id == occurrence.definition) destination = candidate;
			if (live == null || destination == null) throw 'Missing included definition "$currentId"';
			destination.connectors = [for (connector in live.connectors)
				{name: connector.name, frame: copyFrame(connector.frame)}];
		}
	}

	static function exposeNested(sub:AssemblySubdefinition, library:Array<AssemblySubdefinition>,
			path:String, connector:String):String {
		var direct = false;
		for (candidate in sub.occurrences) if (candidate.id == path) direct = true;
		if (!direct) {
			var slash = path.indexOf("/");
			if (slash < 0) throw 'Missing nested member "$path"';
			var head = path.substr(0, slash), tail = path.substr(slash + 1);
			var found = requireNestedOccurrence(sub, head);
			if (found.assembly == null) throw 'Missing nested member "$path"';
			var child = requireSubdefinition(library, found.assembly);
			connector = exposeNested(child, library, tail, connector);
			path = head;
		}
		if (sub.exposedConnectors == null) sub.exposedConnectors = [];
		for (exposed in sub.exposedConnectors)
			if (exposed.occurrence == path && exposed.connector == connector) return exposed.name;
		var name = '__machinekit_${sub.exposedConnectors.length}';
		sub.exposedConnectors.push({name: name, occurrence: path, connector: connector});
		return name;
	}

	static function requireNestedOccurrence(sub:AssemblySubdefinition, id:String):
			materia.assembly.AssemblyDefinition.AssemblyComponentOccurrence {
		for (candidate in sub.occurrences) if (candidate.id == id) return candidate;
		throw 'Missing nested member "$id"';
	}

	static function requireSubdefinition(library:Array<AssemblySubdefinition>, id:String):AssemblySubdefinition {
		for (candidate in library) if (candidate.id == id) return candidate;
		throw 'Missing nested definition for "$id"';
	}

	public function addComponent(id:String, component:MachineComponent, ?pose:AssemblyFrame):Void {
		InstancePath.segment(id);
		addComponentPath(id, component, pose);
	}

	public function addComponentAt(path:InstancePath, component:MachineComponent, ?pose:AssemblyFrame):Void
		addComponentPath(path, component, pose);

	function addComponentPath(id:String, component:MachineComponent, ?pose:AssemblyFrame):Void {
		if (id == null || id.length == 0 || component == null) throw "Assembly component needs an id and component";
		InstancePath.of(id);
		for (member in members) if (member.id == id) throw 'Duplicate assembly component "$id"';
		members.push({id: id, component: component});
		mechanical.definitions.push({id: id, connectors: [for (connector in component.connectors())
			{name: connector.name, frame: copyFrame(connector.frame)}]});
		mechanical.occurrences.push({id: id, definition: id,
			initialPose: pose == null ? AssemblyFrames.identity() : copyFrame(pose)});
		for (port in component.ports()) portRecords.push({occurrence: id, name: port.name,
			kind: port.kind, role: port.role, iface: port.iface, required: port.required,
			connector: port.connector});
		for (link in component.bridges()) portBridges.push({occurrence: id,
			fromPort: link.from, toPort: link.to});
		for (link in component.conversions()) portConversions.push({occurrence: id,
			fromPort: link.from, toPort: link.to});
	}

	/** Include another assembly below a local namespace, optionally moving all of its parts. */
	public function include(id:String, assembly:MachineAssembly, ?pose:AssemblyFrame):Void {
		if (id == null || id.length == 0 || assembly == null || assembly == this)
			throw "Included assembly needs a distinct id and assembly";
		InstancePath.segment(id);
		for (entry in included) if (entry.id == id) throw 'Duplicate included assembly "$id"';
		assembly.validateStructure();
		for (member in assembly.members) {
			var memberPose = assembly.occurrencePose(member.id);
			var localPose = pose == null ? memberPose : AssemblyFrames.compose(pose, memberPose);
			addComponentPath(join(id, member.id), member.component, localPose);
		}
		for (connector in assembly.memberConnectorFrames)
			addMemberConnector(join(id, connector.instanceId), connector.name, connector.frame);
		for (connector in assembly.externalConnectors)
			exposeConnector(join(id, connector.name), join(id, connector.instanceId), connector.connectorName);
		// Ports define required service inputs, so each containing assembly must expose its own interface.
		for (joint in assembly.mechanical.joints) addJoint({id: join(id, joint.id), type: joint.type,
			role: joint.role, parent: join(id, joint.parent), parentConnector: joint.parentConnector,
			child: join(id, joint.child), childConnector: joint.childConnector,
			axis: joint.axis, limits: joint.limits, defaultValue: joint.defaultValue},
			joint.closureTolerance);
		if (assembly.mechanical.couplings != null) for (coupling in assembly.mechanical.couplings)
			addCoupling(join(id, coupling.id), join(id, coupling.source), join(id, coupling.target),
				coupling.ratio, coupling.offset, coupling.efficiency, coupling.stiffness, coupling.backlash, coupling.drag, coupling.assumed);
		for (transmission in assembly.transmissions) {
			var record = copyTransmission(transmission, id);
			applyTransmission(record);
			transmissions.push(record);
		}
		for (motor in assembly.motors) addMotorRecord(copyMotor(motor, id));
		for (encoder in assembly.encoders) addEncoderRecord(copyEncoder(encoder, id));
		for (connection in assembly.portConnections)
			connectPorts(join(id, connection.id), join(id, connection.fromInstance),
				connection.fromPort, join(id, connection.toInstance), connection.toPort);
		for (entry in assembly.bomItems) {
			var mass = switch entry.mass {
				case AssemblyBomMass.Unknown: AssemblyBomMass.Unknown;
				case AssemblyBomMass.Attached(kg, instanceId, centre): AssemblyBomMass.Attached(kg, join(id, instanceId), centre);
				case AssemblyBomMass.Point(kg, centre):
					if (pose == null) AssemblyBomMass.Point(kg, centre);
					else {
						var point = AssemblyFrames.transformPoint(pose, centre.x, centre.y, centre.z);
						AssemblyBomMass.Point(kg, new Vector(point.x, point.y, point.z));
					}
			};
			addBomItem(entry.item, entry.quantity, mass);
		}
		included.push({id: id, assembly: assembly.snapshot(), pose: pose == null ? null : copyFrame(pose)});
		nestedEntries.push({id: id, pose: pose == null ? AssemblyFrames.identity() : copyFrame(pose),
			mechanical: FrozenAssemblyDefinitions.thaw(assembly.describe().mechanical)});
	}

	public function addMate(id:String, kind:String, parent:String, parentConnector:String,
			child:String, childConnector:String, value:Float = 0):Void
		addMateOnAxis(id, kind, parent, parentConnector, child, childConnector,
			{x: 0, y: 1, z: 0}, value);

	public function addMateOnAxis(id:String, kind:String, parent:String, parentConnector:String,
			child:String, childConnector:String, axis:AssemblyVector, value:Float = 0,
			?limits:AssemblyJointLimits):Void
		addJoint({id: id, type: cast kind, role: AssemblyJointRole.Tree, parent: parent,
			parentConnector: parentConnector, child: child, childConnector: childConnector,
			axis: axis, limits: resolvedLimits(limits), defaultValue: value});

	public function addConstraint(id:String, kind:String, parent:String, parentConnector:String,
			child:String, childConnector:String, ?tolerance:Float):Void
		addConstraintOnAxis(id, kind, parent, parentConnector, child, childConnector,
			{x: 0, y: 1, z: 0}, tolerance);

	public function addConstraintOnAxis(id:String, kind:String, parent:String, parentConnector:String,
			child:String, childConnector:String, axis:AssemblyVector, ?tolerance:Float,
			?limits:AssemblyJointLimits):Void
		addJoint({id: id, type: cast kind, role: AssemblyJointRole.Closure, parent: parent,
			parentConnector: parentConnector, child: child, childConnector: childConnector,
			axis: axis, limits: resolvedLimits(limits), defaultValue: 0}, tolerance);

	/**
	 * Couple joint `follower` to `leader` through the parts of `source`, which set the ratio: the
	 * follower sits at zero where the leader is at `leaderZero`. Rebuilding the assembly from its
	 * description works the ratio out again from the parts' values then. Returns the ratio.
	 */
	public function addTransmission(id:String, leader:String, follower:String, source:Transmission,
			sense:Sense = Same, leaderZero:Float = 0):Float {
		var record:machinekit.assembly.MachineAssemblyDescription.TransmissionRecord = {
			coupling: id, source: source, sense: sense, leaderZero: leaderZero};
		var relation = machinekit.transmission.TransmissionResolver.resolve(record, requireMember);
		var ratio = relation.ratio;
		addCoupling(id, leader, follower, ratio, -ratio * leaderZero, relation.efficiency);
		writeTransmission(record, relation);
		transmissions.push(record);
		return ratio;
	}

	/** State measured or datasheet allowances explicitly; rebuilding keeps these overrides. */
	public function setTransmissionOverrides(id:String, ?stiffness:Float, ?backlash:Float, ?drag:Float):Void {
		for (entry in transmissions) if (entry.coupling == id) {
			var proposed = copyTransmission(entry, "");
			proposed.stiffness = stiffness;
			proposed.backlash = backlash;
			proposed.drag = drag;
			var relation = machinekit.transmission.TransmissionResolver.resolve(proposed, requireMember);
			entry.stiffness = stiffness;
			entry.backlash = backlash;
			entry.drag = drag;
			writeTransmission(entry, relation);
			return;
		}
		throw 'No transmission "$id"';
	}

	/**
	 * Says how the ends of lead screw transmission `id` are held (`near` is the end by its motor), over
	 * `unsupported` mm (the whole screw by default). The screw's joint is then capped at 80% of its first
	 * bending speed (`LeadScrew.criticalSpeed`); the cap flows through `RobotModel.coupledLimits` to
	 * its axis. Call it once the screw's joint exists. Returns the cap in rad/s.
	 */
	public function supportScrew(id:String, near:machinekit.motion.ScrewSupport, far:machinekit.motion.ScrewSupport,
			?unsupported:Float):Float {
		for (entry in transmissions) if (entry.coupling == id) {
			if (!switch entry.source { case LeadScrew(_, _): true; default: false; }) throw 'Transmission "$id" is not a lead screw';
			var proposed = copyTransmission(entry, "");
			proposed.near = near;
			proposed.far = far;
			proposed.unsupported = unsupported;
			var relation = machinekit.transmission.TransmissionResolver.resolve(proposed, requireMember);
			entry.near = near;
			entry.far = far;
			entry.unsupported = unsupported;
			writeTransmission(entry, relation);
			var cap = relation.followerSpeedCap;
			if (cap == null) throw 'Transmission "$id" has no supports';
			return cap;
		}
		throw 'No transmission "$id"';
	}

	/** The transmission behind coupling `id`, or null when its ratio is a plain number. */
	public function transmissionFor(id:String):Null<machinekit.assembly.MachineAssemblyDescription.TransmissionRecord> {
		for (entry in transmissions) if (entry.coupling == id) return copyTransmission(entry, "");
		return null;
	}

	/** Recompute all derived coupling data together from the transmission's parts. */
	function applyTransmission(transmission:machinekit.assembly.MachineAssemblyDescription.TransmissionRecord):Bool {
		if (mechanical.couplings != null) for (coupling in mechanical.couplings) if (coupling.id == transmission.coupling) {
			return writeTransmission(transmission, machinekit.transmission.TransmissionResolver.resolve(transmission, requireMember));
		}
		return false;
	}

	function writeTransmission(transmission:machinekit.assembly.MachineAssemblyDescription.TransmissionRecord,
			relation:machinekit.transmission.TransmissionRelation):Bool {
		if (mechanical.couplings != null) for (coupling in mechanical.couplings) if (coupling.id == transmission.coupling) {
			coupling.ratio = relation.ratio;
			coupling.offset = -relation.ratio * transmission.leaderZero;
			coupling.efficiency = relation.efficiency;
			coupling.stiffness = relation.stiffness;
			coupling.backlash = relation.backlash;
			coupling.drag = relation.drag;
			var assumed = relation.assumed;
			coupling.assumed = assumed.length == 0 ? null : assumed;
			if (relation.followerSpeedCap != null) for (joint in mechanical.joints)
				if (joint.id == coupling.target) joint.limits.velocity = relation.followerSpeedCap;
			return true;
		}
		return false;
	}

	/**
	 * Motor member `motor` (a stepper, or any part that is a `MotorDrive`) drives joint `joint` on a
	 * driver member `driver`. Its actuator, `id`, gets the motor's drive kind and torque-speed curve, its rotor
	 * inertia and its usable effort and rate: for a stepper `margin` of its holding torque and the
	 * speed up to where pull-out torque falls to that, for a servo its peak torque and maximum speed.
	 * Rebuilding the assembly works them out again from the motor and driver settings.
	 */
	public function addMotor(id:String, joint:String, motor:String, driver:String, margin:Float = 0.5, ?gearbox:String):Void
		addMotorRecord({actuator: id, joint: joint, motor: motor, driver: driver, margin: margin,
			gearbox: gearbox});

	function motorDriver(record:machinekit.assembly.MachineAssemblyDescription.MotorRecord):machinekit.motion.MotorDriver {
		var member = requireMember(record.motor);
		if (!Std.isOfType(member, machinekit.motion.MotorDrive)) throw 'Motor "${record.actuator}": "${record.motor}" is not a motor part';
		var driver = requireMember(record.driver);
		if (!Std.isOfType(driver, machinekit.motion.MotorDriver))
			throw 'Motor "${record.actuator}": "${record.driver}" is not a driver part';
		return cast driver;
	}

	/** A modelled supply wins over the explicit fallback; an exposed boundary has no voltage of its own. */
	function driverVoltage(id:String, driver:machinekit.motion.MotorDriver, complete:Bool):Null<Float> {
		var trace = traceUpstream(portRef(id, "power"));
		var voltage = driver.statedVoltage;
		if (trace.supplied && !trace.external) {
			var source = complete ? upstream(id, "power").port : trace.port;
			var part = requireMember(source.instanceId);
			if (!Std.isOfType(part, machinekit.motion.ElectricalSource))
				throw 'Power source "${source.instanceId}/${source.portName}" does not state an output voltage';
			var supply:machinekit.motion.ElectricalSource = cast part;
			voltage = supply.outputVoltage(source.portName);
		} else if (!trace.supplied && trace.chain.length > 1) throw unsuppliedMessage(trace.chain);
		if (voltage != null) machinekit.motion.MotorDriver.validateVoltage(driver.rating, voltage);
		return voltage;
	}

	function resolveMotor(record:machinekit.assembly.MachineAssemblyDescription.MotorRecord,
			complete:Bool):materia.assembly.AssemblyDefinition.AssemblyActuator {
		var driver = motorDriver(record);
		var voltage = driverVoltage(record.driver, driver, complete);
		if (voltage == null) throw 'Motor "${record.actuator}" needs a wired supply or a stated driver voltage';
		var motor:machinekit.motion.MotorDrive = cast requireMember(record.motor);
		var added = motor.actuator(record.actuator, record.joint, voltage, record.margin, driver.current);
		var stepper = driver.rating.family == machinekit.motion.MotorDriver.MotorDriverFamily.Stepper;
		if ((stepper && added.drive != "stepper") || (!stepper && added.drive != "servo"))
			throw 'Motor "${record.actuator}" and driver "${record.driver}" have different drive families';
		if (stepper) {
			added.microsteps = driver.microsteps;
			added.maxStepRate = driver.rating.maximumStepRate;
		}
		var assumed = added.assumed == null ? [] : [for (label in added.assumed) label];
		assumed.push("driver ratings");
		added.assumed = assumed;
		var gearbox = record.gearbox;
		if (gearbox != null) {
			var part = requireMember(gearbox);
			if (!Std.isOfType(part, machinekit.motion.Gearbox)) throw 'Motor "${record.actuator}": "$gearbox" is not a gearbox part';
			var gear:machinekit.motion.Gearbox = cast part;
			var housing = requireMember(record.motor);
			if (Std.isOfType(housing, machinekit.robotics.GearedArmJoint)) {
				var pocket:machinekit.robotics.GearedArmJoint = cast housing;
				if (gear.diameter >= pocket.pocketDiameter || gear.length + 0.5 >= pocket.pocketLength)
					throw 'Gearbox "$gearbox" does not fit motor housing "${record.motor}"';
			}
			added.gearRatio = gear.ratio;
			added.gearEfficiency = gear.efficiency;
			if (gear.assumed) {
				assumed.push("gearbox ratio");
				assumed.push("gearbox efficiency");
			}
		}
		return added;
	}

	function addMotorRecord(record:machinekit.assembly.MachineAssemblyDescription.MotorRecord):Void {
		for (motor in motors) if (motor.actuator == record.actuator)
			throw 'Duplicate assembly actuator "${record.actuator}"';
		var driver = motorDriver(record);
		// An incomplete module may be wired by its containing assembly. Validate known ratings now,
		// and resolve the complete power graph when exporting the assembly's model.
		if (driverVoltage(record.driver, driver, false) != null) {
			var checked = resolveMotor(record, false);
		}
		motors.push(copyMotor(record, ""));
	}

	/** Compile every binding anew; connecting a supply after binding must not leave an old curve. */
	function compileMotors():MotorCompilation {
		var actuators = [for (record in motors) resolveMotor(record, true)];
		var sensors:Array<materia.assembly.AssemblyDefinition.AssemblyEncoder> = [];
		var sensorIds:Map<String, Bool> = [];
		for (record in encoders) {
			var part:machinekit.motion.EncoderPart = cast requireMember(record.part);
			var sensor = part.encoder(record.encoder, record.joint);
			sensors.push(sensor);
			sensorIds.set(sensor.id, true);
			if (record.actuator != null) for (actuator in actuators) if (actuator.id == record.actuator) {
				actuator.encoder = record.encoder;
				actuator.encoderCounts = null;
			}
		}
		for (actuator in actuators) {
			var counts = actuator.encoderCounts;
			if (actuator.drive == "servo" && actuator.encoder == null && counts != null && counts > 0) {
				var id = actuator.id + ".encoder";
				if (sensorIds.exists(id)) throw 'Duplicate assembly encoder "$id"';
				var gear = actuator.gearRatio;
				sensors.push({id: id, joint: actuator.joint, kind: "incremental", counts: counts * (gear == null ? 1 : gear)});
				sensorIds.set(id, true);
				actuator.encoder = id;
				actuator.encoderCounts = null;
			}
		}
		return {actuators: actuators, encoders: sensors};
	}

	/**
	 * Encoder member `part` (a shaft encoder, a linear scale, or any part that is an `EncoderPart`) reads
	 * joint `joint`, as encoder `id`. Which joint it is on says what it sees: a motor's own joint, motor-side
	 * (lost steps), or a joint the load moves through a drive, load-side (where the load is). `actuator` names
	 * the motor it is the feedback of, when it is: that actuator then points at this encoder and holds no
	 * count of its own. Rebuilding the assembly asks the part again.
	 */
	public function addEncoder(id:String, joint:String, part:String, ?actuator:String):Void
		addEncoderRecord({encoder: id, joint: joint, part: part, actuator: actuator});

	function addEncoderRecord(record:machinekit.assembly.MachineAssemblyDescription.EncoderRecord):Void {
		var member = requireMember(record.part);
		if (!Std.isOfType(member, machinekit.motion.EncoderPart)) throw 'Encoder "${record.encoder}": "${record.part}" is not an encoder part';
		for (existing in encoders) if (existing.encoder == record.encoder)
			throw 'Duplicate assembly encoder "${record.encoder}"';
		if (record.actuator != null) {
			var found = false;
			for (motor in motors) if (motor.actuator == record.actuator) found = true;
			if (!found) throw 'Encoder "${record.encoder}" reads unknown motor "${record.actuator}"';
		}
		var part:machinekit.motion.EncoderPart = cast member;
		var checked = part.encoder(record.encoder, record.joint);
		var saved = copyEncoder(record, "");
		if (record.actuator != null) for (previous in encoders)
			if (previous.actuator == record.actuator) previous.actuator = null;
		encoders.push(saved);
	}

	static function copyEncoder(encoder:machinekit.assembly.MachineAssemblyDescription.EncoderRecord,
			prefix:String):machinekit.assembly.MachineAssemblyDescription.EncoderRecord
		return {encoder: join(prefix, encoder.encoder), joint: join(prefix, encoder.joint), part: join(prefix, encoder.part),
			actuator: encoder.actuator == null ? null : join(prefix, encoder.actuator)};

	static function copyMotor(motor:machinekit.assembly.MachineAssemblyDescription.MotorRecord,
			prefix:String):machinekit.assembly.MachineAssemblyDescription.MotorRecord
		return {actuator: join(prefix, motor.actuator), joint: join(prefix, motor.joint),
			motor: join(prefix, motor.motor), driver: join(prefix, motor.driver), margin: motor.margin,
			gearbox: motor.gearbox == null ? null : join(prefix, motor.gearbox)};

	static function copyActuator(actuator:materia.assembly.AssemblyDefinition.AssemblyActuator,
			prefix:String):materia.assembly.AssemblyDefinition.AssemblyActuator {
		var copy = materia.assembly.AssemblyDefinitionFlattener.copyActuator(actuator, join(prefix, actuator.id),
			join(prefix, actuator.joint));
		if (actuator.encoder != null) copy.encoder = join(prefix, actuator.encoder);
		return copy;
	}

	static function copyTransmission(transmission:machinekit.assembly.MachineAssemblyDescription.TransmissionRecord,
			prefix:String):machinekit.assembly.MachineAssemblyDescription.TransmissionRecord
	{
		var copy:machinekit.assembly.MachineAssemblyDescription.TransmissionRecord = {coupling: join(prefix, transmission.coupling),
			source: machinekit.transmission.TransmissionResolver.mapSource(transmission.source, member -> join(prefix, member)), sense: transmission.sense,
			leaderZero: transmission.leaderZero};
		copy.stiffness = transmission.stiffness;
		copy.backlash = transmission.backlash;
		copy.drag = transmission.drag;
		copy.near = transmission.near;
		copy.far = transmission.far;
		copy.unsupported = transmission.unsupported;
		return copy;
	}

	public function addCoupling(id:String, source:String, target:String, ratio:Float, offset:Float = 0,
			?efficiency:Float, ?stiffness:Float, ?backlash:Float, ?drag:Float, ?assumed:ReadOnlyArray<String>):Void {
		requireOperationId(id);
		if (source == null || source.length == 0 || target == null || target.length == 0)
			throw 'Assembly coupling "$id" needs source and target';
		var coupling:materia.assembly.AssemblyDefinition.AssemblyJointCoupling = {id: id, source: source,
			target: target, ratio: ratio, offset: offset};
		if (efficiency != null) coupling.efficiency = efficiency;
		if (stiffness != null) coupling.stiffness = stiffness;
		if (backlash != null) coupling.backlash = backlash;
		if (drag != null) coupling.drag = drag;
		if (assumed != null && assumed.length > 0) coupling.assumed = [for (label in assumed) label];
		mechanical.couplings.push(coupling);
	}

	public function connectPorts(id:String, fromInstance:String, fromPort:String,
			toInstance:String, toPort:String, ?line:BomItem,
			lineMass:AssemblyBomMass = Unknown):Void {
		var from = portRef(fromInstance, fromPort), to = portRef(toInstance, toPort);
		var first = requirePort(from), second = requirePort(to);
		if ((first.role == Consumer && second.role != Consumer) ||
			(first.role == Passive && second.role == Supply)) {
			var old = from; from = to; to = old;
		}
		if (line == null) switch lineMass {
			case Point(_, _) | Attached(_, _, _): throw "Connection line mass needs a BOM item";
			case Unknown:
		}
		requireOperationId(id);
		portConnections.push({id: id, fromInstance: from.instanceId, fromPort: from.portName,
			toInstance: to.instanceId, toPort: to.portName});
		if (line != null) addBomItem(line, 1, lineMass);
	}

	/** Add a calculated connector on one member, such as a screw seat above a housing face. */
	public function addMemberConnector(instanceId:String, name:String, frame:AssemblyFrame):Void {
		if (name == null || name.length == 0) throw "Assembly connector needs a name";
		requireMember(instanceId);
		var definition = mechanicalDefinition(instanceId);
		for (connector in definition.connectors) if (connector.name == name)
			throw 'Duplicate assembly connector "$instanceId/$name"';
		memberConnectorFrames.push({instanceId: instanceId, name: name, frame: copyFrame(frame)});
		definition.connectors.push({name: name, frame: copyFrame(frame)});
	}

	/** Publish a stable assembly-level name that resolves to one member connector. */
	public function exposeConnector(name:String, instanceId:String, connectorName:String):Void {
		if (name == null || name.length == 0) throw "External assembly connector needs a name";
		requireConnector(ref(instanceId, connectorName));
		for (existing in externalConnectors) if (existing.name == name)
			throw 'Duplicate external assembly connector "$name"';
		externalConnectors.push({name: name, instanceId: instanceId, connectorName: connectorName});
	}

	public function exposePort(name:String, instanceId:String, portName:String):Void {
		if (name == null || name.length == 0) throw "External assembly port needs a name";
		requirePort(portRef(instanceId, portName));
		for (existing in externalPorts) if (existing.name == name) {
			throw 'Duplicate external assembly port "$name"';
		}
		externalPorts.push({name: name, instanceId: instanceId, portName: portName});
	}

	public function addBomItem(item:BomItem, quantity:Int = 1,
			mass:AssemblyBomMass = Unknown):Void {
		if (item == null || quantity <= 0) throw "Assembly BOM entry needs an item and positive quantity";
		switch mass {
			case Point(kg, centre) | Attached(kg, _, centre):
				if (!Math.isFinite(kg) || kg <= 0 || centre == null ||
					!Math.isFinite(centre.x) || !Math.isFinite(centre.y) || !Math.isFinite(centre.z))
					throw "Assembly BOM mass and centre must be finite and positive";
			case Unknown:
		}
		switch mass {
			case Attached(_, instanceId, _): requireMember(instanceId);
			case _:
		}
		bomItems.push({item: item, quantity: quantity, mass: mass});
	}

	/** Collect structural and service faults without stopping at the first one. */
	public function check():Diagnostics {
		var result = new Diagnostics();
		checkStructure(result);
		checkServices(result);
		return result;
	}

	public function validateStructure():Void {
		var result = new Diagnostics();
		checkStructure(result);
		result.throwIfErrors();
	}

	public function validate():Array<String> {
		var result = check();
		result.throwIfErrors();
		return result.warnings();
	}

	function checkStructure(result:Diagnostics):Void {
		var parents:Map<String, String> = [];
		var joints:Map<String, Bool> = [];
		for (joint in mechanical.joints) {
			if (joint.role == AssemblyJointRole.Tree) {
				if (parents.exists(joint.child))
					result.error("assembly.multiple-parents", joint.child,
						'Assembly member "${joint.child}" has two parent joints');
				else parents.set(joint.child, joint.parent);
			}
			joints.set(joint.id, true);
		}
		var cycles:Map<String, Bool> = [];
		for (member in members) {
			var seen:Map<String, Bool> = [];
			var current = member.id;
			while (parents.exists(current)) {
				if (seen.exists(current)) {
					if (!cycles.exists(current)) result.error("assembly.mate-cycle", current,
						'Assembly mate cycle at "$current"');
					cycles.set(current, true);
					break;
				}
				seen.set(current, true);
				current = parents.get(current);
			}
		}
		if (mechanical.couplings != null) for (coupling in mechanical.couplings)
			if (!joints.exists(coupling.source) || !joints.exists(coupling.target))
				result.error("assembly.missing-coupling-joint", coupling.id,
					'Assembly coupling "${coupling.id}" refers to a missing joint');
	}

	function checkServices(result:Diagnostics, required:Bool = true):Void {
		var connected:Map<String, Bool> = [];
		for (connection in portConnections) {
			var id = connection.id;
			var from = portRef(connection.fromInstance, connection.fromPort);
			var to = portRef(connection.toInstance, connection.toPort);
			var first = requirePort(from), second = requirePort(to);
			var fromKey = portKey(from), toKey = portKey(to);
			if (fromKey == toKey) result.error("port.self-connection", id,
				'Port connection "$id" joins a port to itself');
			if (connected.exists(fromKey) || connected.exists(toKey))
				result.error("port.reused", id, 'Port connection "$id" uses a port more than once');
			connected.set(fromKey, true);
			connected.set(toKey, true);
			if (first.kind != second.kind) result.error("port.kind-mismatch", id,
				'Port connection "$id" has mismatched kinds');
			if ((first.role == Supply && second.role == Supply) ||
				(first.role == Consumer && second.role == Consumer))
				result.error("port.role-mismatch", id, 'Port connection "$id" has incompatible roles');
			if (!PortInterfaces.compatible(first.iface, second.iface))
				result.error("port.interface-mismatch", id,
					'Port connection "$id" has mismatched interfaces: ${Std.string(first.iface)} and ${Std.string(second.iface)}');
		}
		if (required) for (port in portRecords)
			if (port.required && port.role == Consumer) {
				var reference = portRef(port.occurrence, port.name);
				if (connected.exists(portKey(reference))) {
					try {
						var trace = traceUpstream(reference);
						if (!trace.supplied) result.error("port.unsupplied", '${port.occurrence}/${port.name}',
							unsuppliedMessage(trace.chain));
					} catch (error:String) result.error("port.service-cycle", '${port.occurrence}/${port.name}', error);
				} else if (!isExposed(port.occurrence, port.name))
					result.error("port.required-unconnected", '${port.occurrence}/${port.name}',
						'Required consumer port "${port.occurrence}/${port.name}" is unconnected');
			}
	}

	function checkConnections():{connected:Map<String, Bool>, warnings:Array<String>} {
		var result = new Diagnostics();
		checkServices(result, false);
		result.throwIfErrors();
		var connected:Map<String, Bool> = [];
		for (connection in portConnections) {
			connected.set(portKey(portRef(connection.fromInstance, connection.fromPort)), true);
			connected.set(portKey(portRef(connection.toInstance, connection.toPort)), true);
		}
		return {connected: connected, warnings: result.warnings()};
	}

	/** Populate an existing model. All member and joint ids receive the supplied prefix. */
	public function addTo(model:AssemblyModel, prefix:String, ?pose:AssemblyFrame):Void {
		validateStructure();
		for (occurrence in mechanical.occurrences) {
			var localPose = pose == null ? occurrence.initialPose :
				AssemblyFrames.compose(pose, occurrence.initialPose);
			var id = join(prefix, occurrence.id);
			model.add(id, localPose);
			for (connector in mechanicalDefinition(occurrence.definition).connectors)
				model.connector(id, connector.name, connector.frame);
		}
		for (joint in mechanical.joints) {
			if (joint.role == AssemblyJointRole.Tree)
				model.mateOnAxis(join(prefix, joint.id), joint.type, join(prefix, joint.parent),
					joint.parentConnector, join(prefix, joint.child), joint.childConnector,
					joint.axis, joint.defaultValue, joint.limits);
			else model.constrainOnAxis(join(prefix, joint.id), joint.type, join(prefix, joint.parent),
					joint.parentConnector, join(prefix, joint.child), joint.childConnector,
					joint.axis, joint.closureTolerance, joint.limits);
		}
		if (mechanical.couplings != null) for (coupling in mechanical.couplings)
			model.couple(join(prefix, coupling.id), join(prefix, coupling.source),
				join(prefix, coupling.target), coupling.ratio, coupling.offset, coupling.efficiency,
				coupling.stiffness, coupling.backlash, coupling.drag, coupling.assumed);
		var compiled = compileMotors();
		for (actuator in compiled.actuators) model.actuateDrive(copyActuator(actuator, prefix));
		for (encoder in compiled.encoders)
			model.addEncoder(materia.assembly.AssemblyDefinitionFlattener.copyEncoder(encoder, join(prefix, encoder.id),
				join(prefix, encoder.joint)));
	}

	public function components():Array<MachineAssemblyComponent>
		return [for (member in members) {id: member.id, component: member.component}];

	/** Parent links in the mate tree; constraints do not attach members. */
	public function mateParents():Map<String, String> {
		var result:Map<String, String> = [];
		for (joint in mechanical.joints) if (joint.role == AssemblyJointRole.Tree)
			result.set(joint.child, joint.parent);
		return result;
	}

	/** A member connector in that member's local frame. */
	public function memberConnectorFrame(instanceId:String, connectorName:String):AssemblyFrame {
		requireConnector(ref(instanceId, connectorName));
		for (connector in mechanicalDefinition(instanceId).connectors)
			if (connector.name == connectorName) return copyFrame(connector.frame);
		throw 'Unknown connector "$instanceId/$connectorName"';
	}

	public function subassemblies():Array<MachineSubassembly> return included.copy();

	public function billOfMaterials():Bom {
		var result = new Bom();
		for (member in members) result.addComponent(member.component);
		for (entry in bomItems) result.add(entry.item, entry.quantity);
		return result;
	}

	/** Sum posed component masses; separately report BOM extras with no mass model. */
	public function massProperties(?state:AssemblyState):MachineAssemblyMassProperties {
		return massPropertiesFromPoses(solvedPoses(state));
	}

	/** Solve every member pose from one AssemblyState, or read them from a supplied state. */
	public function solvedPoses(?state:AssemblyState):Map<String, AssemblyFrame> {
		if (state == null) {
			state = new AssemblyState(cloneDefinition(mechanical));
			state.checkClosures();
		}
		var result:Map<String, AssemblyFrame> = [];
		for (member in members)
			result.set(member.id, state.worldPose(member.id));
		return result;
	}

	/** Mass using poses already solved for this assembly. */
	public function massPropertiesFromPoses(poses:Map<String, AssemblyFrame>):MachineAssemblyMassProperties {
		var mass = 0.0, weightedX = 0.0, weightedY = 0.0, weightedZ = 0.0;
		var posed:Array<{id:String, properties:machinekit.component.MassProperties, pose:AssemblyFrame,
			centre:Vector}> = [];
		for (member in members) {
			var key = definitionKey(member.id, member.component);
			var properties = massByDefinition.get(key);
			if (properties == null) {
				properties = member.component.massProperties();
				massByDefinition.set(key, properties);
			}
			var pose = poses.get(member.id);
			if (pose == null) throw 'Missing solved pose for "${member.id}"';
			var centre = properties.centreOfMass;
			var world = AssemblyFrames.transformPoint(pose, centre.x, centre.y, centre.z);
			posed.push({id: member.id, properties: properties, pose: pose,
				centre: new Vector(world.x, world.y, world.z)});
			mass += properties.mass;
			weightedX += properties.mass * world.x;
			weightedY += properties.mass * world.y;
			weightedZ += properties.mass * world.z;
		}
		var unaccounted:Array<String> = [];
		var bomPoints:Array<{mass:Float, centre:Vector}> = [];
		for (entry in bomItems) {
			switch entry.mass {
				case Unknown:
					if (unaccounted.indexOf(entry.item.partNumber) < 0)
						unaccounted.push(entry.item.partNumber);
				case Point(kg, centre) | Attached(kg, _, centre):
					var worldCentre = switch entry.mass {
						case Attached(_, instanceId, _):
							var pose = poses.get(instanceId);
							if (pose == null) throw 'Missing solved pose for "$instanceId"';
							var point = AssemblyFrames.transformPoint(pose, centre.x, centre.y, centre.z);
							new Vector(point.x, point.y, point.z);
						case _: centre;
					};
					var itemMass = kg * entry.item.quantity * entry.quantity;
					bomPoints.push({mass: itemMass, centre: worldCentre});
					mass += itemMass;
					weightedX += itemMass * worldCentre.x;
					weightedY += itemMass * worldCentre.y;
					weightedZ += itemMass * worldCentre.z;
			}
		}
		var combinedCentre = mass == 0 ? new Vector() :
			new Vector(weightedX / mass, weightedY / mass, weightedZ / mass);
		var inertia = InertiaTensor.zero();
		var unaccountedInertia:Array<String> = [];
		for (entry in posed) {
			var tensor = entry.properties.inertia;
			if (tensor == null) {
				unaccountedInertia.push(entry.id);
				continue;
			}
			var pose = entry.pose, centre = entry.centre;
			inertia = inertia.add(tensor.rotated(pose.qx, pose.qy, pose.qz, pose.qw)
				.shifted(entry.properties.mass, centre.x - combinedCentre.x,
					centre.y - combinedCentre.y, centre.z - combinedCentre.z));
		}
		// BOM-only masses are represented as point masses at their declared centres.
		for (point in bomPoints) inertia = inertia.shifted(point.mass,
			point.centre.x - combinedCentre.x, point.centre.y - combinedCentre.y,
			point.centre.z - combinedCentre.z);
		return {mass: mass, centreOfMass: combinedCentre,
			inertia: unaccountedInertia.length == 0 ? inertia : null,
			unaccounted: unaccounted, unaccountedInertia: unaccountedInertia};
	}

	static function definitionKey(id:String, component:MachineComponent):String {
		var recipe = component.componentType();
		return recipe == null ? "code:" + id + "|" + component.materialSpec() :
			"typed:" + recipe.id + "|" + recipe.key(component.values()) + "|" + component.materialSpec();
	}

	public function connectorNames():Array<String>
		return [for (connector in externalConnectors) connector.name];

	public function portNames():Array<String> return [for (port in externalPorts) port.name];

	public function port(name:String, prefix:String = ""):PortRef {
		for (entry in externalPorts) if (entry.name == name)
			return portRef(join(prefix, entry.instanceId), entry.portName);
		throw 'Missing assembly port "$name"';
	}

	/** Trace a service through connections, bridges, and a single-input converter. */
	public function upstream(instanceId:String, portName:String):UpstreamResult {
		checkConnections();
		var trace = traceUpstream(portRef(instanceId, portName));
		if (!trace.supplied) throw unsuppliedMessage(trace.chain);
		return {port: trace.port, external: trace.external};
	}

	/** Ordered member/port path from a consumer to its supplied boundary. */
	public function upstreamChain(instanceId:String, portName:String):Array<String> {
		checkConnections();
		var trace = traceUpstream(portRef(instanceId, portName));
		if (!trace.supplied) throw unsuppliedMessage(trace.chain);
		return trace.chain.copy();
	}

	static function unsuppliedMessage(chain:Array<String>):String
		return 'Service chain ${chain.join(" ← ")} is not supplied';

	function traceUpstream(start:PortRef):ServiceTrace {
		var current = start;
		var seen:Map<String, Bool> = [];
		var chain:Array<String> = [];
		while (true) {
			var key = portKey(current);
			if (seen.exists(key)) throw 'Port service cycle at "${start.instanceId}/${start.portName}"';
			seen.set(key, true);
			chain.push('${current.instanceId}/${current.portName}');
			var currentPort = requirePort(current);
			var previous:Array<PortRef> = [];
			for (connection in portConnections)
				if (portKey(portRef(connection.toInstance, connection.toPort)) == key)
					previous.push(portRef(connection.fromInstance, connection.fromPort));
			for (bridge in portBridges) if (bridge.occurrence == current.instanceId &&
				bridge.toPort == current.portName)
				previous.push(portRef(current.instanceId, bridge.fromPort));
			for (conversion in portConversions) if (conversion.occurrence == current.instanceId &&
				conversion.toPort == current.portName)
				previous.push(portRef(current.instanceId, conversion.fromPort));
			if (previous.length == 0) {
				if (currentPort.role == Supply)
					return {port: current, external: false, supplied: true, chain: chain};
				if (isExposed(current.instanceId, current.portName))
					return {port: current, external: true, supplied: true, chain: chain};
				return {port: current, external: false, supplied: false, chain: chain};
			}
			if (previous.length != 1) throw 'Port "${start.instanceId}/${start.portName}" has ambiguous upstream supply';
			current = previous[0];
		}
	}

	public function connector(name:String, prefix:String = ""):MachineAssemblyConnector {
		for (connector in externalConnectors) if (connector.name == name)
			return {instanceId: join(prefix, connector.instanceId), connectorName: connector.connectorName};
		throw 'Missing assembly connector "$name"';
	}

	public static function join(prefix:String, id:String):String {
		var child = InstancePath.of(id);
		return prefix == null || prefix.length == 0 ? child : InstancePath.of(prefix + "/" + id);
	}

	function addJoint(joint:KinematicJoint, ?tolerance:Float):Void {
		requireOperationId(joint.id);
		if (joint.type != AssemblyJointType.Fixed && joint.type != AssemblyJointType.Revolute &&
			joint.type != AssemblyJointType.Continuous && joint.type != AssemblyJointType.Prismatic)
			throw 'Unsupported assembly joint "${joint.type}"';
		requireConnector(ref(joint.parent, joint.parentConnector));
		requireConnector(ref(joint.child, joint.childConnector));
		var saved:KinematicJoint = {id: joint.id, type: joint.type, role: joint.role,
			parent: joint.parent, parentConnector: joint.parentConnector,
			child: joint.child, childConnector: joint.childConnector,
			axis: {x: joint.axis.x, y: joint.axis.y, z: joint.axis.z},
			limits: {lower: joint.limits.lower, upper: joint.limits.upper,
				velocity: joint.limits.velocity, effort: joint.limits.effort, overtravel: joint.limits.overtravel,
				acceleration: joint.limits.acceleration},
			defaultValue: joint.defaultValue};
		if (joint.role == AssemblyJointRole.Closure && tolerance != null)
			saved.closureTolerance = tolerance;
		mechanical.joints.push(saved);
	}

	function requireOperationId(id:String):Void {
		if (id == null || id.length == 0) throw "Assembly operation needs an id";
		for (joint in mechanical.joints) if (joint.id == id)
			throw 'Duplicate assembly operation "$id"';
		if (mechanical.couplings != null) for (coupling in mechanical.couplings) if (coupling.id == id)
			throw 'Duplicate assembly operation "$id"';
		for (existing in portConnections) if (existing.id == id)
			throw 'Duplicate assembly operation "$id"';
	}

	function occurrencePose(id:String):AssemblyFrame {
		for (occurrence in mechanical.occurrences) if (occurrence.id == id) return occurrence.initialPose;
		throw 'Missing mechanical occurrence "$id"';
	}

	function mechanicalDefinition(id:String):materia.assembly.AssemblyDefinition.AssemblyComponentDefinition {
		for (definition in mechanical.definitions) if (definition.id == id) return definition;
		throw 'Missing mechanical definition "$id"';
	}

	static function resolvedLimits(limits:Null<AssemblyJointLimits>):AssemblyJointLimits
		return limits == null ? {lower: null, upper: null, velocity: null, effort: null} : limits;

	function requireMember(id:String):MachineComponent {
		for (member in members) if (member.id == id) return member.component;
		throw 'Unknown assembly member "$id"';
	}

	function requireConnector(reference:MachineAssemblyConnector):Void {
		requireMember(reference.instanceId);
		if (reference.connectorName == null || reference.connectorName.length == 0)
			throw 'Unknown connector "${reference.instanceId}/${reference.connectorName}"';
		for (connector in mechanicalDefinition(reference.instanceId).connectors)
			if (connector.name == reference.connectorName) return;
		throw 'Unknown connector "${reference.instanceId}/${reference.connectorName}"';
	}

	public function hasPort(name:String):Bool {
		for (entry in externalPorts) if (entry.name == name) return true;
		return false;
	}

	public function hasMemberConnector(instanceId:String, connectorName:String):Bool {
		for (occurrence in mechanical.occurrences) if (occurrence.id == instanceId)
			for (connector in mechanicalDefinition(occurrence.definition).connectors)
				if (connector.name == connectorName) return true;
		return false;
	}

	function requirePort(reference:PortRef):ComponentPort {
		requireMember(reference.instanceId);
		if (reference.portName == null || reference.portName.length == 0)
			throw 'Unknown port "${reference.instanceId}/${reference.portName}"';
		for (port in portRecords) if (port.occurrence == reference.instanceId &&
			port.name == reference.portName) return {name: port.name, kind: port.kind,
				role: port.role, iface: port.iface, required: port.required, connector: port.connector};
		throw 'Unknown port "${reference.instanceId}/${reference.portName}"';
	}

	function isConnected(reference:PortRef):Bool {
		var key = portKey(reference);
		for (connection in portConnections)
			if (portKey(portRef(connection.fromInstance, connection.fromPort)) == key ||
				portKey(portRef(connection.toInstance, connection.toPort)) == key) return true;
		return false;
	}

	function isExposed(instanceId:String, portName:String):Bool {
		for (entry in externalPorts) if (entry.instanceId == instanceId &&
			entry.portName == portName) return true;
		return false;
	}

	static function portKey(reference:PortRef):String return reference.instanceId + "\x1f" + reference.portName;

	static function portRef(instanceId:String, portName:String):PortRef
		return {instanceId: instanceId, portName: portName};

	static function ref(instanceId:String, connectorName:String):MachineAssemblyConnector
		return {instanceId: instanceId, connectorName: connectorName};

	static function copyFrame(frame:AssemblyFrame):AssemblyFrame
		return {x: frame.x, y: frame.y, z: frame.z, qx: frame.qx, qy: frame.qy, qz: frame.qz, qw: frame.qw};
}
