package machinekit.assembly;

import cadkit.modeling.Vector;
import haxeon.Equality;
import haxeon.wire.JsonWire;
import machinekit.component.ComponentRegistry;
import machinekit.component.ComponentValue;
import machinekit.component.ComponentValues;
import machinekit.component.MachineComponent;
import machinekit.component.MachineKitComponents;
import machinekit.assembly.MachineAssembly.AssemblyBomMass;
import machinekit.assembly.MachineAssemblyDescription;
import materia.assembly.AssemblyDefinition;
import materia.assembly.AssemblyDefinition.AssemblyComponentDefinition;
import materia.assembly.AssemblyDefinition.AssemblyComponentOccurrence;
import materia.assembly.AssemblyDefinition.AssemblyExposedConnector;
import materia.assembly.AssemblyDefinition.AssemblyJointCoupling;
import materia.assembly.AssemblyDefinition.AssemblyJointRole;
import materia.assembly.AssemblyDefinition.AssemblySubdefinition;
import materia.assembly.AssemblyDefinition.KinematicJoint;
import materia.assembly.AssemblyDefinitionCodec;
import materia.assembly.AssemblyFrames;
import materia.assembly.AssemblyRecord.AssemblyFrame;

/** One level's share of the nested mechanical definition. */
private typedef LevelMechanical = {
	var definitions:Array<AssemblyComponentDefinition>;
	var occurrences:Array<AssemblyComponentOccurrence>;
	var joints:Array<KinematicJoint>;
	var couplings:Array<AssemblyJointCoupling>;
	var exposedConnectors:Array<AssemblyExposedConnector>;
}

/**
 * Writes and reads `MachineAssemblyDescription`. Each subassembly becomes an entry of the nested
 * ProjectKit definition's table, with the facts of its own level beside it. A joint or exposure that
 * reaches into a subassembly goes through a connector that subassembly exposes, named `~path/connector`.
 */
class MachineAssemblyCodec {
	public static function describe(assembly:MachineAssembly):MachineAssemblyDescription {
		assembly.validateStructure();
		var table:Array<AssemblySubdefinition> = [];
		var records:Array<SubassemblyRecord> = [];
		var level = describeLevel(assembly, "", [], table, records);
		var mechanical:AssemblyDefinition = {schemaVersion: AssemblyDefinitionCodec.VERSION, id: "assembly",
			lengthUnit: "mm", definitions: level.mechanical.definitions, occurrences: level.mechanical.occurrences,
			joints: level.mechanical.joints, couplings: level.mechanical.couplings,
			exposedConnectors: level.mechanical.exposedConnectors};
		if (table.length > 0) mechanical.assemblies = table;
		canonicalize(mechanical);
		AssemblyDefinitionCodec.validate(mechanical);
		records.sort((a, b) -> Reflect.compare(a.path, b.path));
		return {schemaVersion: MachineAssembly.SCHEMA_VERSION, mechanical: mechanical, machine: level.machine,
			subassemblies: records};
	}

	/** The generated codec, after rejecting every member that has no recipe to rebuild it. */
	public static function encode(description:MachineAssemblyDescription):String {
		var problems = new Diagnostics();
		checkSavable(description.machine, "", problems);
		for (record in description.subassemblies) checkSavable(record.machine, record.path + "/", problems);
		problems.throwIfErrors();
		return JsonWire.encode(description);
	}

	public static function checkSavable(machine:MachineLevelRecord, prefix:String, problems:Diagnostics):Void
		for (member in machine.members) switch member.source {
			case Code(designation): problems.error("assembly.code-member", prefix + member.occurrence,
				'Code-only member "$designation" has no rebuild recipe');
			case _:
		}

	public static function decode(text:String):MachineAssemblyDescription {
		var version:DescriptionVersion = JsonWire.decode(text);
		requireVersion(version.schemaVersion);
		return JsonWire.decode(text);
	}

	public static function requireVersion(version:Null<Int>):Void
		if (version != MachineAssembly.SCHEMA_VERSION)
			throw 'schema v$version is unsupported; expected v${MachineAssembly.SCHEMA_VERSION}';

	static function describeLevel(assembly:MachineAssembly, path:String, extraConnectors:Map<String, Array<MemberConnectorRecord>>,
			table:Array<AssemblySubdefinition>, records:Array<SubassemblyRecord>):{mechanical:LevelMechanical, machine:MachineLevelRecord} {
		var mechanical = assembly.mechanical;
		var flat = assembly.derived();
		var level:LevelMechanical = {definitions: [], occurrences: [], joints: [], couplings: [], exposedConnectors: []};
		var entries = new Map<String, AssemblySubdefinition>();
		// Connectors added on a subassembly's members, by subassembly and the path below it.
		var nestedConnectors = new Map<String, Map<String, Array<MemberConnectorRecord>>>();
		var below = (instanceId:String) -> {
			var slash = instanceId.indexOf("/");
			return slash > 0 && mechanical.subassembly(instanceId.substr(0, slash)) != null ?
				{head: instanceId.substr(0, slash), tail: instanceId.substr(slash + 1)} : null;
		};
		var addNested = (record:MemberConnectorRecord) -> {
			var split = below(record.instanceId);
			if (split == null) return false;
			var byPath = nestedConnectors.get(split.head);
			if (byPath == null) {
				byPath = new Map();
				nestedConnectors.set(split.head, byPath);
			}
			var list = byPath.get(split.tail);
			if (list == null) {
				list = [];
				byPath.set(split.tail, list);
			}
			list.push({instanceId: split.tail, name: record.name, frame: record.frame});
			return true;
		};
		var ownExtra = new Map<String, Array<MemberConnectorRecord>>();
		var added:Array<MemberConnectorRecord> = [];
		for (id in extraConnectors.keys()) for (record in extraConnectors.get(id)) added.push(record);
		for (record in mechanical.memberConnectors) added.push(record);
		for (record in added) if (!addNested(record)) {
			var own = ownExtra.get(record.instanceId);
			if (own == null) {
				own = [];
				ownExtra.set(record.instanceId, own);
			}
			own.push(record);
		}
		// Members share a definition when their recipe identity and connector frames agree.
		var shared:Map<String, Array<AssemblyComponentDefinition>> = [];
		// Sorted, so the member that names a shared definition does not depend on the order of edits.
		var order = mechanical.order.copy();
		order.sort(Reflect.compare);
		for (id in order) {
			var member = mechanical.member(id);
			if (member == null) {
				var sub = mechanical.requireSubassembly(id);
				var childPath = path.length == 0 ? id : path + "/" + id;
				var extra = nestedConnectors.get(id);
				var child = describeLevel(sub.assembly, childPath, extra == null ? [] : extra, table, records);
				var tableId = 'machinekit-sub-${table.length}';
				var entry:AssemblySubdefinition = {id: tableId, definitions: child.mechanical.definitions,
					occurrences: child.mechanical.occurrences, joints: child.mechanical.joints,
					couplings: child.mechanical.couplings, exposedConnectors: child.mechanical.exposedConnectors};
				table.push(entry);
				entries.set(id, entry);
				records.push({path: childPath, definition: tableId, machine: child.machine});
				level.occurrences.push({id: id, definition: tableId, assembly: tableId,
					initialPose: sub.pose == null ? AssemblyFrames.identity() : MachineAssembly.copyFrame(sub.pose)});
				continue;
			}
			var connectors:Array<materia.assembly.AssemblyRecord.AssemblyConnector> = [for (connector in member.component.connectors())
				{name: connector.name, frame: MachineAssembly.copyFrame(connector.frame)}];
			var added = ownExtra.get(id);
			if (added != null) for (record in added) connectors.push({name: record.name, frame: MachineAssembly.copyFrame(record.frame)});
			connectors.sort((a, b) -> Reflect.compare(a.name, b.name));
			var collision = machinekit.component.CollisionHullFacet.of(member.component);
			var collisionHulls = collision == null ? null : collision.hulls;
			var key = MachineAssembly.definitionKey(id, member.component);
			var candidates = shared.get(key);
			var definition:Null<AssemblyComponentDefinition> = null;
			if (candidates != null) for (candidate in candidates)
				if (Equality.equals(candidate.connectors, connectors) &&
					Equality.equals(candidate.collisionHulls, collisionHulls)) definition = candidate;
			if (definition == null) {
				definition = {id: id, connectors: connectors};
				var flange = machinekit.robotics.RobotFlangeFacet.of(member.component);
				if (flange != null) definition.robotFlangeConnector = flange.connector;
				if (collision != null) definition.collisionHulls = [for (hull in collision.hulls) hull.copy()];
				level.definitions.push(definition);
				if (candidates == null) {
					candidates = [];
					shared.set(key, candidates);
				}
				candidates.push(definition);
			}
			level.occurrences.push({id: id, definition: definition.id, initialPose: MachineAssembly.copyFrame(member.pose)});
		}
		var endpoint = (instanceId:String, connector:String) -> {
			var split = below(instanceId);
			return split == null ? {occurrence: instanceId, connector: connector} :
				{occurrence: split.head, connector: expose(entries.get(split.head), table, split.tail, connector)};
		};
		for (joint in mechanical.joints) {
			var saved = MechanicalAssembly.copyJoint(derivedJoint(flat, joint), id -> id);
			var parent = endpoint(joint.parent, joint.parentConnector);
			var child = endpoint(joint.child, joint.childConnector);
			saved.parent = parent.occurrence;
			saved.parentConnector = parent.connector;
			saved.child = child.occurrence;
			saved.childConnector = child.connector;
			level.joints.push(saved);
		}
		for (coupling in mechanical.couplings)
			level.couplings.push(MechanicalAssembly.copyCoupling(derivedCoupling(flat, coupling), id -> id));
		for (exposure in mechanical.connectorExposures) {
			var target = endpoint(exposure.instanceId, exposure.connectorName);
			level.exposedConnectors.push({name: exposure.name, occurrence: target.occurrence, connector: target.connector});
		}
		return {mechanical: level, machine: levelRecord(assembly)};
	}

	/** The name subassembly `entry` exposes member connector `path/connector` under, adding it if needed. */
	static function expose(entry:AssemblySubdefinition, table:Array<AssemblySubdefinition>, path:String, connector:String):String {
		var direct = false;
		for (occurrence in entry.occurrences) if (occurrence.id == path && occurrence.assembly == null) direct = true;
		var occurrence = path, inner = connector;
		if (!direct) {
			var slash = path.indexOf("/");
			if (slash < 0) throw 'Missing nested member "$path"';
			occurrence = path.substr(0, slash);
			var nested:Null<AssemblySubdefinition> = null;
			for (candidate in entry.occurrences) if (candidate.id == occurrence && candidate.assembly != null)
				for (item in table) if (item.id == candidate.assembly) nested = item;
			if (nested == null) throw 'Missing nested member "$path"';
			inner = expose(nested, table, path.substr(slash + 1), connector);
		}
		for (exposed in entry.exposedConnectors) if (exposed.occurrence == occurrence && exposed.connector == inner) return exposed.name;
		var name = '~$path/$connector';
		for (exposed in entry.exposedConnectors) if (exposed.name == name) throw 'Exposed connector "$name" is taken';
		entry.exposedConnectors.push({name: name, occurrence: occurrence, connector: inner});
		return name;
	}

	static function derivedJoint(flat:FlatAssembly, joint:KinematicJoint):KinematicJoint {
		for (entry in flat.definition.joints) if (entry.id == joint.id) return entry;
		return joint;
	}

	static function derivedCoupling(flat:FlatAssembly, coupling:AssemblyJointCoupling):AssemblyJointCoupling {
		if (flat.definition.couplings != null) for (entry in flat.definition.couplings) if (entry.id == coupling.id) return entry;
		return coupling;
	}

	static function levelRecord(assembly:MachineAssembly):MachineLevelRecord {
		var members:Array<MemberRecord> = [for (member in assembly.mechanical.members) {
			occurrence: member.id, source: memberSource(member.component), material: member.component.materialSpec()}];
		members.sort((a, b) -> Reflect.compare(a.occurrence, b.occurrence));
		var connections = [for (entry in assembly.services.connections) {id: entry.id, fromInstance: entry.fromInstance,
			fromPort: entry.fromPort, toInstance: entry.toInstance, toPort: entry.toPort}];
		connections.sort((a, b) -> Reflect.compare(a.id, b.id));
		var transmissions = [for (entry in assembly.drives.transmissions) DriveSystem.copyTransmission(entry, id -> id)];
		transmissions.sort((a, b) -> Reflect.compare(a.coupling, b.coupling));
		var motors = [for (entry in assembly.drives.motors) DriveSystem.copyMotor(entry, id -> id)];
		motors.sort((a, b) -> Reflect.compare(a.actuator, b.actuator));
		var encoders = [for (entry in assembly.drives.encoders) DriveSystem.copyEncoder(entry, id -> id)];
		encoders.sort((a, b) -> Reflect.compare(a.encoder, b.encoder));
		return {members: members,
			memberConnectors: [for (entry in assembly.mechanical.memberConnectors) {instanceId: entry.instanceId,
				name: entry.name, frame: MachineAssembly.copyFrame(entry.frame)}],
			connectorExposures: [for (entry in assembly.mechanical.connectorExposures) {name: entry.name,
				instanceId: entry.instanceId, connectorName: entry.connectorName}],
			portConnections: connections,
			portExposures: [for (entry in assembly.services.exposures) {name: entry.name,
				instanceId: entry.instanceId, portName: entry.portName}],
			transmissions: transmissions,
			beltPaths: [for (entry in assembly.drives.beltPaths) DriveSystem.copyBeltPath(entry, id -> id)],
			motors: motors,
			encoders: encoders,
			cylinders: {
				var saved = [for (entry in assembly.drives.cylinders) DriveSystem.copyCylinder(entry, id -> id)];
				saved.sort((a, b) -> Reflect.compare(a.actuator, b.actuator));
				saved;
			},
			sensors: {
				var saved = [for (entry in assembly.drives.sensors) DriveSystem.copySensor(entry, id -> id)];
				saved.sort((a, b) -> Reflect.compare(a.id, b.id));
				saved;
			},
			switches: [for (entry in assembly.drives.switches) DriveSystem.copySwitch(entry, id -> id)],
			bomExtras: [for (entry in assembly.inventory.items) {item: {partNumber: entry.item.partNumber,
				description: entry.item.description, quantity: entry.item.quantity, material: entry.item.material,
				typeId: entry.item.typeId, valuesKey: entry.item.valuesKey}, quantity: entry.quantity,
				mass: switch entry.mass {
					case Unknown: SavedBomMass.Unknown;
					case Point(kg, centre): SavedBomMass.Point(kg, centre.x, centre.y, centre.z);
					case Attached(kg, id, centre): SavedBomMass.Attached(kg, id, centre.x, centre.y, centre.z);
				}}]};
	}

	public static function memberSource(component:MachineComponent):MemberSource {
		var recipe = component.componentType();
		if (recipe == null) return MemberSource.Code(component.designation);
		var values = component.values();
		var names = values.names();
		names.sort(Reflect.compare);
		return MemberSource.Typed(recipe.id, [for (name in names) {name: name, value: savedValue(name, values.get(name))}]);
	}

	public static function savedValue(name:String, value:Null<ComponentValue>):SavedValue return switch value {
		case Number(value): SavedValue.Number(value);
		case Integer(value): SavedValue.Integer(value);
		case Boolean(value): SavedValue.Boolean(value);
		case Token(value): SavedValue.Token(value);
		case Unset: SavedValue.Unset;
		case null: throw 'Missing value "$name"';
	};

	public static function componentValue(value:SavedValue):ComponentValue return switch value {
		case SavedValue.Number(value): ComponentValue.Number(value);
		case SavedValue.Integer(value): ComponentValue.Integer(value);
		case SavedValue.Boolean(value): ComponentValue.Boolean(value);
		case SavedValue.Token(value): ComponentValue.Token(value);
		case SavedValue.Unset: ComponentValue.Unset;
	};

	/** Build one member through its recipe. */
	public static function rebuildMember(member:MemberRecord, registry:ComponentRegistry):MachineComponent {
		var component = switch member.source {
			case Code(designation): throw 'Code-only member "$designation" has no rebuild recipe';
			case Typed(typeId, saved):
				var values = new ComponentValues();
				for (entry in saved) values.set(entry.name, componentValue(entry.value));
				registry.byId(typeId).create(values);
		};
		component.setMaterial(member.material);
		return component;
	}

	public static function read(description:MachineAssemblyDescription, target:MachineAssembly, ?registry:ComponentRegistry):Void {
		if (description == null || description.machine == null) throw "Missing machine assembly description";
		requireVersion(description.schemaVersion);
		if (registry == null) registry = MachineKitComponents.defaultRegistry();
		var mechanical = description.mechanical;
		AssemblyDefinitionCodec.validate(mechanical);
		var table = new Map<String, AssemblySubdefinition>();
		if (mechanical.assemblies != null) for (entry in mechanical.assemblies) table.set(entry.id, entry);
		var records = new Map<String, SubassemblyRecord>();
		for (record in description.subassemblies) {
			if (records.exists(record.path)) throw 'Duplicate subassembly record "${record.path}"';
			records.set(record.path, record);
		}
		var root:LevelMechanical = {definitions: mechanical.definitions, occurrences: mechanical.occurrences,
			joints: mechanical.joints, couplings: mechanical.couplings == null ? [] : mechanical.couplings,
			exposedConnectors: mechanical.exposedConnectors == null ? [] : mechanical.exposedConnectors};
		readLevel(target, "", root, description.machine, table, records, registry);
	}

	static function readLevel(target:MachineAssembly, path:String, level:LevelMechanical, machine:MachineLevelRecord,
			table:Map<String, AssemblySubdefinition>, records:Map<String, SubassemblyRecord>, registry:ComponentRegistry):Void {
		var sources = new Map<String, MemberRecord>();
		for (member in machine.members) {
			if (sources.exists(member.occurrence)) throw 'Duplicate member source "${member.occurrence}"';
			sources.set(member.occurrence, member);
		}
		var subassemblies = new Map<String, AssemblySubdefinition>();
		for (occurrence in level.occurrences) {
			if (occurrence.assembly != null) {
				var entry = table.get(occurrence.assembly);
				var childPath = path.length == 0 ? occurrence.id : path + "/" + occurrence.id;
				var record = records.get(childPath);
				if (entry == null || record == null || record.definition != entry.id)
					throw 'Missing subassembly "$childPath"';
				var child = new MachineAssembly();
				readLevel(child, childPath, {definitions: entry.definitions, occurrences: entry.occurrences,
					joints: entry.joints, couplings: entry.couplings == null ? [] : entry.couplings,
					exposedConnectors: entry.exposedConnectors == null ? [] : entry.exposedConnectors},
					record.machine, table, records, registry);
				subassemblies.set(occurrence.id, entry);
				target.mechanical.addSubassembly({id: occurrence.id, assembly: child,
					pose: MachineAssembly.copyFrame(occurrence.initialPose)});
				target.changed();
				continue;
			}
			var member = sources.get(occurrence.id);
			if (member == null) throw 'Missing member source "${occurrence.id}"';
			var component = rebuildMember(member, registry);
			var definition = [for (entry in level.definitions) if (entry.id == occurrence.definition) entry][0];
			if (definition.collisionHulls != null) {
				var collision = machinekit.component.CollisionHullFacet.of(component);
				if (collision == null)
					@:privateAccess component.addFacet(new machinekit.component.CollisionHullFacet(definition.collisionHulls));
				else if (!Equality.equals(collision.hulls, definition.collisionHulls))
					throw 'Collision hulls disagree with recipe for "${occurrence.id}"';
			}
			target.addMember(occurrence.id, component, occurrence.initialPose);
		}
		for (connector in machine.memberConnectors)
			target.addMemberConnector(connector.instanceId, connector.name, connector.frame);
		// A joint end on a subassembly goes through the connector it exposes; follow it to the member.
		var resolve = (occurrence:String, connector:String) -> {
			var entry = subassemblies.get(occurrence);
			if (entry == null) return {instanceId: occurrence, connectorName: connector};
			var inner = exposedMember(entry, table, connector);
			return {instanceId: occurrence + "/" + inner.instanceId, connectorName: inner.connectorName};
		};
		for (joint in level.joints) {
			var parent = resolve(joint.parent, joint.parentConnector);
			var child = resolve(joint.child, joint.childConnector);
			if (joint.role == AssemblyJointRole.Tree)
				target.addMateOnAxis(joint.id, joint.type, parent.instanceId, parent.connectorName,
					child.instanceId, child.connectorName, joint.axis, joint.defaultValue, joint.limits);
			else target.addConstraintOnAxis(joint.id, joint.type, parent.instanceId, parent.connectorName,
				child.instanceId, child.connectorName, joint.axis, joint.closureTolerance, joint.limits);
		}
		for (coupling in level.couplings)
			target.addCoupling(coupling.id, coupling.source, coupling.target, coupling.ratio, coupling.offset,
				coupling.efficiency, coupling.stiffness, coupling.backlash, coupling.drag, coupling.assumed, coupling.assumptions);
		for (path in machine.beltPaths) target.addBeltPath(path);
		// A transmission whose coupling is gone goes too. The rest take their ratios from their parts as
		// they are now, not as they were saved; a change is reported.
		for (transmission in machine.transmissions)
			if (target.mechanical.coupling(transmission.coupling) != null)
				target.drives.transmissions.push(DriveSystem.copyTransmission(transmission, id -> id));
		target.changed();
		if (target.drives.transmissions.length > 0) {
			var flat = target.derived();
			for (transmission in target.drives.transmissions) {
				var saved = target.mechanical.coupling(transmission.coupling);
				if (saved == null) continue;
				var current = derivedCoupling(flat, saved);
				if (!Equality.equals(couplingValues(saved), couplingValues(current)) || !Equality.equals(saved.assumed, current.assumed))
					target.diagnostics.warning("transmission.snapshot", transmission.coupling,
						'Transmission "${transmission.coupling}" changed; its current parts replace the stored coupling');
			}
		}
		for (connection in machine.portConnections)
			target.connectPorts(connection.id, connection.fromInstance, connection.fromPort,
				connection.toInstance, connection.toPort);
		for (entry in machine.portExposures) target.exposePort(entry.name, entry.instanceId, entry.portName);
		// Motors too: their actuators follow the motor parts as they are now.
		for (motor in machine.motors) target.addMotorRecord(motor);
		for (cylinder in machine.cylinders) target.addCylinder(cylinder.actuator, cylinder.joint, cylinder.cylinder, cylinder.valve);
		for (sensor in machine.sensors) target.addSensorRecord(sensor);
		for (contact in machine.switches) target.addTripSwitch(contact.id, contact.joint, contact.part,
			{instanceId: contact.trigger, connectorName: contact.triggerConnector}, contact.side, contact.role, contact.seed, contact.driveJoint);
		for (contact in machine.switches) if (contact.homeAfter != null && contact.homeAfter.length > 0)
			target.homeAfter(contact.joint, contact.homeAfter);
		// Encoders after the motors they read, whose actuators they point at.
		for (encoder in machine.encoders) target.addEncoderRecord(encoder);
		for (entry in machine.connectorExposures) target.exposeConnector(entry.name, entry.instanceId, entry.connectorName);
		for (entry in machine.bomExtras) {
			var mass:AssemblyBomMass = switch entry.mass {
				case SavedBomMass.Unknown: AssemblyBomMass.Unknown;
				case SavedBomMass.Point(kg, x, y, z): AssemblyBomMass.Point(kg, new Vector(x, y, z));
				case SavedBomMass.Attached(kg, id, x, y, z): AssemblyBomMass.Attached(kg, id, new Vector(x, y, z));
			};
			target.addBomItem(entry.item, entry.quantity, mass);
		}
	}

	static function couplingValues(coupling:AssemblyJointCoupling):Array<Null<Float>>
		return [coupling.ratio, coupling.offset, coupling.efficiency, coupling.stiffness, coupling.backlash, coupling.drag];

	/** The member path and connector behind exposed connector `name` of a table entry. */
	static function exposedMember(entry:AssemblySubdefinition, table:Map<String, AssemblySubdefinition>,
			name:String):{instanceId:String, connectorName:String} {
		if (entry.exposedConnectors != null) for (exposed in entry.exposedConnectors) if (exposed.name == name) {
			for (occurrence in entry.occurrences) if (occurrence.id == exposed.occurrence && occurrence.assembly != null) {
				var nested = table.get(occurrence.assembly);
				if (nested == null) throw 'Missing nested definition "${occurrence.assembly}"';
				var inner = exposedMember(nested, table, exposed.connector);
				return {instanceId: exposed.occurrence + "/" + inner.instanceId, connectorName: inner.connectorName};
			}
			return {instanceId: exposed.occurrence, connectorName: exposed.connector};
		}
		throw 'Subassembly "${entry.id}" exposes no connector "$name"';
	}

	static function canonicalize(definition:AssemblyDefinition):Void {
		definition.definitions.sort((a, b) -> Reflect.compare(a.id, b.id));
		for (component in definition.definitions)
			component.connectors.sort((a, b) -> Reflect.compare(a.name, b.name));
		definition.occurrences.sort((a, b) -> Reflect.compare(a.id, b.id));
		definition.joints.sort((a, b) -> Reflect.compare(a.id, b.id));
		if (definition.couplings != null) definition.couplings.sort((a, b) -> Reflect.compare(a.id, b.id));
		if (definition.exposedConnectors != null)
			definition.exposedConnectors.sort((a, b) -> Reflect.compare(a.name, b.name));
		if (definition.assemblies != null) for (nested in definition.assemblies) {
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
