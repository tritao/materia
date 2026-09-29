package machinekit.assembly;

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
import materia.assembly.AssemblyFrames;
import materia.assembly.AssemblyDefinition.AssemblyJointType;
import materia.assembly.AssemblyDefinition.AssemblyJointLimits;
import materia.assembly.AssemblyDefinition.AssemblyVector;
import materia.assembly.AssemblyRecord.AssemblyFrame;

typedef MachineAssemblyComponent = { var id:String; var component:MachineComponent; }
typedef MachineAssemblyConnector = { var instanceId:String; var connectorName:String; }
typedef PortRef = { var instanceId:String; var portName:String; }
typedef UpstreamResult = { var port:PortRef; var external:Bool; }
private typedef ServiceTrace = { var port:PortRef; var external:Bool; var supplied:Bool; var chain:Array<String>; }
typedef MachineSubassembly = { var id:String; var assembly:MachineAssembly; }
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

enum MachineAssemblyOperation {
	Mate(id:String, kind:AssemblyJointType, parent:MachineAssemblyConnector,
		child:MachineAssemblyConnector, value:Float, axis:Null<AssemblyVector>, limits:Null<AssemblyJointLimits>);
	Constrain(id:String, kind:AssemblyJointType, parent:MachineAssemblyConnector,
		child:MachineAssemblyConnector, axis:Null<AssemblyVector>, tolerance:Null<Float>, limits:Null<AssemblyJointLimits>);
	Couple(id:String, source:String, target:String, ratio:Float, offset:Float);
	ConnectPorts(id:String, from:PortRef, to:PortRef, line:Null<BomItem>);
}

private typedef AssemblyMember = {
	var id:String;
	var component:MachineComponent;
	var pose:AssemblyFrame;
}

/** Reusable, prefixable assembly made from MachineComponents and named connector references. */
class MachineAssembly {
	final members:Array<AssemblyMember> = [];
	final included:Array<MachineSubassembly> = [];
	final externalConnectors:Array<{name:String, instanceId:String, connectorName:String}> = [];
	final externalPorts:Array<{name:String, instanceId:String, portName:String}> = [];
	final operations:Array<MachineAssemblyOperation> = [];
	final bomItems:Array<{item:BomItem, quantity:Int, mass:AssemblyBomMass}> = [];
	final memberConnectorFrames:Array<{instanceId:String, name:String, frame:AssemblyFrame}> = [];

	public function new() {}

	public function addComponent(id:String, component:MachineComponent, ?pose:AssemblyFrame):Void {
		if (id == null || id.length == 0 || component == null) throw "Assembly component needs an id and component";
		for (member in members) if (member.id == id) throw 'Duplicate assembly component "$id"';
		members.push({id: id, component: component,
			pose: pose == null ? AssemblyFrames.identity() : copyFrame(pose)});
	}

	/** Include another assembly below a local namespace, optionally moving all of its parts. */
	public function include(id:String, assembly:MachineAssembly, ?pose:AssemblyFrame):Void {
		if (id == null || id.length == 0 || assembly == null || assembly == this)
			throw "Included assembly needs a distinct id and assembly";
		for (entry in included) if (entry.id == id) throw 'Duplicate included assembly "$id"';
		assembly.validateStructure();
		for (member in assembly.members) {
			var localPose = pose == null ? member.pose : AssemblyFrames.compose(pose, member.pose);
			addComponent(join(id, member.id), member.component, localPose);
		}
		for (connector in assembly.memberConnectorFrames)
			addMemberConnector(join(id, connector.instanceId), connector.name, connector.frame);
		for (connector in assembly.externalConnectors)
			exposeConnector(join(id, connector.name), join(id, connector.instanceId), connector.connectorName);
		// Ports define required service inputs, so each containing assembly must expose its own interface.
		for (operation in assembly.operations) addOperation(prefixed(operation, id));
		for (entry in assembly.bomItems) {
			var mass = switch entry.mass {
				case Unknown: Unknown;
				case Attached(kg, instanceId, centre): Attached(kg, join(id, instanceId), centre);
				case Point(kg, centre):
					if (pose == null) Point(kg, centre);
					else {
						var point = AssemblyFrames.transformPoint(pose, centre.x, centre.y, centre.z);
						Point(kg, new Vector(point.x, point.y, point.z));
					}
			};
			addBomItem(entry.item, entry.quantity, mass);
		}
		included.push({id: id, assembly: assembly});
	}

	public function addMate(id:String, kind:String, parent:String, parentConnector:String,
			child:String, childConnector:String, value:Float = 0):Void
		addOperation(Mate(id, cast kind, ref(parent, parentConnector), ref(child, childConnector), value, null, null));

	public function addMateOnAxis(id:String, kind:String, parent:String, parentConnector:String,
			child:String, childConnector:String, axis:AssemblyVector, value:Float = 0,
			?limits:AssemblyJointLimits):Void
		addOperation(Mate(id, cast kind, ref(parent, parentConnector), ref(child, childConnector), value, axis, limits));

	public function addConstraint(id:String, kind:String, parent:String, parentConnector:String,
			child:String, childConnector:String, ?tolerance:Float):Void
		addOperation(Constrain(id, cast kind, ref(parent, parentConnector), ref(child, childConnector), null, tolerance, null));

	public function addConstraintOnAxis(id:String, kind:String, parent:String, parentConnector:String,
			child:String, childConnector:String, axis:AssemblyVector, ?tolerance:Float,
			?limits:AssemblyJointLimits):Void
		addOperation(Constrain(id, cast kind, ref(parent, parentConnector), ref(child, childConnector), axis, tolerance, limits));

	public function addCoupling(id:String, source:String, target:String, ratio:Float, offset:Float = 0):Void
		addOperation(Couple(id, source, target, ratio, offset));

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
		addOperation(ConnectPorts(id, from, to, line));
		if (line != null) addBomItem(line, 1, lineMass);
	}

	/** Add a calculated connector on one member, such as a screw seat above a housing face. */
	public function addMemberConnector(instanceId:String, name:String, frame:AssemblyFrame):Void {
		if (name == null || name.length == 0) throw "Assembly connector needs a name";
		requireMember(instanceId);
		for (entry in memberConnectorFrames) if (entry.instanceId == instanceId && entry.name == name)
			throw 'Duplicate assembly connector "$instanceId/$name"';
		for (connector in requireMember(instanceId).connectors()) if (connector.name == name)
			throw 'Duplicate assembly connector "$instanceId/$name"';
		memberConnectorFrames.push({instanceId: instanceId, name: name, frame: copyFrame(frame)});
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
		for (op in operations) switch op {
			case Mate(id, _, parent, child, _, _, _):
				if (parents.exists(child.instanceId))
					result.error("assembly.multiple-parents", child.instanceId,
						'Assembly member "${child.instanceId}" has two parent joints');
				else parents.set(child.instanceId, parent.instanceId);
				joints.set(id, true);
			case Constrain(id, _, _, _, _, _, _): joints.set(id, true);
			case _:
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
		for (op in operations) switch op {
			case Couple(id, source, target, _, _):
				if (!joints.exists(source) || !joints.exists(target))
					result.error("assembly.missing-coupling-joint", id,
						'Assembly coupling "$id" refers to a missing joint');
			case _:
		}
	}

	function checkServices(result:Diagnostics, required:Bool = true):Void {
		var connected:Map<String, Bool> = [];
		for (op in operations) switch op {
			case ConnectPorts(id, from, to, _):
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
			case _:
		}
		if (required) for (member in members) for (port in member.component.ports())
			if (port.required && port.role == Consumer) {
				var reference = portRef(member.id, port.name);
				if (connected.exists(portKey(reference))) {
					try {
						var trace = traceUpstream(reference);
						if (!trace.supplied) result.error("port.unsupplied", '${member.id}/${port.name}',
							unsuppliedMessage(trace.chain));
					} catch (error:String) result.error("port.service-cycle", '${member.id}/${port.name}', error);
				} else if (!isExposed(member.id, port.name))
					result.error("port.required-unconnected", '${member.id}/${port.name}',
						'Required consumer port "${member.id}/${port.name}" is unconnected');
			}
	}

	function checkConnections():{connected:Map<String, Bool>, warnings:Array<String>} {
		var result = new Diagnostics();
		checkServices(result, false);
		result.throwIfErrors();
		var connected:Map<String, Bool> = [];
		for (op in operations) switch op {
			case ConnectPorts(_, from, to, _): connected.set(portKey(from), true); connected.set(portKey(to), true);
			case _:
		}
		return {connected: connected, warnings: result.warnings()};
	}

	/** Populate an existing model. All member and joint ids receive the supplied prefix. */
	public function addTo(model:AssemblyModel, prefix:String, ?pose:AssemblyFrame):Void {
		validateStructure();
		for (member in members) {
			var localPose = pose == null ? member.pose : AssemblyFrames.compose(pose, member.pose);
			member.component.addTo(model, join(prefix, member.id), localPose);
		}
		for (connector in memberConnectorFrames)
			model.connector(join(prefix, connector.instanceId), connector.name, connector.frame);
		for (op in operations) addOperationToModel(model, prefix, op);
	}

	function addOperationToModel(model:AssemblyModel, prefix:String, op:MachineAssemblyOperation):Void {
		switch op {
			case Mate(id, kind, parent, child, value, axis, limits):
				if (axis == null) model.mate(join(prefix, id), kind, join(prefix, parent.instanceId),
					parent.connectorName, join(prefix, child.instanceId), child.connectorName, value);
				else model.mateOnAxis(join(prefix, id), kind, join(prefix, parent.instanceId),
					parent.connectorName, join(prefix, child.instanceId), child.connectorName, axis, value, limits);
			case Constrain(id, kind, parent, child, axis, tolerance, limits):
				if (axis == null) model.constrain(join(prefix, id), kind, join(prefix, parent.instanceId),
					parent.connectorName, join(prefix, child.instanceId), child.connectorName, tolerance);
				else model.constrainOnAxis(join(prefix, id), kind, join(prefix, parent.instanceId),
					parent.connectorName, join(prefix, child.instanceId), child.connectorName, axis, tolerance, limits);
			case Couple(id, source, target, ratio, offset):
				model.couple(join(prefix, id), join(prefix, source), join(prefix, target), ratio, offset);
			case ConnectPorts(_, _, _, _):
		}
	}

	public function components():Array<MachineAssemblyComponent>
		return [for (member in members) {id: member.id, component: member.component}];

	/** Parent links in the mate tree; constraints do not attach members. */
	public function mateParents():Map<String, String> {
		var result:Map<String, String> = [];
		for (op in operations) switch op {
			case Mate(_, _, parent, child, _, _, _): result.set(child.instanceId, parent.instanceId);
			case _:
		}
		return result;
	}

	/** A member connector in that member's local frame. */
	public function memberConnectorFrame(instanceId:String, connectorName:String):AssemblyFrame {
		requireConnector(ref(instanceId, connectorName));
		for (entry in memberConnectorFrames)
			if (entry.instanceId == instanceId && entry.name == connectorName) return copyFrame(entry.frame);
		for (connector in requireMember(instanceId).connectors())
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
			var model = new AssemblyModel();
			addTo(model, "");
			state = model.solve();
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
			var properties = member.component.massProperties();
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
			for (op in operations) switch op {
				case ConnectPorts(_, from, to, _): if (portKey(to) == key) previous.push(from);
				case _:
			}
			for (bridge in requireMember(current.instanceId).bridges())
				if (bridge.to == current.portName) previous.push(portRef(current.instanceId, bridge.from));
			for (conversion in requireMember(current.instanceId).conversions())
				if (conversion.to == current.portName)
					previous.push(portRef(current.instanceId, conversion.from));
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

	public static function join(prefix:String, id:String):String
		return prefix == null || prefix.length == 0 ? id : '$prefix/$id';

	function addOperation(op:MachineAssemblyOperation):Void {
		var id = operationId(op);
		if (id == null || id.length == 0) throw "Assembly operation needs an id";
		for (existing in operations) if (operationId(existing) == id)
			throw 'Duplicate assembly operation "$id"';
		switch op {
			case Mate(_, kind, parent, child, _, _, _) | Constrain(_, kind, parent, child, _, _, _):
				if (kind != AssemblyJointType.Fixed && kind != AssemblyJointType.Revolute &&
					kind != AssemblyJointType.Continuous && kind != AssemblyJointType.Prismatic)
					throw 'Unsupported assembly joint "$kind"';
				requireConnector(parent);
				requireConnector(child);
			case Couple(_, source, target, _, _):
				if (source == null || source.length == 0 || target == null || target.length == 0)
					throw 'Assembly coupling "$id" needs source and target';
			case ConnectPorts(_, from, to, _):
				requirePort(from);
				requirePort(to);
		}
		operations.push(op);
	}

	function requireMember(id:String):MachineComponent {
		for (member in members) if (member.id == id) return member.component;
		throw 'Unknown assembly member "$id"';
	}

	function requireConnector(reference:MachineAssemblyConnector):Void {
		var component = requireMember(reference.instanceId);
		if (reference.connectorName == null || reference.connectorName.length == 0)
			throw 'Unknown connector "${reference.instanceId}/${reference.connectorName}"';
		for (entry in memberConnectorFrames)
			if (entry.instanceId == reference.instanceId && entry.name == reference.connectorName) return;
		for (connector in component.connectors()) if (connector.name == reference.connectorName) return;
		throw 'Unknown connector "${reference.instanceId}/${reference.connectorName}"';
	}

	public function hasPort(name:String):Bool {
		for (entry in externalPorts) if (entry.name == name) return true;
		return false;
	}

	public function hasMemberConnector(instanceId:String, connectorName:String):Bool {
		for (entry in memberConnectorFrames)
			if (entry.instanceId == instanceId && entry.name == connectorName) return true;
		for (member in members) if (member.id == instanceId)
			for (connector in member.component.connectors()) if (connector.name == connectorName) return true;
		return false;
	}

	function requirePort(reference:PortRef):ComponentPort {
		var component = requireMember(reference.instanceId);
		if (reference.portName == null || reference.portName.length == 0)
			throw 'Unknown port "${reference.instanceId}/${reference.portName}"';
		for (port in component.ports()) if (port.name == reference.portName) return port;
		throw 'Unknown port "${reference.instanceId}/${reference.portName}"';
	}

	function isConnected(reference:PortRef):Bool {
		var key = portKey(reference);
		for (op in operations) switch op {
			case ConnectPorts(_, from, to, _): if (portKey(from) == key || portKey(to) == key) return true;
			case _:
		}
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

	static function operationId(op:MachineAssemblyOperation):String return switch op {
		case Mate(id, _, _, _, _, _, _) | Constrain(id, _, _, _, _, _, _) | Couple(id, _, _, _, _) |
			ConnectPorts(id, _, _, _): id;
	}

	static function ref(instanceId:String, connectorName:String):MachineAssemblyConnector
		return {instanceId: instanceId, connectorName: connectorName};

	static function prefixed(op:MachineAssemblyOperation, prefix:String):MachineAssemblyOperation return switch op {
		case Mate(id, kind, parent, child, value, axis, limits):
			Mate(join(prefix, id), kind, ref(join(prefix, parent.instanceId), parent.connectorName),
				ref(join(prefix, child.instanceId), child.connectorName), value, axis, limits);
		case Constrain(id, kind, parent, child, axis, tolerance, limits):
			Constrain(join(prefix, id), kind, ref(join(prefix, parent.instanceId), parent.connectorName),
				ref(join(prefix, child.instanceId), child.connectorName), axis, tolerance, limits);
		case Couple(id, source, target, ratio, offset):
			Couple(join(prefix, id), join(prefix, source), join(prefix, target), ratio, offset);
		case ConnectPorts(id, from, to, line):
			ConnectPorts(join(prefix, id), portRef(join(prefix, from.instanceId), from.portName),
				portRef(join(prefix, to.instanceId), to.portName), line);
	}

	static function copyFrame(frame:AssemblyFrame):AssemblyFrame
		return {x: frame.x, y: frame.y, z: frame.z, qx: frame.qx, qy: frame.qy, qz: frame.qz, qw: frame.qw};
}
