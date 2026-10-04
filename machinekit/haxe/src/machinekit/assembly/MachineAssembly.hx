package machinekit.assembly;

import cadkit.modeling.AssemblyModel;
import cadkit.modeling.AssemblyState;
import cadkit.modeling.Vector;
import cadkit.InertiaTensor;
import machinekit.component.Bom;
import machinekit.component.BomItem;
import machinekit.component.ComponentRegistry;
import machinekit.component.MachineComponent;
import machinekit.component.MassProperties;
import machinekit.assembly.MachineAssemblyDescription;
import machinekit.assembly.MachineAssemblyDescription.TransmissionRecord;
import materia.assembly.AssemblyDefinitionCodec;
import materia.assembly.AssemblyDefinition;
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

/**
 * A reusable machine assembly: MachineComponents, the subassemblies it includes, and how they are
 * mated, coupled, driven, wired and itemised. Subassemblies stay whole; records on this level name
 * members below it by path (`arm/link1`). Checks and exports work on one flattened view, built on
 * demand from the four parts of a level: `MechanicalAssembly`, `ServiceNetwork`, `DriveSystem` and
 * `AssemblyInventory`.
 */
@:allow(machinekit.assembly)
class MachineAssembly {
	public static inline var SCHEMA_VERSION:Int = 13;
	/** Findings from rebuilding saved data, such as transmissions whose parts changed since saving. */
	public final diagnostics:Diagnostics = new Diagnostics();
	final mechanical:MechanicalAssembly = new MechanicalAssembly();
	final services:ServiceNetwork = new ServiceNetwork();
	final drives:DriveSystem = new DriveSystem();
	final inventory:AssemblyInventory = new AssemblyInventory();
	final massByDefinition:Map<String, MassProperties> = [];
	var flatCache:Null<FlatAssembly>;

	public function new() {}

	// Saved form ------------------------------------------------------------------------------

	/** Capture a portable description. Code-only members remain visible but cannot be saved. */
	public function describe():MachineAssemblyDescription return MachineAssemblyCodec.describe(this);

	/** Generated wire codec; reject every unbuildable member before emitting a file. */
	public function encode():String return MachineAssemblyCodec.encode(describe());

	public static function decode(text:String, ?registry:ComponentRegistry):MachineAssembly
		return fromDescription(MachineAssemblyCodec.decode(text), registry);

	/** Rebuild through registered recipes; no component object is stored in the description. */
	public static function fromDescription(description:MachineAssemblyDescription, ?registry:ComponentRegistry):MachineAssembly {
		var result = new MachineAssembly();
		MachineAssemblyCodec.read(description, result, registry);
		return result;
	}

	/** Copy builder state for a derived assembly or an owned tool snapshot. */
	public function copyInto(target:MachineAssembly):Void {
		for (item in diagnostics.items)
			target.diagnostics.add(item.severity, item.code, item.subject, item.message);
		for (id in mechanical.order) {
			var member = mechanical.member(id);
			if (member != null) target.mechanical.addMember(id, copyComponent(member.component), member.pose);
			else {
				// Subassemblies are owned snapshots that nothing edits, so copies share them.
				var entry = mechanical.requireSubassembly(id);
				target.mechanical.addSubassembly({id: entry.id, assembly: entry.assembly,
					pose: entry.pose == null ? null : copyFrame(entry.pose)});
			}
		}
		for (entry in mechanical.memberConnectors) target.mechanical.memberConnectors.push({instanceId: entry.instanceId,
			name: entry.name, frame: copyFrame(entry.frame)});
		for (entry in mechanical.connectorExposures) target.mechanical.connectorExposures.push({name: entry.name,
			instanceId: entry.instanceId, connectorName: entry.connectorName});
		for (joint in mechanical.joints) target.mechanical.joints.push(MechanicalAssembly.copyJoint(joint, same));
		for (coupling in mechanical.couplings) target.mechanical.couplings.push(MechanicalAssembly.copyCoupling(coupling, same));
		for (entry in services.connections) target.services.connections.push({id: entry.id,
			fromInstance: entry.fromInstance, fromPort: entry.fromPort, toInstance: entry.toInstance, toPort: entry.toPort});
		for (entry in services.exposures) target.services.exposures.push({name: entry.name,
			instanceId: entry.instanceId, portName: entry.portName});
		for (entry in drives.transmissions) target.drives.transmissions.push(DriveSystem.copyTransmission(entry, same));
		for (entry in drives.beltPaths) target.drives.beltPaths.push(DriveSystem.copyBeltPath(entry, same));
		for (entry in drives.motors) target.drives.motors.push(DriveSystem.copyMotor(entry, same));
		for (entry in drives.encoders) target.drives.encoders.push(DriveSystem.copyEncoder(entry, same));
		for (entry in drives.cylinders) target.drives.cylinders.push(DriveSystem.copyCylinder(entry, same));
		for (entry in drives.sensors) target.drives.sensors.push(DriveSystem.copySensor(entry, same));
		for (entry in inventory.items) target.inventory.items.push({item: copyBomItem(entry.item),
			quantity: entry.quantity, mass: entry.mass});
		target.changed();
	}

	public function snapshot():MachineAssembly {
		var result = new MachineAssembly();
		copyInto(result);
		return result;
	}

	static function same(id:String):String return id;

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

	// Mechanical -------------------------------------------------------------------------------

	public function addComponent(id:String, component:MachineComponent, ?pose:AssemblyFrame):Void {
		InstancePath.segment(id);
		addMember(id, component, pose);
	}

	/**
	 * Add a member whose local name is a path, such as `bin-01/indicator`, to group members without
	 * a subassembly. The first segment cannot name a subassembly.
	 */
	public function addComponentAt(path:InstancePath, component:MachineComponent, ?pose:AssemblyFrame):Void {
		if (mechanical.subassembly(path.segments()[0]) != null)
			throw 'Member "$path" would be inside subassembly "${path.segments()[0]}"';
		addMember(path, component, pose);
	}

	function addMember(id:String, component:MachineComponent, ?pose:AssemblyFrame):Void {
		if (id == null || id.length == 0 || component == null) throw "Assembly component needs an id and component";
		mechanical.addMember(id, component, pose == null ? AssemblyFrames.identity() : pose);
		changed();
	}

	/**
	 * Include a snapshot of another assembly as subassembly `id`, optionally placed at `pose`. Its
	 * members are then `id/...` here. Its exposed connectors are visible as `id/name`; its exposed
	 * ports are not, so this assembly connects or exposes the services its subassemblies need.
	 */
	public function include(id:String, assembly:MachineAssembly, ?pose:AssemblyFrame):Void {
		if (id == null || id.length == 0 || assembly == null || assembly == this)
			throw "Included assembly needs a distinct id and assembly";
		InstancePath.segment(id);
		for (member in mechanical.members) if (StringTools.startsWith(member.id, id + "/"))
			throw 'Included assembly "$id" would hide member "${member.id}"';
		assembly.validateStructure();
		mechanical.addSubassembly({id: id, assembly: assembly.snapshot(), pose: pose == null ? null : copyFrame(pose)});
		changed();
	}

	public function subassemblies():Array<MachineSubassembly> return mechanical.subassemblies.copy();

	public function addMate(id:String, kind:AssemblyJointType, parent:String, parentConnector:String,
			child:String, childConnector:String, value:Float = 0):Void
		addMateOnAxis(id, kind, parent, parentConnector, child, childConnector,
			{x: 0, y: 1, z: 0}, value);

	public function addMateOnAxis(id:String, kind:AssemblyJointType, parent:String, parentConnector:String,
			child:String, childConnector:String, axis:AssemblyVector, value:Float = 0,
			?limits:AssemblyJointLimits):Void
		addJoint({id: id, type: kind, role: AssemblyJointRole.Tree, parent: parent,
			parentConnector: parentConnector, child: child, childConnector: childConnector,
			axis: axis, limits: resolvedLimits(limits), defaultValue: value});

	public function addConstraint(id:String, kind:AssemblyJointType, parent:String, parentConnector:String,
			child:String, childConnector:String, ?tolerance:Float):Void
		addConstraintOnAxis(id, kind, parent, parentConnector, child, childConnector,
			{x: 0, y: 1, z: 0}, tolerance);

	public function addConstraintOnAxis(id:String, kind:AssemblyJointType, parent:String, parentConnector:String,
			child:String, childConnector:String, axis:AssemblyVector, ?tolerance:Float,
			?limits:AssemblyJointLimits):Void
		addJoint({id: id, type: kind, role: AssemblyJointRole.Closure, parent: parent,
			parentConnector: parentConnector, child: child, childConnector: childConnector,
			axis: axis, limits: resolvedLimits(limits), defaultValue: 0}, tolerance);

	function addJoint(joint:KinematicJoint, ?tolerance:Float):Void {
		requireOperationId(joint.id);
		if (joint.type != AssemblyJointType.Fixed && joint.type != AssemblyJointType.Revolute &&
			joint.type != AssemblyJointType.Continuous && joint.type != AssemblyJointType.Prismatic)
			throw 'Unsupported assembly joint "${joint.type}"';
		requireConnector(joint.parent, joint.parentConnector);
		requireConnector(joint.child, joint.childConnector);
		var saved = MechanicalAssembly.copyJoint(joint, same);
		if (joint.role == AssemblyJointRole.Closure && tolerance != null) saved.closureTolerance = tolerance;
		mechanical.joints.push(saved);
		changed();
	}

	public function addCoupling(id:String, source:String, target:String, ratio:Float, offset:Float = 0,
			?efficiency:Float, ?stiffness:Float, ?backlash:Float, ?drag:Float, ?assumed:haxe.ds.ReadOnlyArray<String>,
			?assumptions:haxe.ds.ReadOnlyArray<materia.assembly.AssemblyDefinition.QuantityAssumption>):Void {
		if (drives.transmission(id) != null) throw 'Transmission coupling "$id" is read-only; edit its parts or allowances';
		requireOperationId(id);
		if (source == null || source.length == 0 || target == null || target.length == 0)
			throw 'Assembly coupling "$id" needs source and target';
		var coupling:materia.assembly.AssemblyDefinition.AssemblyJointCoupling = {id: id, source: source,
			target: target, ratio: ratio, offset: offset};
		if (efficiency != null) coupling.efficiency = efficiency;
		if (stiffness != null) coupling.stiffness = stiffness;
		if (backlash != null) coupling.backlash = backlash;
		if (drag != null) coupling.drag = drag;
		if (assumptions != null && assumptions.length > 0) coupling.assumptions = [for (value in assumptions)
			{quantity: value.quantity, label: value.label}];
		if (assumed != null && assumed.length > 0) coupling.assumed = [for (label in assumed) label];
		mechanical.couplings.push(coupling);
		changed();
	}

	/** Add a calculated connector on one member, such as a screw seat above a housing face. */
	public function addMemberConnector(instanceId:String, name:String, frame:AssemblyFrame):Void {
		if (name == null || name.length == 0) throw "Assembly connector needs a name";
		requireMember(instanceId);
		if (connectorFrame(instanceId, name) != null) throw 'Duplicate assembly connector "$instanceId/$name"';
		mechanical.memberConnectors.push({instanceId: instanceId, name: name, frame: copyFrame(frame)});
		changed();
	}

	/** Publish a stable assembly-level name that resolves to one member connector. */
	public function exposeConnector(name:String, instanceId:String, connectorName:String):Void {
		if (name == null || name.length == 0) throw "External assembly connector needs a name";
		requireConnector(instanceId, connectorName);
		if (connectorNames().indexOf(name) >= 0) throw 'Duplicate external assembly connector "$name"';
		mechanical.connectorExposures.push({name: name, instanceId: instanceId, connectorName: connectorName});
	}

	public function connectorNames():Array<String> {
		var result = [for (entry in mechanical.connectorExposures) entry.name];
		for (entry in mechanical.subassemblies) for (name in entry.assembly.connectorNames())
			result.push(entry.id + "/" + name);
		return result;
	}

	/** The member connector behind exposed connector `name`, its member path under `prefix`. */
	public function connector(name:String, prefix:String = ""):MachineAssemblyConnector {
		for (entry in mechanical.connectorExposures) if (entry.name == name)
			return {instanceId: join(prefix, entry.instanceId), connectorName: entry.connectorName};
		var slash = name.indexOf("/");
		if (slash > 0) {
			var entry = mechanical.subassembly(name.substr(0, slash));
			if (entry != null) return entry.assembly.connector(name.substr(slash + 1), join(prefix, entry.id));
		}
		throw 'Missing assembly connector "$name"';
	}

	// Services ---------------------------------------------------------------------------------

	public function connectPorts(id:String, fromInstance:String, fromPort:String,
			toInstance:String, toPort:String, ?line:BomItem,
			lineMass:AssemblyBomMass = Unknown):Void {
		var from = resolvePort(fromInstance, fromPort), to = resolvePort(toInstance, toPort);
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
		services.connections.push({id: id, fromInstance: from.instanceId, fromPort: from.portName,
			toInstance: to.instanceId, toPort: to.portName});
		changed();
		if (line != null) addBomItem(line, 1, lineMass);
	}

	public function exposePort(name:String, instanceId:String, portName:String):Void {
		if (name == null || name.length == 0) throw "External assembly port needs a name";
		requirePort(ServiceNetwork.portRef(instanceId, portName));
		if (hasPort(name)) throw 'Duplicate external assembly port "$name"';
		services.exposures.push({name: name, instanceId: instanceId, portName: portName});
		changed();
	}

	public function portNames():Array<String> return [for (port in services.exposures) port.name];

	public function hasPort(name:String):Bool {
		for (entry in services.exposures) if (entry.name == name) return true;
		return false;
	}

	public function port(name:String, prefix:String = ""):PortRef {
		for (entry in services.exposures) if (entry.name == name)
			return ServiceNetwork.portRef(join(prefix, entry.instanceId), entry.portName);
		throw 'Missing assembly port "$name"';
	}

	/** Trace a service through connections, bridges, and a single-input converter. */
	public function upstream(instanceId:String, portName:String):UpstreamResult {
		var trace = suppliedTrace(instanceId, portName);
		return {port: trace.port, external: trace.external};
	}

	/** Ordered member/port path from a consumer to its supplied boundary. */
	public function upstreamChain(instanceId:String, portName:String):Array<String>
		return suppliedTrace(instanceId, portName).chain.copy();

	function suppliedTrace(instanceId:String, portName:String):ServiceNetwork.ServiceTrace {
		var flat = flat();
		var connections = new Diagnostics();
		ServiceNetwork.check(flat, connections, false);
		connections.throwIfErrors();
		var trace = ServiceNetwork.trace(flat, ServiceNetwork.portRef(instanceId, portName));
		if (!trace.supplied) throw ServiceNetwork.unsuppliedMessage(trace.chain);
		return trace;
	}

	// Drives -----------------------------------------------------------------------------------

	/**
	 * Couple joint `follower` to `leader` through the parts of `source`, which set the ratio: the
	 * follower sits at zero where the leader is at `leaderZero`. Rebuilding the assembly from its
	 * description works the ratio out again from the parts' values then. Returns the ratio.
	 */
	public function addTransmission(id:String, leader:String, follower:String, source:Transmission,
			sense:Sense, leaderZero:Float = 0):Float {
		var record:TransmissionRecord = {coupling: id, source: source, sense: sense, leaderZero: leaderZero};
		var relation = DriveSystem.resolve(derived(), record, leader, false, follower);
		var ratio = relation.ratio;
		addCoupling(id, leader, follower, ratio, -ratio * leaderZero, relation.efficiency);
		DriveSystem.write(mechanical.couplings, mechanical.joints, record, relation);
		drives.transmissions.push(record);
		changed();
		return ratio;
	}

	/** Change each stated allowance independently; rebuilding keeps the fields left alone. */
	public function setTransmissionOverrides(id:String, stiffness:AllowanceEdit = Leave,
			backlash:AllowanceEdit = Leave, drag:AllowanceEdit = Leave):Void {
		var entry = ownTransmission(id);
		var proposed = DriveSystem.copyTransmission(entry, same);
		proposed.stiffness = editAllowance(entry.stiffness, stiffness);
		proposed.backlash = editAllowance(entry.backlash, backlash);
		proposed.drag = editAllowance(entry.drag, drag);
		var relation = DriveSystem.resolve(derived(), proposed, null, false);
		entry.stiffness = proposed.stiffness;
		entry.backlash = proposed.backlash;
		entry.drag = proposed.drag;
		DriveSystem.write(mechanical.couplings, mechanical.joints, entry, relation);
		changed();
	}

	static function editAllowance(current:Null<Float>, edit:AllowanceEdit):Null<Float> return switch edit {
		case Leave: current;
		case Clear: null;
		case State(value): value;
	};

	/**
	 * Says how the ends of lead screw transmission `id` are held (`near` is the end by its motor), over
	 * `unsupported` mm (the whole screw by default). The screw's joint is then capped at 80% of its first
	 * bending speed (`LeadScrew.criticalSpeed`); the cap flows through `RobotModel.coupledLimits` to
	 * its axis. Call it once the screw's joint exists. Returns the cap in rad/s.
	 */
	public function supportScrew(id:String, near:machinekit.motion.ScrewSupport, far:machinekit.motion.ScrewSupport,
			?unsupported:Float):Float {
		var entry = ownTransmission(id);
		if (!switch entry.source { case LeadScrew(_, _): true; default: false; }) throw 'Transmission "$id" is not a lead screw';
		var proposed = DriveSystem.copyTransmission(entry, same);
		proposed.near = near;
		proposed.far = far;
		proposed.unsupported = unsupported;
		var relation = DriveSystem.resolve(derived(), proposed, null, false);
		entry.near = near;
		entry.far = far;
		entry.unsupported = unsupported;
		DriveSystem.write(mechanical.couplings, mechanical.joints, entry, relation);
		changed();
		var cap = relation.followerSpeedCap;
		if (cap == null) throw 'Transmission "$id" has no supports';
		return cap;
	}

	function ownTransmission(id:String):TransmissionRecord {
		var entry = drives.transmission(id);
		if (entry == null) throw 'No transmission "$id"';
		return entry;
	}

	/** The transmission behind coupling `id`, or null when its ratio is a plain number. */
	public function transmissionFor(id:String):Null<TransmissionRecord> {
		for (entry in flat().transmissions) if (entry.coupling == id) return DriveSystem.copyTransmission(entry, same);
		return null;
	}

	/** State the belt's physical clamp and pulley centres, in path order. */
	public function addBeltPath(path:BeltPathRecord):Void {
		requireMember(path.belt);
		var clamp = path.clamp;
		if (clamp != null) requireConnector(clamp.instanceId, clamp.connectorName);
		for (wrap in path.wraps) requireConnector(wrap.instanceId, wrap.connectorName);
		for (existing in flat().beltPaths) if (existing.belt == path.belt) throw 'Duplicate belt path "${path.belt}"';
		drives.beltPaths.push(DriveSystem.copyBeltPath(path, same));
		changed();
	}

	/**
	 * Motor member `motor` (a stepper, or any part that is a `MotorDrive`) drives joint `joint` on a
	 * driver member `driver`. Its actuator, `id`, gets the motor's drive kind and torque-speed curve, its rotor
	 * inertia and its usable effort and rate: for a stepper `margin` of its holding torque and the
	 * speed up to where pull-out torque falls to that, for a servo its peak torque and maximum speed.
	 * Rebuilding the assembly works them out again from the motor and driver settings.
	 */
	public function addMotor(id:String, joint:String, motor:String, driver:String, margin:Float = 0.5,
			?gearbox:String, ?processVelocity:materia.assembly.AssemblyDefinition.AssemblyProcessVelocityDrive):Void
		addMotorRecord({actuator: id, joint: joint, motor: motor, driver: driver, margin: margin, gearbox: gearbox,
			processVelocity: processVelocity});

	function addMotorRecord(record:MotorRecord):Void {
		DriveSystem.checkMotor(flat(), record);
		drives.motors.push(DriveSystem.copyMotor(record, same));
		changed();
	}

	/** Resolve one motor binding from the current parts and power connections. */
	public function actuatorFor(id:String):materia.assembly.AssemblyDefinition.AssemblyActuator {
		var flat = flat();
		for (record in flat.motors) if (record.actuator == id) return DriveSystem.resolveMotor(flat, record, true);
		throw 'Missing assembly actuator "$id"';
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

	function addEncoderRecord(record:EncoderRecord):Void {
		DriveSystem.checkEncoder(flat(), record);
		if (record.actuator != null) for (previous in drives.encoders)
			if (previous.actuator == record.actuator) previous.actuator = null;
		drives.encoders.push(DriveSystem.copyEncoder(record, same));
		changed();
	}

	/** Bind a cylinder to an existing prismatic guide joint; pressure and direction are rebuilt from parts. */
	public function addCylinder(id:String, joint:String, cylinderMember:String, valveMember:String):Void {
		requireOperationId(id);
		var record = {actuator: id, joint: joint, cylinder: cylinderMember, valve: valveMember};
		DriveSystem.checkCylinder(flat(), record);
		drives.cylinders.push(record);
		changed();
	}

	/** Gauge pressure at a pneumatic consumer, resolved from its current service wiring. */
	public function airPressure(instanceId:String, portName:String):Float
		return DriveSystem.airPressure(flat(), instanceId, portName);

	/** A reed or limit switch with an inclusive coordinate window and release hysteresis. */
	public function addSwitch(id:String, joint:String, window:AssemblyJointLimits, hysteresis:Float = 0):Void {
		if (window == null || window.lower == null || window.upper == null)
			throw 'Switch "$id" needs a closed coordinate window';
		addSensorRecord({id: id, kind: "joint_switch", joint: joint, windowLower: window.lower,
			windowUpper: window.upper, hysteresis: hysteresis});
	}

	/** A digital spindle feedback sensor that turns on at the requested absolute joint speed. */
	public function addAtSpeed(id:String, joint:String, minimumSpeed:Float):Void
		addSensorRecord({id: id, kind: "at_speed", joint: joint, windowLower: minimumSpeed, hysteresis: 0.0});

	/** A one-ray presence sensor normal to a connector face: an exposed connector's name, or `member/connector`. */
	public function addPresence(id:String, connector:String, range:Float):Void {
		var target = presenceConnector(connector);
		addSensorRecord({id: id, kind: "presence", occurrence: target.instanceId,
			connector: target.connectorName, range: range});
	}

	function presenceConnector(name:String):MachineAssemblyConnector {
		if (name == null) throw "Presence sensor needs an exposed or member connector";
		if (connectorNames().indexOf(name) >= 0) return connector(name);
		var at = name.lastIndexOf("/");
		if (at <= 0 || at == name.length - 1) throw 'Presence sensor connector "$name" must name occurrence/connector';
		var instance = name.substr(0, at), connectorName = name.substr(at + 1);
		requireConnector(instance, connectorName);
		return {instanceId: instance, connectorName: connectorName};
	}

	function addSensorRecord(record:materia.assembly.AssemblyDefinition.AssemblySensor):Void {
		requireOperationId(record.id);
		DriveSystem.checkSensor(flat(), record);
		drives.sensors.push(DriveSystem.copySensor(record, same));
		changed();
	}

	// Inventory --------------------------------------------------------------------------------

	public function addBomItem(item:BomItem, quantity:Int = 1, mass:AssemblyBomMass = Unknown):Void {
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
		inventory.items.push({item: item, quantity: quantity, mass: mass});
		changed();
	}

	public function billOfMaterials():Bom return AssemblyInventory.billOfMaterials(flat());

	/** Sum posed component masses; separately report BOM extras with no mass model. */
	public function massProperties(?state:AssemblyState):MachineAssemblyMassProperties
		return massPropertiesFromPoses(solvedPoses(state));

	/** Mass using poses already solved for this assembly. */
	public function massPropertiesFromPoses(poses:Map<String, AssemblyFrame>):MachineAssemblyMassProperties
		return AssemblyInventory.massProperties(flat(), poses, massByDefinition);

	// Checks and exports -----------------------------------------------------------------------

	/** Collect structural, drive and service faults without stopping at the first one. */
	public function check():Diagnostics {
		var flat = derived();
		var result = new Diagnostics();
		MechanicalAssembly.checkStructure(flat, result);
		for (item in diagnostics.items) result.add(item.severity, item.code, item.subject, item.message);
		for (item in flat.diagnostics.items) result.add(item.severity, item.code, item.subject, item.message);
		ServiceNetwork.check(flat, result);
		return result;
	}

	public function validateStructure():Void {
		var result = new Diagnostics();
		MechanicalAssembly.checkStructure(flat(), result);
		result.throwIfErrors();
	}

	public function validate():Array<String> {
		var result = check();
		result.throwIfErrors();
		return result.warnings();
	}

	/**
	 * The flattened ProjectKit definition, with member paths as occurrence ids and couplings worked
	 * out from their transmissions. Actuators and encoders are compiled by `addTo`.
	 */
	public function definition():AssemblyDefinition
		return AssemblyDefinitionCodec.decode(AssemblyDefinitionCodec.encode(derived().definition));

	/** Populate an existing model. All member and joint ids receive the supplied prefix. */
	public function addTo(model:AssemblyModel, prefix:String, ?pose:AssemblyFrame):Void {
		var flat = derived();
		var structure = new Diagnostics();
		MechanicalAssembly.checkStructure(flat, structure);
		structure.throwIfErrors();
		diagnostics.throwIfErrors();
		flat.diagnostics.throwIfErrors();
		var mechanical = flat.definition;
		for (occurrence in mechanical.occurrences) {
			var localPose = pose == null ? occurrence.initialPose :
				AssemblyFrames.compose(pose, occurrence.initialPose);
			var id = join(prefix, occurrence.id);
			model.add(id, localPose);
			for (connector in flat.requireDefinition(occurrence.definition).connectors)
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
				coupling.stiffness, coupling.backlash, coupling.drag, coupling.assumed, coupling.assumptions);
		if (mechanical.elasticNetworks != null) for (network in mechanical.elasticNetworks)
			model.addElasticNetwork(materia.assembly.AssemblyDefinitionFlattener.copyElasticNetwork(network, id -> join(prefix, id)));
		var compiled = DriveSystem.compileMotors(flat);
		for (actuator in compiled.actuators) {
			var copy = materia.assembly.AssemblyDefinitionFlattener.copyActuator(actuator, join(prefix, actuator.id),
				join(prefix, actuator.joint));
			if (actuator.encoder != null) copy.encoder = join(prefix, actuator.encoder);
			model.actuateDrive(copy);
		}
		for (encoder in compiled.encoders)
			model.addEncoder(materia.assembly.AssemblyDefinitionFlattener.copyEncoder(encoder, join(prefix, encoder.id),
				join(prefix, encoder.joint)));
		for (sensor in compiled.sensors)
			model.addSensor(materia.assembly.AssemblyDefinitionFlattener.copySensor(sensor, sensor.id, prefix));
	}

	/** Every member, subassembly members included, by path. */
	public function components():Array<MachineAssemblyComponent>
		return [for (member in flat().members) {id: member.id, component: member.component}];

	/** Parent links in the mate tree; constraints do not attach members. */
	public function mateParents():Map<String, String> {
		var result:Map<String, String> = [];
		for (joint in flat().definition.joints) if (joint.role == AssemblyJointRole.Tree)
			result.set(joint.child, joint.parent);
		return result;
	}

	/** The members mated to `instanceId` by a mate, on either side of it. */
	public function matedMembers(instanceId:String):Array<String> {
		requireMember(instanceId);
		var result:Array<String> = [];
		for (joint in flat().definition.joints) {
			var other = joint.parent == instanceId ? joint.child : joint.child == instanceId ? joint.parent : null;
			if (other != null && result.indexOf(other) < 0) result.push(other);
		}
		return result;
	}

	/** A member connector in that member's local frame. */
	public function memberConnectorFrame(instanceId:String, connectorName:String):AssemblyFrame {
		requireConnector(instanceId, connectorName);
		return copyFrame(connectorFrame(instanceId, connectorName));
	}

	public function hasMemberConnector(instanceId:String, connectorName:String):Bool
		return memberAt(instanceId) != null && connectorFrame(instanceId, connectorName) != null;

	/** Solve every member pose from one AssemblyState, or read them from a supplied state. */
	public function solvedPoses(?state:AssemblyState):Map<String, AssemblyFrame> {
		var flat = derived();
		if (state == null) {
			state = new AssemblyState(AssemblyDefinitionCodec.decode(AssemblyDefinitionCodec.encode(flat.definition)));
			state.checkClosures();
		}
		var result:Map<String, AssemblyFrame> = [];
		for (member in flat.members) result.set(member.id, state.worldPose(member.id));
		return result;
	}

	// Flattened view ---------------------------------------------------------------------------

	function changed():Void flatCache = null;

	/** This assembly and its subassemblies in one namespace; rebuilt after any edit. */
	function flat():FlatAssembly {
		var cached = flatCache;
		if (cached != null) return cached;
		var result = new FlatAssembly();
		flattenInto(result, "", null, true);
		flatCache = result;
		return result;
	}

	/**
	 * The flattened view with every transmission and belt worked out from its parts as they are now.
	 * Parts can change in place (a driver's rating, say), so this always derives afresh.
	 */
	function derived():FlatAssembly {
		changed();
		var result = flat();
		DriveSystem.derive(result);
		return result;
	}

	function flattenInto(flat:FlatAssembly, prefix:String, pose:Null<AssemblyFrame>, top:Bool):Void {
		var map = prefix.length == 0 ? same : (id:String) -> prefix + "/" + id;
		for (id in mechanical.order) {
			var member = mechanical.member(id);
			if (member != null) {
				flat.addMember(map(id), member.component,
					pose == null ? copyFrame(member.pose) : AssemblyFrames.compose(pose, member.pose));
				continue;
			}
			var entry = mechanical.requireSubassembly(id);
			var local = entry.pose == null ? AssemblyFrames.identity() : entry.pose;
			entry.assembly.flattenInto(flat, map(id), pose == null ? copyFrame(local) : AssemblyFrames.compose(pose, local), false);
		}
		for (entry in mechanical.memberConnectors) flat.addConnector(map(entry.instanceId), entry.name, entry.frame);
		for (joint in mechanical.joints) flat.definition.joints.push(MechanicalAssembly.copyJoint(joint, map));
		for (coupling in mechanical.couplings) flat.definition.couplings.push(MechanicalAssembly.copyCoupling(coupling, map));
		for (entry in services.connections) flat.connections.push({id: map(entry.id),
			fromInstance: map(entry.fromInstance), fromPort: entry.fromPort,
			toInstance: map(entry.toInstance), toPort: entry.toPort});
		if (top) for (entry in services.exposures) flat.portExposures.push({name: entry.name,
			instanceId: entry.instanceId, portName: entry.portName});
		for (entry in drives.transmissions) flat.transmissions.push(DriveSystem.copyTransmission(entry, map));
		for (entry in drives.beltPaths) flat.beltPaths.push(DriveSystem.copyBeltPath(entry, map));
		for (entry in drives.motors) flat.motors.push(DriveSystem.copyMotor(entry, map));
		for (entry in drives.encoders) flat.encoders.push(DriveSystem.copyEncoder(entry, map));
		for (entry in drives.cylinders) flat.cylinders.push(DriveSystem.copyCylinder(entry, map));
		for (entry in drives.sensors) flat.sensors.push(DriveSystem.copySensor(entry, map));
		for (entry in inventory.items) flat.bomItems.push({item: entry.item, quantity: entry.quantity,
			mass: switch entry.mass {
				case Unknown: Unknown;
				case Attached(kg, instanceId, centre): Attached(kg, map(instanceId), centre);
				case Point(kg, centre):
					if (pose == null) Point(kg, centre);
					else {
						var point = AssemblyFrames.transformPoint(pose, centre.x, centre.y, centre.z);
						Point(kg, new Vector(point.x, point.y, point.z));
					}
			}});
	}

	// Lookups by path --------------------------------------------------------------------------

	/** The member at `path` below this level, or null. */
	function memberAt(path:String):Null<MachineComponent> {
		if (path == null) return null;
		var own = mechanical.member(path);
		if (own != null) return own.component;
		var slash = path.indexOf("/");
		if (slash < 0) return null;
		var entry = mechanical.subassembly(path.substr(0, slash));
		return entry == null ? null : entry.assembly.memberAt(path.substr(slash + 1));
	}

	/** Connector `name` of the member at `path` as this level sees it, or null. */
	function connectorFrame(path:String, name:String):Null<AssemblyFrame> {
		for (entry in mechanical.memberConnectors) if (entry.instanceId == path && entry.name == name) return entry.frame;
		var own = mechanical.member(path);
		if (own != null) {
			for (connector in own.component.connectors()) if (connector.name == name) return connector.frame;
			return null;
		}
		var slash = path.indexOf("/");
		if (slash < 0) return null;
		var entry = mechanical.subassembly(path.substr(0, slash));
		return entry == null ? null : entry.assembly.connectorFrame(path.substr(slash + 1), name);
	}

	function requireMember(path:String):MachineComponent {
		var member = memberAt(path);
		if (member == null) throw 'Unknown assembly member "$path"';
		return member;
	}

	function requireConnector(path:String, name:String):Void {
		requireMember(path);
		if (name == null || name.length == 0 || connectorFrame(path, name) == null)
			throw 'Unknown connector "$path/$name"';
	}

	/** A port of a member, or a port a subassembly exposes, named by the subassembly's path. */
	function resolvePort(instanceId:String, portName:String):PortRef {
		if (memberAt(instanceId) != null) return ServiceNetwork.portRef(instanceId, portName);
		var level:MachineAssembly = this, prefix = "";
		for (segment in instanceId.split("/")) {
			var entry = level.mechanical.subassembly(segment);
			if (entry == null) return ServiceNetwork.portRef(instanceId, portName);
			prefix = prefix.length == 0 ? segment : prefix + "/" + segment;
			level = entry.assembly;
		}
		return level.port(portName, prefix);
	}

	function requirePort(reference:PortRef):machinekit.component.ComponentPort {
		var member = requireMember(reference.instanceId);
		if (reference.portName == null || reference.portName.length == 0 || !member.hasPort(reference.portName))
			throw 'Unknown port "${reference.instanceId}/${reference.portName}"';
		return member.port(reference.portName);
	}

	/** Joint, coupling and connection ids share one namespace with those of subassemblies. */
	function requireOperationId(id:String):Void {
		if (id == null || id.length == 0) throw "Assembly operation needs an id";
		for (joint in mechanical.joints) if (joint.id == id) throw 'Duplicate assembly operation "$id"';
		for (coupling in mechanical.couplings) if (coupling.id == id) throw 'Duplicate assembly operation "$id"';
		for (existing in services.connections) if (existing.id == id) throw 'Duplicate assembly operation "$id"';
		if (id.indexOf("/") < 0) return;
		var flat = flat();
		for (joint in flat.definition.joints) if (joint.id == id) throw 'Duplicate assembly operation "$id"';
		for (coupling in flat.definition.couplings) if (coupling.id == id) throw 'Duplicate assembly operation "$id"';
		for (existing in flat.connections) if (existing.id == id) throw 'Duplicate assembly operation "$id"';
	}

	public static function join(prefix:String, id:String):String {
		var child = InstancePath.of(id);
		return prefix == null || prefix.length == 0 ? child : InstancePath.of(prefix + "/" + id);
	}

	static function resolvedLimits(limits:Null<AssemblyJointLimits>):AssemblyJointLimits
		return limits == null ? {lower: null, upper: null, velocity: null, effort: null} : limits;

	/** Members with the same recipe values, material and connectors share one definition. */
	public static function definitionKey(id:String, component:MachineComponent):String {
		var recipe = component.componentType();
		return recipe == null ? "code:" + id + "|" + component.materialSpec() :
			"typed:" + recipe.id + "|" + recipe.key(component.values()) + "|" + component.materialSpec();
	}

	public static function copyFrame(frame:AssemblyFrame):AssemblyFrame
		return {x: frame.x, y: frame.y, z: frame.z, qx: frame.qx, qy: frame.qy, qz: frame.qz, qw: frame.qw};
}
