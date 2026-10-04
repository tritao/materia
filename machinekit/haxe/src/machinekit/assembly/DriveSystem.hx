package machinekit.assembly;

import machinekit.assembly.MachineAssemblyDescription.BeltPathRecord;
import machinekit.assembly.MachineAssemblyDescription.EncoderRecord;
import machinekit.assembly.MachineAssemblyDescription.MotorRecord;
import machinekit.assembly.MachineAssemblyDescription.TransmissionRecord;
import machinekit.transmission.BeltPoseContext;
import machinekit.transmission.TransmissionDesignError;
import machinekit.transmission.TransmissionRelation;
import machinekit.transmission.TransmissionResolver;
import materia.assembly.AssemblyDefinition;
import materia.assembly.AssemblyDefinition.AssemblyActuator;
import materia.assembly.AssemblyDefinition.AssemblyEncoder;
import materia.assembly.AssemblyDefinition.AssemblyJointRole;
import materia.assembly.AssemblyDefinition.AssemblyJointType;
import materia.assembly.AssemblyDefinition.AssemblySensor;
import materia.assembly.AssemblyFrames;
import materia.assembly.AssemblyRecord.AssemblyFrame;
import machinekit.assembly.MachineAssemblyDescription.CylinderRecord;
import materia.assembly.AssemblyDefinition.AssemblyJointCoupling;
import materia.assembly.AssemblyDefinition.KinematicJoint;
import materia.assembly.AssemblyDefinition.QuantityAssumption;

typedef MotorCompilation = {
	var actuators:Array<AssemblyActuator>;
	var encoders:Array<AssemblyEncoder>;
	var sensors:Array<AssemblySensor>;
}

/**
 * One level's drives and their feedback: transmissions that set coupling ratios from their parts,
 * belt paths, motors, pneumatic cylinders, encoders and switch, at-speed and presence sensors.
 * Resolving them works on the flattened assembly, since a motor's supply or a belt's pulleys may sit
 * in another level.
 */
class DriveSystem {
	public final transmissions:Array<TransmissionRecord> = [];
	public final beltPaths:Array<BeltPathRecord> = [];
	public final motors:Array<MotorRecord> = [];
	public final encoders:Array<EncoderRecord> = [];
	public final cylinders:Array<CylinderRecord> = [];
	public final sensors:Array<AssemblySensor> = [];

	public function new() {}

	public function transmission(coupling:String):Null<TransmissionRecord> {
		for (entry in transmissions) if (entry.coupling == coupling) return entry;
		return null;
	}

	/**
	 * Write every transmission's coupling from its parts into `flat.definition`, and the elastic
	 * networks of shared belts. Findings go to `flat.diagnostics`; an unresolved transmission keeps
	 * its stated coupling.
	 */
	public static function derive(flat:FlatAssembly):Void {
		if (flat.derived) return;
		flat.derived = true;
		var mechanical = flat.definition;
		var context:Null<BeltPoseContext> = flat.beltPaths.length == 0 ? null : new BeltPoseContext(mechanical);
		for (transmission in flat.transmissions) apply(flat, transmission, context);
		mechanical.elasticNetworks = null;
		for (path in flat.beltPaths) {
			var reduction = false;
			for (record in flat.transmissions) switch record.source {
				case BeltReduction(belt, _, _) if (belt == path.belt): reduction = true;
				case _:
			}
			if (!reduction) continue;
			try {
				// Only the motors that turn one of this belt's pulleys load it.
				var beltContext:BeltPoseContext = cast context;
				var beltActuators:Array<AssemblyActuator> = [];
				for (record in flat.motors) {
					var drives = false;
					for (wrap in path.wraps)
						if (machinekit.transmission.BeltElasticity.rotaryJoint(beltContext.definition, wrap.instanceId) == record.joint) drives = true;
					if (drives) {
						var driver = motorDriver(flat, record);
						if (driverVoltage(flat, record.driver, driver, false) != null)
							beltActuators.push(resolveMotor(flat, record, false));
					}
				}
				var belt:machinekit.transmission.TimingBelt = cast flat.member(path.belt);
				var network = machinekit.transmission.BeltElasticity.build(belt, path, flat.transmissions, mechanical,
					flat.member, beltActuators, context);
				if (mechanical.elasticNetworks == null) mechanical.elasticNetworks = [];
				mechanical.elasticNetworks.push(network);
			} catch (error:TransmissionDesignError) {
				flat.diagnostics.error("transmission.parts", path.belt, error.message);
			} catch (error:Dynamic) {
				flat.diagnostics.error("transmission.parts", path.belt, Std.string(error));
			}
		}
	}

	/** Resolve `transmission` and write its coupling; false when the coupling does not exist. */
	public static function apply(flat:FlatAssembly, transmission:TransmissionRecord, ?context:BeltPoseContext):Bool {
		var coupling = findCoupling(flat.definition, transmission.coupling);
		if (coupling == null) return false;
		// Missing members are malformed data, even when another member has the wrong kind.
		TransmissionResolver.mapSource(transmission.source, id -> {
			flat.member(id);
			return id;
		});
		try {
			var relation = resolve(flat, transmission, null, true, null, context);
			write(flat.definition.couplings, flat.definition.joints, transmission, relation);
		} catch (error:TransmissionDesignError) {
			flat.diagnostics.error("transmission.parts", transmission.coupling,
				'Transmission "${transmission.coupling}" is unresolved: ${error.message}');
		}
		return true;
	}

	public static function resolve(flat:FlatAssembly, record:TransmissionRecord, ?leader:String, strict:Bool = true,
			?follower:String, ?context:BeltPoseContext):TransmissionRelation {
		var mechanical = flat.definition;
		var relation = TransmissionResolver.resolve(record, flat.member);
		switch record.source {
			case TimingBelt(beltId, pulley):
				if (leader == null) {
					var coupling = findCoupling(mechanical, record.coupling);
					if (coupling != null) leader = coupling.source;
				}
				var paths = [for (path in flat.beltPaths) if (path.belt == beltId) path];
				if (paths.length != 1 || leader == null) {
					if (strict) throw new TransmissionDesignError(
						'Belt "$beltId" needs its clamp and wrap attachments; add a belt path');
				} else {
					var belt:machinekit.transmission.TimingBelt = cast flat.member(beltId);
					var derived = machinekit.transmission.BeltStretch.stiffness(belt, paths[0], leader, pulley, mechanical, context);
					if (record.stiffness == null) relation.stiffness = derived;
				}
			case BeltReduction(beltId, driver, driven):
				var coupling = findCoupling(mechanical, record.coupling);
				if (coupling != null) {
					leader = coupling.source;
					follower = coupling.target;
				}
				var paths = [for (path in flat.beltPaths) if (path.belt == beltId) path];
				if (paths.length != 1 || leader == null || follower == null) {
					if (strict) throw new TransmissionDesignError('Belt "$beltId" needs its wrap attachments; add a belt path');
				} else {
					var belt:machinekit.transmission.TimingBelt = cast flat.member(beltId);
					for (pulleyId in [driver, driven]) {
						var part:machinekit.transmission.TimingPulley = cast flat.member(pulleyId);
						var index = machinekit.transmission.BeltStretch.wrapIndex(paths[0], pulleyId);
						if (Math.abs(belt.wraps()[index].radius - part.pitchDiameter / 2) > 1e-5)
							throw new TransmissionDesignError('Pulley "$pulleyId" tooth count does not match its belt wrap');
					}
					var derived = machinekit.transmission.BeltStretch.reduction(belt, paths[0], leader, follower, driver, driven,
						mechanical, relation, context);
					if (record.stiffness == null) relation.stiffness = derived;
				}
			case _:
		}
		return relation;
	}

	/** Write a resolved relation into the coupling and its follower joint's speed limit. */
	public static function write(couplings:Null<Array<AssemblyJointCoupling>>, joints:Array<KinematicJoint>,
			transmission:TransmissionRecord, relation:TransmissionRelation):Bool {
		var coupling:Null<AssemblyJointCoupling> = null;
		if (couplings != null) for (entry in couplings) if (entry.id == transmission.coupling) coupling = entry;
		if (coupling == null) return false;
		coupling.ratio = relation.ratio;
		coupling.offset = -relation.ratio * transmission.leaderZero;
		coupling.efficiency = relation.efficiency;
		coupling.stiffness = relation.stiffness;
		coupling.backlash = relation.backlash;
		coupling.drag = relation.drag;
		var fields = relation.assumptions();
		coupling.assumptions = [for (value in fields) if (value.quantity != "speed limit") value];
		var assumed = [for (value in fields) if (value.quantity != "speed limit") value.label];
		coupling.assumed = assumed.length == 0 ? null : assumed;
		for (joint in joints) if (joint.id == coupling.target) {
			var hadScrewCap = false;
			if (joint.limits.assumptions != null) for (value in joint.limits.assumptions)
				if (value.quantity == "speed limit" && value.label == "screw critical-speed margin") hadScrewCap = true;
			if (relation.followerSpeedCap != null) {
				joint.limits.velocity = relation.followerSpeedCap;
				joint.limits.assumptions = [for (value in fields) if (value.quantity == "speed limit") value];
			} else if (hadScrewCap) {
				joint.limits.velocity = null;
				var remaining:Array<QuantityAssumption> = [];
				if (joint.limits.assumptions != null) for (value in joint.limits.assumptions)
					if (!(value.quantity == "speed limit" && value.label == "screw critical-speed margin")) remaining.push(value);
				joint.limits.assumptions = remaining;
			}
		}
		return true;
	}

	static function findCoupling(mechanical:AssemblyDefinition, id:String):Null<AssemblyJointCoupling> {
		if (mechanical.couplings != null) for (coupling in mechanical.couplings) if (coupling.id == id) return coupling;
		return null;
	}

	public static function motorDriver(flat:FlatAssembly, record:MotorRecord):machinekit.motion.MotorDriver {
		var member = flat.member(record.motor);
		if (!Std.isOfType(member, machinekit.motion.MotorDrive)) throw 'Motor "${record.actuator}": "${record.motor}" is not a motor part';
		var driver = flat.member(record.driver);
		if (!Std.isOfType(driver, machinekit.motion.MotorDriver))
			throw 'Motor "${record.actuator}": "${record.driver}" is not a driver part';
		return cast driver;
	}

	/** A modelled supply wins over the explicit fallback; an exposed boundary has no voltage of its own. */
	public static function driverVoltage(flat:FlatAssembly, id:String, driver:machinekit.motion.MotorDriver,
			complete:Bool):Null<Float> {
		var trace = ServiceNetwork.trace(flat, ServiceNetwork.portRef(id, "power"));
		var voltage = driver.statedVoltage;
		if (trace.supplied && !trace.external) {
			var source = trace.port;
			if (complete) {
				var checked = new Diagnostics();
				ServiceNetwork.check(flat, checked, false);
				checked.throwIfErrors();
			}
			var part = flat.member(source.instanceId);
			if (!Std.isOfType(part, machinekit.motion.ElectricalSource))
				throw 'Power source "${source.instanceId}/${source.portName}" does not state an output voltage';
			var supply:machinekit.motion.ElectricalSource = cast part;
			voltage = supply.outputVoltage(source.portName);
		} else if (!trace.supplied && trace.chain.length > 1) throw ServiceNetwork.unsuppliedMessage(trace.chain);
		if (voltage != null) machinekit.motion.MotorDriver.validateVoltage(driver.rating, voltage);
		return voltage;
	}

	public static function resolveMotor(flat:FlatAssembly, record:MotorRecord, complete:Bool):AssemblyActuator {
		var driver = motorDriver(flat, record);
		var voltage = driverVoltage(flat, record.driver, driver, complete);
		if (voltage == null) throw 'Motor "${record.actuator}" needs a wired supply or a stated driver voltage';
		var motor:machinekit.motion.MotorDrive = cast flat.member(record.motor);
		var added = motor.actuator(record.actuator, record.joint, voltage, record.margin, driver.current);
		if (record.processVelocity != null) added.processVelocity = record.processVelocity;
		if (driver.rating.positionLoopRate > 0) added.positionLoopRate = driver.rating.positionLoopRate;
		var stepper = driver.rating.family == machinekit.motion.MotorDriver.MotorDriverFamily.Stepper;
		if ((stepper && added.drive != "stepper") || (!stepper && added.drive != "servo"))
			throw 'Motor "${record.actuator}" and driver "${record.driver}" have different drive families';
		if (stepper) {
			added.microsteps = driver.microsteps;
			added.maxStepRate = driver.rating.maximumStepRate;
		}
		var quantities = added.assumptions == null ? [] : [for (value in added.assumptions) value];
		added.assumptions = quantities;
		quantities.push({quantity: "speed limit", label: "driver ratings"});
		var assumed = added.assumed == null ? [] : [for (label in added.assumed) label];
		assumed.push("driver ratings");
		added.assumed = assumed;
		var gearbox = record.gearbox;
		if (gearbox != null) {
			var part = flat.member(gearbox);
			if (!Std.isOfType(part, machinekit.motion.Gearbox)) throw 'Motor "${record.actuator}": "$gearbox" is not a gearbox part';
			var gear:machinekit.motion.Gearbox = cast part;
			var housing = flat.member(record.motor);
			if (Std.isOfType(housing, machinekit.robotics.GearedArmJoint)) {
				var pocket:machinekit.robotics.GearedArmJoint = cast housing;
				if (gear.diameter >= pocket.pocketDiameter || gear.length + 0.5 >= pocket.pocketLength)
					throw 'Gearbox "$gearbox" does not fit motor housing "${record.motor}"';
			}
			added.rotorInertia = (added.rotorInertia == null ? 0.0 : added.rotorInertia) + gear.inputInertia;
			if (gear.inertiaAssumed) {
				assumed.push("gearbox input inertia");
				quantities.push({quantity: "inertia", label: "gearbox input inertia"});
			}
			added.gearRatio = gear.ratio;
			added.gearEfficiency = gear.efficiency;
			if (gear.assumed) {
				assumed.push("gearbox ratio");
				assumed.push("gearbox efficiency");
				quantities.push({quantity: "efficiency", label: "gearbox efficiency"});
			}
		}
		return added;
	}

	/**
	 * Check a motor binding against the parts as far as they are wired now. An incomplete module may
	 * be wired by the assembly that includes it; its complete power graph is resolved on export.
	 */
	public static function checkMotor(flat:FlatAssembly, record:MotorRecord):Void {
		if (record.processVelocity != null) {
			var process = record.processVelocity;
			if (process.speedChannel == null || process.speedChannel.length == 0 ||
				process.directionChannel == null || process.directionChannel.length == 0 ||
				process.speedChannel == process.directionChannel || !(process.radiansPerSpeedUnit > 0) ||
				!Math.isFinite(process.radiansPerSpeedUnit))
				throw 'Motor "${record.actuator}" has an invalid process velocity binding';
			var continuous = false;
			for (joint in flat.definition.joints) if (joint.id == record.joint && joint.type == AssemblyJointType.Continuous)
				continuous = true;
			if (!continuous) throw 'Process velocity motor "${record.actuator}" needs a continuous joint';
		}
		for (motor in flat.motors) if (motor.actuator == record.actuator)
			throw 'Duplicate assembly actuator "${record.actuator}"';
		for (cylinder in flat.cylinders) if (cylinder.actuator == record.actuator || cylinder.joint == record.joint)
			throw 'Joint "${record.joint}" already has a pneumatic drive';
		var driver = motorDriver(flat, record);
		if (driverVoltage(flat, record.driver, driver, false) != null) resolveMotor(flat, record, false);
	}

	public static function checkEncoder(flat:FlatAssembly, record:EncoderRecord):Void {
		var member = flat.member(record.part);
		if (!Std.isOfType(member, machinekit.motion.EncoderPart)) throw 'Encoder "${record.encoder}": "${record.part}" is not an encoder part';
		for (existing in flat.encoders) if (existing.encoder == record.encoder)
			throw 'Duplicate assembly encoder "${record.encoder}"';
		if (record.actuator != null) {
			var found = false;
			for (motor in flat.motors) if (motor.actuator == record.actuator) found = true;
			if (!found) throw 'Encoder "${record.encoder}" reads unknown motor "${record.actuator}"';
		}
		var part:machinekit.motion.EncoderPart = cast member;
		part.encoder(record.encoder, record.joint);
	}

	/** Compile every binding anew; connecting a supply after binding must not leave an old curve. */
	public static function compileMotors(flat:FlatAssembly):MotorCompilation {
		var actuators = [for (record in flat.motors) resolveMotor(flat, record, true)];
		for (cylinder in flat.cylinders) actuators.push(compileCylinder(flat, cylinder));
		var sensors:Array<AssemblyEncoder> = [];
		var sensorIds:Map<String, Bool> = [];
		for (record in flat.encoders) {
			var part:machinekit.motion.EncoderPart = cast flat.member(record.part);
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
		return {actuators: actuators, encoders: sensors, sensors: compileSensors(flat)};
	}

	/** Check a cylinder binding: typed cylinder and valve parts on a prismatic tree joint with no other drive. */
	public static function checkCylinder(flat:FlatAssembly, record:CylinderRecord):Void {
		if (flat.member(record.cylinder).pneumaticCylinderSpec() == null || flat.member(record.valve).pneumaticValveSpec() == null)
			throw "Cylinder binding needs cylinder and valve parts";
		for (cylinder in flat.cylinders) if (cylinder.actuator == record.actuator || cylinder.joint == record.joint)
			throw "Duplicate cylinder actuator or joint";
		for (motor in flat.motors) if (motor.actuator == record.actuator || motor.joint == record.joint)
			throw "Joint already has a motor";
		for (encoder in flat.encoders) if (encoder.encoder == record.actuator) throw 'Duplicate assembly actuator "${record.actuator}"';
		var found = false;
		for (joint in flat.definition.joints)
			if (joint.id == record.joint && joint.type == AssemblyJointType.Prismatic && joint.role == AssemblyJointRole.Tree) found = true;
		if (!found) throw 'Cylinder "${record.actuator}" needs a prismatic tree joint';
	}

	/** A cylinder's actuator: force from the supply pressure, direction from its hoses and geometry. */
	static function compileCylinder(flat:FlatAssembly, record:CylinderRecord):AssemblyActuator {
		var cylinder = flat.member(record.cylinder).pneumaticCylinderSpec();
		var valve = flat.member(record.valve).pneumaticValveSpec();
		if (cylinder == null || valve == null) throw "Cylinder binding lost its typed cylinder or valve part";
		var pressure = airPressure(flat, record.valve, "P");
		var a = directUpstream(flat, record.cylinder, "A"), b = directUpstream(flat, record.cylinder, "B");
		if (a.instanceId != record.valve || b.instanceId != record.valve || a.portName == b.portName ||
			(a.portName != "A" && a.portName != "B") || (b.portName != "A" && b.portName != "B"))
			throw "Cylinder hoses must connect to the bound valve's A/B outlets";
		var state = new cadkit.modeling.AssemblyState(materia.assembly.AssemblyDefinitionCodec.decode(
			materia.assembly.AssemblyDefinitionCodec.encode(flat.definition)));
		var extension = state.worldConnector(record.cylinder, "extension");
		var direction = AssemblyFrames.transformVector(extension, 0, 1, 0);
		var slider:Null<AssemblyFrame> = null;
		var axis = {x: 0.0, y: 0.0, z: 0.0};
		for (joint in flat.definition.joints) if (joint.id == record.joint) {
			slider = state.worldConnector(joint.child, joint.childConnector);
			axis = AssemblyFrames.transformVector(state.worldConnector(joint.parent, joint.parentConnector), joint.axis.x, joint.axis.y, joint.axis.z);
			if (joint.limits.lower == null || joint.limits.upper == null ||
				Math.abs(joint.limits.upper - joint.limits.lower - cylinder.strokeMm()) > 1e-6)
				throw "Cylinder stroke must match the guide's geometric travel";
		}
		var dot = direction.x * axis.x + direction.y * axis.y + direction.z * axis.z;
		if (Math.abs(dot) < 0.999999) throw "Cylinder extension must align with its guide axis";
		if (slider == null || Math.sqrt(Math.pow(extension.x - slider.x, 2) + Math.pow(extension.y - slider.y, 2) +
			Math.pow(extension.z - slider.z, 2)) > 1e-5)
			throw "Cylinder extension tip must meet the guided member at its default pose";
		var hoseSign = a.portName == "A" ? 1.0 : -1.0;
		return {id: record.actuator, joint: record.joint, maxEffort: cylinder.extendForce(pressure), maxRate: cylinder.ratedSpeedMmPerSecond(),
			drive: "pneumatic", assumed: ["cylinder envelope", "flow-limited cylinder speed", "installed pneumatic fittings"],
			pneumatic: {bore: cylinder.boreMm(), rod: cylinder.rodMm(), stroke: cylinder.strokeMm(), ratedSpeed: cylinder.ratedSpeedMmPerSecond(),
				pressurePa: pressure, channelA: record.valve + "/coilA", channelB: valve.isDoubleSolenoid() ? record.valve + "/coilB" : null,
				normallyToA: hoseSign > 0 ? valve.defaultsToA() : !valve.defaultsToA(), extendSign: dot > 0 ? 1.0 : -1.0}};
	}

	static function directUpstream(flat:FlatAssembly, instanceId:String, portName:String):machinekit.assembly.MachineAssembly.PortRef {
		for (connection in flat.connections) if (connection.toInstance == instanceId && connection.toPort == portName)
			return {instanceId: connection.fromInstance, portName: connection.fromPort};
		throw 'Unwired cylinder port "$instanceId/$portName"';
	}

	/** Gauge pressure at a pneumatic consumer, traced through its current air wiring to a modelled supply. */
	public static function airPressure(flat:FlatAssembly, instanceId:String, portName:String):Float {
		var consumer = ServiceNetwork.portRef(instanceId, portName);
		var input = ServiceNetwork.requirePort(flat, consumer);
		if (input.kind != Pneumatic || input.role != Consumer)
			throw '"$instanceId/$portName" is not a pneumatic consumer';
		var checked = new Diagnostics();
		ServiceNetwork.check(flat, checked, false);
		checked.throwIfErrors();
		var source = ServiceNetwork.trace(flat, consumer);
		if (!source.supplied) throw ServiceNetwork.unsuppliedMessage(source.chain);
		if (source.external) throw 'Air pressure at "$instanceId/$portName" needs a modelled supply';
		var part = flat.member(source.port.instanceId);
		if (!Std.isOfType(part, machinekit.pneumatic.PressureSource))
			throw 'Air source "${source.port.instanceId}/${source.port.portName}" does not state pressure';
		var supply:machinekit.pneumatic.PressureSource = cast part;
		var pressure = supply.outputPressure(source.port.portName);
		if (!(pressure >= 0) || !Math.isFinite(pressure)) throw "Air pressure must be finite and nonnegative";
		return pressure;
	}

	public static function checkSensor(flat:FlatAssembly, record:AssemblySensor):Void {
		for (sensor in flat.sensors) if (sensor.id == record.id) throw 'Duplicate assembly sensor "${record.id}"';
		for (encoder in flat.encoders) if (encoder.encoder == record.id) throw 'Duplicate assembly sensor "${record.id}"';
	}

	static function compileSensors(flat:FlatAssembly):Array<AssemblySensor> {
		var result:Array<AssemblySensor> = [];
		for (record in flat.sensors) {
			if (record.kind == "joint_switch" || record.kind == "at_speed") {
				var edge:Null<materia.assembly.AssemblyDefinition.KinematicJoint> = null;
				for (joint in flat.definition.joints) if (joint.id == record.joint) edge = joint;
				if (edge == null || edge.role != AssemblyJointRole.Tree ||
					(edge.type != AssemblyJointType.Prismatic && edge.type != AssemblyJointType.Revolute &&
						edge.type != AssemblyJointType.Continuous))
					throw 'Native sensor "${record.id}" needs a moving tree joint';
				if (record.kind == "joint_switch") {
					var lower = record.windowLower, upper = record.windowUpper, hysteresis = record.hysteresis;
					if (lower == null || upper == null || !Math.isFinite(lower) || !Math.isFinite(upper) ||
						lower > upper || hysteresis == null || !Math.isFinite(hysteresis) || hysteresis < 0)
						throw 'Joint switch "${record.id}" has an invalid window or hysteresis';
					if ((edge.limits.lower != null && lower < edge.limits.lower) ||
						(edge.limits.upper != null && upper > edge.limits.upper))
						throw 'Joint switch "${record.id}" lies outside joint travel';
					result.push({id: record.id, kind: record.kind, joint: record.joint,
						windowLower: lower, windowUpper: upper, hysteresis: hysteresis});
				} else {
					var threshold = record.windowLower;
					if (threshold == null || !(threshold > 0) || !Math.isFinite(threshold))
						throw 'At-speed sensor "${record.id}" needs a positive threshold';
					result.push({id: record.id, kind: record.kind, joint: record.joint,
						windowLower: threshold, hysteresis: 0.0});
				}
			} else {
				var range = record.range, occurrence = record.occurrence, connector = record.connector;
				if (occurrence == null || connector == null || range == null || !(range > 0) || !Math.isFinite(range))
					throw 'Presence sensor "${record.id}" needs a connector and positive range';
				var found = false;
				for (entry in flat.requireDefinition(occurrence).connectors) if (entry.name == connector) found = true;
				if (!found) throw 'Unknown connector "$occurrence/$connector"';
				result.push({id: record.id, kind: record.kind, occurrence: occurrence, connector: connector, range: range});
			}
		}
		return result;
	}

	public static function copyCylinder(record:CylinderRecord, map:String->String):CylinderRecord
		return {actuator: map(record.actuator), joint: map(record.joint), cylinder: map(record.cylinder), valve: map(record.valve)};

	public static function copySensor(sensor:AssemblySensor, map:String->String):AssemblySensor {
		var copy:AssemblySensor = {id: map(sensor.id), kind: sensor.kind};
		if (sensor.joint != null) copy.joint = map(sensor.joint);
		if (sensor.occurrence != null) copy.occurrence = map(sensor.occurrence);
		if (sensor.connector != null) copy.connector = sensor.connector;
		if (sensor.windowLower != null) copy.windowLower = sensor.windowLower;
		if (sensor.windowUpper != null) copy.windowUpper = sensor.windowUpper;
		if (sensor.hysteresis != null) copy.hysteresis = sensor.hysteresis;
		if (sensor.range != null) copy.range = sensor.range;
		return copy;
	}

	public static function copyTransmission(transmission:TransmissionRecord, map:String->String):TransmissionRecord {
		var copy:TransmissionRecord = {coupling: map(transmission.coupling),
			source: TransmissionResolver.mapSource(transmission.source, map), sense: transmission.sense,
			leaderZero: transmission.leaderZero};
		copy.stiffness = transmission.stiffness;
		copy.backlash = transmission.backlash;
		copy.drag = transmission.drag;
		copy.near = transmission.near;
		copy.far = transmission.far;
		copy.unsupported = transmission.unsupported;
		return copy;
	}

	public static function copyMotor(motor:MotorRecord, map:String->String):MotorRecord {
		var copy:MotorRecord = {actuator: map(motor.actuator), joint: map(motor.joint), motor: map(motor.motor),
			driver: map(motor.driver), margin: motor.margin, gearbox: motor.gearbox == null ? null : map(motor.gearbox)};
		var process = motor.processVelocity;
		if (process != null) copy.processVelocity = {speedChannel: process.speedChannel,
			directionChannel: process.directionChannel, radiansPerSpeedUnit: process.radiansPerSpeedUnit};
		return copy;
	}

	public static function copyEncoder(encoder:EncoderRecord, map:String->String):EncoderRecord
		return {encoder: map(encoder.encoder), joint: map(encoder.joint), part: map(encoder.part),
			actuator: encoder.actuator == null ? null : map(encoder.actuator)};

	public static function copyBeltPath(path:BeltPathRecord, map:String->String):BeltPathRecord {
		var clamp = path.clamp;
		return {belt: map(path.belt),
			clamp: clamp == null ? null : {instanceId: map(clamp.instanceId), connectorName: clamp.connectorName},
			wraps: [for (wrap in path.wraps) {instanceId: map(wrap.instanceId), connectorName: wrap.connectorName}]};
	}
}
