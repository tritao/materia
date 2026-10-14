package machinekit.assembly;

import machinekit.assembly.MachineAssemblyDescription.BeltPathRecord;
import machinekit.assembly.MachineAssemblyDescription.EncoderRecord;
import machinekit.assembly.MachineAssemblyDescription.MotorRecord;
import machinekit.assembly.MachineAssemblyDescription.TransmissionRecord;
import machinekit.transmission.BeltPoseContext;
import machinekit.transmission.TransmissionDesignError;
import machinekit.transmission.TransmissionRelation;
import machinekit.transmission.TransmissionResolver;
import machinekit.transmission.ValueBasis;
import materia.assembly.AssemblyDefinition;
import materia.assembly.AssemblyDefinition.AssemblyActuator;
import materia.assembly.AssemblyDefinition.AssemblyEncoder;
import materia.assembly.AssemblyDefinition.AssemblyJointRole;
import materia.assembly.AssemblyDefinition.AssemblyJointType;
import materia.assembly.AssemblyDefinition.AssemblySensor;
import materia.assembly.AssemblyFrames;
import cadkit.modeling.AssemblyState;
import materia.assembly.AssemblyRecord.AssemblyFrame;
import machinekit.assembly.MachineAssemblyDescription.CylinderRecord;
import machinekit.fluid.CylinderControlSpec;
import machinekit.fluid.CylinderPhaseSpeeds;
import machinekit.fluid.ValveControlSpec;
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
	public final switches:Array<materia.assembly.AssemblyDefinition.AssemblySwitch> = [];

	public function new() {}

	static function followsSwitchJoint(flat:FlatAssembly, member:String, child:String):Bool {
		for (_ in 0...flat.definition.joints.length + 1) {
			if (member == child) return true;
			var parent:Null<String> = null;
			for (edge in flat.definition.joints) if (edge.role == AssemblyJointRole.Tree && edge.child == member)
				parent = edge.parent;
			if (parent == null) return false;
			member = parent;
		}
		throw "Switch geometry contains a cyclic member chain";
	}

	public static function resolveSwitch(flat:FlatAssembly, contact:materia.assembly.AssemblyDefinition.AssemblySwitch, state:AssemblyState):Void {
		var edge:Null<materia.assembly.AssemblyDefinition.KinematicJoint> = null;
		for (candidate in flat.definition.joints) if (candidate.id == contact.joint) edge = candidate;
		if (edge == null || edge.type != materia.assembly.AssemblyDefinition.AssemblyJointType.Prismatic ||
			!followsSwitchJoint(flat, contact.trigger, edge.child) || followsSwitchJoint(flat, contact.part, edge.child))
			throw 'Switch "${contact.id}" needs a fixed switch and a trigger carried by its prismatic joint';
		if (contact.id == null || contact.id.length == 0 || (contact.role != "home" && contact.role != "limit") ||
			(contact.side != -1 && contact.side != 1)) throw "Switch needs a valid ID, role and side";
		if (contact.driveJoint != null) {
			var driveFound = false;
			for (drive in flat.definition.joints) if (drive.id == contact.driveJoint &&
				drive.type != materia.assembly.AssemblyDefinition.AssemblyJointType.Fixed) driveFound = true;
			if (!driveFound) throw 'Switch "${contact.id}" has no movable drive joint "${contact.driveJoint}"';
		}
		var member = flat.member(contact.part);
		if (!Std.isOfType(member, machinekit.motion.SwitchPart)) throw 'Part "${contact.part}" is not a switch';
		var source:machinekit.motion.SwitchPart = cast member;
		contact.connector = source.tripConnector();
		contact.hysteresis = source.switchHysteresis();
		contact.repeatability = source.switchRepeatability();
		if (!Math.isFinite(contact.hysteresis) || contact.hysteresis < 0 ||
			!Math.isFinite(contact.repeatability) || contact.repeatability < 0)
			throw "Switch hysteresis and repeatability must be finite and non-negative";
		var base = state.worldConnector(edge.parent, edge.parentConnector);
		var axis = AssemblyFrames.transformVector(base, edge.axis.x, edge.axis.y, edge.axis.z);
		var norm = axis.x * axis.x + axis.y * axis.y + axis.z * axis.z;
		if (!(norm > 0) || !Math.isFinite(norm)) throw "Switch joint has no finite direction";
		var switchPoint = state.worldConnector(contact.part, contact.connector);
		var triggerPoint = state.worldConnector(contact.trigger, contact.triggerConnector);
		contact.trip = state.joint(contact.joint) + ((switchPoint.x - triggerPoint.x) * axis.x +
			(switchPoint.y - triggerPoint.y) * axis.y + (switchPoint.z - triggerPoint.z) * axis.z) / Math.sqrt(norm);
		var movement = contact.trip - state.joint(contact.joint), length = Math.sqrt(norm);
		var dx = switchPoint.x - triggerPoint.x - axis.x * movement / length;
		var dy = switchPoint.y - triggerPoint.y - axis.y * movement / length;
		var dz = switchPoint.z - triggerPoint.z - axis.z * movement / length;
		if (dx * dx + dy * dy + dz * dz > 1e-10)
			throw 'Switch "${contact.id}" trigger path misses its trip connector';
		// The stated uncertainty band must leave room on both sides before an end stop.
		var limits = edge.limits, room = limits.overtravel == null ? 0.0 : limits.overtravel;
		var bound = contact.side < 0 ? limits.lower : limits.upper;
		if (bound == null || !(room > 0) || !Math.isFinite(contact.trip) ||
			contact.side * (contact.trip - bound) <= contact.repeatability ||
			contact.side * (contact.trip - bound) + contact.repeatability >= room)
			throw 'Switch "${contact.id}" trip must lie strictly between the soft limit and its end stop';
	}

	public static function copySwitch(contact:materia.assembly.AssemblyDefinition.AssemblySwitch, map:String->String):materia.assembly.AssemblyDefinition.AssemblySwitch
		return materia.assembly.AssemblyDefinitionFlattener.copySwitch(contact, map);


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
		if (flat.switches.length > 0) {
			mechanical.switches = [];
			var state = new AssemblyState(mechanical);
			for (record in flat.switches) {
				var contact = copySwitch(record, id -> id);
				try {
					resolveSwitch(flat, contact, state);
					mechanical.switches.push(contact);
				} catch (error:Dynamic) flat.diagnostics.error("assembly.switch-geometry", contact.id, Std.string(error));
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
					beltDamping(relation, belt, record);
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
					beltDamping(relation, belt, record);
				}
			case _:
		}
		return relation;
	}

	/** A belt's damping is the belt part's, unless the transmission states its own. */
	static function beltDamping(relation:TransmissionRelation, belt:machinekit.transmission.TimingBelt, record:TransmissionRecord):Void {
		if (record.dampingRatio != null) {
			relation.dampingRatio = record.dampingRatio;
			relation.setBasis("dampingRatio", ValueBasis.Stated);
		} else if (belt.statedDampingRatio > 0) {
			relation.dampingRatio = belt.statedDampingRatio;
			relation.setBasis("dampingRatio", ValueBasis.Stated);
		}
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
		coupling.dampingRatio = relation.dampingRatio;
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
		if (member.motorDrive() == null) throw 'Motor "${record.actuator}": "${record.motor}" is not a motor part';
		var driver = flat.member(record.driver).motorDriver();
		if (driver == null)
			throw 'Motor "${record.actuator}": "${record.driver}" is not a driver part';
		return driver;
	}

	/** A modelled supply wins over the explicit fallback; an exposed boundary has no voltage of its own. */
	public static function driverVoltage(flat:FlatAssembly, id:String, driver:machinekit.motion.MotorDriver,
			complete:Bool):Null<machinekit.power.DcVoltage> {
		var trace = ServiceNetwork.trace(flat, ServiceNetwork.portRef(id, "power"));
		var voltage = driver.statedVoltage;
		if (trace.supplied) {
			// Energy traces to the pack, but a converter's output sets the driver's voltage.
			var delivered = ServiceNetwork.trace(flat, ServiceNetwork.portRef(id, "power"),
				(part, port) -> part.electricalSource() != null);
			var source = delivered.port;
			if (complete) {
				var checked = new Diagnostics();
				ServiceNetwork.check(flat, checked, false);
				checked.throwIfErrors();
			}
			var part = flat.member(source.instanceId);
			if (!delivered.external && part.electricalSource() == null)
				throw 'Power source "${source.instanceId}/${source.portName}" does not state an output voltage';
			if (!delivered.external) {
				var supply = part.electricalSource();
				voltage = supply.outputRating(source.portName).voltage;
			}
		} else if (!trace.supplied && trace.chain.length > 1) throw ServiceNetwork.unsuppliedMessage(trace.chain);
		if (voltage != null) machinekit.motion.MotorDriver.validateVoltage(driver.rating, voltage);
		return voltage;
	}

	public static function resolveMotor(flat:FlatAssembly, record:MotorRecord, complete:Bool):AssemblyActuator {
		var driver = motorDriver(flat, record);
		var voltage = driverVoltage(flat, record.driver, driver, complete);
		if (voltage == null) throw 'Motor "${record.actuator}" needs a wired supply or a stated driver voltage';
		var motor = flat.member(record.motor).motorDrive();
		if (motor == null) throw 'Motor "${record.motor}" has no typed drive';
		var added = motor.actuator(record.actuator, record.joint, voltage.volts(), record.margin, driver.current.amps());
		if (record.processVelocity != null) added.processVelocity = record.processVelocity;
		if (record.brake != null) {
			var binding = record.brake;
			var brake = flat.member(binding.part).motorBrakeSpec();
			if (brake == null || added.drive != "servo" || record.gearbox != null)
				throw 'Motor "${record.actuator}" needs a direct servo brake part on its motor joint';
			added.brake = {id: binding.part, releaseChannel: binding.releaseChannel,
				releasedSensor: binding.releasedSensor, holdingEffort: brake.holdingTorque(),
				releaseDelay: brake.releaseDelaySeconds(), applicationDelay: brake.applicationDelaySeconds(),
				releaseTimeoutMs: binding.releaseTimeoutMs, feedbackMaxAgeMs: binding.feedbackMaxAgeMs};
		}
		if (driver.rating.positionLoopRate > 0) added.positionLoopRate = driver.rating.positionLoopRate;
		var stepper = driver.rating.family == machinekit.motion.MotorDriver.MotorDriverFamily.Stepper;
		if ((stepper && added.drive != "stepper") || (!stepper && added.drive != "servo"))
			throw 'Motor "${record.actuator}" and driver "${record.driver}" have different drive families';
		if (!stepper) {
			var winding = motor.electricalRating();
			if (winding == null) throw 'Servo "${record.motor}" needs winding ratings';
			var load = new machinekit.motion.ServoElectricalLoad(added, winding, driver);
			load.check(driver, voltage, record.actuator);
		}
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

	/** Check the whole wired source, including simultaneous peak loads and regeneration. */
	public static function checkElectricalSources(flat:FlatAssembly):Void {
		var demands:Map<String, {rating:machinekit.motion.ElectricalSupplyRating, peak:Float, regeneration:Float}> = [];
		for (record in flat.motors) {
			var motor = flat.member(record.motor).motorDrive();
			if (motor == null) throw 'Motor "${record.motor}" has no typed drive';
			var winding = motor.electricalRating();
			if (winding == null) continue; // Steppers retain their explicit current/pull-out model.
			var driver = motorDriver(flat, record);
			var trace = ServiceNetwork.trace(flat, ServiceNetwork.portRef(record.driver, "power"));
			if (!trace.supplied)
				throw 'Servo "${record.actuator}" needs a wired source with current and regenerative ratings';
			var port = ServiceNetwork.trace(flat, ServiceNetwork.portRef(record.driver, "power"),
				(part, port) -> part.electricalSource() != null).port;
			var source = flat.member(port.instanceId).electricalSource();
			if (source == null) throw 'Source "${port.instanceId}" has no electrical ratings';
			var rating = source.outputRating(port.portName);
			var actuator = motor.actuator(record.actuator, record.joint, rating.voltage.volts(), record.margin, driver.current.amps());
			var load = new machinekit.motion.ServoElectricalLoad(actuator, winding, driver);
			load.check(driver, rating.voltage, record.actuator);
			var demand = demands.get(port.instanceId);
			if (demand == null) {demand = {rating: rating, peak: 0, regeneration: 0}; demands.set(port.instanceId, demand);}
			demand.peak += load.peakPower;
			demand.regeneration += load.regenerativePower;
		}
		for (id in demands.keys()) {
			var demand = demands.get(id), rating = demand.rating;
			if (demand.peak > rating.voltage.volts() * rating.current.amps() + 1e-9)
				throw 'Source "$id" supplies ${rating.voltage.volts() * rating.current.amps()} W DC; simultaneous servo demand is ${demand.peak} W';
			if (demand.regeneration > rating.regenerativePower + 1e-9)
				throw 'Source "$id" absorbs ${rating.regenerativePower} W; servo regeneration needs ${demand.regeneration} W';
		}
	}

	/** Compile every binding anew; connecting a supply after binding must not leave an old curve. */
	public static function compileMotors(flat:FlatAssembly):MotorCompilation {
		checkElectricalSources(flat);
		var actuators = [for (record in flat.motors) resolveMotor(flat, record, true)];
		for (cylinder in flat.cylinders) actuators.push(compileCylinder(flat, cylinder));
		var sensors:Array<AssemblyEncoder> = [];
		var nativeSensors = compileSensors(flat);
		for (record in flat.motors) if (record.brake != null) {
			var binding = record.brake;
			for (sensor in nativeSensors) if (sensor.id == binding.releasedSensor)
				throw 'Duplicate brake contact "${binding.releasedSensor}"';
			nativeSensors.push({id: binding.releasedSensor, kind: "brake_released", joint: record.joint});
		}
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
		return {actuators: actuators, encoders: sensors, sensors: nativeSensors};
	}

	/** Check a cylinder binding: typed cylinder and valve parts on a prismatic tree joint with no other drive. */
	public static function checkCylinder(flat:FlatAssembly, record:CylinderRecord,
			phases:Null<CylinderPhaseSpeeds>):Void {
		var cylinder = flat.member(record.cylinder).cylinderSpec();
		var valve = flat.member(record.valve).valveSpec();
		if (cylinder == null || valve == null)
			throw "Cylinder binding needs cylinder and valve parts";
		switch cylinder.controlSpec() {
			case CylinderControlSpec.PressureEffort(_):
				if (record.phases != null) throw "Pneumatic binding does not accept hydraulic phase speeds";
				switch valve.controlSpec() {
					case ValveControlSpec.ClosedCentre: throw "Pneumatic cylinder needs a pneumatic directional valve";
					case ValveControlSpec.PressureEffort(_, _):
				}
			case CylinderControlSpec.FlowLimited:
				if (phases == null || record.phases == null ||
					Math.abs(phases.stroke - cylinder.strokeMm()) > 1e-6 ||
					cylinderPhases(record, cylinder.strokeMm()) == null)
					throw "Hydraulic binding needs explicit phase speeds for its cylinder stroke";
				switch valve.controlSpec() {
					case ValveControlSpec.PressureEffort(_, _): throw "Hydraulic cylinder needs a closed-centre hydraulic valve";
					case ValveControlSpec.ClosedCentre:
				}
		}
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

	/** A fluid cylinder drive: pressure, direction and control law come from its installed parts. */
	static function compileCylinder(flat:FlatAssembly, record:CylinderRecord):AssemblyActuator {
		var cylinder = flat.member(record.cylinder).cylinderSpec();
		var valve = flat.member(record.valve).valveSpec();
		if (cylinder == null || valve == null) throw "Cylinder binding lost its typed cylinder or valve part";
		var pressure = fluidPressure(flat, record.valve, "P");
		var valvePart = flat.member(record.valve);
		function toValvePort(portName:String):machinekit.assembly.MachineAssembly.PortRef {
			var trace = ServiceNetwork.trace(flat, ServiceNetwork.portRef(record.cylinder, portName),
				(component, port) -> component == valvePart && (port.name == "A" || port.name == "B"));
			if (!trace.supplied || trace.external || trace.port.instanceId != record.valve ||
				(trace.port.portName != "A" && trace.port.portName != "B"))
				throw "Cylinder hoses must connect to the bound valve's A/B outlets";
			return trace.port;
		}
		var a = toValvePort("A"), b = toValvePort("B");
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
		var reversed = a.portName == "B";
		var extendSign = dot > 0 ? 1.0 : -1.0;
		return switch cylinder.controlSpec() {
			case CylinderControlSpec.PressureEffort(speed):
				switch valve.controlSpec() {
					case ValveControlSpec.ClosedCentre: throw "Pneumatic cylinder needs a pneumatic directional valve";
					case ValveControlSpec.PressureEffort(doubleSolenoid, normallyToA):
						{id: record.actuator, joint: record.joint, maxEffort: cylinder.extendForce(pressure), maxRate: speed,
							drive: "pneumatic", assumed: ["cylinder envelope", "flow-limited cylinder speed", "installed pneumatic fittings"],
							pneumatic: {bore: cylinder.boreMm(), rod: cylinder.rodMm(), stroke: cylinder.strokeMm(), ratedSpeed: speed,
								pressurePa: pressure, channelA: record.valve + (doubleSolenoid && reversed ? "/coilB" : "/coilA"),
								channelB: doubleSolenoid ? record.valve + (reversed ? "/coilA" : "/coilB") : null,
								normallyToA: reversed ? !normallyToA : normallyToA, extendSign: extendSign}};
				}
			case CylinderControlSpec.FlowLimited:
				switch valve.controlSpec() {
					case ValveControlSpec.PressureEffort(_, _): throw "Hydraulic cylinder needs a closed-centre hydraulic valve";
					case ValveControlSpec.ClosedCentre:
						for (joint in flat.definition.joints) if (joint.id == record.joint) {
							var home = extendSign > 0 ? joint.limits.lower : joint.limits.upper;
							if (home == null || Math.abs(joint.defaultValue - home) > 1e-6)
								throw "Hydraulic cylinder tip must meet its guide at the retracted limit";
						}
						var phases = cylinderPhases(record, cylinder.strokeMm());
						if (phases == null) throw "Hydraulic cylinder binding lost its explicit phase profile";
						{id: record.actuator, joint: record.joint, maxEffort: cylinder.extendForce(pressure), maxRate: phases.maximumSpeed(),
							drive: "hydraulic", assumed: ["cylinder envelope", "hydraulic phase speeds", "installed hydraulic fittings"],
							hydraulic: {bore: cylinder.boreMm(), rod: cylinder.rodMm(), stroke: cylinder.strokeMm(), pressurePa: pressure,
								approachSpeed: phases.approach, workingSpeed: phases.working, returnSpeed: phases.returning,
								changePoint: phases.changePoint, channelA: record.valve + (reversed ? "/coilB" : "/coilA"),
								channelB: record.valve + (reversed ? "/coilA" : "/coilB"), extendSign: extendSign}};
				}
		};
	}

	/** Gauge pressure at a pneumatic consumer. */
	public static function airPressure(flat:FlatAssembly, instanceId:String, portName:String):Float
		return fluidPressure(flat, instanceId, portName);

	/** Gauge pressure at a fluid consumer, traced through its current service wiring to a modelled supply. */
	public static function fluidPressure(flat:FlatAssembly, instanceId:String, portName:String):Float {
		var consumer = ServiceNetwork.portRef(instanceId, portName);
		var input = ServiceNetwork.requirePort(flat, consumer);
		if ((input.kind != Pneumatic && input.kind != Hydraulic) || input.role != Consumer)
			throw '"$instanceId/$portName" is not a fluid consumer';
		var checked = new Diagnostics();
		ServiceNetwork.check(flat, checked, false);
		checked.throwIfErrors();
		var source = ServiceNetwork.trace(flat, consumer);
		if (!source.supplied) throw ServiceNetwork.unsuppliedMessage(source.chain);
		if (source.external) throw 'Fluid pressure at "$instanceId/$portName" needs a modelled supply';
		var part = flat.member(source.port.instanceId);
		var supply = part.pressureSource();
		if (supply == null)
			throw 'Fluid source "${source.port.instanceId}/${source.port.portName}" does not state pressure';
		var pressure = supply.outputPressure(source.port.portName);
		if (!(pressure >= 0) || !Math.isFinite(pressure)) throw "Fluid pressure must be finite and nonnegative";
		return pressure;
	}

	static function cylinderPhases(record:CylinderRecord, stroke:Float):Null<CylinderPhaseSpeeds> {
		var saved = record.phases;
		return saved == null ? null : new CylinderPhaseSpeeds(stroke, saved.approach, saved.working,
			saved.returning, saved.changePoint);
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
					// Switch heads can sense a member while it compresses a real end stop.
					// `overtravel` is the mechanical room beyond the commanded joint limits.
					var overtravel = edge.limits.overtravel == null ? 0.0 : edge.limits.overtravel;
					if ((edge.limits.lower != null && lower < edge.limits.lower - overtravel) ||
						(edge.limits.upper != null && upper > edge.limits.upper + overtravel))
						throw 'Joint switch "${record.id}" lies outside mechanical travel including end-stop room';
					result.push({id: record.id, kind: record.kind, joint: record.joint,
						windowLower: lower, windowUpper: upper, hysteresis: hysteresis});
				} else {
					var threshold = record.windowLower;
					if (threshold == null || !(threshold > 0 && threshold < 1) ||
						record.hysteresis == null || !(record.hysteresis >= 0 && record.hysteresis < 1) ||
						!Math.isFinite(threshold + record.hysteresis))
						throw 'At-speed sensor "${record.id}" needs fractional tolerance and hysteresis';
					result.push({id: record.id, kind: record.kind, joint: record.joint,
						windowLower: threshold, hysteresis: record.hysteresis});
				}
			} else {
				var range = record.range, occurrence = record.occurrence, connector = record.connector;
				var boxes = record.zoneBoxes;
				if (occurrence == null || connector == null) throw 'Volume sensor "${record.id}" needs a connector';
				if (record.kind == "robot_zone") {
					if (!materia.assembly.AssemblyDefinitionCodec.validZoneBoxes(boxes))
						throw 'Robot zone "${record.id}" needs boxes of a centre and three positive half extents';
				} else if (range == null || !(range > 0) || !Math.isFinite(range))
					throw 'Presence sensor "${record.id}" needs a connector and positive range';
				var found = false;
				for (entry in flat.requireDefinition(occurrence).connectors) if (entry.name == connector) found = true;
				if (!found) throw 'Unknown connector "$occurrence/$connector"';
				if (record.kind == "robot_zone")
					result.push({id: record.id, kind: record.kind, occurrence: occurrence, connector: connector,
						zoneBoxes: (cast boxes : Array<Float>).copy()});
				else result.push({id: record.id, kind: record.kind, occurrence: occurrence, connector: connector, range: range});
			}
		}
		return result;
	}

	public static function copyCylinder(record:CylinderRecord, map:String->String):CylinderRecord {
		var copy:CylinderRecord = {actuator: map(record.actuator), joint: map(record.joint),
			cylinder: map(record.cylinder), valve: map(record.valve)};
		if (record.phases != null) copy.phases = {approach: record.phases.approach, working: record.phases.working,
			returning: record.phases.returning, changePoint: record.phases.changePoint};
		return copy;
	}

	public static function copySensor(sensor:AssemblySensor, map:String->String):AssemblySensor {
		var copy:AssemblySensor = {id: map(sensor.id), kind: sensor.kind};
		if (sensor.joint != null) copy.joint = map(sensor.joint);
		if (sensor.occurrence != null) copy.occurrence = map(sensor.occurrence);
		if (sensor.connector != null) copy.connector = sensor.connector;
		if (sensor.windowLower != null) copy.windowLower = sensor.windowLower;
		if (sensor.windowUpper != null) copy.windowUpper = sensor.windowUpper;
		if (sensor.hysteresis != null) copy.hysteresis = sensor.hysteresis;
		if (sensor.range != null) copy.range = sensor.range;
		if (sensor.zoneBoxes != null) copy.zoneBoxes = sensor.zoneBoxes.copy();
		return copy;
	}

	public static function copyTransmission(transmission:TransmissionRecord, map:String->String):TransmissionRecord {
		var copy:TransmissionRecord = {coupling: map(transmission.coupling),
			source: TransmissionResolver.mapSource(transmission.source, map), sense: transmission.sense,
			leaderZero: transmission.leaderZero};
		copy.stiffness = transmission.stiffness;
		copy.dampingRatio = transmission.dampingRatio;
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
		if (process != null) copy.processVelocity = {speedChannel: map(process.speedChannel),
			directionChannel: map(process.directionChannel), radiansPerSpeedUnit: process.radiansPerSpeedUnit};
		if (motor.brake != null) {
			var brake = motor.brake;
			copy.brake = {part: map(brake.part), releaseChannel: map(brake.releaseChannel),
				releasedSensor: map(brake.releasedSensor), releaseTimeoutMs: brake.releaseTimeoutMs,
				feedbackMaxAgeMs: brake.feedbackMaxAgeMs};
		}
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
