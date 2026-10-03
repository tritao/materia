package machinekit.robotics;

import machinekit.component.PortKind;
import machinekit.component.PortRole;

/** One digital control of an end effector: the channel that works it and the inlet it drives. */
enum EndEffectorControl {
	Gripper(channel:String, member:String, openPort:String, closePort:String);
	Vacuum(channel:String, member:String, inletPort:String);
	Lock(channel:String, member:String, inletPort:String);
	/** An arc torch's trigger: the channel that lights its arc and the control inlet it drives. */
	Arc(channel:String, member:String, controlPort:String);
}

/** Where a suction cup meets what it holds: its member and its contact connector. */
typedef SuctionContact = {member:String, connector:String};

/**
 * An arc torch and the channels that work it, named `<prefix>/<member>.<signal>`:
 * - `channel`, digital: lights the arc (the `Arc` control);
 * - `wireSpeedChannel`, analog: the wire feed speed, in metres per minute;
 * - `voltageChannel`, analog: the voltage setpoint, in volts;
 * - `sensor`, a `tool_weld` frame: arc established, current (A), voltage (V), touch, fault code and the
 *   mains power the supply draws (W), in that order.
 * `controlPort` is the inlet the channels drive through the feeder and the supply, and
 * `tcpConnector` is the wire tip.
 */
typedef ArcTorchControl = {
	member:String, channel:String, wireSpeedChannel:String, voltageChannel:String, sensor:String,
	controlPort:String, tcpConnector:String
};

/**
 * What it takes to run an end effector, read from the capabilities its parts declare: the digital
 * controls that work its gripper, vacuum and changer lock, each on a channel named
 * `<prefix>/<member>.<action>`; the vacuum pressure sensor, if it has one, as
 * `<prefix>/<member>.<signal port>`; where its suction cups touch; and its arc torches, each with
 * the arc channel `<prefix>/<member>.arc` and, with it, the wire speed, voltage and weld sensor of an
 * `ArcTorchControl`. Each control drives a consumer inlet of the right
 * service that its configuration supplies. A runtime, real or simulated, binds these; nothing here
 * depends on one.
 */
class EndEffectorControls {
	public final controls:Array<EndEffectorControl>;
	public final vacuumSensor:Null<String>;
	public final suctions:Array<SuctionContact>;
	public final arcs:Array<ArcTorchControl>;

	function new(controls:Array<EndEffectorControl>, vacuumSensor:Null<String>, suctions:Array<SuctionContact>,
			arcs:Array<ArcTorchControl>) {
		this.controls = controls;
		this.vacuumSensor = vacuumSensor;
		this.suctions = suctions;
		this.arcs = arcs;
	}

	/** The first arc torch's channel, or null when the effector has none. */
	public function arcChannel():Null<String> return arcs.length == 0 ? null : arcs[0].channel;

	/** The vacuum control's channel, or null when the effector has none. */
	public function vacuumChannel():Null<String> {
		for (control in controls) switch control {
			case Vacuum(channel, _, _): return channel;
			case _:
		}
		return null;
	}

	public static function derive(configuration:EndEffector, prefix:String):EndEffectorControls {
		if (configuration == null) throw "End effector configuration is required";
		var controls:Array<EndEffectorControl> = [];
		var suctions:Array<SuctionContact> = [];
		var arcs:Array<ArcTorchControl> = [];
		var sensorId:Null<String> = null;
		var hasGripper = false, hasVacuum = false, hasLock = false;
		var hasExplicitVacuumValve = false;
		for (member in configuration.components()) for (intent in member.component.capabilities())
			switch intent {
				case VacuumValve(_): hasExplicitVacuumValve = true;
				case _:
			}
		for (member in configuration.components()) for (intent in member.component.capabilities()) {
			var name = '$prefix/${member.id}';
			switch intent {
				case Grip(_, _, openPort, closePort):
					if (hasGripper || openPort == closePort) throw "Ambiguous gripper runtime ports";
					requireInlet(configuration, member.id, openPort, [PortKind.Pneumatic, PortKind.Signal]);
					requireInlet(configuration, member.id, closePort, [PortKind.Pneumatic, PortKind.Signal]);
					controls.push(Gripper('$name.close', member.id, openPort, closePort));
					hasGripper = true;
				case VacuumActuator(inletPort):
					if (hasExplicitVacuumValve) continue;
					if (hasVacuum) throw "Ambiguous vacuum runtime ports";
					requireInlet(configuration, member.id, inletPort, [PortKind.Pneumatic, PortKind.Signal]);
					controls.push(Vacuum('$name.enable', member.id, inletPort));
					hasVacuum = true;
				case VacuumValve(controlPort):
					if (hasVacuum) throw "Ambiguous vacuum runtime ports";
					requireInlet(configuration, member.id, controlPort, [PortKind.Signal]);
					controls.push(Vacuum('$name.enable', member.id, controlPort));
					hasVacuum = true;
				case ChangerLock(inletPort):
					if (hasLock) throw "Ambiguous changer-lock runtime ports";
					requireInlet(configuration, member.id, inletPort, [PortKind.Pneumatic]);
					controls.push(Lock('$name.lock', member.id, inletPort));
					hasLock = true;
				case VacuumPressureSensor(vacuumPort, signalPort):
					if (sensorId != null) throw "Ambiguous vacuum pressure sensors";
					requireInlet(configuration, member.id, vacuumPort, [PortKind.Vacuum]);
					var signal = member.component.port(signalPort);
					if (signal.role != PortRole.Supply || signal.kind != PortKind.Signal)
						throw 'Pressure sensor "$name" needs a signal supply';
					sensorId = '$name.$signalPort';
				case Suction(_, _, _, contactConnector):
					suctions.push({member: member.id, connector: contactConnector});
				case ArcTorch(tcpConnector, controlPort):
					requireInlet(configuration, member.id, controlPort, [PortKind.Signal]);
					if (arcs.length > 0) throw "Ambiguous arc torch runtime ports";
					controls.push(Arc('$name.arc', member.id, controlPort));
					arcs.push({member: member.id, channel: '$name.arc', wireSpeedChannel: '$name.wire_speed',
						voltageChannel: '$name.voltage', sensor: '$name.weld', controlPort: controlPort, tcpConnector: tcpConnector});
				case _:
			}
		}
		if (sensorId != null && !hasVacuum) throw "Vacuum pressure sensor has no bound vacuum actuator";
		return new EndEffectorControls(controls, sensorId, suctions, arcs);
	}

	static function requireInlet(configuration:EndEffector, instanceId:String, portName:String, allowed:Array<PortKind>):Void {
		for (member in configuration.components()) if (member.id == instanceId) {
			var port = member.component.port(portName);
			if (port.role != PortRole.Consumer) throw 'Control port "$instanceId/$portName" must be a consumer';
			for (kind in allowed) if (port.kind == kind) {
				configuration.upstream(instanceId, portName);
				return;
			}
			throw 'Control port "$instanceId/$portName" has the wrong service kind';
		}
		throw 'Unknown control member "$instanceId"';
	}
}
