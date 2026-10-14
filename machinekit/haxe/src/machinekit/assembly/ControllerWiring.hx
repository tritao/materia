package machinekit.assembly;

import machinekit.control.ControllerCabinet;
import machinekit.control.DigitalSensor;
import materia.assembly.AssemblyDefinition;
import materia.project.SceneArtifact.SceneArtifactCellController;
import materia.project.SceneArtifact.SceneArtifactCellSignal;

typedef CellWiring = {
	var controllers:Array<SceneArtifactCellController>;
	var signals:Array<SceneArtifactCellSignal>;
}

/** Derives control domains from cabinet terminals, drive inputs and valve coils. */
class ControllerWiring {
	public static function of(assembly:MachineAssembly, definition:AssemblyDefinition):Null<CellWiring> {
		var cabinets = new Map<String, ControllerCabinet>();
		var heads = new Map<String, DigitalSensor>();
		for (member in assembly.components()) if (Std.isOfType(member.component, ControllerCabinet))
			cabinets.set(member.id, cast member.component);
		else if (Std.isOfType(member.component, DigitalSensor)) heads.set(member.id, cast member.component);
		var ids = [for (id in cabinets.keys()) id];
		ids.sort(Reflect.compare);
		if (ids.length == 0) return null;
		var flat = assembly.derived();
		var owner = new Map<String, String>();
		var byId = new Map<String, SceneArtifactCellController>();
		var controllers:Array<SceneArtifactCellController> = [];
		for (id in ids) {
			var cabinet = cabinets.get(id);
			var entry:SceneArtifactCellController = {id: id, kind: cabinet.kind, joints: [], sensors: [], channels: []};
			controllers.push(entry);
			byId.set(id, entry);
		}
		function owned(id:String):SceneArtifactCellController {
			var entry = byId.get(id);
			if (entry == null) throw 'Unknown controller "$id"';
			return entry;
		}
		function terminal(instance:String, port:String):Null<String> {
			for (wire in flat.connections)
				if (wire.toInstance == instance && wire.toPort == port && cabinets.exists(wire.fromInstance))
					return wire.fromInstance;
			return null;
		}
		function claim(joint:String, cabinet:String):Void {
			var prior = owner.get(joint);
			if (prior != null && prior != cabinet)
				throw 'Joint "$joint" is wired to both "$prior" and "$cabinet"';
			owner.set(joint, cabinet);
		}
		if (flat.motors != null) for (motor in flat.motors) {
			var cabinet = terminal(motor.driver, "bus");
			if (cabinet == null) cabinet = terminal(motor.driver, "step-dir");
			if (cabinet == null) throw 'Motor "${motor.actuator}" has no controller wiring';
			claim(motor.joint, cabinet);
			var process = motor.processVelocity;
			if (process != null) {
				owned(cabinet).channels.push(process.speedChannel);
				owned(cabinet).channels.push(process.directionChannel);
			}
			// The drive bus carries the integrated brake release and contact, like motor feedback.
			var brake = motor.brake;
			if (brake != null) owned(cabinet).channels.push(brake.releaseChannel);
		}
		if (flat.cylinders != null) for (cylinder in flat.cylinders) {
			var cabinet = terminal(cylinder.valve, "coilA");
			if (cabinet == null) throw 'Cylinder "${cylinder.actuator}" has no controller wiring';
			var other = terminal(cylinder.valve, "coilB");
			if (other != null && other != cabinet) throw 'Valve "${cylinder.valve}" has coils on different controllers';
			claim(cylinder.joint, cabinet);
			owned(cabinet).channels.push(cylinder.valve + "/coilA");
			if (other != null) owned(cabinet).channels.push(cylinder.valve + "/coilB");
		}
		// A transmission follower is part of the same motion domain as its leader.
		var changed = true;
		while (changed) {
			changed = false;
			if (definition.couplings != null) for (coupling in definition.couplings) {
				var source = owner.get(coupling.source), target = owner.get(coupling.target);
				if (source != null && target == null) {claim(coupling.target, source); changed = true;}
				else if (target != null && source == null) {claim(coupling.source, target); changed = true;}
				else if (source != null && target != null && source != target)
					throw 'Coupling "${coupling.id}" crosses controllers';
			}
		}
		for (joint in definition.joints) if (Std.string(joint.role) == "tree" && Std.string(joint.type) != "fixed") {
			var cabinet = owner.get(joint.id);
			if (cabinet == null) throw 'Joint "${joint.id}" has no controller';
			owned(cabinet).joints.push(joint.id);
		}
		if (definition.sensors != null) for (sensor in definition.sensors) {
			var cabinet:Null<String> = null;
			if (sensor.kind == "at_speed" || sensor.kind == "brake_released" || sensor.kind == "joint_switch")
				cabinet = owner.get(sensor.joint);
			else {
				var head = heads.get(sensor.id);
				var expected = sensor.kind == "presence" ? "presence" : sensor.kind == "robot_zone" ? "scanner" : "reed";
				if (head == null || head.kind != expected) throw 'Sensor "${sensor.id}" has no matching field head';
				for (wire in flat.connections)
				if (wire.fromInstance == sensor.id && wire.fromPort == "signal" && cabinets.exists(wire.toInstance)) {
					if (cabinet != null && cabinet != wire.toInstance)
						throw 'Sensor "${sensor.id}" is wired to multiple controllers';
					cabinet = wire.toInstance;
				}
			}
			if (cabinet == null) throw 'Sensor "${sensor.id}" has no controller';
			if (sensor.joint != null && owner.exists(sensor.joint) && owner.get(sensor.joint) != cabinet)
				throw 'Sensor "${sensor.id}" observes joint "${sensor.joint}" owned by "${owner.get(sensor.joint)}", '
					+ 'but is wired to "$cabinet"; wire its field head to the joint owner and route a cell signal to the other controller';
			owned(cabinet).sensors.push(sensor.id);
		}
		var signals:Array<SceneArtifactCellSignal> = [];
		for (wire in flat.connections)
			if (cabinets.exists(wire.fromInstance) && cabinets.exists(wire.toInstance))
				{
					var source = cabinets.get(wire.fromInstance).port(wire.fromPort);
					var target = cabinets.get(wire.toInstance).port(wire.toPort);
					if (source.kind != machinekit.component.PortKind.Signal ||
						target.kind != machinekit.component.PortKind.Signal ||
						source.role != machinekit.component.PortRole.Supply ||
						target.role != machinekit.component.PortRole.Consumer)
						throw 'Cell signal "${wire.id}" must connect a controller output to a controller input';
					owned(wire.fromInstance).channels.push(wire.id);
					signals.push({name: wire.id, from: {controller: wire.fromInstance, port: wire.fromPort},
						to: {controller: wire.toInstance, port: wire.toPort}});
				}
		for (entry in controllers) {
			var channels:Array<String> = [];
			for (channel in entry.channels) if (channels.indexOf(channel) < 0) channels.push(channel);
			entry.channels = channels;
			entry.joints.sort(Reflect.compare);
			entry.sensors.sort(Reflect.compare);
			entry.channels.sort(Reflect.compare);
		}
		signals.sort((a, b) -> Reflect.compare(a.name, b.name));
		return {controllers: controllers, signals: signals};
	}
}
