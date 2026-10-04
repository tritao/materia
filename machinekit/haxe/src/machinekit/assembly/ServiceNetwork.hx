package machinekit.assembly;

import machinekit.component.ComponentPort;
import machinekit.component.PortInterfaces;
import machinekit.assembly.MachineAssembly.PortRef;
import machinekit.assembly.MachineAssemblyDescription.PortConnectionRecord;
import machinekit.assembly.MachineAssemblyDescription.PortExposureRecord;

typedef ServiceTrace = { var port:PortRef; var external:Bool; var supplied:Bool; var chain:Array<String>; }

/**
 * One level's service wiring: connections between member ports and the ports it publishes. Tracing
 * and checks work on the flattened assembly, where a subassembly's connections are prefixed.
 */
class ServiceNetwork {
	public final connections:Array<PortConnectionRecord> = [];
	public final exposures:Array<PortExposureRecord> = [];

	public function new() {}

	public static function requirePort(flat:FlatAssembly, reference:PortRef):ComponentPort {
		var component = flat.member(reference.instanceId);
		if (reference.portName == null || reference.portName.length == 0 || !component.hasPort(reference.portName))
			throw 'Unknown port "${reference.instanceId}/${reference.portName}"';
		return component.port(reference.portName);
	}

	/** Connection faults, and with `required` every required consumer left without a supply. */
	public static function check(flat:FlatAssembly, result:Diagnostics, required:Bool = true):Void {
		var connected:Map<String, Bool> = [];
		for (connection in flat.connections) {
			var id = connection.id;
			var from = portRef(connection.fromInstance, connection.fromPort);
			var to = portRef(connection.toInstance, connection.toPort);
			var first = requirePort(flat, from), second = requirePort(flat, to);
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
		if (required) for (member in flat.members) for (port in member.component.ports())
			if (port.required && port.role == Consumer) {
				var reference = portRef(member.id, port.name);
				var subject = '${member.id}/${port.name}';
				if (connected.exists(portKey(reference))) {
					try {
						var found = trace(flat, reference);
						if (!found.supplied) result.error("port.unsupplied", subject, unsuppliedMessage(found.chain));
					} catch (error:String) result.error("port.service-cycle", subject, error);
				} else if (!isExposed(flat, member.id, port.name))
					result.error("port.required-unconnected", subject,
						'Required consumer port "$subject" is unconnected');
			}
	}

	/** Follow a service upstream through connections, bridges and a single-input converter. */
	public static function trace(flat:FlatAssembly, start:PortRef):ServiceTrace {
		var current = start;
		var seen:Map<String, Bool> = [];
		var chain:Array<String> = [];
		while (true) {
			var key = portKey(current);
			if (seen.exists(key)) throw 'Port service cycle at "${start.instanceId}/${start.portName}"';
			seen.set(key, true);
			chain.push('${current.instanceId}/${current.portName}');
			var currentPort = requirePort(flat, current);
			var component = flat.member(current.instanceId);
			var previous:Array<PortRef> = [];
			for (connection in flat.connections)
				if (connection.toInstance == current.instanceId && connection.toPort == current.portName)
					previous.push(portRef(connection.fromInstance, connection.fromPort));
			for (bridge in component.bridges()) if (bridge.to == current.portName)
				previous.push(portRef(current.instanceId, bridge.from));
			for (conversion in component.conversions()) if (conversion.to == current.portName)
				previous.push(portRef(current.instanceId, conversion.from));
			if (previous.length == 0) {
				if (currentPort.role == Supply)
					return {port: current, external: false, supplied: true, chain: chain};
				if (isExposed(flat, current.instanceId, current.portName))
					return {port: current, external: true, supplied: true, chain: chain};
				return {port: current, external: false, supplied: false, chain: chain};
			}
			if (previous.length != 1) throw 'Port "${start.instanceId}/${start.portName}" has ambiguous upstream supply';
			current = previous[0];
		}
	}

	public static function unsuppliedMessage(chain:Array<String>):String
		return 'Service chain ${chain.join(" ← ")} is not supplied';

	static function isExposed(flat:FlatAssembly, instanceId:String, portName:String):Bool {
		for (entry in flat.portExposures) if (entry.instanceId == instanceId && entry.portName == portName) return true;
		return false;
	}

	public static function portKey(reference:PortRef):String return reference.instanceId + "\x1f" + reference.portName;

	public static function portRef(instanceId:String, portName:String):PortRef
		return {instanceId: instanceId, portName: portName};
}
