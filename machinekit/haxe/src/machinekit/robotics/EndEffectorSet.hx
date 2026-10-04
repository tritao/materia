package machinekit.robotics;

import machinekit.assembly.MachineAssembly;
import machinekit.assembly.Diagnostics;
import haxeon.wire.JsonWire;
import machinekit.assembly.MachineAssemblyCodec;
import machinekit.component.ComponentRegistry;
import machinekit.robotics.EndEffectorDescription;

typedef ChangerPortMap = {robot:String, tool:String};

private typedef ChangerInterface = {
	var name:String;
	var instanceId:String;
	var connectorName:String;
	var ports:Array<ChangerPortMap>;
}

/** Robot-side end effector (EOAT) stack with independently coupled tools. */
class EndEffectorSet extends EndEffector {
	var changerRef:Null<ChangerInterface>;
	final tools:Map<String, EndEffector> = [];

	public function new() super();

	public function describeSet():EndEffectorSetDescription {
		var changer = changerRef;
		if (changer == null) throw "End effector set needs a changer";
		var ids = toolIds();
		ids.sort(Reflect.compare);
		return {base: describeEndEffector(), changer: {name: changer.name,
			instanceId: changer.instanceId, connectorName: changer.connectorName,
			ports: [for (entry in changer.ports) {robot: entry.robot, tool: entry.tool}]},
			tools: [for (id in ids) {id: id, tool: requireTool(id).describeEndEffector()}]};
	}

	function requireTool(id:String):EndEffector {
		var tool = tools.get(id);
		if (tool == null) throw 'Unknown changer tool "$id"';
		return tool;
	}

	override public function encode():String {
		var description = describeSet();
		MachineAssemblyCodec.encode(description.base.assembly);
		for (entry in description.tools) MachineAssemblyCodec.encode(entry.tool.assembly);
		return JsonWire.encode(description);
	}

	public static function decode(text:String, ?registry:ComponentRegistry):EndEffectorSet {
		var description:EndEffectorSetDescription = JsonWire.decode(text);
		return fromDescription(description, registry);
	}

	public static function fromDescription(description:EndEffectorSetDescription, ?registry:ComponentRegistry):EndEffectorSet {
		var result = new EndEffectorSet();
		MachineAssembly.fromDescription(description.base.assembly, registry).copyInto(result);
		result.readEndEffector(description.base.endEffector);
		var changer = description.changer;
		result.changer(changer.name, changer.instanceId, changer.connectorName,
			[for (entry in changer.ports) {robot: entry.robot, tool: entry.tool}]);
		for (entry in description.tools) result.addTool(entry.id, EndEffector.fromDescription(entry.tool, registry));
		return result;
	}

	public function changer(name:String, instanceId:String, connector:String,
			portMap:Array<ChangerPortMap>):Void {
		if (changerRef != null) throw "End effector set already has a changer";
		if (name == null || name.length == 0 || portMap == null)
			throw "Changer needs a name and port map";
		memberConnectorFrame(instanceId, connector);
		var master = componentAt(this, instanceId);
		var coupling = master.coupling();
		if (coupling != null && connector != coupling.connector)
			throw 'Changer connector "$connector" does not match master coupling connector';
		var robotPorts:Map<String, Bool> = [], toolPorts:Map<String, Bool> = [];
		var copied:Array<ChangerPortMap> = [];
		for (mapping in portMap) {
			if (mapping == null || mapping.robot == null || mapping.robot.length == 0 ||
				mapping.tool == null || mapping.tool.length == 0)
				throw "Changer port mapping needs robot and tool names";
			if (robotPorts.exists(mapping.robot) || toolPorts.exists(mapping.tool))
				throw "Changer port mapping repeats a port";
			port(mapping.robot);
			robotPorts.set(mapping.robot, true);
			toolPorts.set(mapping.tool, true);
			copied.push({robot: mapping.robot, tool: mapping.tool});
		}
		changerRef = {name: name, instanceId: instanceId, connectorName: connector, ports: copied};
	}

	public function addTool(id:String, tool:EndEffector):Void {
		if (changerRef == null) throw "End effector set needs a changer before adding tools";
		var changer = changerRef;
		if (id == null || id.length == 0 || tool == null || tool == this)
			throw "Changer tool needs a distinct id and end effector";
		if (tools.exists(id)) throw 'Duplicate changer tool "$id"';
		tool.mountReference();
		var findings = new Diagnostics();
		for (mapping in changer.ports) if (!tool.hasPort(mapping.tool))
			findings.error("changer.missing-tool-port", id + "/" + mapping.tool,
				'Changer tool "$id" does not expose mapped port "${mapping.tool}"');
		findings.throwIfErrors();
		var master = componentAt(this, changer.instanceId);
		var mount = tool.mountReference();
		var half = componentAt(tool, mount.instanceId);
		var masterCoupling = master.coupling();
		var toolCoupling = half.coupling();
		var masterCoupled = masterCoupling != null;
		var toolCoupled = toolCoupling != null;
		if (masterCoupled != toolCoupled)
			throw 'Changer tool "$id" needs a matching coupling interface';
		if (masterCoupled) {
			var robotHalf:machinekit.component.CouplingFacet = cast masterCoupling;
			var toolHalf:machinekit.component.CouplingFacet = cast toolCoupling;
			if (changer.connectorName != robotHalf.connector ||
				mount.connectorName != toolHalf.connector ||
				robotHalf.key != toolHalf.key)
				throw 'Changer tool "$id" does not fit the master interface';
		}
		tools.set(id, tool.snapshot());
	}

	function componentAt(assembly:MachineAssembly, instanceId:String):machinekit.component.MachineComponent {
		for (member in assembly.components()) if (member.id == instanceId) return member.component;
		throw 'Unknown assembly member "$instanceId"';
	}


	override public function check():Diagnostics {
		var result = super.check();
		if (changerRef == null) result.error("changer.missing", "changer", "End effector set needs a changer");
		for (id in tools.keys()) {
			var tool = tools.get(id);
			for (finding in tool.check().items)
				result.add(finding.severity, finding.code, id + "/" + finding.subject, finding.message);
			if (changerRef != null) for (mapping in changerRef.ports)
				if (!tool.hasPort(mapping.tool)) result.error("changer.missing-tool-port", id + "/" + mapping.tool,
					'Changer tool "$id" does not expose mapped port "${mapping.tool}"');
		}
		return result;
	}

	public function toolIds():Array<String> return [for (id in tools.keys()) id];

	/** Build one coupled configuration, with no ports to other tools. */
	public function configuration(toolId:String):EndEffector {
		return buildConfiguration(toolId);
	}

	/** Evaluate a configuration from portable data without modifying the source description. */
	public static function configurationFromDescription(description:EndEffectorSetDescription,
			toolId:String, ?registry:ComponentRegistry):EndEffector
		return EndEffectorSet.fromDescription(description, registry).buildConfiguration(toolId);

	function buildConfiguration(toolId:String):EndEffector {
		if (changerRef == null) throw "End effector set needs a changer";
		var changer = changerRef;
		var tool = tools.get(toolId);
		if (tool == null) throw 'Unknown changer tool "$toolId"';
		validate();
		tool.validate();
		var result = new EndEffector();
		result.include("robot", this);
		result.include("tool", tool);
		for (member in components()) if (collisionExcluded(member.id))
			result.excludeFromCollision(MachineAssembly.join("robot", member.id));
		for (member in tool.components()) if (tool.collisionExcluded(member.id))
			result.excludeFromCollision(MachineAssembly.join("tool", member.id));
		var robotMount = mountReference();
		var toolMount = tool.mountReference();
		result.mount(MachineAssembly.join("robot", robotMount.instanceId), robotMount.connectorName);
		result.addMate(changer.name, "fixed", MachineAssembly.join("robot", changer.instanceId),
			changer.connectorName, MachineAssembly.join("tool", toolMount.instanceId), toolMount.connectorName);
		for (mapping in changer.ports) {
			var from = port(mapping.robot);
			var to = tool.port(mapping.tool);
			result.connectPorts(changer.name + "/" + mapping.robot,
				MachineAssembly.join("robot", from.instanceId), from.portName,
				MachineAssembly.join("tool", to.instanceId), to.portName);
		}
		for (name in portNames()) {
			var mapped = false;
			for (mapping in changer.ports) if (mapping.robot == name) mapped = true;
			if (!mapped) {
				var reference = port(name);
				result.exposePort(name, MachineAssembly.join("robot", reference.instanceId), reference.portName);
			}
		}
		for (name in workingFrameNames()) {
			var reference = workingFrameReference(name);
			result.workingFrame("robot/" + name, MachineAssembly.join("robot", reference.instanceId),
				reference.connectorName, tool.primaryFrame == null && primaryFrame == name);
		}
		for (name in tool.workingFrameNames()) {
			var reference = tool.workingFrameReference(name);
			result.workingFrame(name, MachineAssembly.join("tool", reference.instanceId),
				reference.connectorName, tool.primaryFrame == name);
		}
		result.validate();
		return result;
	}
}
