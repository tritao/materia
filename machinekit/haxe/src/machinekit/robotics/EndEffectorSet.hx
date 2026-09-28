package machinekit.robotics;

import machinekit.assembly.MachineAssembly;

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

	public function changer(name:String, instanceId:String, connector:String,
			portMap:Array<ChangerPortMap>):Void {
		if (changerRef != null) throw "End effector set already has a changer";
		if (name == null || name.length == 0 || portMap == null)
			throw "Changer needs a name and port map";
		memberConnectorFrame(instanceId, connector);
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
		if (id == null || id.length == 0 || tool == null || tool == this)
			throw "Changer tool needs a distinct id and end effector";
		if (tools.exists(id)) throw 'Duplicate changer tool "$id"';
		tool.mountReference();
		tools.set(id, tool);
	}

	public function toolIds():Array<String> return [for (id in tools.keys()) id];

	/** Build one coupled configuration, with no ports to other tools. */
	public function configuration(toolId:String):EndEffector {
		if (changerRef == null) throw "End effector set needs a changer";
		var changer = changerRef;
		var tool = tools.get(toolId);
		if (tool == null) throw 'Unknown changer tool "$toolId"';
		validate();
		tool.validate();
		var result = new EndEffector();
		result.include("robot", this);
		result.include("tool", tool);
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
