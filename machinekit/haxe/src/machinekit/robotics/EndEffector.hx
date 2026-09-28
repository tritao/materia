package machinekit.robotics;

import cadkit.modeling.AssemblyModel;
import cadkit.modeling.AssemblyState;
import cadkit.modeling.Vector;
import machinekit.assembly.MachineAssembly;
import machinekit.assembly.MachineAssembly.MachineAssemblyMassProperties;
import materia.assembly.AssemblyFrames;
import materia.assembly.AssemblyRecord.AssemblyFrame;

private typedef WorkingFrame = {
	var name:String;
	var instanceId:String;
	var connectorName:String;
}

/** One mountable end effector (EOAT), with mass and frames relative to its mount. */
class EndEffector extends MachineAssembly {
	var mountRef:Null<{instanceId:String, connectorName:String}>;
	final frames:Array<WorkingFrame> = [];
	public var primaryFrame(default, null):Null<String>;

	public function new() super();

	public function mount(instanceId:String, connector:String):Void {
		if (mountRef != null) throw "End effector already has a mount";
		memberConnectorFrame(instanceId, connector);
		mountRef = {instanceId: instanceId, connectorName: connector};
	}

	public function workingFrame(name:String, instanceId:String, connector:String,
			primary:Bool = false):Void {
		if (name == null || name.length == 0) throw "Working frame needs a name";
		for (frame in frames) if (frame.name == name) throw 'Duplicate working frame "$name"';
		if (primary && primaryFrame != null) throw "End effector already has a primary working frame";
		frames.push({name: name, instanceId: instanceId, connectorName: connector});
		if (primary) primaryFrame = name;
	}

	public function workingFrameNames():Array<String> return [for (frame in frames) frame.name];

	override public function validate():Array<String> {
		var warnings = super.validate();
		if (mountRef == null) throw "End effector needs a mount";
		var mount = mountRef;
		memberConnectorFrame(mount.instanceId, mount.connectorName);
		var parents = mateParents();
		if (parents.exists(mount.instanceId)) throw "End effector mount must be on a root member";
		for (member in components()) {
			var current = member.id;
			while (parents.exists(current)) current = parents.get(current);
			if (current != mount.instanceId) throw 'End effector member "${member.id}" is not attached to the mount root';
		}
		for (frame in frames) memberConnectorFrame(frame.instanceId, frame.connectorName);
		return warnings;
	}

	/** MachineKit frame in mm, relative to the robot-facing mount connector. */
	public function mountTFrame(name:String, ?state:AssemblyState):AssemblyFrame {
		if (mountRef == null) throw "End effector needs a mount";
		var mount = mountRef;
		var frame:Null<WorkingFrame> = null;
		for (entry in frames) if (entry.name == name) frame = entry;
		if (frame == null) throw 'Unknown working frame "$name"';
		var mountWorld = worldConnector(mount.instanceId, mount.connectorName, state);
		var frameWorld = worldConnector(frame.instanceId, frame.connectorName, state);
		return AssemblyFrames.compose(AssemblyFrames.inverse(mountWorld), frameWorld);
	}

	/** Centre in mm and centroidal inertia in kg mm², both in the mount frame. */
	public function massPropertiesAtMount(?state:AssemblyState):MachineAssemblyMassProperties {
		if (mountRef == null) throw "End effector needs a mount";
		var mount = mountRef;
		var properties = massProperties(state);
		var mountWorld = worldConnector(mount.instanceId, mount.connectorName, state);
		var inverse = AssemblyFrames.inverse(mountWorld);
		var centre = AssemblyFrames.transformPoint(inverse, properties.centreOfMass.x,
			properties.centreOfMass.y, properties.centreOfMass.z);
		return {mass: properties.mass, centreOfMass: new Vector(centre.x, centre.y, centre.z),
			inertia: properties.inertia == null ? null : properties.inertia.rotated(
				inverse.qx, inverse.qy, inverse.qz, inverse.qw),
			unaccounted: properties.unaccounted.copy(),
			unaccountedInertia: properties.unaccountedInertia.copy()};
	}

	function worldConnector(instanceId:String, connectorName:String, state:AssemblyState):AssemblyFrame {
		var local = memberConnectorFrame(instanceId, connectorName);
		if (state != null) return AssemblyFrames.compose(state.worldPose(instanceId), local);
		var model = new AssemblyModel();
		addTo(model, "");
		return AssemblyFrames.compose(model.pose(instanceId), local);
	}
}
