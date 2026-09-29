package machinekit.robotics;

import cadkit.modeling.AssemblyState;
import cadkit.modeling.Vector;
import machinekit.assembly.MachineAssembly;
import machinekit.assembly.Diagnostics;
import machinekit.assembly.MachineAssemblyDescription;
import machinekit.assembly.MachineAssembly.MachineAssemblyMassProperties;
import materia.assembly.AssemblyFrames;
import materia.assembly.AssemblyRecord.AssemblyFrame;

private typedef WorkingFrame = {
	var name:String;
	var instanceId:String;
	var connectorName:String;
}

typedef EndEffectorSolvedContext = {
	var poses:Map<String, AssemblyFrame>;
	var mountWorld:AssemblyFrame;
}

/** One mountable end effector (EOAT), with mass and frames relative to its mount. */
class EndEffector extends MachineAssembly {
	var mountRef:Null<{instanceId:String, connectorName:String}>;
	final frames:Array<WorkingFrame> = [];
	final collisionExclusions:Map<String, Bool> = [];
	public var primaryFrame(default, null):Null<String>;

	public function new() super();

	override public function describe():MachineAssemblyDescription {
		var description = super.describe();
		description.machine.endEffector = {mount: mountRef == null ? null :
			{instanceId: mountRef.instanceId, connectorName: mountRef.connectorName},
			frames: [for (frame in frames) {name: frame.name, instanceId: frame.instanceId,
				connectorName: frame.connectorName}], primaryFrame: primaryFrame,
			collisionExclusions: [for (member in components()) if (collisionExclusions.exists(member.id)) member.id]};
		return description;
	}

	public static function fromDescription(description:MachineAssemblyDescription):EndEffector {
		var saved = description.machine.endEffector;
		if (saved == null) throw "Description has no end-effector data";
		var base = MachineAssembly.fromDescription(description);
		var result = new EndEffector();
		base.copyInto(result);
		if (saved.mount != null) result.mount(saved.mount.instanceId, saved.mount.connectorName);
		for (frame in saved.frames) result.workingFrame(frame.name, frame.instanceId,
			frame.connectorName, frame.name == saved.primaryFrame);
		for (id in saved.collisionExclusions) result.excludeFromCollision(id);
		return result;
	}

	/** Own a stable builder snapshot when a changer accepts this tool. */
	override public function snapshot():EndEffector {
		var result = new EndEffector();
		copyInto(result);
		if (mountRef != null) result.mountRef = {instanceId: mountRef.instanceId,
			connectorName: mountRef.connectorName};
		for (frame in frames) result.frames.push({name: frame.name, instanceId: frame.instanceId,
			connectorName: frame.connectorName});
		result.primaryFrame = primaryFrame;
		for (id in collisionExclusions.keys()) result.collisionExclusions.set(id, true);
		return result;
	}

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

	/** Explicitly omit a member from generated collision geometry. */
	public function excludeFromCollision(instanceId:String):Void {
		var found = false;
		for (member in components()) if (member.id == instanceId) found = true;
		if (!found) throw 'Unknown end effector member "$instanceId"';
		collisionExclusions.set(instanceId, true);
	}

	public function collisionExcluded(instanceId:String):Bool return collisionExclusions.exists(instanceId);

	/** The connector that mates this unit to the robot or changer. */
	public function mountReference():{instanceId:String, connectorName:String} {
		if (mountRef == null) throw "End effector needs a mount";
		return {instanceId: mountRef.instanceId, connectorName: mountRef.connectorName};
	}

	public function workingFrameReference(name:String):{instanceId:String, connectorName:String} {
		for (frame in frames) if (frame.name == name)
			return {instanceId: frame.instanceId, connectorName: frame.connectorName};
		throw 'Unknown working frame "$name"';
	}

	override public function check():Diagnostics {
		var result = super.check();
		if (mountRef == null) result.error("eoat.missing-mount", "mount", "End effector needs a mount");
		else {
			var mount = mountRef;
			if (!hasMemberConnector(mount.instanceId, mount.connectorName))
				result.error("eoat.invalid-mount", mount.instanceId + "/" + mount.connectorName,
					'Unknown connector "${mount.instanceId}/${mount.connectorName}"');
			var parents = mateParents();
			if (parents.exists(mount.instanceId)) result.error("eoat.nonroot-mount", mount.instanceId,
				"End effector mount must be on a root member");
			for (member in components()) {
				var current = member.id;
				var seen:Map<String, Bool> = [];
				while (parents.exists(current) && !seen.exists(current)) {
					seen.set(current, true);
					current = parents.get(current);
				}
				if (!seen.exists(current) && current != mount.instanceId)
					result.error("eoat.detached-member", member.id,
						'End effector member "${member.id}" is not attached to the mount root');
			}
		}
		for (frame in frames) if (!hasMemberConnector(frame.instanceId, frame.connectorName))
			result.error("eoat.invalid-frame", frame.name,
				'Unknown connector "${frame.instanceId}/${frame.connectorName}"');
		return result;
	}

	override public function validate():Array<String> {
		var result = check();
		result.throwIfErrors();
		return result.warnings();
	}

	/** MachineKit frame in mm, relative to the robot-facing mount connector. */
	public function mountTFrame(name:String, ?state:AssemblyState,
			?solved:EndEffectorSolvedContext):ConnectorFrame {
		if (mountRef == null) throw "End effector needs a mount";
		var frame:Null<WorkingFrame> = null;
		for (entry in frames) if (entry.name == name) frame = entry;
		if (frame == null) throw 'Unknown working frame "$name"';
		var context = solved == null ? solve(state) : solved;
		var pose = context.poses.get(frame.instanceId);
		if (pose == null) throw 'Missing solved pose for "${frame.instanceId}"';
		var frameWorld = AssemblyFrames.compose(pose,
			memberConnectorFrame(frame.instanceId, frame.connectorName));
		return new ConnectorFrame(AssemblyFrames.compose(AssemblyFrames.inverse(context.mountWorld), frameWorld));
	}

	/** Centre in mm and centroidal inertia in kg mm², both in the mount frame. */
	public function massPropertiesAtMount(?state:AssemblyState,
			?solved:EndEffectorSolvedContext):MachineAssemblyMassProperties {
		if (mountRef == null) throw "End effector needs a mount";
		var context = solved == null ? solve(state) : solved;
		var properties = massPropertiesFromPoses(context.poses);
		var inverse = AssemblyFrames.inverse(context.mountWorld);
		var centre = AssemblyFrames.transformPoint(inverse, properties.centreOfMass.x,
			properties.centreOfMass.y, properties.centreOfMass.z);
		return {mass: properties.mass, centreOfMass: new Vector(centre.x, centre.y, centre.z),
			inertia: properties.inertia == null ? null : properties.inertia.rotated(
				inverse.qx, inverse.qy, inverse.qz, inverse.qw),
			unaccounted: properties.unaccounted.copy(),
			unaccountedInertia: properties.unaccountedInertia.copy()};
	}

	/** Solve all member poses and the mount frame for a single conversion pass. */
	public function solve(?state:AssemblyState):EndEffectorSolvedContext {
		if (mountRef == null) throw "End effector needs a mount";
		var mount = mountRef;
		var poses = solvedPoses(state);
		var pose = poses.get(mount.instanceId);
		if (pose == null) throw 'Missing solved pose for "${mount.instanceId}"';
		return {poses: poses, mountWorld: AssemblyFrames.compose(pose,
			memberConnectorFrame(mount.instanceId, mount.connectorName))};
	}

}
