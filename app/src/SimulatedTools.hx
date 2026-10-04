package app;

import materia.assembly.AssemblyDefinitionFlattener;
import materia.assembly.AssemblyFrames;
import materia.assembly.AssemblyRecord.AssemblyFrame;
import materia.project.SceneArtifact.SceneArtifactRobotTool;
import nativekit.sim.SimObject;
import robotkit.runtime.SimulatedSuctionTool;
import processkit.simulation.SimulatedWelder;
import robotkit.runtime.Simulation;
import robotkit.tool.ConvexSolid;
import processkit.tool.GroundedWork;

/** A scene object the simulation owns, by scene id. */
typedef GripObject = {id:String, object:SimObject};

/**
 * The simulated tools of a session's robots, as a member of the session: they act after each step
 * on their own, so this only resets them with the session and reports what they hold.
 */
class SimulatedTools implements SessionMember {
	public final tools:Array<SimulatedSuctionTool> = [];
	public final welders:Array<SimulatedWelder> = [];
	final objects:Array<GripObject>;

	public function new(objects:Array<GripObject>) this.objects = objects;

	public function add(tool:SimulatedSuctionTool):SimulatedSuctionTool {
		tools.push(tool);
		return tool;
	}

	public function addWelder(welder:SimulatedWelder):SimulatedWelder {
		welders.push(welder);
		return welder;
	}

	/**
	 * The welder a `torch` robot tool describes, on the link of the assembly robot that carries the torch. The wire
	 * tip is the tool's contact connector and the wire runs along its +Z. The grounded work is the collision hulls of
	 * the occurrences the tool names, each following its link; an occurrence without a hull (a free workpiece, or a
	 * part with collision turned off) cannot be work, and is refused rather than silently left out.
	 */
	public static function welderFor(simulation:Simulation, robot:AssemblyRobot, robotIndex:Int, tool:SceneArtifactRobotTool,
			project:ProjectDocumentSession):SimulatedWelder {
		var welding = tool.torch;
		var sensor = tool.sensor;
		if (welding == null || sensor == null) throw 'Robot tool "${tool.channel}" is not a torch with a weld sensor';
		var physical = project.projectPhysical;
		var metres = physical == null ? 1.0 : physical.metresPerUnit;
		var definition = project.projectAssemblyDefinition;
		if (definition == null) throw "A torch needs the project's assembly";
		var flat = AssemblyDefinitionFlattener.flatten(definition);
		var contact:Null<AssemblyFrame> = null;
		for (item in flat.occurrences) if (item.id == tool.contact.occurrence)
			for (component in flat.definitions) if (component.id == item.definition)
				for (connector in component.connectors) if (connector.name == tool.contact.connector) contact = connector.frame;
		if (contact == null) throw 'Torch connector "${tool.contact.occurrence}/${tool.contact.connector}" does not exist';
		var part = robot.part("project:" + tool.contact.occurrence);
		var tip = AssemblyFrames.compose(part.offset, {x: contact.x * metres, y: contact.y * metres, z: contact.z * metres,
			qx: contact.qx, qy: contact.qy, qz: contact.qz, qw: contact.qw});
		var wire = AssemblyRobot.rotate([tip.qx, tip.qy, tip.qz, tip.qw], [0.0, 0.0, 1.0]);
		var work = new GroundedWork();
		for (id in welding.groundedWork) {
			var found = false;
			for (hull in robot.hulls) if (hull.part == id) {
				found = true;
				var link = hull.link;
				work.add(new ConvexSolid(hull.vertices), () -> simulation.linkPose(robotIndex, link));
			}
			if (!found) throw 'Grounded work "$id" has no collision hull in the simulated robot';
		}
		return new SimulatedWelder(simulation, robot.runtime, robotIndex, part.linkIndex, [tip.x, tip.y, tip.z], wire, work, tool.channel,
			welding.wireSpeedChannel, welding.voltageChannel, sensor,
			{maxCurrentA: welding.maxCurrentA, efficiency: welding.efficiency, wireDiameterMm: welding.wireDiameterMm,
				stickoutMm: welding.stickoutMm});
	}

	/** Scene ids of the objects the tools hold right now. */
	public function heldIds():Array<String> {
		var result:Array<String> = [];
		for (tool in tools) {
			var held = tool.held;
			if (held != null) for (entry in objects) if (entry.object == held) result.push(entry.id);
		}
		return result;
	}

	/** The session's free objects, by scene id. */
	public function objectsByScene():Array<GripObject> return objects;

	public function feed():Void {}

	public function beforeReset():Void {}

	public function reset():Void {
		for (tool in tools) tool.reset();
		for (welder in welders) welder.reset();
	}

	public function present():Void {}
}
