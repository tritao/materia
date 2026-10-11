package app;

import materia.assembly.AssemblyDefinitionFlattener;
import materia.assembly.AssemblyFrames;
import materia.assembly.AssemblyRecord.AssemblyFrame;
import materia.project.SceneArtifact.SceneArtifactRobotTool;
import nativekit.sim.SimObject;
import robotkit.runtime.SimulatedSuctionTool;
import robotkit.runtime.SimulatedPneumaticGripper;
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
	public final grippers:Array<SimulatedPneumaticGripper> = [];
	public final welders:Array<SimulatedWelder> = [];
	public final screwdrivers:Array<processkit.simulation.SimulatedScrewdriver> = [];
	public final screwFeeders:Array<SimulatedScrewFeeder> = [];
	final objects:Array<GripObject>;
	final simulation:Simulation;

	public function new(objects:Array<GripObject>, simulation:Simulation) {
		this.objects = objects;
		this.simulation = simulation;
	}

	public function add(tool:SimulatedSuctionTool):SimulatedSuctionTool {
		tools.push(tool);
		return tool;
	}

	public function addGripper(gripper:SimulatedPneumaticGripper):SimulatedPneumaticGripper {
		grippers.push(gripper);
		return gripper;
	}

	public function addWelder(welder:SimulatedWelder):SimulatedWelder {
		welders.push(welder);
		return welder;
	}

	/**
	 * The welder a `torch` robot tool describes, on the link of the assembly robot that carries the torch. The wire
	 * tip is the tool's contact connector and the wire runs along its +Z. Grounded CAD hulls follow either their
	 * robot link or their independent scene object. Missing or disabled collision geometry is refused.
	 */
	public function welderFor(simulation:Simulation, robot:AssemblyRobot, robotIndex:Int, tool:SceneArtifactRobotTool,
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
        var parts = new SimulationAssemblyParts(simulation, [robot], objects, project);
		for (id in welding.groundedWork) {
			var body = parts.get("project:" + id);
			if (body.pieces.length == 0) throw 'Grounded work "$id" has no CAD collision hull';
			for (piece in body.pieces) work.add(new ConvexSolid(piece), body.pose);
		}
		return new SimulatedWelder(simulation, robot.runtime, robotIndex, part.linkIndex, [tip.x, tip.y, tip.z], wire, work, tool.channel,
			welding.wireSpeedChannel, welding.voltageChannel, sensor,
			{maxCurrentA: welding.maxCurrentA, efficiency: welding.efficiency, wireDiameterMm: welding.wireDiameterMm,
				stickoutMm: welding.stickoutMm});
	}

	public function addScrewdriver(driver:processkit.simulation.SimulatedScrewdriver):processkit.simulation.SimulatedScrewdriver {
		screwdrivers.push(driver); return driver;
	}

	/** An idle observer for the authored driver. A qualified joint is configured explicitly before use. */
	public function screwdriverFor(simulation:Simulation, robot:AssemblyRobot, robotIndex:Int, tool:SceneArtifactRobotTool,
			project:ProjectDocumentSession):processkit.simulation.SimulatedScrewdriver {
		var driver = tool.screwdriver, sensor = tool.sensor, definition = project.projectAssemblyDefinition;
		if (driver == null || sensor == null || definition == null) throw "A screwdriver needs its capability, sensor and assembly";
		var flat = AssemblyDefinitionFlattener.flatten(definition);
		var physical = project.projectPhysical;
		var metres = physical == null ? 1.0 : physical.metresPerUnit;
		var contact:Null<AssemblyFrame> = null;
		for (occurrence in flat.occurrences) if (occurrence.id == tool.contact.occurrence)
			for (component in flat.definitions) if (component.id == occurrence.definition)
				for (connector in component.connectors) if (connector.name == tool.contact.connector) contact = connector.frame;
		if (contact == null) throw 'Screwdriver connector "${tool.contact.occurrence}/${tool.contact.connector}" does not exist';
		var part = robot.part("project:" + tool.contact.occurrence);
		var tip = AssemblyFrames.compose(part.offset, {x:contact.x*metres,y:contact.y*metres,z:contact.z*metres,
			qx:contact.qx,qy:contact.qy,qz:contact.qz,qw:contact.qw});
		var feed = -1, spindle = -1;
		for (i in 0...robot.model.joints.length) {
			if (robot.model.joints[i].id == driver.feedJoint) feed = i;
			if (robot.model.joints[i].id == driver.spindleJoint) spindle = i;
		}
		if (feed < 0 || spindle < 0) throw "Screwdriver's declared tool joints are absent from its robot";
		return new processkit.simulation.SimulatedScrewdriver(simulation, robot.runtime, robotIndex, part.linkIndex,
			feed, spindle, [tip.x,tip.y,tip.z], AssemblyRobot.rotate([tip.qx,tip.qy,tip.qz,tip.qw],[0.0,0.0,1.0]),
			{start:tool.channel,direction:driver.directionChannel,program:driver.programChannel,vacuum:driver.vacuumChannel},
			sensor,driver.strokeMm,driver.maxSpeed,driver.maxTorque,driver.springRate);
	}

	/** Scene ids of the objects the tools hold right now. */
	public function heldIds():Array<String> {
		var result:Array<String> = [];
		for (driver in screwdrivers) {
			var held = driver.holding.held;
			if (held != null) for (entry in objects) if (entry.object == held) result.push(entry.id);
		}
		for (tool in tools) {
			var held = tool.held;
			if (held != null) for (entry in objects) if (entry.object == held) result.push(entry.id);
		}
		var runtimes:Array<robotkit.runtime.RobotRuntime> = [];
		var images:Array<Array<robotkit.runtime.RobotContact>> = [];
		for (gripper in grippers) {
			var image = runtimes.indexOf(gripper.runtime);
			if (image < 0) {
				image = runtimes.length;
				runtimes.push(gripper.runtime);
				images.push(simulation.robotContacts(gripper.runtime));
			}
			for (handle in gripper.grippedObjects(images[image])) for (entry in objects)
				if (entry.object.handle.rawValue() == handle && result.indexOf(entry.id) < 0) result.push(entry.id);
		}
		return result;
	}

	/** The session's free objects, by scene id. */
	public function objectsByScene():Array<GripObject> return objects;

	public function feed():Void {}

	public function beforeReset():Void {}

	public function safeFeeders():Void {for (feeder in screwFeeders) feeder.safe();}

	public function safeWelders():Void {
		for (welder in welders) if (welder.supply != null) welder.supply.safe();
	}

	public function reset():Void {
		for (tool in tools) tool.reset();
		for (gripper in grippers) gripper.reset();
		for (welder in welders) welder.reset();
		for (driver in screwdrivers) driver.reset();
		for (feeder in screwFeeders) feeder.reset();
	}

	public function present():Void {}
}
