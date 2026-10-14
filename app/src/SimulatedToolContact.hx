package app;

import haxe.Int64;
import robotkit.runtime.RobotRuntime;
import robotkit.runtime.Simulation;
import robotkit.runtime.SimulationStepObserver;
import robotkit.runtime.RobotContact;
import robotkit.runtime.RobotContactOtherKind;
import robotkit.spatial.Transform3;
import robotkit.spatial.Vec3;
import robotkit.spatial.Quat;

/** Publishes fresh digital contact only when the screwdriver bit's front profile touches CAD. */
class SimulatedToolContact implements SimulationStepObserver {
	final simulation:Simulation;
	final runtime:RobotRuntime;
	final robotIndex:Int;
	final linkIndex:Int;
	final sensorId:String;
	final tcp:Transform3;
	var nextSequence:Int64 = Int64.ofInt(0);

	public function new(simulation:Simulation, runtime:RobotRuntime, robotIndex:Int, linkIndex:Int,
		sensorId:String, tcp:Transform3) {
		if (simulation == null || runtime == null || !simulation.ownsRobot(runtime) || robotIndex < 0 || linkIndex < 0 ||
			sensorId == null || sensorId.length == 0 || tcp == null)
			throw "Simulated contact needs its mounted robot link, sensor and TCP";
		this.simulation = simulation;
		this.runtime = runtime;
		this.robotIndex = robotIndex;
		this.linkIndex = linkIndex;
		this.sensorId = sensorId;
		this.tcp = tcp;
	}

	public function afterSimulationStep(sourceTimestampNs:Int64):Void {
		// Timestamp the geometry observation on the robot's own clock. The
		// session callback time can stay monotonic across a physics reset while
		// the robot starts a new source-clock epoch at zero.
		var jointTime = runtime.jointObservation().sourceTimestampNs;
		var link = simulation.linkPose(robotIndex, linkIndex);
		var worldLink = new Transform3(new Vec3(link.position[0], link.position[1], link.position[2]),
			new Quat(link.rotation[0], link.rotation[1], link.rotation[2], link.rotation[3]));
		var worldTcp = worldLink.compose(tcp);
		var touching = false;
		var contacts = simulation.robotContacts(runtime);
		for (contact in contacts) {
			if (contact != null && contact.active && contact.linkIndex == linkIndex &&
				(contact.otherKind == RobotContactOtherKind.Object || contact.otherKind == RobotContactOtherKind.RobotLink) &&
				contact.position != null) {
				var hit = nearTip(contact, worldTcp);
				if (hit) touching = true;
			}
		}
		nextSequence = Int64.add(nextSequence, Int64.ofInt(1));
		runtime.publishSensorFrame(sensorId, [touching ? 1.0 : 0.0], nextSequence,
			jointTime, runtime.sourceClockId);
	}

	function nearTip(contact:RobotContact, worldTcp:Transform3):Bool {
		if (contact == null || !contact.active || contact.linkIndex != linkIndex ||
			(contact.otherKind != RobotContactOtherKind.Object && contact.otherKind != RobotContactOtherKind.RobotLink) ||
			contact.position == null) return false;
		var point = worldTcp.inverse().transformPoint(contact.position);
		// The CAD bit's front profile is at the declared TCP. Side contact on its
		// shank or sleeve must not masquerade as a touch-probe observation.
		var radius = processkit.tool.ToolContactProfile.FRONT_RADIUS_METERS;
		return point.z >= -0.001 && point.z <= 0.001 && point.x * point.x + point.y * point.y <= radius * radius;
	}

	public function reset():Void {
		// RobotRuntime retains the last published frame across a physics reset,
		// so this source's sequence must remain monotonic for the runtime's life.
	}
}
