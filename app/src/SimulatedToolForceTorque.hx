package app;

import haxe.Int64;
import robotkit.runtime.RobotRuntime;
import robotkit.runtime.Simulation;
import robotkit.runtime.SimulationStepObserver;
import robotkit.runtime.RobotContact;
import robotkit.spatial.Transform3;
import robotkit.spatial.Vec3;
import robotkit.spatial.Quat;
import robotkit.spatial.Wrench3;

/** Publishes the MuJoCo contact wrench transmitted through the sensor's mounted robot link. */
class SimulatedToolForceTorque implements SimulationStepObserver {
	final simulation:Simulation;
	final runtime:RobotRuntime;
	final robotIndex:Int;
	final sensorLinkIndex:Int;
	final contactLinks:Map<Int, Bool> = new Map();
	final sensorId:String;
	final linkFromSensor:Transform3;
	var nextSequence:Int64 = Int64.ofInt(0);
	final contactSequences:Array<Int64> = [];
	final contactSamples:Array<Array<RobotContact>> = [];
	final wrenchSamples:Array<Wrench3> = [];

	public function new(simulation:Simulation, runtime:RobotRuntime, robotIndex:Int, sensorLinkIndex:Int,
		contactLinks:Array<Int>, sensorId:String, linkFromSensor:Transform3) {
		if (simulation == null || !simulation.supportsContactWrenches() || runtime == null ||
			!simulation.ownsRobot(runtime) || robotIndex < 0 || sensorLinkIndex < 0 ||
			contactLinks == null || contactLinks.length == 0 || sensorId == null || sensorId.length == 0 || linkFromSensor == null)
			throw "Simulated force/torque requires MuJoCo, its mounted robot link and sensor frame";
		this.simulation = simulation;
		this.runtime = runtime;
		this.robotIndex = robotIndex;
		this.sensorLinkIndex = sensorLinkIndex;
		for (link in contactLinks) {
			if (link < 0 || this.contactLinks.exists(link)) throw "Simulated force/torque has invalid or duplicate contact links";
			this.contactLinks.set(link, true);
		}
		this.sensorId = sensorId;
		this.linkFromSensor = linkFromSensor;
	}

	public function afterSimulationStep(sourceTimestampNs:Int64):Void {
		// The contact wrench belongs to this physics tick. Use the robot's
		// sampled time so it stays in the same epoch as the joint observation.
		var jointTime = runtime.jointObservation().sourceTimestampNs;
		var link = simulation.linkPose(robotIndex, sensorLinkIndex);
		var worldLink = new Transform3(new Vec3(link.position[0], link.position[1], link.position[2]),
			new Quat(link.rotation[0], link.rotation[1], link.rotation[2], link.rotation[3]));
		var worldSensor = worldLink.compose(linkFromSensor);
		var forceWorld = Vec3.zero();
		var torqueWorldOrigin = Vec3.zero();
		var supported = true;
		var contacts = simulation.robotContacts(runtime);
		for (contact in contacts) {
			if (!contactLinks.exists(contact.linkIndex) || !contact.active) continue;
			if (!contact.wrenchValid) { supported = false; break; }
			forceWorld = forceWorld.add(contact.force);
			torqueWorldOrigin = torqueWorldOrigin.add(contact.torque).add(contact.position.cross(contact.force));
		}
		nextSequence = Int64.add(nextSequence, Int64.ofInt(1));
		if (Int64.compare(nextSequence, Int64.ofInt(0)) <= 0) throw 'Force/torque sensor "$sensorId" sequence exhausted';
		// A missing wrench on any active contact invalidates the whole observation;
		// publishing a partial sum would turn unsupported physics into a false reading.
		if (!supported) return;
		var wrench = worldSensor.inverse().transformWrench(new Wrench3(forceWorld, torqueWorldOrigin));
		contactSequences.push(nextSequence);
		contactSamples.push(contacts);
		wrenchSamples.push(wrench);
		if (contactSequences.length > 4) {
			contactSequences.shift();
			contactSamples.shift();
			wrenchSamples.shift();
		}
		runtime.publishSensorFrame(sensorId,
			[wrench.force.x, wrench.force.y, wrench.force.z, wrench.torque.x, wrench.torque.y, wrench.torque.z],
			nextSequence, jointTime, runtime.sourceClockId);
	}

	/** Contact snapshot that produced one recent force/torque acquisition. */
	public function contactsForSequence(sequence:Int64):Null<Array<RobotContact>> {
		for (index in 0...contactSequences.length)
			if (Int64.compare(contactSequences[index], sequence) == 0) return contactSamples[index].copy();
		return null;
	}

	/** Exact wrench computed from the contacts published in one recent sensor frame. */
	public function wrenchForSequence(sequence:Int64):Null<Wrench3> {
		for (index in 0...contactSequences.length)
			if (Int64.compare(contactSequences[index], sequence) == 0) return wrenchSamples[index];
		return null;
	}

	public function reset():Void {
		// RobotRuntime retains the last published frame across a physics reset,
		// so this source's sequence must remain monotonic for the runtime's life.
	}
}
