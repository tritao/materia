package humankit.sim;

import humankit.CapsulePlacement;
import humankit.HumanBodyProxy;
import humankit.rig.HumanPose;
import nativekit.sim.SimActor;
import nativekit.sim.SimPose;
import nativekit.sim.SimSession;
import nativekit.sim.SimShape;

/**
 * A person taking part in a SimKit session: the body proxy's capsules as one
 * kinematic actor. The person pushes whatever they walk into and nothing
 * pushes them back. Poses are keyframes in simulation time: a writer pushes
 * one per tick, for the tick about to complete.
 */
class HumanActor {
	public final actor:SimActor;
	public final proxy:HumanBodyProxy;

	/** Joins a stopped session with the body at a pose, through a root transform. */
	public function new(session:SimSession, proxy:HumanBodyProxy, pose:HumanPose, root:Array<Float>) {
		this.proxy = proxy;
		actor = session.createActor([for (capsule in proxy.capsules) SimShape.capsule(capsule.radius, capsule.length)],
			simPoses(proxy.place(pose, root)));
	}

	/** Where the body is at simulation time `time`. */
	public function pushPose(time:Float, pose:HumanPose, root:Array<Float>):Void
		actor.pushKeyframe(time, simPoses(proxy.place(pose, root)));

	public function dispose():Void
		actor.dispose();

	static function simPoses(placements:Array<CapsulePlacement>):Array<SimPose>
		return [
			for (placement in placements)
				new SimPose(placement.center[0], placement.center[1], placement.center[2], placement.rotation[0],
					placement.rotation[1], placement.rotation[2], placement.rotation[3])
		];
}
