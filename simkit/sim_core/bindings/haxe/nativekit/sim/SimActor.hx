package nativekit.sim;

import NativeKitSim;

/**
 * A group of kinematic parts a session moves along a timed trajectory. They
 * push dynamic bodies in their way and nothing pushes them back.
 */
class SimActor {
    final session:SimSession;
    public final handle:nksim_actor;
    public final partCount:Int;

    @:allow(SimSession)
    private function new(session:SimSession, handle:nksim_actor, partCount:Int) {
        this.session = session;
        this.handle = handle;
        this.partCount = partCount;
    }

    /**
     * Every part's pose at simulation time `time`. Ticks interpolate between
     * keyframes and hold the last one; a keyframe at or before an existing
     * one replaces it and every later one.
     */
    public function pushKeyframe(time:Float, poses:Array<SimPose>):Void {
        if (poses.length != partCount)
            throw "Actor keyframe needs one pose per part";
        var values:Array<nksim_pose> = [for (pose in poses) pose.toNative()];
        SimWorld.check(NativeKitSim.nksim_session_push_actor_keyframe(session.nativeHandle(),
            handle, time, values), "actor.pushKeyframe");
    }

    /** Removes the actor while stopped. */
    public function dispose():Void
        SimWorld.check(NativeKitSim.nksim_session_destroy_actor(session.nativeHandle(), handle),
            "actor.dispose");
}
