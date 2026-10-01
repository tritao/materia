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
    var pushScratch:Null<Array<nksim_pose>>;

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
        // Keyframes are pushed every tick; the native values are reused rather than allocated per part each time.
        if (pushScratch == null) {
            var made:Array<nksim_pose> = [];
            for (_ in 0...partCount) {
                var value = new nksim_pose();
                value.set_struct_size(nksim_pose.size());
                made.push(value);
            }
            pushScratch = made;
        }
        var values:Array<nksim_pose> = pushScratch;
        for (index in 0...partCount) poses[index].writeNative(values[index]);
        SimWorld.check(NativeKitSim.nksim_session_push_actor_keyframe(session.nativeHandle(),
            handle, time, values), "actor.pushKeyframe");
    }

    /** The session body for a capsule or other actor part. */
    public function partBody(index:Int):nksim_body {
        if (index < 0 || index >= partCount)
            throw "Actor part index is out of range";
        var result = NativeKitSim.nksim_session_get_actor_body(session.nativeHandle(), handle,
            index);
        SimWorld.check(result.status, "actor.partBody");
        return result.out_body;
    }

    /** Removes the actor while stopped. */
    public function dispose():Void
        SimWorld.check(NativeKitSim.nksim_session_destroy_actor(session.nativeHandle(), handle),
            "actor.dispose");
}
