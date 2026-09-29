package nativekit.sim;

import NativeKitSim;

/** One environment body owned by a session. */
class SimObject {
    final session:SimSession;
    public final handle:nksim_object;
    public final motion:MotionType;
    /** The collision shape the object was created with. */
    public final shape:SimShape;

    @:allow(SimSession)
    private function new(session:SimSession, handle:nksim_object, motion:MotionType, shape:SimShape) {
        this.session = session;
        this.handle = handle;
        this.motion = motion;
        this.shape = shape;
    }

    /** Moves a kinematic object for the next tick, while stopped or running. */
    public function drive(pose:SimPose):Void
        SimWorld.check(NativeKitSim.nksim_session_drive_object(session.nativeHandle(), handle,
            pose.toNative()), "object.drive");

    /** Moves the object at rest while stopped; the pose becomes its reset pose. */
    public function teleport(pose:SimPose):Void
        SimWorld.check(NativeKitSim.nksim_session_teleport_object(session.nativeHandle(), handle,
            pose.toNative()), "object.teleport");

    /** Removes the object while stopped. */
    public function dispose():Void
        SimWorld.check(NativeKitSim.nksim_session_destroy_object(session.nativeHandle(), handle),
            "object.dispose");
}
