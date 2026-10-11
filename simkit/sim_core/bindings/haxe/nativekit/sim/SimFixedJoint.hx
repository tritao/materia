package nativekit.sim;

import NativeKitSim;

/** A removable rigid connection between two objects, owned by their session. */
class SimFixedJoint {
    final session:SimSession;
    final handle:nksim_joint;

    @:allow(SimSession)
    private function new(session:SimSession, handle:nksim_joint) {
        this.session = session;
        this.handle = handle;
    }

    /** Remove the connection while stopped, before removing either object. */
    public function dispose():Void
        SimWorld.check(NativeKitSim.nksim_session_destroy_fixed_joint(session.nativeHandle(), handle),
            "session.destroyFixedJoint");
}
