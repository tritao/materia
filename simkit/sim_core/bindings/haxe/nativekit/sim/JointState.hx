package nativekit.sim;

import NativeKitSim;

/** Immutable copy of one simulated joint's state. */
class JointState {
    public final joint:nksim_joint;
    public final position:Float;
    public final velocity:Float;
    public final effort:Float;

    private function new(value:nksim_joint_state) {
        joint = value.get_joint();
        position = value.get_position();
        velocity = value.get_velocity();
        effort = value.get_effort();
    }

    @:allow(Joint)
    static function fromNative(value:nksim_joint_state):JointState
        return new JointState(value);
}
