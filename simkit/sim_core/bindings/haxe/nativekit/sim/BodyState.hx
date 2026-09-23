package nativekit.sim;

import NativeKitSim;
import NativeKitScene;

/** Immutable copy of one simulated body's state. */
class BodyState {
    public final body:nksim_body;
    public final node:nkscene_node_id;
    public final x:Float;
    public final y:Float;
    public final z:Float;
    public final linearVelocityX:Float;
    public final linearVelocityY:Float;
    public final linearVelocityZ:Float;
    public final sleeping:Bool;

    private function new(value:nksim_body_state) {
        body = value.get_body();
        node = value.get_node();
        x = value.get_position(0);
        y = value.get_position(1);
        z = value.get_position(2);
        linearVelocityX = value.get_linear_velocity(0);
        linearVelocityY = value.get_linear_velocity(1);
        linearVelocityZ = value.get_linear_velocity(2);
        sleeping = value.get_sleeping() != 0;
    }

    @:allow(SimWorld, SimSnapshot)
    static function fromNative(value:nksim_body_state):BodyState
        return new BodyState(value);
}
