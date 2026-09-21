package nativekit.sim;

import NativeKitSim;

/** Owns one simulation joint handle. */
class Joint {
    final world:SimWorld;
    final value:nksim_joint;
    var disposed:Bool = false;

    @:allow(SimWorld)
    private function new(world:SimWorld, value:nksim_joint) {
        this.world = world;
        this.value = value;
    }

    public function nativeHandle():nksim_joint {
        ensureLive();
        return value;
    }

    public function state():JointState {
        ensureLive();
        var state = new nksim_joint_state();
        state.set_struct_size(nksim_joint_state.size());
        var result = NativeKitSim.nksim_joint_get_state(world.nativeHandle(), value, state);
        SimWorld.check(result.status, "joint.state");
        return JointState.fromNative(state);
    }

    /** Stages a position command for the next world step. */
    public function setTargetPosition(target:Float, maxForce:Float = 0.0):Void {
        ensureLive();
        world.stageJointTarget(this, JointTargetMode.Position, target, maxForce);
    }

    /** Stages a velocity command for the next world step. */
    public function setTargetVelocity(target:Float, maxForce:Float = 0.0):Void {
        ensureLive();
        world.stageJointTarget(this, JointTargetMode.Velocity, target, maxForce);
    }

    /** Stages an effort command for the next world step. */
    public function setTargetEffort(target:Float, maxForce:Float = 0.0):Void {
        ensureLive();
        world.stageJointTarget(this, JointTargetMode.Effort, target, maxForce);
    }

    public function dispose():Void {
        if (disposed)
            return;
        NativeKitSim.nksim_joint_destroy(world.nativeHandle(), value);
        disposed = true;
    }

    public function isDisposed():Bool
        return disposed;

    function ensureLive():Void {
        if (disposed)
            throw "Simulation joint has been disposed";
        world.ensureLive();
    }
}
