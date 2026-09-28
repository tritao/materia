package nativekit.sim;

import NativeKitSim;

/** Immutable view of a session's bodies and clock after one tick. */
class SimFrame {
    final owner:Ownednksim_frame;
    var disposed:Bool = false;

    @:allow(SimSession)
    private function new(owner:Ownednksim_frame) {
        this.owner = owner;
    }

    public function nativeHandle():nksim_frame {
        ensureLive();
        return owner.borrow();
    }

    public function stepIndex():haxe.Int64
        return clock().get_step_index();

    public function simulationTime():Float
        return clock().get_time();

    public function objectPose(object:SimObject):SimPose {
        ensureLive();
        var pose = new nksim_pose();
        pose.set_struct_size(nksim_pose.size());
        SimWorld.check(NativeKitSim.nksim_frame_get_object_pose(owner.borrow(), object.handle,
            pose).status, "frame.objectPose");
        return SimPose.fromNative(pose);
    }

    public function actorPose(actor:SimActor, part:Int):SimPose {
        ensureLive();
        var pose = new nksim_pose();
        pose.set_struct_size(nksim_pose.size());
        SimWorld.check(NativeKitSim.nksim_frame_get_actor_pose(owner.borrow(), actor.handle,
            part, pose).status, "frame.actorPose");
        return SimPose.fromNative(pose);
    }

    public function dispose():Void {
        if (disposed)
            return;
        owner.close();
        disposed = true;
    }

    function clock():nksim_clock {
        ensureLive();
        var value = new nksim_clock();
        value.set_struct_size(nksim_clock.size());
        SimWorld.check(NativeKitSim.nksim_frame_get_clock(owner.borrow(), value).status,
            "frame.clock");
        return value;
    }

    function ensureLive():Void {
        if (disposed)
            throw "Simulation frame has been disposed";
    }
}
