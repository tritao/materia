package nativekit.sim;

import NativeKitScene;
import NativeKitSim;
import nativekit.scene.Scene;

/**
 * One shared simulated space around a borrowed world and scene: the fixed
 * clock, stepping, realtime ticking, and the objects and actors every
 * participant shares. Dispose it before the world and scene.
 */
class SimSession {
    final owner:Ownednksim_session;
    final stepObservers:Array<{id:Int, callback:Void->Void}> = [];
    var nextStepObserverId:Int = 1;
    var disposed:Bool = false;

    public function new(scene:Scene, world:nksim_world) {
        var desc = new nksim_session_desc();
        desc.set_struct_size(nksim_session_desc.size());
        desc.set_scene(scene.nativeHandle());
        desc.set_world(world);
        var result = NativeKitSim.nksim_session_create(desc);
        SimWorld.check(result.status, "session.create");
        owner = result.out_session;
    }

    public function nativeHandle():nksim_session {
        ensureLive();
        return owner.borrow();
    }

    /** Advances exactly one fixed tick and returns the new simulation time. */
    public function step(?ownerTimeNs:haxe.Int64):Float {
        ensureLive();
        var clock = new nksim_clock();
        clock.set_struct_size(nksim_clock.size());
        var result = NativeKitSim.nksim_session_step(owner.borrow(),
            ownerTimeNs == null ? haxe.Int64.ofInt(0) : ownerTimeNs, clock);
        SimWorld.check(result.status, "session.step");
        for (entry in stepObservers.copy()) {
            var registered = false;
            for (current in stepObservers)
                if (current.id == entry.id) registered = true;
            if (registered) entry.callback();
        }
        return clock.get_time();
    }

    /** Runs after each successful explicit step, once the shared contacts are available. */
    public function addStepObserver(callback:Void->Void):Int {
        ensureLive();
        if (callback == null) throw "Simulation session step observer is required";
        var id = nextStepObserverId++;
        stepObservers.push({id: id, callback: callback});
        return id;
    }

    public function removeStepObserver(id:Int):Void {
        for (index in 0...stepObservers.length)
            if (stepObservers[index].id == id) {
                stepObservers.splice(index, 1);
                return;
            }
    }

    public function start():Void {
        ensureLive();
        SimWorld.check(NativeKitSim.nksim_session_start(owner.borrow()), "session.start");
    }

    public function stop():Void {
        ensureLive();
        SimWorld.check(NativeKitSim.nksim_session_stop(owner.borrow()), "session.stop");
    }

    public function reset():Void {
        ensureLive();
        SimWorld.check(NativeKitSim.nksim_session_reset(owner.borrow()), "session.reset");
    }

    public function isRunning():Bool
        return status().get_running() != 0;

    public function isSealed():Bool
        return status().get_sealed() != 0;

    public function stepIndex():haxe.Int64
        return status().get_step_index();

    public function simulationTime():Float
        return status().get_simulation_time();

    public function fixedTimestep():Float
        return status().get_fixed_timestep();

    public function createObject(motion:MotionType, shape:SimShape, pose:SimPose,
            mass:Float = 0.0):SimObject {
        ensureLive();
        var desc = new nksim_object_desc();
        desc.set_struct_size(nksim_object_desc.size());
        desc.set_motion_type(motion);
        desc.set_shape(shape.toNative());
        desc.set_pose(pose.toNative());
        desc.set_mass(mass);
        var result = NativeKitSim.nksim_session_create_object(owner.borrow(), desc);
        SimWorld.check(result.status, "session.createObject");
        return new SimObject(this, result.out_object, motion, shape);
    }

    /** Drive a dynamic object from a carrier body's frame on every tick. */
    public function holdObject(object:SimObject, carrier:nksim_body, ?offset:SimPose):Void {
        ensureLive();
        var local = offset == null ? new SimPose(0, 0, 0) : offset;
        SimWorld.check(NativeKitSim.nksim_session_hold_object(owner.borrow(), object.handle,
            carrier, local.toNative()), "session.holdObject");
    }

    public function releaseObject(object:SimObject):Void {
        ensureLive();
        SimWorld.check(NativeKitSim.nksim_session_release_object(owner.borrow(), object.handle),
            "session.releaseObject");
    }

    /** Zero means the object is free. */
    public function objectCarrier(object:SimObject):nksim_body {
        ensureLive();
        var result = NativeKitSim.nksim_session_get_object_carrier(owner.borrow(), object.handle);
        SimWorld.check(result.status, "session.objectCarrier");
        return result.out_carrier;
    }

    /** Adds a group of kinematic parts, each starting at its pose. */
    public function createActor(shapes:Array<SimShape>, poses:Array<SimPose>):SimActor {
        ensureLive();
        if (shapes.length != poses.length)
            throw "Simulation actor needs one pose per shape";
        var parts:Array<nksim_actor_part> = [];
        for (index in 0...shapes.length) {
            var part = new nksim_actor_part();
            part.set_struct_size(nksim_actor_part.size());
            part.set_shape(shapes[index].toNative());
            part.set_pose(poses[index].toNative());
            parts.push(part);
        }
        var result = NativeKitSim.nksim_session_create_actor(owner.borrow(), parts);
        SimWorld.check(result.status, "session.createActor");
        return new SimActor(this, result.out_actor, shapes.length);
    }

    /** Distance along a unit direction to the nearest object or actor, up to maxDistance. */
    public function raycast(origin:SimPose, dirX:Float, dirY:Float, dirZ:Float,
            maxDistance:Float):Float {
        ensureLive();
        var ray = new nksim_ray();
        ray.set_struct_size(nksim_ray.size());
        ray.set_origin(0, origin.x);
        ray.set_origin(1, origin.y);
        ray.set_origin(2, origin.z);
        ray.set_direction(0, dirX);
        ray.set_direction(1, dirY);
        ray.set_direction(2, dirZ);
        ray.set_max_distance(maxDistance);
        var result = NativeKitSim.nksim_session_raycast(owner.borrow(), ray);
        SimWorld.check(result.status, "session.raycast");
        return result.out_distance;
    }

    /** Captures a consistent view of every body after the latest tick. */
    public function capture():SimFrame {
        ensureLive();
        var result = NativeKitSim.nksim_session_capture(owner.borrow());
        SimWorld.check(result.status, "session.capture");
        return new SimFrame(result.out_frame);
    }

    public function dispose():Void {
        if (disposed)
            return;
        stepObservers.resize(0);
        owner.close();
        disposed = true;
    }

    public function isDisposed():Bool
        return disposed;

    function status():nksim_session_status {
        ensureLive();
        var value = new nksim_session_status();
        value.set_struct_size(nksim_session_status.size());
        SimWorld.check(NativeKitSim.nksim_session_get_status(owner.borrow(), value).status,
            "session.status");
        return value;
    }

    @:allow(SimObject, SimActor)
    function ensureLive():Void {
        if (disposed)
            throw "Simulation session has been disposed";
    }
}
