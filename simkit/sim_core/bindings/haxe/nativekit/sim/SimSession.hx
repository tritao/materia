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
    final world:nksim_world;
    final stepObservers:Array<{id:Int, callback:Void->Void}> = [];
    final resetObservers:Array<{id:Int, callback:Void->Void}> = [];
    var nextStepObserverId:Int = 1;
    var disposed:Bool = false;

    public function new(scene:Scene, world:nksim_world) {
        this.world = world;
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
        notifyStepObservers();
        return clock.get_time();
    }

    function notifyStepObservers():Void {
        for (entry in stepObservers.copy()) {
            var registered = false;
            for (current in stepObservers)
                if (current.id == entry.id) registered = true;
            if (registered) entry.callback();
        }
    }

    /**
     * Owner-paced realtime: adds `elapsedNs` of wall time and returns how many
     * fixed ticks are due, at most `maxTicks` (the excess is dropped, so a slow
     * owner slows simulation time). Run each with `stepPaced`, acting on the
     * session between them.
     */
    public function dueTicks(elapsedNs:haxe.Int64, maxTicks:Int):Int {
        ensureLive();
        var result = NativeKitSim.nksim_session_due_ticks(owner.borrow(), elapsedNs, maxTicks);
        SimWorld.check(result.status, "session.dueTicks");
        return result.out_ticks;
    }

    /** Like `step`, but participants see a realtime tick on the native monotonic clock. */
    public function stepPaced():Float {
        ensureLive();
        var clock = new nksim_clock();
        clock.set_struct_size(nksim_clock.size());
        var result = NativeKitSim.nksim_session_step_paced(owner.borrow(), clock);
        SimWorld.check(result.status, "session.stepPaced");
        notifyStepObservers();
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
        for (entry in resetObservers.copy()) {
            var registered = false;
            for (current in resetObservers) if (current.id == entry.id) registered = true;
            if (registered) entry.callback();
        }
    }

    /** Runs after a successful reset. Clock owners invalidate the preceding observation epoch. */
    public function addResetObserver(callback:Void->Void):Int {
        ensureLive();
        if (callback == null) throw "Simulation session reset observer is required";
        var id = nextStepObserverId++;
        resetObservers.push({id:id, callback:callback});
        return id;
    }

    public function removeResetObserver(id:Int):Void {
        for (index in 0...resetObservers.length) if (resetObservers[index].id == id) {
            resetObservers.splice(index, 1);
            return;
        }
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

    /** Batch topology changes after stop(), preserving the shared clock and other participants. */
    public function beginTopologyUpdate():Void {
        ensureLive();
        if (isRunning()) throw "Stop the session before changing topology";
        SimWorld.check(NativeKitSim.nksim_world_begin_topology_update(world), "session.beginTopologyUpdate");
    }
    public function endTopologyUpdate():Void {
        ensureLive();
        SimWorld.check(NativeKitSim.nksim_world_end_topology_update(world), "session.endTopologyUpdate");
    }

    public function createObject(motion:MotionType, shape:SimShape, pose:SimPose,
            mass:Float = 0.0, ?restContactGroup:haxe.Int64,
            ?surface:SimContactSurface, ?inertial:SimInertialProperties):SimObject {
        ensureLive();
        var desc = new nksim_object_desc();
        desc.set_struct_size(nksim_object_desc.size());
        desc.set_motion_type(motion);
        desc.set_shape(shape.toNative());
        desc.set_pose(pose.toNative());
        desc.set_mass(mass);
        if (inertial != null) inertial.writeNative(desc);
        if (restContactGroup != null) desc.set_rest_contact_group(restContactGroup);
        var nativeSurface = new nksim_surface();
        nativeSurface.set_struct_size(nksim_surface.size());
        nativeSurface.set_contact_filter(NativeKitSimConstants.NKSIM_CONTACT_LAYERS);
        if (surface != null) surface.writeNative(nativeSurface);
        desc.set_surface(nativeSurface);
        var handle:nksim_object;
        if (shape.type == ShapeType.Compound) {
            var result = NativeKitSim.nksim_session_create_compound_convex_object(owner.borrow(), desc,
                shape.convexVertices(), shape.convexPieceCounts());
            SimWorld.check(result.status, "session.createCompoundConvexObject");
            handle = result.out_object;
        } else if (shape.type == ShapeType.Convex) {
            var result = NativeKitSim.nksim_session_create_convex_object(owner.borrow(), desc, shape.convexVertices());
            SimWorld.check(result.status, "session.createConvexObject");
            handle = result.out_object;
        } else {
            var result = NativeKitSim.nksim_session_create_object(owner.borrow(), desc);
            SimWorld.check(result.status, "session.createObject");
            handle = result.out_object;
        }
        return new SimObject(this, handle, motion, shape);
    }

    /** Rigidly connect objects using their creation frames; the parent retains its motion. */
    public function createFixedJoint(parent:SimObject, child:SimObject):SimFixedJoint {
        ensureLive();
        if (parent == null || child == null || parent.session != this || child.session != this)
            throw "A fixed connection needs two objects from this session";
        var result = NativeKitSim.nksim_session_create_fixed_joint(owner.borrow(), parent.handle, child.handle);
        SimWorld.check(result.status, "session.createFixedJoint");
        return new SimFixedJoint(this, result.out_joint);
    }

    /** Tests one free dynamic object against an oriented query box. */
    @:allow(SimObject)
    private function objectOverlapsVolume(object:SimObject, pose:SimPose,
            halfExtents:Array<Float>):Bool {
        ensureLive();
        if (object.session != this || halfExtents == null || halfExtents.length != 3)
            throw "Object volume query needs this session's object and three half extents";
        var result = NativeKitSim.nksim_session_object_overlaps_volume(owner.borrow(), object.handle,
            pose.toNative(), halfExtents[0], halfExtents[1], halfExtents[2]);
        SimWorld.check(result.status, "session.objectOverlapsVolume");
        return result.out_overlaps != 0;
    }

    /** Updates a dynamic object's mass while preserving its current state. */
    @:allow(SimObject)
    private function setObjectMass(object:SimObject, mass:Float):Void {
        ensureLive();
        if (object.session != this || object.motion != MotionType.Dynamic ||
                !Math.isFinite(mass) || mass <= 0.0)
            throw "Object mass update needs this session's dynamic object and a positive finite mass";
        SimWorld.check(NativeKitSim.nksim_session_set_object_mass(owner.borrow(), object.handle, mass),
            "session.setObjectMass");
    }

    /** Excludes contacts between two simulation bodies before this session's first tick. */
    public function excludeContactPair(first:nksim_body, second:nksim_body):Void {
        ensureLive();
        SimWorld.check(NativeKitSim.nksim_session_exclude_body_pair(owner.borrow(), first, second),
            "session.excludeContactPair");
    }

    /** Excludes a robot body from a session-owned object without exposing the object's body handle. */
    public function excludeObjectContactPair(body:nksim_body, object:SimObject):Void {
        ensureLive();
        if (object == null || object.session != this)
            throw "Contact exclusion needs this session's object";
        var found = NativeKitSim.nksim_session_get_object_body(owner.borrow(), object.handle);
        SimWorld.check(found.status, "session.objectBodyForContactExclusion");
        excludeContactPair(body, found.out_body);
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

    /** Read this object's measured world pose, including while stopped. */
    public function objectPose(object:SimObject):SimPose {
        ensureLive();
        if (object == null || object.session != this) throw "Pose observation needs this session's object";
        var found = NativeKitSim.nksim_session_get_object_body(owner.borrow(), object.handle);
        SimWorld.check(found.status, "session.objectPoseBody");
        return bodyPose(found.out_body);
    }

    /** Read a carrier's measured world pose from this shared world. */
    public function bodyPose(body:nksim_body):SimPose {
        ensureLive();
        var pose = new nksim_pose(); pose.set_struct_size(nksim_pose.size());
        SimWorld.check(NativeKitSim.nksim_session_get_body_pose(owner.borrow(), body, pose).status, "session.bodyPose");
        return SimPose.fromNative(pose);
    }

    /** Hold one session object from another's measured frame without exposing a body handle. */
    public function holdObjectFromObject(object:SimObject, carrier:SimObject, offset:SimPose):Void {
        ensureLive();
        if (object == null || carrier == null || object.session != this || carrier.session != this)
            throw "Object hold needs two objects from this session";
        var found = NativeKitSim.nksim_session_get_object_body(owner.borrow(), carrier.handle);
        SimWorld.check(found.status, "session.objectCarrierBody");
        holdObject(object, found.out_body, offset);
    }

    /** Retains a dynamic part relative to an explicitly identified environment object. */
    public function holdObjectOnObject(object:SimObject, carrier:SimObject, offset:SimPose):Void {
        ensureLive();
        if (object == null || carrier == null || object == carrier || object.session != this || carrier.session != this)
            throw "Object holding needs two distinct objects from this session";
        holdObject(object, objectBody(carrier), offset);
    }

    /** The physics body of this session's explicitly identified environment object. */
    public function objectBody(object:SimObject):nksim_body {
        ensureLive();
        if (object == null || object.session != this) throw "Object body needs this session's object";
        var found = NativeKitSim.nksim_session_get_object_body(owner.borrow(), object.handle);
        SimWorld.check(found.status, "session.objectBody");
        return found.out_body;
    }

    /**
     * Places a free dynamic object at the start of the next physics tick and clears its velocity.
     * Unlike teleport(), this is safe while the session is hosted and does not change the reset pose.
     */
    @:allow(SimObject)
    private function setObjectPoseForNextTick(object:SimObject, pose:SimPose):Void {
        ensureLive();
        if (object.session != this || object.motion != MotionType.Dynamic)
            throw "Only this session's dynamic objects can be placed for the next tick";
        // Releasing a free object reports INVALID_STATE; a held object stages its release before
        // the body-state write, so both commands take effect in order on the next physics tick.
        var releaseStatus = NativeKitSim.nksim_session_release_object(owner.borrow(), object.handle);
        if (releaseStatus != NativeKitSimConstants.NKSIM_ERROR_INVALID_STATE)
            SimWorld.check(releaseStatus, "session.releaseObjectForPlacement");
        var found = NativeKitSim.nksim_session_get_object_body(owner.borrow(), object.handle);
        SimWorld.check(found.status, "session.objectBody");
        var state = new nksim_body_state();
        state.set_struct_size(nksim_body_state.size());
        state.set_body(found.out_body);
        state.set_position(0, pose.x); state.set_position(1, pose.y); state.set_position(2, pose.z);
        state.set_rotation(0, pose.qx); state.set_rotation(1, pose.qy);
        state.set_rotation(2, pose.qz); state.set_rotation(3, pose.qw);
        for (axis in 0...3) {
            state.set_linear_velocity(axis, 0.0);
            state.set_angular_velocity(axis, 0.0);
        }
        state.set_sleeping(0);
        SimWorld.check(NativeKitSim.nksim_session_set_body_states(owner.borrow(), [state]),
            "session.setObjectPoseForNextTick");
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
        resetObservers.resize(0);
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
