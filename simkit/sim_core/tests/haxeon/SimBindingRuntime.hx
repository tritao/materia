import nativekit.scene.Scene;
import nativekit.scene.Transform;
import nativekit.sim.MotionType;
import nativekit.sim.SimPose;
import nativekit.sim.SimSession;
import nativekit.sim.SimShape;
import nativekit.sim.SimWorld;

/** Runtime smoke for scene ownership, simulation stepping, and Haxe resource cleanup. */
class SimBindingRuntime {
    static function main():Int {
        var scene = Scene.create();
        var transaction = scene.beginTransaction();
        var node = transaction.createNode();
        transaction.setTransform(node, Transform.identity().translated(0.0, 0.0, 10.0));
        var initialChanges = transaction.commitWithChanges();
        initialChanges.dispose();

        var world = new SimWorld(scene, {
            timestep: 0.1,
            physicsSubsteps: 2,
            gravity: [0.0, 0.0, -9.81]
        });
        var body = world.createBody(node, MotionType.Dynamic, 1.0);
        var step = world.step();
        if (haxe.Int64.toInt(step.stepIndex) != 1 || step.physicsSubsteps != 2)
            return 1;
        if (step.simulationTime <= 0.0)
            return 2;
        if (haxe.Int64.toInt(step.sceneChanges.revision()) < 2)
            return 3;
        step.sceneChanges.dispose();

        var state = body.state();
        if (state.z >= 10.0 || state.linearVelocityZ >= 0.0)
            return 4;
        var snapshot = world.snapshot();
        if (snapshot.bodyCount() != 1)
            return 5;
        var snapshotState = snapshot.bodyAt(0);
        if (snapshotState.z != state.z)
            return 6;
        snapshot.dispose();

        body.dispose();
        world.dispose();
        scene.dispose();
        return sessionRuntime();
    }

    /** A shared session with a falling object and a keyframed actor. */
    static function sessionRuntime():Int {
        var scene = Scene.create();
        var world = new SimWorld(scene, {timestep: 0.01, gravity: [0.0, 0.0, -9.81]});
        var session = new SimSession(scene, world.nativeHandle());
        var crate = session.createObject(MotionType.Dynamic, SimShape.box(0.2, 0.2, 0.2),
            new SimPose(3.0, 0.0, 5.0), 1.0);
        var walker = session.createActor([SimShape.capsule(0.15, 0.8)],
            [new SimPose(0.0, 0.0, 0.6)]);
        walker.pushKeyframe(0.0, [new SimPose(0.0, 0.0, 0.6)]);
        walker.pushKeyframe(1.0, [new SimPose(1.0, 0.0, 0.6)]);
        for (_ in 0...50)
            session.step();
        if (haxe.Int64.toInt(session.stepIndex()) != 50 || Math.abs(session.simulationTime() - 0.5) > 1e-9)
            return 10;
        var frame = session.capture();
        var walked = frame.actorPose(walker, 0);
        if (Math.abs(walked.x - 0.5) > 1e-6 || Math.abs(walked.z - 0.6) > 1e-6)
            return 11;
        if (frame.objectPose(crate).z >= 5.0)
            return 12;
        frame.dispose();
        var hit = session.raycast(new SimPose(-1.0, 0.0, 0.6), 1.0, 0.0, 0.0, 10.0);
        if (Math.abs(hit - 1.35) > 1e-6)
            return 13;
        session.stop();
        session.reset();
        frame = session.capture();
        if (Math.abs(frame.actorPose(walker, 0).x) > 1e-9 || frame.objectPose(crate).z != 5.0)
            return 14;
        frame.dispose();
        session.dispose();
        world.dispose();
        scene.dispose();
        return 42;
    }
}
