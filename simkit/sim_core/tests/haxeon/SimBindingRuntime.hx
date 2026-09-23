import nativekit.scene.Scene;
import nativekit.scene.Transform;
import nativekit.sim.MotionType;
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
        return 42;
    }
}
