import nativekit.scene.Scene;
import nativekit.scene.Transform;
import nativekit.sim.JointType;
import nativekit.sim.MujocoSimWorld;
import nativekit.sim.MotionType;

/** Runtime smoke for the typed MuJoCo world façade and shared Sim API. */
class MujocoBindingRuntime {
    static function main():Int {
        var scene = Scene.create();
        var transaction = scene.beginTransaction();
        var floorNode = transaction.createNode();
        var node = transaction.createNode();
        transaction.setTransform(floorNode, Transform.identity());
        transaction.setTransform(node, Transform.identity().translated(0.0, 0.0, 2.0));
        var initialChanges = transaction.commitWithChanges();
        initialChanges.dispose();

        var world = MujocoSimWorld.create(scene, {
            timestep: 0.01,
            physicsSubsteps: 2,
            gravity: [0.0, 0.0, -9.81]
        });
        var floorShape = world.createShapePlane(0.0, 0.0, 1.0, 0.0);
        var shape = world.createShapeBox(0.25, 0.25, 0.25);
        var floor = world.createBody(floorNode, MotionType.Static, 0.0, floorShape);
        var body = world.createBody(node, MotionType.Dynamic, 1.0, shape);
        for (index in 0...300) {
            var step = world.step();
            if (index == 0 && (haxe.Int64.toInt(step.stepIndex) != 1 ||
                    step.physicsSubsteps != 2 || step.simulationTime <= 0.0))
                return 1;
            step.sceneChanges.dispose();
        }

        var state = body.state();
        if (state.z <= 0.1 || state.z >= 0.6 || Math.abs(state.linearVelocityZ) >= 0.2)
            return 2;
        var snapshot = world.snapshot();
        if (snapshot.bodyCount() != 2)
            return 3;
        snapshot.dispose();

        body.dispose();
        floor.dispose();
        shape.dispose();
        floorShape.dispose();
        world.dispose();

        var jointTransaction = scene.beginTransaction();
        var baseNode = jointTransaction.createNode();
        var armNode = jointTransaction.createNode();
        jointTransaction.setTransform(baseNode, Transform.identity());
        jointTransaction.setTransform(armNode,
            Transform.identity().translated(1.0, 0.0, 0.0));
        var jointChanges = jointTransaction.commitWithChanges();
        jointChanges.dispose();

        var jointWorld = MujocoSimWorld.create(scene, {
            timestep: 0.01,
            physicsSubsteps: 1,
            gravity: [0.0, 0.0, 0.0]
        });
        var armShape = jointWorld.createShapeBox(0.1, 0.1, 0.5);
        var base = jointWorld.createBody(baseNode, MotionType.Static, 0.0);
        var arm = jointWorld.createBody(armNode, MotionType.Dynamic, 1.0, armShape);
        var joint = jointWorld.createJoint(JointType.Revolute, base, arm);
        joint.setTargetPosition(0.2, 20.0);
        for (index in 0...20) {
            var step = jointWorld.step();
            step.sceneChanges.dispose();
        }
        if (joint.state().position <= 0.0)
            return 4;

        joint.dispose();
        arm.dispose();
        base.dispose();
        armShape.dispose();
        jointWorld.dispose();
        scene.dispose();
        return 42;
    }
}
