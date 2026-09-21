package nativekit.sim;

import NativeKitSim;
import NativeKitSimMujoco;
import nativekit.scene.Occurrence;
import nativekit.scene.Scene;

/** Creates the engine-neutral simulation façade with the MuJoCo backend. */
class MujocoSimWorld {
    final world:SimWorld;

    private function new(world:SimWorld) {
        this.world = world;
    }

    public static function create(scene:Scene, ?options:SimWorldOptions):MujocoSimWorld {
        var desc = SimWorld.makeDescription(scene, options);
        var result = NativeKitSimMujoco.nksim_mujoco_world_create(desc);
        SimWorld.check(result.status, "mujocoWorld.create");
        return new MujocoSimWorld(SimWorld.fromNativeOwner(scene, result.out_world));
    }

    public function nativeHandle():nksim_world
        return world.nativeHandle();

    public function createShapeBox(x:Float, y:Float, z:Float):Shape
        return world.createShapeBox(x, y, z);

    public function createShapeSphere(radius:Float):Shape
        return world.createShapeSphere(radius);

    public function createShapeCapsule(radius:Float, height:Float):Shape
        return world.createShapeCapsule(radius, height);

    public function createShapePlane(normalX:Float, normalY:Float,
            normalZ:Float, offset:Float):Shape
        return world.createShapePlane(normalX, normalY, normalZ, offset);

    public function createBody(occurrence:Occurrence, motion:MotionType,
            mass:Float, ?shape:Shape):Body
        return world.createBody(occurrence, motion, mass, shape);

    public function createJoint(type:JointType, bodyA:Body, bodyB:Body):Joint
        return world.createJoint(type, bodyA, bodyB);

    public function step():StepResult
        return world.step();

    public function snapshot():SimSnapshot
        return world.snapshot();

    public function dispose():Void
        world.dispose();

    public function isDisposed():Bool
        return world.isDisposed();
}
