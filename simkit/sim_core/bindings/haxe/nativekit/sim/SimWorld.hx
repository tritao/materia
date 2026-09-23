package nativekit.sim;

import NativeKitScene;
import NativeKitSim;
import nativekit.scene.Node;
import nativekit.scene.Scene;

typedef SimWorldOptions = {
    ?timestep:Float,
    ?physicsSubsteps:Int,
    ?gravity:Array<Float>
};

/** Thin typed Haxeon façade over an explicitly stepped simulation world. */
class SimWorld {
    final scene:Scene;
    final owner:Ownednksim_world;
    final resources:Array<Void->Void> = [];
    var pendingJointTargets:Array<nksim_joint_target> = [];
    var disposed:Bool = false;

    public function new(scene:Scene, ?options:SimWorldOptions,
            ?nativeOwner:Ownednksim_world) {
        this.scene = scene;
        if (nativeOwner != null) {
            owner = nativeOwner;
            return;
        }
        var desc = makeDescription(scene, options);
        var result = NativeKitSim.nksim_world_create(desc);
        check(result.status, "world.create");
        owner = result.out_world;
    }

    @:allow(MujocoSimWorld)
    static function makeDescription(scene:Scene, options:SimWorldOptions):nksim_world_desc {
        var desc = new nksim_world_desc();
        desc.set_struct_size(nksim_world_desc.size());
        desc.set_scene(scene.nativeHandle());
        desc.set_fixed_timestep(options != null && options.timestep != null ? options.timestep : 0.001);
        desc.set_physics_substeps(options != null && options.physicsSubsteps != null
            ? options.physicsSubsteps : 1);
        var gravity = options != null && options.gravity != null ? options.gravity : [0.0, 0.0, -9.81];
        if (gravity.length != 3)
            throw "Simulation gravity must contain three values";
        for (index in 0...3)
            desc.set_gravity(index, gravity[index]);
        return desc;
    }

    @:allow(MujocoSimWorld)
    static function fromNativeOwner(scene:Scene, owner:Ownednksim_world):SimWorld
        return new SimWorld(scene, null, owner);

    @:allow(MujocoSimWorld)
    static function check(status:Int, operation:String):Void {
        if (status != NativeKitSimConstants.NKSIM_OK)
            throw '$operation failed with NativeKit simulation status $status';
    }

    public function nativeHandle():nksim_world {
        ensureLive();
        return owner.borrow();
    }

    public function createShapeBox(x:Float, y:Float, z:Float):Shape {
        ensureLive();
        var desc = new nksim_shape_desc();
        desc.set_struct_size(nksim_shape_desc.size());
        desc.set_type(NativeKitSimConstants.NKSIM_SHAPE_BOX);
        desc.set_parameters(0, x);
        desc.set_parameters(1, y);
        desc.set_parameters(2, z);
        var result = NativeKitSim.nksim_shape_create(owner.borrow(), desc);
        check(result.status, "world.createShapeBox");
        var shape = new Shape(this, result.out_shape);
        resources.push(shape.dispose);
        return shape;
    }

    public function createShapeSphere(radius:Float):Shape {
        ensureLive();
        var result = NativeKitSim.nksim_shape_create_sphere(owner.borrow(), radius);
        check(result.status, "world.createShapeSphere");
        var shape = new Shape(this, result.out_shape);
        resources.push(shape.dispose);
        return shape;
    }

    public function createShapeCapsule(radius:Float, height:Float):Shape {
        ensureLive();
        var result = NativeKitSim.nksim_shape_create_capsule(owner.borrow(), radius, height);
        check(result.status, "world.createShapeCapsule");
        var shape = new Shape(this, result.out_shape);
        resources.push(shape.dispose);
        return shape;
    }

    /** Creates a plane in the shape's local frame. */
    public function createShapePlane(normalX:Float, normalY:Float,
            normalZ:Float, offset:Float):Shape {
        ensureLive();
        var desc = new nksim_shape_desc();
        desc.set_struct_size(nksim_shape_desc.size());
        desc.set_type(NativeKitSimConstants.NKSIM_SHAPE_PLANE);
        desc.set_parameters(0, normalX);
        desc.set_parameters(1, normalY);
        desc.set_parameters(2, normalZ);
        desc.set_parameters(3, offset);
        var result = NativeKitSim.nksim_shape_create(owner.borrow(), desc);
        check(result.status, "world.createShapePlane");
        var shape = new Shape(this, result.out_shape);
        resources.push(shape.dispose);
        return shape;
    }

    public function createBody(node:Node, motion:MotionType,
            mass:Float, ?shape:Shape):Body {
        ensureLive();
        var desc = new nksim_body_desc();
        desc.set_struct_size(nksim_body_desc.size());
        var nodeValue = new nkscene_node_id();
        nodeValue.set_value(node.stableValue());
        desc.set_node(nodeValue);
        desc.set_motion_type(motion);
        desc.set_mass(mass);
        var shapeValue = new nksim_shape();
        if (shape != null)
            shapeValue = shape.nativeHandle();
        desc.set_shape(shapeValue);
        desc.set_collision_layer(1);
        desc.set_collision_mask(1);
        var result = NativeKitSim.nksim_body_create(owner.borrow(), desc);
        check(result.status, "world.createBody");
        var body = new Body(this, result.out_body);
        resources.push(body.dispose);
        return body;
    }

    public function createJoint(type:JointType, bodyA:Body, bodyB:Body):Joint {
        ensureLive();
        var desc = new nksim_joint_desc();
        desc.set_struct_size(nksim_joint_desc.size());
        desc.set_type(type);
        desc.set_body_a(bodyA.nativeHandle());
        desc.set_body_b(bodyB.nativeHandle());
        desc.set_axis_a(2, 1.0);
        var result = NativeKitSim.nksim_joint_create(owner.borrow(), desc);
        check(result.status, "world.createJoint");
        var joint = new Joint(this, result.out_joint);
        resources.push(joint.dispose);
        return joint;
    }

    public function step():StepResult {
        ensureLive();
        if (pendingJointTargets.length > 0) {
            var targetStatus = NativeKitSim.nksim_world_set_joint_targets(
                owner.borrow(), pendingJointTargets);
            check(targetStatus, "world.setJointTargets");
            pendingJointTargets = [];
        }
        var value = new nksim_step_result();
        value.set_struct_size(nksim_step_result.size());
        var result = NativeKitSim.nksim_world_step(owner.borrow(), value);
        check(result.status, "world.step");
        return new StepResult(value);
    }

    public function snapshot():SimSnapshot {
        ensureLive();
        var result = NativeKitSim.nksim_world_snapshot(owner.borrow());
        check(result.status, "world.snapshot");
        return new SimSnapshot(result.out_snapshot);
    }

    public function dispose():Void {
        if (disposed)
            return;
        for (index in 0...resources.length)
            resources[resources.length - index - 1]();
        owner.close();
        disposed = true;
    }

    public function isDisposed():Bool
        return disposed;

    @:allow(Body, Joint, Shape, SimSnapshot)
    function ensureLive():Void {
        if (disposed)
            throw "Simulation world has been disposed";
    }

    @:allow(Joint)
    function stageJointTarget(joint:Joint, mode:JointTargetMode,
            target:Float, maxForce:Float):Void {
        ensureLive();
        var value = new nksim_joint_target();
        value.set_struct_size(nksim_joint_target.size());
        value.set_joint(joint.nativeHandle());
        value.set_mode(mode);
        value.set_target(target);
        value.set_max_force(maxForce);
        pendingJointTargets.push(value);
    }
}
