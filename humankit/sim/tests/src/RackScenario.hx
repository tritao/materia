import animkit.AnimationAsset;
import humankit.HumanBodyProxy;
import humankit.HumanCharacter;
import humankit.HumanDescription;
import humankit.HumanJobSpec;
import humankit.HumanoidRig;
import humankit.sim.HumanWorker;
import nativekit.scene.Scene;
import nativekit.sim.MotionType;
import nativekit.sim.MujocoSimWorld;
import nativekit.sim.SimPose;
import nativekit.sim.SimSession;
import nativekit.sim.SimShape;

/** The rack-to-table job on a bare session: a worker fetches a part from a pedestal and sets it on a table. */
class RackScenario {
    public final session:SimSession;
    public final worker:HumanWorker;
    public final part:nativekit.sim.SimObject;
    public final partStart:Array<Float>;
    final world:MujocoSimWorld;

    function new(session:SimSession, world:MujocoSimWorld, worker:HumanWorker, part:nativekit.sim.SimObject,
            partStart:Array<Float>) {
        this.session = session;
        this.world = world;
        this.worker = worker;
        this.part = part;
        this.partStart = partStart;
    }

    public static function build(restingOn:Bool = true):RackScenario {
        var asset = AnimationAsset.load("../../../animkit/assets/quaternius/worker.glb");
        var rig = HumanoidRig.detect(asset);
        var scene = Scene.create();
        var world = MujocoSimWorld.create(scene, {timestep: 0.01, physicsSubsteps: 4, gravity: [0.0, 0.0, -9.81]});
        var session = new SimSession(scene, world.nativeHandle());
        var human = new HumanCharacter(scene, asset, rig, null, "worker");
        human.advance(0.0);
        var proxy = HumanBodyProxy.standard(human.pose, HumanDescription.measure(human.pose, human.height()));
        var worker = new HumanWorker(session, human, proxy, new SimPose(0.0, 0.0, 0.0));
        var partHalf = 0.04, surface = 1.06;
        var partStart = [0.9, -0.2, surface + partHalf];
        var part = session.createObject(MotionType.Dynamic, SimShape.box(partHalf, partHalf, partHalf),
            new SimPose(partStart[0], partStart[1], partStart[2]), 0.1);
        session.createObject(MotionType.Static, SimShape.box(0.2, 0.2, 0.05), new SimPose(partStart[0], partStart[1], surface - 0.05));
        session.createObject(MotionType.Static, SimShape.box(4.0, 4.0, 0.1), new SimPose(0.0, 0.0, -0.05));
        var placePoint = [partStart[0] + 1.0, partStart[1], surface + partHalf];
        var table = session.createObject(MotionType.Static, SimShape.box(0.2, 0.2, 0.05),
            new SimPose(placePoint[0], placePoint[1], surface - 0.05));
        var targets = new JobTargets();
        targets.boxes.set("part", {center: partStart.copy(), halfExtents: [partHalf, partHalf, partHalf], yaw: 0.0});
        targets.boxes.set("table", {center: [placePoint[0], placePoint[1], surface - 0.05], halfExtents: [0.2, 0.2, 0.05], yaw: 0.0});
        targets.boxes.set("rack", {center: [partStart[0], partStart[1], surface - 0.05], halfExtents: [0.2, 0.2, 0.05], yaw: 0.0});
        var objects:Map<String, nativekit.sim.SimObject> = new Map();
        objects.set("part", part);
        objects.set("table", table);
        worker.runSpec(HumanJobSpec.parse('{"version":1,"loop":false,"steps":[{"action":"pick","object":"part"' + (restingOn ? ',"from":"rack"' : '') + '},{"action":"place","onto":"table"}]}'), targets, objects);
        return new RackScenario(session, world, worker, part, partStart);
    }

    /** Feeds the worker and runs one tick; returns where the part is afterwards. */
    public function tick():Array<Float> {
        worker.advance();
        session.stepPaced();
        var captured = session.capture();
        var pose = captured.objectPose(part);
        captured.dispose();
        return [pose.x, pose.y, pose.z];
    }

}
