import animkit.AnimationAsset;
import humankit.HumanBodyProxy;
import humankit.HumanCharacter;
import humankit.HumanDescription;
import humankit.HumanJobSpec;
import humankit.HumanLimb;
import humankit.rig.HumanoidRig;
import humankit.sim.HumanWorker;
import nativekit.scene.Scene;
import nativekit.sim.MotionType;
import nativekit.sim.MujocoSimWorld;
import nativekit.sim.SimPose;
import nativekit.sim.SimSession;
import nativekit.sim.SimShape;

/** A wall panel at a height, turned about the worker, pressed with one hand. */
class PressScenario {
    public final session:SimSession;
    public final worker:HumanWorker;
    /** The point on the panel's near face that the press is aimed at, in the world. */
    public final pressPoint:Array<Float>;
    public final hand:HumanLimb;
    final world:MujocoSimWorld;

    function new(session:SimSession, world:MujocoSimWorld, worker:HumanWorker, pressPoint:Array<Float>, hand:HumanLimb) {
        this.session = session;
        this.world = world;
        this.worker = worker;
        this.pressPoint = pressPoint;
        this.hand = hand;
    }

    /** `yaw` turns the panel about the worker's start, which faces +X; `height` is the panel centre's height in metres. */
    public static function build(hand:String, height:Float, yaw:Float):PressScenario {
        var asset = AnimationAsset.load("../../../animkit/assets/quaternius/worker.glb");
        var scene = Scene.create();
        var world = MujocoSimWorld.create(scene, {timestep: 1.0 / 60.0, physicsSubsteps: 4, gravity: [0.0, 0.0, -9.81]});
        var session = new SimSession(scene, world.nativeHandle());
        var human = new HumanCharacter(scene, asset, HumanoidRig.detect(asset), null, "presser");
        human.advance(0.0);
        var proxy = HumanBodyProxy.standard(human.pose, HumanDescription.measure(human.pose, human.height()));
        var worker = new HumanWorker(session, human, proxy, new SimPose(0, 0, 0));
        var halfThickness = 0.02, distance = 1.5;
        var cosine = Math.cos(yaw), sine = Math.sin(yaw);
        var center = [distance * cosine, distance * sine, height];
        session.createObject(MotionType.Static, SimShape.box(halfThickness, 0.12, 0.08),
            new SimPose(center[0], center[1], center[2], 0.0, 0.0, Math.sin(yaw * 0.5), Math.cos(yaw * 0.5)));
        session.createObject(MotionType.Static, SimShape.box(6, 6, 0.1), new SimPose(0, 0, -0.05));
        var targets = new JobTargets();
        targets.boxes.set("panel", {center: center, halfExtents: [halfThickness, 0.12, 0.08], yaw: yaw});
        worker.runSpec(HumanJobSpec.parse('{"version":1,"loop":false,"steps":[{"action":"press","target":{"object":"panel","anchor":"front"},"hand":"' + hand + '"}]}'),
            targets, new Map());
        // The near face, toward the worker, where the press lands.
        var face = [center[0] - halfThickness * cosine, center[1] - halfThickness * sine, height];
        return new PressScenario(session, world, worker, face, hand == "left" ? ArmL : ArmR);
    }

    public function tick():Void {
        worker.advance();
        session.step();
    }
}
