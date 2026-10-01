import animkit.AnimationAsset;
import humankit.HumanBodyProxy;
import humankit.HumanCharacter;
import humankit.HumanDescription;
import humankit.HumanJobSpec;
import humankit.HumanLimb;
import humankit.HumanTargetBox;
import humankit.HumanoidRig;
import humankit.sim.HumanWorker;
import nativekit.scene.Scene;
import nativekit.sim.MotionType;
import nativekit.sim.MujocoSimWorld;
import nativekit.sim.SimPose;
import nativekit.sim.SimSession;
import nativekit.sim.SimShape;

/** How the rack and table are laid out: which hand fetches, the surfaces' height, and the layout's turn about the worker. */
typedef RackLayout = {
    /** "right", "left" or "both"; the part sits on a single hand's side, or between both hands. */
    var hand:String;
    /** Height of the surfaces' tops, in metres. */
    var surface:Float;
    /** Turn of the whole layout about the worker's start, in radians. */
    var yaw:Float;
    /** Half the part's side, in metres; 0.04 when not given. */
    @:optional var partHalf:Float;
    /** Half the part's extents (x, y, z) when it is not a cube; overrides partHalf. */
    @:optional var partSize:Array<Float>;
    /** Half the rack's and table's side, in metres; 0.2 when not given. */
    @:optional var surfaceHalf:Float;
    /** The character's glTF, relative to the tests' folder; the bundled Quaternius worker when not given. */
    @:optional var character:String;
}

/** The rack-to-table job on a bare session: a worker fetches a part from a pedestal and sets it on a table. */
class RackScenario {
    public final session:SimSession;
    public final worker:HumanWorker;
    public final part:nativekit.sim.SimObject;
    public final partStart:Array<Float>;
    public final placePoint:Array<Float>;
    public final rack:HumanTargetBox;
    public final table:HumanTargetBox;
    public final layout:RackLayout;
    final world:MujocoSimWorld;

    function new(session:SimSession, world:MujocoSimWorld, worker:HumanWorker, part:nativekit.sim.SimObject,
            partStart:Array<Float>, placePoint:Array<Float>, rack:HumanTargetBox, table:HumanTargetBox, layout:RackLayout) {
        this.session = session;
        this.world = world;
        this.worker = worker;
        this.part = part;
        this.partStart = partStart;
        this.placePoint = placePoint;
        this.rack = rack;
        this.table = table;
        this.layout = layout;
    }

    /** The limb of the hand that does the work. */
    public function limb():HumanLimb return layout.hand == "left" ? ArmL : ArmR;

    /** Every hand that does the work. */
    public function limbs():Array<HumanLimb> return layout.hand == "both" ? [ArmL, ArmR] : [limb()];

    static function yawPose(x:Float, y:Float, z:Float, yaw:Float):SimPose
        return new SimPose(x, y, z, 0.0, 0.0, Math.sin(yaw * 0.5), Math.cos(yaw * 0.5));

    public static function build(restingOn:Bool = true, ?layout:RackLayout):RackScenario {
        if (layout == null) layout = {hand: "right", surface: 1.06, yaw: 0.0};
        var asset = AnimationAsset.load("../../../animkit/assets/" + (layout.character == null ? "quaternius/worker.glb" : layout.character));
        var rig = HumanoidRig.detect(asset);
        var scene = Scene.create();
        var world = MujocoSimWorld.create(scene, {timestep: 0.01, physicsSubsteps: 4, gravity: [0.0, 0.0, -9.81]});
        var session = new SimSession(scene, world.nativeHandle());
        var human = new HumanCharacter(scene, asset, rig, null, "worker");
        human.advance(0.0);
        var proxy = HumanBodyProxy.standard(human.pose, HumanDescription.measure(human.pose, human.height()));
        var worker = new HumanWorker(session, human, proxy, new SimPose(0.0, 0.0, 0.0));
        var side0 = layout.partHalf == null ? 0.04 : layout.partHalf;
        var half:Array<Float> = layout.partSize == null ? [side0, side0, side0] : layout.partSize;
        var partHalf = half[2], surface = layout.surface, top = layout.surfaceHalf == null ? 0.2 : layout.surfaceHalf;
        // The layout is built facing +X with the part on the working hand's side, then turned about the start.
        var side = layout.hand == "left" ? 1.0 : layout.hand == "both" ? 0.0 : -1.0, cosine = Math.cos(layout.yaw), sine = Math.sin(layout.yaw);
        var turned = function(x:Float, y:Float):Array<Float> return [cosine * x - sine * y, sine * x + cosine * y];
        var rackXY = turned(0.9, side * 0.2), tableXY = turned(1.9, side * 0.2);
        var partStart = [rackXY[0], rackXY[1], surface + partHalf];
        var part = session.createObject(MotionType.Dynamic, SimShape.box(half[0], half[1], half[2]),
            new SimPose(partStart[0], partStart[1], partStart[2]), 0.1);
        session.createObject(MotionType.Static, SimShape.box(top, top, 0.05), yawPose(partStart[0], partStart[1], surface - 0.05, layout.yaw));
        session.createObject(MotionType.Static, SimShape.box(4.0, 4.0, 0.1), new SimPose(0.0, 0.0, -0.05));
        var placePoint = [tableXY[0], tableXY[1], surface + partHalf];
        var tableObject = session.createObject(MotionType.Static, SimShape.box(top, top, 0.05),
            yawPose(placePoint[0], placePoint[1], surface - 0.05, layout.yaw));
        var targets = new JobTargets();
        var rackBox:HumanTargetBox = {center: [partStart[0], partStart[1], surface - 0.05], halfExtents: [top, top, 0.05], yaw: layout.yaw};
        var tableBox:HumanTargetBox = {center: [placePoint[0], placePoint[1], surface - 0.05], halfExtents: [top, top, 0.05], yaw: layout.yaw};
        targets.boxes.set("part", {center: partStart.copy(), halfExtents: half.copy(), yaw: layout.yaw});
        targets.boxes.set("table", tableBox);
        targets.boxes.set("rack", rackBox);
        var objects:Map<String, nativekit.sim.SimObject> = new Map();
        objects.set("part", part);
        objects.set("table", tableObject);
        var hand = ',"hand":"' + layout.hand + '"';
        worker.runSpec(HumanJobSpec.parse('{"version":1,"loop":false,"steps":[{"action":"pick","object":"part"' + hand +
            (restingOn ? ',"from":"rack"' : '') + '},{"action":"place","onto":"table"' + hand + '}]}'), targets, objects);
        return new RackScenario(session, world, worker, part, partStart, placePoint, rackBox, tableBox, layout);
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
