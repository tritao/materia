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

/**
 * The worker is fed on the simulation clock, one call per tick, so the owner's
 * frame pattern cannot change what a carried part does. A UI frame that stalls
 * for many ticks must leave the part on exactly the same path as smooth frames.
 */
class PacedStepTests {
    static inline var TICK_NS = 10000000;

    static function trace(framePattern:Array<Int>):Array<Array<Float>> {
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
        var objects:Map<String, nativekit.sim.SimObject> = new Map();
        objects.set("part", part);
        objects.set("table", table);
        worker.runSpec(HumanJobSpec.parse('{"version":1,"loop":false,"steps":[{"action":"pick","object":"part"},{"action":"place","onto":"table"}]}'), targets, objects);
        var poses:Array<Array<Float>> = [];
        var frame = 0;
        while (poses.length < 900 && !worker.currentJobDone()) {
            var elapsed = haxe.Int64.ofInt(framePattern[frame++ % framePattern.length] * TICK_NS);
            var due = session.dueTicks(elapsed, 1000);
            for (_ in 0...due) {
                worker.advance();
                session.stepPaced();
                var captured = session.capture();
                var pose = captured.objectPose(part);
                captured.dispose();
                poses.push([pose.x, pose.y, pose.z]);
            }
        }
        if (!worker.currentJobDone()) throw "Paced job did not finish: " + worker.currentJobFailure();
        if (worker.currentJobFailure() != null) throw "Paced job failed: " + worker.currentJobFailure();
        return poses;
    }

    public static function run():Void {
        var smooth = trace([1]);
        // Frames of 1 to 12 ticks, well past the old three-tick lead.
        var stalled = trace([1, 1, 12, 2, 1, 7, 1, 4]);
        var count = smooth.length < stalled.length ? smooth.length : stalled.length;
        if (Math.abs(smooth.length - stalled.length) > 12)
            throw 'Frame pattern changed the job length: ${smooth.length} vs ${stalled.length} ticks';
        var worst = 0.0;
        for (tick in 0...count) for (axis in 0...3)
            worst = Math.max(worst, Math.abs(smooth[tick][axis] - stalled[tick][axis]));
        if (worst > 1e-9) throw 'A stalled frame moved the carried part off its path by $worst m';
        Sys.println('paced stepping: ${smooth.length} ticks, stalled frames left the part path identical');
    }
}
