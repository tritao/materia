import animkit.AnimationAsset;
import humankit.HumanBodyProxy;
import humankit.HumanCharacter;
import humankit.HumanDescription;
import humankit.HumanJob;
import humankit.HumanBone;
import humankit.HumanoidRig;
import humankit.Pick;
import humankit.Place;
import humankit.WalkTo;
import humankit.Wait;
import humankit.sim.HumanWorker;
import humankit.sim.HumanZone;
import nativekit.scene.Scene;
import nativekit.sim.MotionType;
import nativekit.sim.MujocoSimWorld;
import nativekit.sim.SimPose;
import nativekit.sim.SimSession;
import nativekit.sim.SimShape;
import nativekit.sim.SimFrame;

class HumanSimTests {
    static function main():Void {
        var asset = AnimationAsset.load("../../../animkit/assets/quaternius/worker.glb");
        var rig = HumanoidRig.detect(asset);
        var scene = Scene.create();
        var world = MujocoSimWorld.create(scene, {timestep: 1.0 / 60.0, physicsSubsteps: 4,
            gravity: [0.0, 0.0, -9.81]});
        var session = new SimSession(scene, world.nativeHandle());
        var human = new HumanCharacter(scene, asset, rig, null, "worker");
        human.advance(0.0);
        var proxy = HumanBodyProxy.standard(human.pose, HumanDescription.measure(human.pose, human.height()));
        var worker = new HumanWorker(session, human, proxy, new SimPose(0.0, 0.0, 0.0));
        worker.addZone(new HumanZone("start", [[-0.5, -0.5], [0.5, -0.5], [0.5, 0.5], [-0.5, 0.5]]));
        var samples = 0, seenStart = false, leftStart = false;
        worker.onTick = function(_, signals) {
            samples++;
            if (signals.zones.indexOf("start") >= 0) seenStart = true;
            if (seenStart && signals.zones.indexOf("start") < 0) leftStart = true;
        };
        var hand = human.pose.bonePosition(HumanBone.HandR);
        var pickPoint = [hand[0] + 0.15, hand[1], hand[2]];
        var gripOffset = 0.30;
        var part = session.createObject(MotionType.Dynamic, SimShape.box(0.08, 0.08, 0.08),
            new SimPose(pickPoint[0] + gripOffset, pickPoint[1], pickPoint[2]), 0.1);
        var robot = session.createActor([SimShape.sphere(0.05)], [new SimPose(3.0, 0.0, 1.5)]);
        worker.addRobotLink("arm", robot.partBody(0), 0.05);
        session.createObject(MotionType.Static, SimShape.box(0.4, 0.4, 0.08),
            new SimPose(pickPoint[0] + gripOffset, pickPoint[1], pickPoint[2] - 0.16));
        session.createObject(MotionType.Static, SimShape.box(4.0, 4.0, 0.1), new SimPose(0.0, 0.0, -0.05));
        var pick = new Pick(pickPoint, [ArmR], 0.1);
        var placePoint = [pickPoint[0] + 0.9, pickPoint[1], pickPoint[2]];
        session.createObject(MotionType.Static, SimShape.box(0.8, 0.8, 0.08),
            new SimPose(placePoint[0] + gripOffset, placePoint[1], placePoint[2] - 0.16));
        var place = new Place(placePoint, [ArmR], 0.1);
        var job = new HumanJob().add(pick).add(new WalkTo([0.9, 0.0], 1.0)).add(place);
        worker.bindPick(pick, part, pickPoint);
        worker.bindPlace(place, part, placePoint);
        worker.run(job);
        for (_ in 0...600) {
            worker.advance();
            session.step();
            if (job.isDone()) break;
        }
        for (_ in 0...90) {
            worker.advance();
            session.step();
        }
        var frame = session.capture();
        var finalPose = frame.objectPose(part);
        frame.dispose();
        if (samples == 0 || !seenStart || !leftStart || job.failure() != null || !job.isDone())
            throw 'Worker did not complete job or zone transitions: ${job.failure()} samples=$samples start=$seenStart left=$leftStart';
        var error = Math.sqrt(Math.pow(finalPose.x - placePoint[0] - gripOffset, 2) +
            Math.pow(finalPose.y - placePoint[1], 2) + Math.pow(finalPose.z - placePoint[2], 2));
        if (error > 0.01) throw 'Placed part missed target by $error: ${finalPose.x}, ${finalPose.y}, ${finalPose.z}';
        var now = session.simulationTime();
        robot.pushKeyframe(now, [new SimPose(3.0, 0.0, 1.5)]);
        robot.pushKeyframe(now + 0.5, [new SimPose(2.0, 0.0, 1.5)]);
        var previous = Math.POSITIVE_INFINITY;
        worker.onTick = function(_, signals) {
            var value = signals.separation.get("arm");
            if (value == null || value > previous + 0.005)
                throw 'Robot separation did not decrease: $value after $previous';
            var captured = session.capture();
            var manual = expectedSeparation(captured, worker, captured.actorPose(robot, 0), 0.05);
            captured.dispose();
            if (Math.abs(manual - value) > 1e-6) throw 'Separation differs from capsule bounds: $value vs $manual';
            previous = value;
        };
        for (_ in 0...30) { worker.advance(); session.step(); }
        if (previous == Math.POSITIVE_INFINITY) throw "No separation samples";
        worker.dispose();
        session.dispose();
        world.dispose();
        human.dispose();
        scene.dispose();
        releaseMidWalk(asset, rig);
    }

    static function expectedSeparation(frame:SimFrame, worker:HumanWorker, point:SimPose, radius:Float):Float {
        var minimum = Math.POSITIVE_INFINITY;
        for (index in 0...worker.actor.proxy.capsules.length) {
            var capsule = worker.actor.proxy.capsules[index];
            var pose = frame.actorPose(worker.actor.actor, index);
            var axis = [2 * (pose.qx * pose.qz + pose.qw * pose.qy),
                2 * (pose.qy * pose.qz - pose.qw * pose.qx),
                1 - 2 * (pose.qx * pose.qx + pose.qy * pose.qy)];
            var half = capsule.length * 0.5;
            var along = (point.x - pose.x) * axis[0] + (point.y - pose.y) * axis[1] +
                (point.z - pose.z) * axis[2];
            along = Math.max(-half, Math.min(half, along));
            var dx = point.x - pose.x - axis[0] * along;
            var dy = point.y - pose.y - axis[1] * along;
            var dz = point.z - pose.z - axis[2] * along;
            minimum = Math.min(minimum, Math.max(0, Math.sqrt(dx * dx + dy * dy + dz * dz) -
                capsule.radius - radius));
        }
        return minimum;
    }

    static function releaseMidWalk(asset:AnimationAsset, rig:HumanoidRig):Void {
        var scene = Scene.create();
        var world = MujocoSimWorld.create(scene, {timestep: 1.0 / 60.0, physicsSubsteps: 4,
            gravity: [0.0, 0.0, -9.81]});
        var session = new SimSession(scene, world.nativeHandle());
        var human = new HumanCharacter(scene, asset, rig, null, "release worker");
        human.advance(0.0);
        var proxy = HumanBodyProxy.standard(human.pose, HumanDescription.measure(human.pose, human.height()));
        var worker = new HumanWorker(session, human, proxy, new SimPose(0, 0, 0));
        var hand = human.pose.bonePosition(HumanBone.HandR);
        var point = [hand[0] + 0.15, hand[1], hand[2]];
        var part = session.createObject(MotionType.Dynamic, SimShape.box(0.04, 0.04, 0.04),
            new SimPose(point[0] + 0.2, point[1], point[2]), 0.05);
        session.createObject(MotionType.Static, SimShape.box(0.35, 0.35, 0.06),
            new SimPose(point[0] + 0.2, point[1], point[2] - 0.10));
        session.createObject(MotionType.Static, SimShape.box(4, 4, 0.1), new SimPose(0, 0, -0.1));
        var freeBox = session.createObject(MotionType.Dynamic, SimShape.box(0.1, 0.1, 0.1),
            new SimPose(0.5, 0, 0.1), 0.2);
        var pick = new Pick(point, [ArmR], 0.1);
        worker.bindPick(pick, part, point);
        worker.run(new HumanJob().add(pick).add(new WalkTo([1.5, 0], 1.0)));
        var released = false, releaseHeight = 0.0;
        for (_ in 0...300) {
            worker.advance();
            session.step();
            if (!released && worker.body.rootTransform()[12] > 0.5 &&
                session.objectCarrier(part) == worker.actor.actor.partBody(11)) {
                var frame = session.capture();
                releaseHeight = frame.objectPose(part).z;
                frame.dispose();
                session.releaseObject(part);
                released = true;
            }
        }
        var frame = session.capture();
        var fallen = frame.objectPose(part);
        var pushed = frame.objectPose(freeBox);
        frame.dispose();
        if (!released || fallen.z >= releaseHeight - 0.2)
            throw 'Mid-walk release did not fall: released=$released $releaseHeight to ${fallen.z}';
        if (Math.abs(pushed.x - 0.5) < 0.02 && Math.abs(pushed.y) < 0.02)
            throw 'HumanActor did not push the free box: ${pushed.x}, ${pushed.y}';
        worker.dispose();
        session.dispose();
        world.dispose();
        human.dispose();
        scene.dispose();
    }
}
