import animkit.AnimationAsset;
import humankit.HumanBodyProxy;
import humankit.HumanCharacter;
import humankit.HumanDescription;
import humankit.HumanJob;
import humankit.HumanBone;
import humankit.HumanoidRig;
import humankit.Pick;
import humankit.Place;
import humankit.ApproachFor;
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
        // An 8 cm part on a pedestal whose top is at 1.06 m, and a table of the
        // same height 1 m further on. The palm grasps just above the part's top;
        // Place puts the part's centre where it rests on the table.
        var partHalf = 0.04, surface = 1.06;
        var partStart = [0.9, -0.2, surface + partHalf];
        var part = session.createObject(MotionType.Dynamic, SimShape.box(partHalf, partHalf, partHalf),
            new SimPose(partStart[0], partStart[1], partStart[2]), 0.1);
        var robot = session.createActor([SimShape.sphere(0.05)], [new SimPose(3.0, 0.0, 1.5)]);
        worker.addRobotLink("arm", robot.partBody(0), 0.05);
        session.createObject(MotionType.Static, SimShape.box(0.2, 0.2, 0.05),
            new SimPose(partStart[0], partStart[1], surface - 0.05));
        session.createObject(MotionType.Static, SimShape.box(4.0, 4.0, 0.1), new SimPose(0.0, 0.0, -0.05));
        var pickPoint = [partStart[0], partStart[1], surface + 2 * partHalf + 0.01];
        var pick = new Pick(pickPoint, [ArmR], 0.3);
        var placePoint = [partStart[0] + 1.0, partStart[1], surface + partHalf];
        session.createObject(MotionType.Static, SimShape.box(0.2, 0.2, 0.05),
            new SimPose(placePoint[0], placePoint[1], surface - 0.05));
        var place = new Place(placePoint, [ArmR], 0.35);
        var job = new HumanJob().add(new ApproachFor(pickPoint, ArmR)).add(pick)
            .add(new ApproachFor(placePoint, ArmR)).add(place)
            .add(new WalkTo([0.3, 0.0], 1.0));
        worker.bindPick(pick, part, pickPoint);
        worker.bindPlace(place, part, placePoint);
        worker.run(job);
        var noCarrier = session.objectCarrier(part);
        var sawHold = false, sawRelease = false;
        var beforeRelease:Null<SimPose> = null;
        for (tick in 0...600) {
            worker.advance();
            session.step();
            var captured = session.capture();
            var currentPose = captured.objectPose(part);
            captured.dispose();
            var carrier = session.objectCarrier(part);
            // Reaching for the part must not knock it before the hand closes.
            if (!sawHold && Math.sqrt(Math.pow(currentPose.x - partStart[0], 2) +
                Math.pow(currentPose.y - partStart[1], 2) + Math.pow(currentPose.z - partStart[2], 2)) > 0.01)
                throw 'The reach disturbed the part before the grip at tick $tick: ${currentPose.x}, ${currentPose.y}, ${currentPose.z}';
            if (carrier != noCarrier) sawHold = true;
            if (job.currentAction() == place && carrier != noCarrier && beforeRelease != null) {
                var moving = Math.sqrt(Math.pow(currentPose.x - beforeRelease.x, 2) +
                    Math.pow(currentPose.y - beforeRelease.y, 2) +
                    Math.pow(currentPose.z - beforeRelease.z, 2));
                if (moving > 0.15) throw 'Carried part jumped during Place at tick $tick: $moving';
            }
            if (sawHold && carrier == noCarrier && !sawRelease) {
                sawRelease = true;
                if (beforeRelease == null) throw "Release lacked a previous hand pose";
                var jump = Math.sqrt(Math.pow(currentPose.x - beforeRelease.x, 2) +
                    Math.pow(currentPose.y - beforeRelease.y, 2) +
                    Math.pow(currentPose.z - beforeRelease.z, 2));
                if (jump > 0.05) throw 'Part jumped away from the hand on release: $jump';
            }
            beforeRelease = currentPose;
            if (job.isDone()) break;
        }
        var finalPose:SimPose = null;
        var speed = Math.POSITIVE_INFINITY;
        for (settleTick in 0...450) {
            worker.advance();
            session.step();
            var frame = session.capture();
            var current = frame.objectPose(part);
            frame.dispose();
            if (finalPose != null) {
                speed = Math.sqrt(Math.pow(current.x - finalPose.x, 2) +
                    Math.pow(current.y - finalPose.y, 2) + Math.pow(current.z - finalPose.z, 2)) /
                    session.fixedTimestep();
                var horizontalNow = Math.sqrt(Math.pow(current.x - placePoint[0], 2) +
                    Math.pow(current.y - placePoint[1], 2));
                if (settleTick >= 90 && speed < 0.001 && horizontalNow < 0.02 &&
                    Math.abs(current.z - placePoint[2]) < 0.01) {
                    finalPose = current;
                    break;
                }
            }
            finalPose = current;
        }
        if (!sawHold || !sawRelease || samples == 0 || !seenStart || !leftStart ||
            job.failure() != null || !job.isDone())
            throw 'Worker did not complete job or zone transitions: ${job.failure()} samples=$samples start=$seenStart left=$leftStart';
        var horizontal = Math.sqrt(Math.pow(finalPose.x - placePoint[0], 2) +
            Math.pow(finalPose.y - placePoint[1], 2));
        var vertical = Math.abs(finalPose.z - placePoint[2]);
        // Place levels the part in the hand: it rests flat, not tipped on an edge.
        var tilt = 2 * Math.asin(Math.min(1.0, Math.sqrt(finalPose.qx * finalPose.qx + finalPose.qy * finalPose.qy)));
        if (tilt > 0.035) throw 'Placed part rests tilted by $tilt rad';
        if (horizontal > 0.02 || vertical > 0.01 || speed > 0.001)
            throw 'Placed part missed target: horizontal=$horizontal vertical=$vertical speed=$speed residual=${place.placementError} pose=${finalPose.x}, ${finalPose.y}, ${finalPose.z}';
        Sys.println('human sim placement: pick=${pick.pickError} horizontal=$horizontal vertical=$vertical speed=$speed residual=${place.placementError}');
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
        // A part on a pedestal whose top is at 1.06 m, picked from above.
        var part = session.createObject(MotionType.Dynamic, SimShape.box(0.04, 0.04, 0.04),
            new SimPose(0.9, -0.3, 1.10), 0.05);
        session.createObject(MotionType.Static, SimShape.box(0.1, 0.1, 0.05), new SimPose(0.9, -0.3, 1.01));
        var point = [0.9, -0.3, 1.15];
        session.createObject(MotionType.Static, SimShape.box(4, 4, 0.1), new SimPose(0, 0, -0.1));
        var freeBox = session.createObject(MotionType.Dynamic, SimShape.box(0.1, 0.1, 0.1),
            new SimPose(0.5, 0, 0.1), 0.2);
        var pick = new Pick(point, [ArmR], 0.1);
        worker.bindPick(pick, part, point);
        worker.run(new HumanJob().add(new ApproachFor(point, ArmR)).add(pick).add(new WalkTo([1.5, 0], 1.0)));
        var released = false, releaseHeight = 0.0;
        for (_ in 0...300) {
            worker.advance();
            session.step();
            if (!released && worker.body.rootTransform()[12] > 1.2 &&
                session.objectCarrier(part) == worker.handBody(ArmR)) {
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
