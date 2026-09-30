/**
 * Resetting a session puts everything in it back at the start, the worker
 * included. After a reset at any point in a job, the worker starts its job
 * again and everything follows exactly the path of a session that never ran:
 * the same pick, the same length, the same part path.
 */
class WorkerResetTests {
    /** Runs the job to its end and lets the part settle. */
    static function runToEnd(scenario:RackScenario):{poses:Array<Array<Float>>, pickedAt:Int, rest:Array<Float>} {
        var free = scenario.session.objectCarrier(scenario.part);
        var poses:Array<Array<Float>> = [];
        var pickedAt = -1;
        while (poses.length < 900 && !scenario.worker.currentJobDone()) {
            poses.push(scenario.tick());
            if (pickedAt < 0 && scenario.session.objectCarrier(scenario.part) != free) pickedAt = poses.length;
        }
        if (!scenario.worker.currentJobDone() || scenario.worker.currentJobFailure() != null)
            throw "Rack job did not finish: " + scenario.worker.currentJobFailure();
        if (pickedAt < 0) throw "The worker never picked the part up";
        var rest = poses[poses.length - 1];
        for (_ in 0...300) rest = scenario.tick();
        return {poses: poses, pickedAt: pickedAt, rest: rest};
    }

    static function gap(a:Array<Float>, b:Array<Float>):Float
        return Math.max(Math.abs(a[0] - b[0]), Math.max(Math.abs(a[1] - b[1]), Math.abs(a[2] - b[2])));

    /** Runs `resetAt` ticks, resets, and checks the rerun against a fresh run. */
    static function checkResetAt(resetAt:Int, fresh:{poses:Array<Array<Float>>, pickedAt:Int, rest:Array<Float>}):Void {
        var scenario = RackScenario.build();
        var free = scenario.session.objectCarrier(scenario.part);
        for (_ in 0...resetAt) scenario.tick();
        var moved = scenario.worker.body.rootTransform();
        var carried = scenario.session.objectCarrier(scenario.part) != free;

        scenario.session.reset();
        scenario.worker.reset();

        var label = 'reset at tick $resetAt' + (carried ? " while carrying" : "");
        if (scenario.session.objectCarrier(scenario.part) != free) throw '$label: the part is still held';
        var captured = scenario.session.capture();
        var pose = captured.objectPose(scenario.part);
        captured.dispose();
        if (gap([pose.x, pose.y, pose.z], scenario.partStart) > 1e-6)
            throw '$label: the part did not return to the rack';
        var root = scenario.worker.body.rootTransform();
        if (Math.abs(root[12]) > 1e-9 || Math.abs(root[13]) > 1e-9)
            throw '$label: the worker did not return to its start: ${root[12]}, ${root[13]}';
        if (resetAt > 0 && Math.abs(moved[12]) + Math.abs(moved[13]) < 0.05)
            throw '$label: the worker had not moved from its start, so the reset proved nothing';
        if (scenario.worker.currentJobDone()) throw '$label: the job should start again';
        if (scenario.worker.currentStep() != 0) throw '$label: the job should be back at its first step';

        var again = runToEnd(scenario);
        if (again.pickedAt != fresh.pickedAt)
            throw '$label: picked at tick ${again.pickedAt}, a fresh run at ${fresh.pickedAt}';
        if (again.poses.length != fresh.poses.length)
            throw '$label: took ${again.poses.length} ticks, a fresh run ${fresh.poses.length}';
        // Leftover state from before the reset shows as millimetres or more. The two runs' float32
        // skeletons can still differ by an ulp for a tick, which contact turns into microns.
        for (tick in 0...fresh.poses.length)
            if (gap(fresh.poses[tick], again.poses[tick]) > 1e-5)
                throw '$label: the part path left the fresh path at tick $tick by ${gap(fresh.poses[tick], again.poses[tick])} m';
        if (gap(fresh.rest, again.rest) > 1e-5)
            throw '$label: the part came to rest ${gap(fresh.rest, again.rest)} m from where a fresh run left it';
    }

    public static function run():Void {
        var fresh = runToEnd(RackScenario.build());
        // At the start, while walking to the rack, while carrying, and while placing.
        var points = [0, 60, fresh.pickedAt + 30, fresh.poses.length - 30];
        for (point in points) checkResetAt(point, fresh);
        Sys.println('worker reset: resets at ticks ${points.join(", ")} each reran the job along the fresh path (${fresh.poses.length} ticks)');
    }
}
