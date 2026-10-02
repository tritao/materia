import humankit.rig.HumanBone;

/**
 * Pressing a wall panel with either hand, at three heights and three turns of the wall, through the same
 * gates as the carrying jobs. The job must finish without a failure, keep both arms inside the
 * motion-quality limits, and really bring the wrist to the panel: a press that stops short is a press that
 * does nothing.
 */
class PressSweepTests {
    static var hands:Array<String> = ["right", "left"];
    static var heights:Array<Float> = [1.05, 1.2, 1.35];
    static var yaws:Array<Float> = [0.0, 1.0, -1.4];
    /** How close the wrist must come to the point on the panel, in metres. */
    static inline var CONTACT = 0.03;

    public static function run():Void {
        var failures:Array<String> = [], report:Array<String> = [], runs = 0;
        for (hand in hands) for (height in heights) for (yaw in yaws) {
            var label = 'press hand=$hand height=$height yaw=$yaw';
            var scenario = PressScenario.build(hand, height, yaw);
            var worker = scenario.worker, body = worker.body;
            var gate = new JobGate(worker, scenario.session, [scenario.hand], []);
            var closest = Math.POSITIVE_INFINITY, ticks = 0;
            while (!worker.currentJobDone() && ticks++ < 900) {
                scenario.tick();
                gate.sample();
                var wrist = body.toWorld(body.character.pose.bonePosition(scenario.hand == humankit.HumanLimb.ArmL ? HumanBone.HandL : HumanBone.HandR));
                closest = Math.min(closest, Math.sqrt(Math.pow(wrist[0] - scenario.pressPoint[0], 2) + Math.pow(wrist[1] - scenario.pressPoint[1], 2) +
                    Math.pow(wrist[2] - scenario.pressPoint[2], 2)));
            }
            if (!worker.currentJobDone()) { failures.push('$label: the job did not finish'); continue; }
            if (worker.currentJobFailure() != null) { failures.push('$label: the job failed: ${worker.currentJobFailure()}'); continue; }
            if (closest > CONTACT) failures.push('$label: the wrist came only ${Math.round(closest * 1000) / 1000} m from the panel');
            gate.check(label, failures);
            var worst = gate.worst();
            report.push('$label contact ${Math.round(closest * 1000) / 1000} turn ${Math.round(worst.turn * 100) / 100} speed ${Math.round(worst.speed * 100) / 100}');
            runs++;
        }
        if (failures.length > 0) throw "Press sweep failures:\n" + failures.join("\n") + "\n" + report.join("\n");
        Sys.println('press sweep: $runs presses within the gates');
    }
}
