/**
 * The rack-to-table job across the layouts a worker meets: either hand, surfaces from a low bench to
 * high shelf, and the layout turned about the worker. Each run must finish without a failure, set the
 * part down where it was sent, keep its arms within the motion-quality limits, and keep the belly off
 * the surfaces it reaches over. One scenario can hide a limit tuned to it; a sweep cannot.
 */
class ScenarioSweepTests {
    static var hands:Array<String> = ["right", "left"];
    static var surfaces:Array<Float> = [1.0, 1.06, 1.16];
    static var yaws:Array<Float> = [0.0, 0.7, -1.1];

    public static function run():Void {
        var worstClearance = Math.POSITIVE_INFINITY, worstTurn = 0.0, worstSpeed = 0.0, worstAccel = 0.0, runs = 0;
        var failures:Array<String> = [];
        for (hand in hands) for (surface in surfaces) for (yaw in yaws) {
            var label = 'hand=$hand surface=$surface yaw=$yaw';
            var scenario = RackScenario.build(true, {hand: hand, surface: surface, yaw: yaw});
            var worker = scenario.worker;
            var gate = new JobGate(worker, scenario.session, scenario.limbs(), [scenario.rack, scenario.table]);
            var ticks = 0;
            while (!worker.currentJobDone() && ticks++ < 1200) {
                scenario.tick();
                gate.sample();
            }
            if (!worker.currentJobDone()) { failures.push('$label: the job did not finish'); continue; }
            if (worker.currentJobFailure() != null) { failures.push('$label: the job failed: ${worker.currentJobFailure()}'); continue; }
            // Let the part settle, then it must rest where it was sent.
            var rest:Array<Float> = null;
            for (_ in 0...150) rest = scenario.tick();
            var miss = Math.sqrt(Math.pow(rest[0] - scenario.placePoint[0], 2) + Math.pow(rest[1] - scenario.placePoint[1], 2));
            if (miss > 0.05 || Math.abs(rest[2] - scenario.placePoint[2]) > 0.03)
                failures.push('$label: the part came to rest ${Math.round(miss * 1000) / 1000} m off the place point, at height ${rest[2]}');
            gate.check(label, failures);
            var worst = gate.worst();
            worstTurn = Math.max(worstTurn, worst.turn);
            worstSpeed = Math.max(worstSpeed, worst.speed);
            worstAccel = Math.max(worstAccel, worst.accel);
            worstClearance = Math.min(worstClearance, gate.clearance);
            runs++;
        }
        if (failures.length > 0) throw "Scenario sweep failures:\n" + failures.join("\n");
        Sys.println('scenario sweep: $runs layouts, worst belly clearance ${r(worstClearance)} m, plane turn ${r(worstTurn)} rad/s, hand ${r(worstSpeed)} m/s ${r(worstAccel)} m/s2');
    }

    static function r(v:Float):Float return Math.round(v * 100) / 100;
}
