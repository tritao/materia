/**
 * The rack-to-table job with the Universal Animation Library character, whose rig has proper leg chains
 * and longer arms for its height than the bundled worker. It runs through the same gates as the bundled
 * worker's layouts, for either hand, to show the planner does not depend on one rig's proportions. The
 * surfaces run down to half a metre, which the bundled worker cannot reach: this one crouches for them, and
 * must not for a shelf.
 */
class UniversalSweepTests {
    static var character = "quaternius-ual/ual-standard.glb";

    public static function run():Void {
        var failures:Array<String> = [], report:Array<String> = [], runs = 0;
        for (hand in ["right", "left"]) for (surface in [0.5, 0.6, 0.7, 0.85, 1.0, 1.06, 1.16]) for (yaw in [0.0, 0.7]) {
            var label = 'universal hand=$hand surface=$surface yaw=$yaw';
            var scenario = RackScenario.build(true, {hand: hand, surface: surface, yaw: yaw, character: character});
            var worker = scenario.worker;
            var gate = new JobGate(worker, scenario.session, scenario.limbs(), [scenario.rack, scenario.table]);
            var ticks = 0, deepest = 0.0;
            while (!worker.currentJobDone() && ticks++ < 1500) {
                scenario.tick();
                gate.sample();
                deepest = Math.max(deepest, worker.body.crouchAmount());
            }
            if (!worker.currentJobDone()) { failures.push('$label: the job did not finish'); continue; }
            if (worker.currentJobFailure() != null) { failures.push('$label: the job failed: ${worker.currentJobFailure()}'); continue; }
            var rest:Array<Float> = null;
            for (_ in 0...150) rest = scenario.tick();
            var miss = Math.sqrt(Math.pow(rest[0] - scenario.placePoint[0], 2) + Math.pow(rest[1] - scenario.placePoint[1], 2));
            if (miss > 0.05 || Math.abs(rest[2] - scenario.placePoint[2]) > 0.03)
                failures.push('$label: the part came to rest ${Math.round(miss * 1000) / 1000} m off the place point, at height ${rest[2]}');
            gate.check(label, failures);
            if (surface <= 0.6 && deepest < 0.4) failures.push('$label: the worker only crouched ${Math.round(deepest * 100) / 100} for a low surface');
            if (surface >= 1.06 && deepest > 0.5) failures.push('$label: the worker crouched ${Math.round(deepest * 100) / 100} for a surface at standing height');
            var worst = gate.worst();
            report.push('$label clearance ${Math.round(gate.clearance * 100) / 100} lean ${Math.round(gate.lean * 100) / 100} crouch ${Math.round(deepest * 100) / 100} turn ${Math.round(worst.turn * 100) / 100} speed ${Math.round(worst.speed * 100) / 100}');
            runs++;
        }
        if (failures.length > 0) Sys.println(report.join("\n"));
        if (failures.length > 0) throw "Universal sweep failures:\n" + failures.join("\n");
        Sys.println('universal sweep: $runs layouts within the gates');
    }
}
