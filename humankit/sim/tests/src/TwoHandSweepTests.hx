/**
 * A wide part carried in both hands from a rack to a table, through the same gates as the one-handed job:
 * three surface heights and three turns of the layout. Each must finish, set the part down where it was
 * sent, keep both arms inside the motion-quality limits, and keep the belly off the rack and table.
 * The part is 36 cm long, so it is carried by its ends and the surfaces under it are wider than the
 * one-handed job's.
 */
class TwoHandSweepTests {
    static var surfaces:Array<Float> = [1.0, 1.06, 1.16];
    static var yaws:Array<Float> = [0.0, 0.7, -1.1];

    public static function run():Void {
        var failures:Array<String> = [], report:Array<String> = [], runs = 0;
        for (surface in surfaces) for (yaw in yaws) {
            var label = 'both hands surface=$surface yaw=$yaw';
            var scenario = RackScenario.build(true, {hand: "both", surface: surface, yaw: yaw, partSize: [0.18, 0.08, 0.04]});
            var worker = scenario.worker;
            var gate = new JobGate(worker, scenario.session, scenario.limbs(), [scenario.rack, scenario.table]);
            var ticks = 0;
            while (!worker.currentJobDone() && ticks++ < 1500) {
                scenario.tick();
                gate.sample();
            }
            if (!worker.currentJobDone()) { failures.push('$label: the job did not finish'); continue; }
            if (worker.currentJobFailure() != null) { failures.push('$label: the job failed: ${worker.currentJobFailure()}'); continue; }
            var rest:Array<Float> = null;
            for (_ in 0...200) rest = scenario.tick();
            var miss = Math.sqrt(Math.pow(rest[0] - scenario.placePoint[0], 2) + Math.pow(rest[1] - scenario.placePoint[1], 2));
            if (miss > 0.06 || Math.abs(rest[2] - scenario.placePoint[2]) > 0.03)
                failures.push('$label: the part came to rest ${Math.round(miss * 1000) / 1000} m off the place point, at height ${rest[2]}');
            // At a metre the arms have about 10 cm of reach left after the drop from the shoulder, and two
            // hands spend some of it sideways, so the worker cannot stand clear of the edge without
            // crouching. That is held to completing, placing the part, moving without spikes, and not
            // standing deeper in the edge than the measured 17 cm.
            var lowest = surface <= 1.0;
            // The planner knows which layouts need that crouch, so the belly's depth is not the only witness. Its estimate
            // is of the stance, not the moving body: a couple of centimetres at 1.06 m that the belly then clears.
            var shortfall = worker.approachShortfall();
            if (lowest != (shortfall > 0.05))
                failures.push('$label: the approach fell ${Math.round(shortfall * 100) / 100} m short, where ${lowest ? "a crouch is needed" : "the stance should clear the edge"}');
            gate.check(label, failures, !lowest);
            if (lowest && gate.clearance < JobGate.CROUCH_CLEARANCE)
                failures.push('$label: the belly stood ${Math.round(-gate.clearance * 100) / 100} m inside a surface, beyond the ${-JobGate.CROUCH_CLEARANCE} m measured for a reach that needs a crouch');
            var worst = gate.worst();
            report.push('$label clearance ${Math.round(gate.clearance * 100) / 100} lean ${Math.round(gate.lean * 100) / 100} turn ${Math.round(worst.turn * 100) / 100} speed ${Math.round(worst.speed * 100) / 100}');
            runs++;
        }
        if (failures.length > 0) throw "Two-hand sweep failures:\n" + failures.join("\n") + "\n" + report.join("\n");
        Sys.println('two-hand sweep: $runs layouts within the gates');
    }
}
