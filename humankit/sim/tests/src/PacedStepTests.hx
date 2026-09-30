/**
 * The worker is fed on the simulation clock, one call per tick, so the owner's
 * frame pattern cannot change what a carried part does. A UI frame that stalls
 * for many ticks must leave the part on exactly the same path as smooth frames.
 */
class PacedStepTests {
    static inline var TICK_NS = 10000000;

    static function trace(framePattern:Array<Int>):Array<Array<Float>> {
        var scenario = RackScenario.build();
        var poses:Array<Array<Float>> = [];
        var frame = 0;
        while (poses.length < 900 && !scenario.worker.currentJobDone()) {
            var elapsed = haxe.Int64.ofInt(framePattern[frame++ % framePattern.length] * TICK_NS);
            var due = scenario.session.dueTicks(elapsed, 1000);
            for (_ in 0...due) poses.push(scenario.tick());
        }
        if (!scenario.worker.currentJobDone()) throw "Paced job did not finish: " + scenario.worker.currentJobFailure();
        if (scenario.worker.currentJobFailure() != null) throw "Paced job failed: " + scenario.worker.currentJobFailure();
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
