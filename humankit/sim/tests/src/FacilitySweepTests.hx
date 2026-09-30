/**
 * Facility fetch-and-deliver jobs through the same gates as the rack job: a rack and a station at
 * three surface heights, joined by a straight lane, one with a corner, and one that turns back on
 * itself. Each must finish, set the part down at the station, keep its arms inside the motion-quality
 * limits, and keep the belly off the rack and station.
 */
class FacilitySweepTests {
    static var surfaces:Array<Float> = [1.0, 1.06, 1.16];

    /** Runs one facility layout and reports the belly clearance, the deepest lean, and how far the part ended from the station. */
    static function measure(layout:FacilityLayout, describe:String):{clearance:Float, lean:Float, rest:Float} {
        var scenario = FacilityScenario.build(layout, describe);
        var gate = new JobGate(scenario.worker, scenario.session, scenario.limb(), scenario.surfaces);
        var ticks = 0;
        while (!scenario.worker.currentJobDone() && ticks++ < 2400) {
            scenario.tick();
            gate.sample();
        }
        var rest:Array<Float> = null;
        for (_ in 0...180) rest = scenario.tick();
        var miss = Math.sqrt(Math.pow(rest[0] - scenario.placePoint[0], 2) + Math.pow(rest[1] - scenario.placePoint[1], 2));
        return {clearance: gate.clearance, lean: gate.lean, rest: miss};
    }

    public static function run():Void {
        var layouts:Array<{name:String, station:Array<Float>, via:Array<Array<Float>>}> = [
            {name: "straight", station: [2.55, -0.2], via: []},
            {name: "corner", station: [2.2, 1.2], via: [[0.9, 1.2]]},
            {name: "u-turn", station: [-1.6, -0.2], via: [[0.9, 1.0], [-0.9, 1.0]]}
        ];
        var failures:Array<String> = [], report:Array<String> = [], runs = 0;
        for (layout in layouts) for (surface in surfaces) {
            var label = 'lane=${layout.name} surface=$surface';
            var scenario = FacilityScenario.build({station: layout.station, via: layout.via, surface: surface});
            var worker = scenario.worker;
            var gate = new JobGate(worker, scenario.session, scenario.limb(), scenario.surfaces);
            var ticks = 0;
            while (!worker.currentJobDone() && ticks++ < 2400) {
                scenario.tick();
                gate.sample();
            }
            if (!worker.currentJobDone()) { failures.push('$label: the job did not finish'); continue; }
            if (worker.currentJobFailure() != null) { failures.push('$label: the job failed: ${worker.currentJobFailure()}'); continue; }
            var rest:Array<Float> = null;
            for (_ in 0...180) rest = scenario.tick();
            var miss = Math.sqrt(Math.pow(rest[0] - scenario.placePoint[0], 2) + Math.pow(rest[1] - scenario.placePoint[1], 2));
            if (miss > 0.06) failures.push('$label: the part came to rest ${Math.round(miss * 1000) / 1000} m from the station');
            gate.check(label, failures);
            var worst = gate.worst();
            report.push('$label clearance ${Math.round(gate.clearance * 100) / 100} lean ${Math.round(gate.lean * 100) / 100} turn ${Math.round(worst.turn * 100) / 100}');
            runs++;
        }
        if (failures.length > 0) throw "Facility sweep failures:\n" + failures.join("\n") + "\n" + report.join("\n");
        // What the facility describes must give the same job as handing the same boxes to the job.
        var layout = {station: [2.2, 1.2], via: [[0.9, 1.2]], surface: 1.06};
        var described = measure(layout, "model"), handed = measure(layout, "explicit");
        if (Math.abs(described.clearance - handed.clearance) > 1e-6 || Math.abs(described.lean - handed.lean) > 1e-6 ||
            Math.abs(described.rest - handed.rest) > 1e-6)
            throw 'A facility that describes its surfaces did not match one whose job was handed them: $described against $handed';
        var plain = measure(layout, "none");
        if (!(described.clearance > plain.clearance + 0.01))
            throw 'Describing the facility kept the belly no further from its surfaces: ${described.clearance} against ${plain.clearance}';
        Sys.println('facility sweep: $runs jobs within the gates');
    }
}
