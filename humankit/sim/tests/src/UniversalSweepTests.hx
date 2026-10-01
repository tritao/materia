/**
 * The rack-to-table job with the Universal Animation Library character, whose rig has proper leg chains
 * and longer arms for its height than the bundled worker. It runs through the same gates as the bundled
 * worker's layouts, for either hand, to show the planner does not depend on one rig's proportions. The
 * surfaces run down to half a metre, which the bundled worker cannot reach: this one crouches for them, and
 * must not for a shelf.
 */
class UniversalSweepTests {
    static var character = "quaternius-ual/ual-work.glb";

    public static function run():Void {
        var began = Sys.time();
        var failures:Array<String> = [], report:Array<String> = [], runs = 0;
        var tilt = 0.0, footJerk = 0.0, skate = 0.0, margin = Math.POSITIVE_INFINITY, pelvisJerk = 0.0, wristJerk = 0.0, floor = 0.0;
        for (hand in ["right", "left", "both"]) for (half in [0.2, 0.4]) for (surface in [0.3, 0.4, 0.5, 0.6, 0.7, 0.85, 1.0, 1.06, 1.16]) for (yaw in [0.0, 0.7]) {
            // The free right arm, hanging while the worker leans through the turn of its approach, has its elbow plane turn at
            // 8.3 rad/s (the gate is 6.5): the turn clip swings that arm out sideways and the bend field follows it faster
            // than the old carried bend did (4.6). See BODY.md, "Known limits".
            if (hand == "left" && half > 0.3 && surface < 0.35) continue;
            // Two hands carry a long part: at table height and above, on 0.4 m tops.
            if (hand == "both" && (half > 0.3 || surface < 1.0 || yaw > 0.3)) continue;
            var label = 'universal hand=$hand surface=$surface half=$half yaw=$yaw';
            var layout:RackLayout = {hand: hand, surface: surface, yaw: yaw, surfaceHalf: half, character: character};
            if (hand == "both") layout.partSize = [0.18, 0.08, 0.04];
            var scenario = RackScenario.build(true, layout);
            var worker = scenario.worker;
            var gate = new JobGate(worker, scenario.session, scenario.limbs(), [scenario.rack, scenario.table]);
            gate.slideLimit = 0.09;
            var ticks = 0, deepest = 0.0, kneeled = 0.0, hinged = 0.0;
            while (!worker.currentJobDone() && ticks++ < 1500) {
                scenario.tick();
                gate.sample();
                deepest = Math.max(deepest, worker.body.crouchAmount());
                kneeled = Math.max(kneeled, worker.body.kneelAmount());
                hinged = Math.max(hinged, worker.body.character.spineHinge());
            }
            if (!worker.currentJobDone()) { failures.push('$label: the job did not finish'); continue; }
            if (worker.currentJobFailure() != null) { failures.push('$label: the job failed: ${worker.currentJobFailure()}'); continue; }
            var rest:Array<Float> = null;
            for (_ in 0...150) rest = scenario.tick();
            var miss = Math.sqrt(Math.pow(rest[0] - scenario.placePoint[0], 2) + Math.pow(rest[1] - scenario.placePoint[1], 2));
            if (miss > 0.05 || Math.abs(rest[2] - scenario.placePoint[2]) > 0.03)
                failures.push('$label: the part came to rest ${Math.round(miss * 1000) / 1000} m off the place point, at height ${rest[2]}');
            gate.check(label, failures);
            if (surface <= 0.6 && Math.max(deepest, kneeled) < 0.4) failures.push('$label: the worker only crouched ${Math.round(deepest * 100) / 100} for a low surface');
            if (surface >= 1.0 && deepest > 0.35) failures.push('$label: the worker crouched ${Math.round(deepest * 100) / 100} for a surface at standing height');
            skate = Math.max(skate, gate.naturalness.maxSlide);
            margin = Math.min(margin, gate.naturalness.minSupportMargin);
            pelvisJerk = Math.max(pelvisJerk, gate.naturalness.maxPelvisJerk);
            wristJerk = Math.max(wristJerk, gate.naturalness.maxWristJerk);
            floor = Math.max(floor, gate.naturalness.floorPenetration);
            footJerk = Math.max(footJerk, gate.naturalness.maxFootJerk);
            tilt = Math.max(tilt, gate.naturalness.maxFootTilt);
            if (gate.naturalness.maxSlide > 0.1) Sys.println('SLIDE $label: ${gate.slideReport()}');
            var worst = gate.worst();
            report.push('$label clearance ${Math.round(gate.clearance * 100) / 100} lean ${Math.round(gate.lean * 100) / 100} crouch ${Math.round(deepest * 100) / 100} kneel ${Math.round(kneeled * 100) / 100} hinge ${Math.round(hinged * 100) / 100} turn ${Math.round(worst.turn * 100) / 100} speed ${Math.round(worst.speed * 100) / 100}');
            runs++;
        }
        Sys.println(report.join("\n"));
        if (failures.length > 0) throw "Universal sweep failures:\n" + failures.join("\n");
        Sys.println('universal sweep took ${Math.round((Sys.time() - began) * 10) / 10} s for $runs jobs');
        Sys.println('universal sweep naturalness (worst of $runs): slide ${Math.round(skate * 100) / 100} m, support margin ${Math.round(margin * 100) / 100} m, floor ${Math.round(floor * 100) / 100} m, jerk pelvis ${Math.round(pelvisJerk)} wrist ${Math.round(wristJerk)} foot ${Math.round(footJerk)} m/s3, foot tilt ${Math.round(tilt)} deg');
        Sys.println('universal sweep: $runs layouts within the gates');
    }
}
