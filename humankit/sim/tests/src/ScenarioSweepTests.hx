import humankit.HumanBone;
import humankit.HumanBody;
import humankit.MotionQuality;

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

    /**
     * Belly clearance allowed: a few millimetres, the walker's stopping precision; and more where the lean
     * is at its limit and cannot make up the whole reach, as at a surface low enough to need a crouch,
     * which a worker does not do. The sweep measured 3 to 4 cm there for either hand.
     */
    static inline var CLEARANCE = -0.01;
    static inline var CAPPED_CLEARANCE = -0.05;

    public static function run():Void {
        var worstClearance = Math.POSITIVE_INFINITY, worstTurn = 0.0, worstSpeed = 0.0, worstAccel = 0.0, runs = 0;
        var failures:Array<String> = [];
        for (hand in hands) for (surface in surfaces) for (yaw in yaws) {
            var label = 'hand=$hand surface=$surface yaw=$yaw';
            var scenario = RackScenario.build(true, {hand: hand, surface: surface, yaw: yaw});
            var worker = scenario.worker, body = worker.body;
            var pose = body.character.pose;
            var quality = new MotionQuality();
            var clearance = Math.POSITIVE_INFINITY, lean = 0.0, ticks = 0;
            while (!worker.currentJobDone() && ticks++ < 1200) {
                scenario.tick();
                quality.sample(pose, scenario.session.fixedTimestep());
                lean = Math.max(lean, body.character.spineLean());
                if (body.reachWeight(scenario.limb()) <= 0.0 || !(worker.currentStep() == 0 || body.grip)) continue;
                var belly = pose.bonePosition(HumanBone.Spine);
                var front = body.toWorld([belly[0] + 0.12, belly[1], belly[2]]);
                clearance = Math.min(clearance, Math.min(TorsoClearanceTests.outside(front, scenario.rack),
                    TorsoClearanceTests.outside(front, scenario.table)));
            }
            if (!worker.currentJobDone()) { failures.push('$label: the job did not finish'); continue; }
            if (worker.currentJobFailure() != null) { failures.push('$label: the job failed: ${worker.currentJobFailure()}'); continue; }
            // Let the part settle, then it must rest where it was sent.
            var rest:Array<Float> = null;
            for (_ in 0...150) rest = scenario.tick();
            var miss = Math.sqrt(Math.pow(rest[0] - scenario.placePoint[0], 2) + Math.pow(rest[1] - scenario.placePoint[1], 2));
            if (miss > 0.05 || Math.abs(rest[2] - scenario.placePoint[2]) > 0.03)
                failures.push('$label: the part came to rest ${Math.round(miss * 1000) / 1000} m off the place point, at height ${rest[2]}');
            for (side in [MotionQuality.RIGHT, MotionQuality.LEFT]) {
                var arm = quality.arm(side), name = side == MotionQuality.RIGHT ? "right" : "left";
                if (arm.planeTurnRate > MotionQualityTests.MAX_PLANE_TURN)
                    failures.push('$label: the $name elbow turned its bend plane at ${r(arm.planeTurnRate)} rad/s at sample ${arm.planeTurnAt}');
                if (arm.maxHandSpeed > MotionQualityTests.MAX_HAND_SPEED)
                    failures.push('$label: the $name hand moved at ${r(arm.maxHandSpeed)} m/s at sample ${arm.handSpeedAt}');
                if (arm.maxHandAcceleration > MotionQualityTests.MAX_HAND_ACCELERATION)
                    failures.push('$label: the $name hand accelerated at ${r(arm.maxHandAcceleration)} m/s2 at sample ${arm.handAccelerationAt}');
                if (arm.elbowAboveShoulder > 0.0) failures.push('$label: the $name elbow rose ${r(arm.elbowAboveShoulder)} m above the shoulder');
                if (arm.minElbowAngle < MotionQualityTests.MIN_ELBOW_ANGLE)
                    failures.push('$label: the $name elbow bent to ${r(arm.minElbowAngle)} degrees');
                worstTurn = Math.max(worstTurn, arm.planeTurnRate);
                worstSpeed = Math.max(worstSpeed, arm.maxHandSpeed);
                worstAccel = Math.max(worstAccel, arm.maxHandAcceleration);
            }
            var capped = lean >= body.posture.maxLean - 0.01;
            if (clearance < (capped ? CAPPED_CLEARANCE : CLEARANCE))
                failures.push('$label: the belly stood ${r(-clearance)} m inside a surface (lean ${r(lean)} rad${capped ? ", at its limit" : ""})');
            worstClearance = Math.min(worstClearance, clearance);
            runs++;
        }
        if (failures.length > 0) throw "Scenario sweep failures:\n" + failures.join("\n");
        Sys.println('scenario sweep: $runs layouts, worst belly clearance ${r(worstClearance)} m, plane turn ${r(worstTurn)} rad/s, hand ${r(worstSpeed)} m/s ${r(worstAccel)} m/s2');
    }

    static function r(v:Float):Float return Math.round(v * 100) / 100;
}
