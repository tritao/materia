import humankit.MotionQuality;

/**
 * Holds the rack job's whole motion, both arms from start to finish, to limits a person's arm
 * would keep. The limits sit just above today's ordinary gait and reach; the failures they were
 * set to catch measured several times higher: an elbow flipping (11 rad/s of bend-plane turn), a
 * clip change or carry pose applied in a single tick (a wrist at 10 m/s, 1000 m/s2).
 */
class MotionQualityTests {
    static inline var MAX_PLANE_TURN = 5.0;
    static inline var MAX_HAND_SPEED = 2.5;
    static inline var MAX_HAND_ACCELERATION = 300.0;
    static inline var MIN_ELBOW_ANGLE = 30.0;

    public static function run():Void {
        var scenario = RackScenario.build();
        var pose = scenario.worker.body.character.pose;
        var quality = new MotionQuality();
        var ticks = 0;
        while (!scenario.worker.currentJobDone() && ticks++ < 900) {
            scenario.tick();
            quality.sample(pose, scenario.session.fixedTimestep());
        }
        if (!scenario.worker.currentJobDone()) throw "The motion-quality job did not finish";
        for (side in [MotionQuality.RIGHT, MotionQuality.LEFT]) {
            var name = side == MotionQuality.RIGHT ? "right" : "left";
            var arm = quality.arm(side);
            if (arm.planeTurnRate > MAX_PLANE_TURN)
                throw 'The $name elbow turned its bend plane at ${r(arm.planeTurnRate)} rad/s at sample ${arm.planeTurnAt}';
            if (arm.maxHandSpeed > MAX_HAND_SPEED)
                throw 'The $name hand moved at ${r(arm.maxHandSpeed)} m/s at sample ${arm.handSpeedAt}';
            if (arm.maxHandAcceleration > MAX_HAND_ACCELERATION)
                throw 'The $name hand accelerated at ${r(arm.maxHandAcceleration)} m/s2 at sample ${arm.handAccelerationAt}';
            if (arm.elbowAboveShoulder > 0.0)
                throw 'The $name elbow rose ${r(arm.elbowAboveShoulder)} m above the shoulder';
            if (arm.minElbowAngle < MIN_ELBOW_ANGLE)
                throw 'The $name elbow bent to ${r(arm.minElbowAngle)} degrees';
            Sys.println('$name arm: plane turn ${r(arm.planeTurnRate)} rad/s, hand ${r(arm.maxHandSpeed)} m/s ${r(arm.maxHandAcceleration)} m/s2, elbow ${r(arm.minElbowAngle)} deg at its sharpest and ${r(arm.elbowAboveShoulder)} m from the shoulder');
        }
    }

    static function r(v:Float):Float return Math.round(v * 100) / 100;
}
