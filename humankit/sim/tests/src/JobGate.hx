import humankit.HumanBone;
import humankit.HumanBody;
import humankit.HumanLimb;
import humankit.HumanTargetBox;
import humankit.MotionQuality;
import humankit.sim.HumanWorker;
import nativekit.sim.SimSession;

/**
 * Holds one worker's job to the same rules wherever it runs: both arms inside the motion-quality limits
 * the whole way, and the belly off the surfaces it reaches over. Call `sample` after every tick and
 * `check` when the job is done; a scenario says which surfaces there are and which hand works.
 */
class JobGate {
    /**
     * Belly clearance allowed: a few millimetres, the walker's stopping precision; and more where the lean
     * is at its limit and cannot make up the whole reach, as at a surface low enough to need a crouch,
     * which a worker does not do. The rack sweep measured 3 to 4 cm there for either hand.
     */
    public static inline var CLEARANCE = -0.01;
    public static inline var CAPPED_CLEARANCE = -0.05;
    static inline var BELLY_FRONT = 0.12;

    public final quality:MotionQuality = new MotionQuality();
    public var clearance(default, null):Float = Math.POSITIVE_INFINITY;
    public var lean(default, null):Float = 0.0;
    final worker:HumanWorker;
    final session:SimSession;
    final surfaces:Array<HumanTargetBox>;
    final limb:HumanLimb;
    /** Whether the hand has held something yet: reaching before that is a pick, and after letting go a retreat. */
    var gripped:Bool = false;

    public function new(worker:HumanWorker, session:SimSession, limb:HumanLimb, surfaces:Array<HumanTargetBox>) {
        this.worker = worker;
        this.session = session;
        this.limb = limb;
        this.surfaces = surfaces;
    }

    public function sample():Void {
        var body = worker.body, pose = body.character.pose;
        quality.sample(pose, session.fixedTimestep());
        lean = Math.max(lean, body.character.spineLean());
        if (body.grip) gripped = true;
        // Only while reaching for the part, or holding it to place it: walking back past a surface after
        // the hand lets go is not standing at it.
        if (body.reachWeight(limb) <= 0.0 || !(!gripped || body.grip)) return;
        var belly = pose.bonePosition(HumanBone.Spine);
        var front = body.toWorld([belly[0] + BELLY_FRONT, belly[1], belly[2]]);
        for (surface in surfaces) clearance = Math.min(clearance, TorsoClearanceTests.outside(front, surface));
    }

    /** Appends to `failures` every rule the run broke, each naming `label`. */
    public function check(label:String, failures:Array<String>):Void {
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
        }
        var capped = lean >= worker.body.posture.maxLean - 0.01;
        if (clearance < (capped ? CAPPED_CLEARANCE : CLEARANCE))
            failures.push('$label: the belly stood ${r(-clearance)} m inside a surface (lean ${r(lean)} rad${capped ? ", at its limit" : ""})');
    }

    /** The worst of each arm measure over both arms, for a one-line report. */
    public function worst():{turn:Float, speed:Float, accel:Float} {
        var turn = 0.0, speed = 0.0, accel = 0.0;
        for (side in [MotionQuality.RIGHT, MotionQuality.LEFT]) {
            var arm = quality.arm(side);
            turn = Math.max(turn, arm.planeTurnRate);
            speed = Math.max(speed, arm.maxHandSpeed);
            accel = Math.max(accel, arm.maxHandAcceleration);
        }
        return {turn: turn, speed: speed, accel: accel};
    }

    static function r(v:Float):Float return Math.round(v * 100) / 100;
}
