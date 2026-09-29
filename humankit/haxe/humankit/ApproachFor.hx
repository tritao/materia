package humankit;

/**
 * Walks to where the limb's shoulder has a comfortable reach to a point and
 * faces so the point lies straight ahead of that shoulder, before a reach.
 * The stand-off accounts for the height between shoulder and point: a low
 * point is approached closer than one at shoulder height.
 */
class ApproachFor extends HumanActionBase {
	/**
	 * Fraction of the arm (shoulder to wrist) a standing reach uses. The palm
	 * adds no reliable length: IK sets only the wrist, so the hand may hang
	 * across the reach direction.
	 */
	static inline var COMFORT = 0.8;

	public final target:Array<Float>;
	public final limb:HumanLimb;
	public final speed:Float;
	public final bothHands:Bool;
	var faceAngle:Float = 0.0;
	var turnIssued:Bool = false;

	public function new(target:Array<Float>, limb:HumanLimb, speed:Float = 1.0, bothHands:Bool = false) {
		super();
		this.target = target.copy();
		this.limb = limb;
		this.speed = speed;
		this.bothHands = bothHands;
	}

	override public function start(worker:HumanBody):Void {
		super.start(worker);
		if (target.length < 3) { fail("Approach target needs x, y, z"); return; }
		if (limb != ArmL && limb != ArmR) { fail("Approach requires an arm"); return; }
		var shoulder = worker.character.pose.bonePosition(limb == ArmL ? UpperArmL : UpperArmR);
		if (shoulder == null) { fail("The rig lacks an arm"); return; }
		var root = worker.rootTransform();
		var comfortable = COMFORT * (worker.description.upperArm + worker.description.forearm);
		var rise = target[2] - (root[14] + shoulder[2]);
		if (rise >= comfortable) {
			fail("Target is above reachable height");
			return;
		}
		if (-rise >= comfortable) {
			fail("Target is below waist height; crouching is unsupported");
			return;
		}
		var ahead = Math.sqrt(comfortable * comfortable - rise * rise);
		var dx = target[0] - root[12], dy = target[1] - root[13];
		var distance = Math.sqrt(dx * dx + dy * dy);
		var ux = distance > 1e-8 ? dx / distance : root[0];
		var uy = distance > 1e-8 ? dy / distance : root[1];
		// Stand so the shoulder (model +X forward, +Y left of the root) sits
		// `ahead` metres behind the target along the facing direction.
		var lateral = bothHands ? 0.0 : shoulder[1];
		var standX = target[0] - ux * (ahead + shoulder[0]) + uy * lateral;
		var standY = target[1] - uy * (ahead + shoulder[0]) - ux * lateral;
		faceAngle = Math.atan2(uy, ux);
		if (Math.sqrt(Math.pow(standX - root[12], 2) + Math.pow(standY - root[13], 2)) > 0.005)
			worker.walker.continueAlong([[root[12], root[13]], [standX, standY]], speed);
		else {
			worker.walker.face(faceAngle);
			turnIssued = true;
		}
	}

	override public function advance(seconds:Float):Void {
		if (!done && !turnIssued && !worker.walker.isWalking()) {
			worker.walker.face(faceAngle);
			turnIssued = true;
		}
	}

	override public function isDone():Bool
		return done || (turnIssued && !worker.walker.isTurning());
}
