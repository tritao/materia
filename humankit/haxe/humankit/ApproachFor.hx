package humankit;

/**
 * Walks to where the limb's shoulder has a comfortable reach to a point and
 * faces so the point lies straight ahead of that shoulder, before a reach.
 * The stand-off accounts for the height between shoulder and point: a low
 * point is approached closer than one at shoulder height.
 *
 * Given the surface the point lies on, the worker also stands far enough back
 * that the belly stays clear of its near edge, and leans the upper body
 * forward over it to keep the shoulder where the reach is comfortable.
 */
class ApproachFor extends HumanActionBase {
	public var target(default, null):Array<Float>;
	public final limb:HumanLimb;
	public final speed:Float;
	public final bothHands:Bool;
	var faceAngle:Float = 0.0;
	var turnIssued:Bool = false;
	final targetProvider:Null<Void->Array<Float>>;
	final support:Null<HumanTargetBox>;

	public function new(target:Array<Float>, limb:HumanLimb, speed:Float = 1.0, bothHands:Bool = false,
		?targetProvider:Void->Array<Float>, ?support:HumanTargetBox) {
		super();
		this.support = support;
		this.target = target.copy();
		this.limb = limb;
		this.speed = speed;
		this.bothHands = bothHands;
		this.targetProvider = targetProvider;
	}

	override public function start(worker:HumanBody):Void {
		super.start(worker);
		if (targetProvider != null) target = targetProvider().copy();
		if (target.length < 3) { fail("Approach target needs x, y, z"); return; }
		if (limb != ArmL && limb != ArmR) { fail("Approach requires an arm"); return; }
		var shoulder = worker.standingBone(limb == ArmL ? UpperArmL : UpperArmR);
		if (shoulder == null) { fail("The rig lacks an arm"); return; }
		var root = worker.rootTransform();
		var length = worker.description.upperArm + worker.description.forearm;
		var comfortable = worker.posture.comfort * length, farthest = worker.posture.stretch * length;
		var rise = target[2] - (root[14] + shoulder[2]);
		// A point is out of reach only beyond what the arm will stretch to, not beyond the comfortable reach.
		if (rise >= farthest) {
			fail("Target is above reachable height");
			return;
		}
		if (-rise >= farthest) {
			fail("Target is below standing arm reach; crouching is unsupported");
			return;
		}
		// Stand where the shoulder is a comfortable reach away; a point as far below the shoulder as that has
		// the shoulder straight over it, and any more reach comes from leaning and stretching below.
		var ahead = Math.abs(rise) < comfortable ? Math.sqrt(comfortable * comfortable - rise * rise) : 0.0;
		var dx = target[0] - root[12], dy = target[1] - root[13];
		var distance = Math.sqrt(dx * dx + dy * dy);
		var ux = distance > 1e-8 ? dx / distance : root[0];
		var uy = distance > 1e-8 ? dy / distance : root[1];
		// Stand so the shoulder (model +X forward, +Y left of the root) sits
		// `ahead` metres behind the target along the facing direction.
		var lateral = bothHands ? 0.0 : shoulder[1];
		var standDistance = ahead + shoulder[0];
		var lean = 0.0;
		if (support != null) {
			var belly = worker.standingBone(Spine);
			if (belly == null) belly = worker.standingBone(Pelvis);
			var required = edgeDistance(support, target, ux, uy) + (belly == null ? 0.0 : belly[0]) +
				worker.posture.bellyFront + worker.posture.edgeGap;
			if (required > standDistance) {
				// Standing back from the edge leaves the shoulder short of the point: lean to make it up.
				var made = worker.leanFor(limb, required - standDistance);
				lean = made.angle;
				standDistance += made.shift;
				// With the lean spent, stretch the arm past its comfortable reach, up to the posture's limit.
				if (required > standDistance) {
					var aheadMost = Math.sqrt(farthest * farthest - rise * rise);
					standDistance += Math.min(required - standDistance, Math.max(0.0, aheadMost - ahead));
				}
			}
		}
		worker.setLean(lean);
		var standX = target[0] - ux * standDistance + uy * lateral;
		var standY = target[1] - uy * standDistance - ux * lateral;
		faceAngle = Math.atan2(uy, ux);
		if (Math.sqrt(Math.pow(standX - root[12], 2) + Math.pow(standY - root[13], 2)) > 0.005)
			worker.walker.continueAlong([[root[12], root[13]], [standX, standY]], speed);
		else {
			worker.walker.face(faceAngle);
			turnIssued = true;
		}
	}

	/**
	 * How far back from `point` along (-ux, -uy) the surface's footprint runs: the walk from the
	 * point out to the edge the worker stands at. Zero when the point is not over the footprint.
	 */
	static function edgeDistance(box:HumanTargetBox, point:Array<Float>, ux:Float, uy:Float):Float {
		var c = Math.cos(box.yaw), s = Math.sin(box.yaw);
		var dx = point[0] - box.center[0], dy = point[1] - box.center[1];
		var px = c * dx + s * dy, py = -s * dx + c * dy;
		var vx = c * -ux + s * -uy, vy = -s * -ux + c * -uy;
		var hx = box.halfExtents[0], hy = box.halfExtents[1];
		if (Math.abs(px) > hx || Math.abs(py) > hy) return 0.0;
		var tx = vx > 1e-9 ? (hx - px) / vx : vx < -1e-9 ? (-hx - px) / vx : Math.POSITIVE_INFINITY;
		var ty = vy > 1e-9 ? (hy - py) / vy : vy < -1e-9 ? (-hy - py) / vy : Math.POSITIVE_INFINITY;
		return Math.max(0.0, Math.min(tx, ty));
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
