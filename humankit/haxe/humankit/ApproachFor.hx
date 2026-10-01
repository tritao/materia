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
	/**
	 * How far short of the surface's edge the belly ends, in metres, once the lean and the stretch are spent:
	 * zero when the stance clears it. A surface low or deep enough to need a crouch leaves this above zero,
	 * since the worker does not crouch; the job still runs, with the belly over the edge by about this much.
	 */
	public var shortfall(default, null):Float = 0.0;
	/** How deep a crouch the worker takes at the stand (0 upright, 1 the clip's full crouch), once it has arrived. */
	public var crouch(default, null):Float = 0.0;
	var crouchIssued:Bool = false;
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
		var bone = limb == ArmL ? UpperArmL : UpperArmR;
		var shoulder = worker.standingBone(bone);
		if (shoulder == null) { fail("The rig lacks an arm"); return; }
		var bellyBone = worker.standingBone(Spine) != null ? Spine : Pelvis;
		var belly = worker.standingBone(bellyBone);
		var root = worker.rootTransform();
		// A body that can crouch tries standing first and then ever deeper, and takes the first stance that is
		// comfortable: clear of the surface's edge, without leaning further than a worker would, and with the
		// point no further below the shoulder than the arm comfortably reaches. A body that cannot crouch has
		// only the standing stance.
		var levels = worker.canCrouch() ? Std.int(Math.max(1.0, worker.posture.crouchLevels)) : 1;
		var chosen:Null<Stance> = null;
		var failure:Null<String> = null;
		for (step in 0...levels) {
			var depth = levels > 1 ? step / (levels - 1) : 0.0;
			var stance = stanceAt(worker, bone, bellyBone, depth, shoulder, belly, root);
			if (stance.failure != null) {
				if (step == 0) failure = stance.failure;
				continue;
			}
			if (chosen == null || stance.shortfall < chosen.shortfall - 0.01 ||
				(Math.abs(stance.shortfall - chosen.shortfall) <= 0.01 && stance.lean < chosen.lean - 0.01))
				chosen = stance;
			if (levels == 1 || (stance.shortfall <= 0.01 && stance.lean <= worker.posture.comfortLean && stance.comfortable)) break;
		}
		if (chosen == null) {
			fail(failure == null ? "Target is out of reach" : failure);
			return;
		}
		shortfall = chosen.shortfall;
		crouch = chosen.crouch;
		worker.setLean(chosen.lean);
		worker.setArmsDown(support != null);
		if (crouch <= 0.0) worker.setCrouch(0.0);
		var standX = target[0] - chosen.ux * chosen.standDistance + chosen.uy * chosen.lateral;
		var standY = target[1] - chosen.uy * chosen.standDistance - chosen.ux * chosen.lateral;
		faceAngle = Math.atan2(chosen.uy, chosen.ux);
		if (Math.sqrt(Math.pow(standX - root[12], 2) + Math.pow(standY - root[13], 2)) > 0.005)
			worker.walker.continueAlong([[root[12], root[13]], [standX, standY]], speed);
		else {
			worker.walker.face(faceAngle);
			turnIssued = true;
		}
	}

	/**
	 * Where the worker would stand, and how it would hold its body, to reach the target with its body crouched
	 * `depth` deep; or why it cannot at that depth.
	 */
	function stanceAt(worker:HumanBody, bone:HumanBone, bellyBone:HumanBone, depth:Float, standing:Array<Float>,
			bellyStanding:Null<Array<Float>>, root:Array<Float>):Stance {
		var drop = worker.crouchShift(bone, depth);
		var shoulder = [standing[0] + drop[0], standing[1] + drop[1], standing[2] + drop[2]];
		var length = worker.description.upperArm + worker.description.forearm;
		// With two hands the stance is centred on the object, so each shoulder stays a little to the side of
		// its hand's grasp point. That sideways gap takes its share of the arm, leaving less for the reach
		// ahead and below; measuring only those would stand the worker further out than the arm reaches.
		var sideways = bothHands ? Math.abs(Math.abs(shoulder[1]) - worker.posture.handSpread) : 0.0;
		var comfortable = inPlane(worker.posture.comfort * length, sideways), farthest = inPlane(worker.posture.stretch * length, sideways);
		var rise = target[2] - (root[14] + shoulder[2]);
		var stance:Stance = {failure: null, crouch: depth, lean: 0.0, standDistance: 0.0, shortfall: 0.0, ux: 0.0, uy: 0.0,
			lateral: 0.0, comfortable: -rise <= comfortable};
		// A point is out of reach only beyond what the arm will stretch to, not beyond the comfortable reach.
		if (rise >= farthest) {
			stance.failure = "Target is above reachable height";
			return stance;
		}
		if (-rise >= farthest) {
			stance.failure = worker.canCrouch() ? "Target is below the arm's reach even crouched" :
				"Target is below standing arm reach; crouching is unsupported";
			return stance;
		}
		// Stand where the shoulder is a comfortable reach away; a point as far below the shoulder as that has
		// the shoulder straight over it, and any more reach comes from leaning and stretching below.
		var ahead = Math.abs(rise) < comfortable ? Math.sqrt(comfortable * comfortable - rise * rise) : 0.0;
		var dx = target[0] - root[12], dy = target[1] - root[13];
		var distance = Math.sqrt(dx * dx + dy * dy);
		stance.ux = distance > 1e-8 ? dx / distance : root[0];
		stance.uy = distance > 1e-8 ? dy / distance : root[1];
		// Stand so the shoulder (model +X forward, +Y left of the root) sits
		// `ahead` metres behind the target along the facing direction.
		stance.lateral = bothHands ? 0.0 : shoulder[1];
		stance.standDistance = ahead + shoulder[0];
		if (support != null) {
			var belly = bellyStanding;
			if (belly != null) {
				var shift = worker.crouchShift(bellyBone, depth);
				belly = [belly[0] + shift[0], belly[1] + shift[1], belly[2] + shift[2]];
			}
			var required = edgeDistance(support, target, stance.ux, stance.uy) + (belly == null ? 0.0 : belly[0]) +
				worker.posture.bellyFront + worker.posture.edgeGap;
			if (required > stance.standDistance) {
				// Standing back from the edge leaves the shoulder short of the point: lean to make it up.
				var made = worker.leanFor(limb, required - stance.standDistance, depth);
				stance.lean = made.angle;
				stance.standDistance += made.shift;
				// With the lean spent, stretch the arm past its comfortable reach, up to the posture's limit.
				if (required > stance.standDistance) {
					var aheadMost = Math.sqrt(farthest * farthest - rise * rise);
					stance.standDistance += Math.min(required - stance.standDistance, Math.max(0.0, aheadMost - ahead));
				}
				stance.shortfall = Math.max(0.0, required - stance.standDistance);
			}
		}
		return stance;
	}

	/** What is left of a reach of `radius` once `sideways` of it is spent off to one side. */
	static function inPlane(radius:Float, sideways:Float):Float
		return radius > sideways ? Math.sqrt(radius * radius - sideways * sideways) : 0.0;

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
		// The body lowers once it stands where it will work and has turned to face the point.
		if (!done && turnIssued && !crouchIssued && crouch > 0.0 && !worker.walker.isTurning()) {
			crouchIssued = true;
			worker.setCrouch(crouch);
		}
	}

	override public function isDone():Bool
		return done || (turnIssued && !worker.walker.isTurning() && (crouchIssued || crouch <= 0.0) && worker.crouchReached());
}
