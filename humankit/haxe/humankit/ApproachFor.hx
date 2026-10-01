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
	 * zero when the stance clears it. A surface too deep for the arm to reach over (a crouch does not help: it
	 * lowers the shoulder, not carries it over the edge), or a body that cannot crouch at a low one, leaves this above
	 * zero; the job still runs, with the belly over the edge by about this much.
	 */
	public var shortfall(default, null):Float = 0.0;
	/** How deep a crouch the worker takes at the stand (0 upright, 1 the clip's full crouch), once it has arrived. */
	public var crouch(default, null):Float = 0.0;
	/** How deep a kneel the worker takes instead of a crouch, for a point too low for one (0 when it does not kneel). */
	public var kneel(default, null):Float = 0.0;
	/** How far the upper body leans over the surface at the stand, in radians, once the worker has arrived. */
	public var lean(default, null):Float = 0.0;
	/** How far it bends at the hips on top of that, for a surface too deep to reach across by leaning alone. */
	public var hinge(default, null):Float = 0.0;
	var postureIssued:Bool = false;
	/** The arm whose shoulder the stance is planned for: the action's own, or for two hands the one further back. */
	var planLimb:HumanLimb = ArmR;
	/** The walk to the stand, held back while the body stands up from a crouch. */
	var waiting:Null<Array<Array<Float>>> = null;
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
		// Two hands reach with both arms: the stance has to suit the shoulder that is further back, which on a body whose idle
		// pose twists a little is the farther one from the point, not the one of the limb the action was given.
		if (bothHands) {
			var otherBone = limb == ArmL ? UpperArmR : UpperArmL;
			var other = worker.standingBone(otherBone);
			if (other != null && other[0] < shoulder[0]) {
				bone = otherBone;
				shoulder = other;
			}
		}
		planLimb = bone == UpperArmL ? ArmL : ArmR;
		var bellyBone = worker.standingBone(Spine) != null ? Spine : Pelvis;
		var belly = worker.standingBone(bellyBone);
		var root = worker.rootTransform();
		// Standing is tried first. A worker crouches only for a point lower than the arm comfortably reaches below
		// the shoulder: crouching brings the shoulder down, not forward over a surface, so it does nothing for a
		// point at table height. For a lower one, ever deeper crouches are tried and the first that is comfortable
		// is taken: clear of the surface's edge, leaning no more than a worker would, the point in comfortable reach.
		// Failing that, the best of those tried: least short of the edge, then least lean.
		var levels = worker.canCrouch() ? Std.int(Math.max(1.0, worker.posture.crouchLevels)) : 1;
		// How hard a stance is: how far short of the edge it leaves the worker, how far past comfortable reach, how far past
		// a comfortable lean, and how far it bends at the hips.
		var discomfort = function(stance:Stance):Float
			return stance.shortfall * 4.0 + (stance.comfortable ? 0.0 : 0.15) + Math.max(0.0, stance.lean - worker.posture.comfortLean) +
				stance.hinge * worker.posture.hingeDiscomfort;
		var chosen:Null<Stance> = null;
		var failure:Null<String> = null;
		// Whether the point is within comfortable reach below the shoulder standing: then no way down helps, and a surface that
		// is merely deep is a matter of bending over it, not of kneeling.
		var reachesStanding = false;
		for (step in 0...levels) {
			var depth = levels > 1 ? step / (levels - 1) : 0.0;
			var stance = stanceAt(worker, bone, bellyBone, depth, shoulder, belly, root);
			if (stance.failure != null) {
				if (step == 0) failure = stance.failure;
				continue;
			}
			if (chosen == null || discomfort(stance) < discomfort(chosen) - 0.01)
				chosen = stance;
			if (step == 0 && stance.comfortable) {
				reachesStanding = true;
				break;
			}
			if (stance.shortfall <= 0.01 && stance.lean <= worker.posture.comfortLean && stance.comfortable) {
				chosen = stance;
				break;
			}
		}
		// A point lower than a crouch reaches comfortably is taken from a knee instead, if that is comfortable and the
		// crouch was not (or reached nothing).
		var easy = function(stance:Stance):Bool return stance.shortfall <= 0.01 && stance.lean <= worker.posture.comfortLean && stance.comfortable;
		if (worker.canKneel() && (chosen == null || (!easy(chosen) && !reachesStanding))) {
			var kneeling:Null<Stance> = null;
			for (step in 1...levels) {
				var stance = stanceAt(worker, bone, bellyBone, step / (levels - 1), shoulder, belly, root, true);
				if (stance.failure != null) continue;
				if (kneeling == null || discomfort(stance) < discomfort(kneeling) - 0.01)
					kneeling = stance;
				if (easy(stance)) {
					kneeling = stance;
					break;
				}
			}
			// Kneel when it is the easier stance by that measure.
			// A crouch is kept unless the kneel is clearly the easier way: people crouch for a bench and kneel for the floor.
			if (kneeling != null && (chosen == null || discomfort(kneeling) < discomfort(chosen) - 0.25)) chosen = kneeling;
		}
		if (chosen == null) {
			fail(failure == null ? "Target is out of reach" : failure);
			return;
		}
		shortfall = chosen.shortfall;
		crouch = chosen.crouch;
		kneel = chosen.kneel;
		lean = chosen.lean;
		hinge = chosen.hinge;
		worker.setLean(lean);
		worker.setArmsDown(support != null);
		var standX = target[0] - chosen.ux * chosen.standDistance + chosen.uy * chosen.lateral;
		var standY = target[1] - chosen.uy * chosen.standDistance - chosen.ux * chosen.lateral;
		faceAngle = Math.atan2(chosen.uy, chosen.ux) - (bothHands ? worker.standingTwist() : 0.0);
		if (Math.sqrt(Math.pow(standX - root[12], 2) + Math.pow(standY - root[13], 2)) > 0.005) {
			var route = [[root[12], root[13]], [standX, standY]];
			// A crouched body does not walk: it stands up first, and the walk starts when it has.
			if (worker.downAmount() > 1e-3 || !worker.downReached()) {
				worker.setCrouch(0.0);
				worker.setKneel(0.0);
				waiting = route;
			} else
				worker.walker.continueAlong(route, speed);
		} else {
			worker.walker.face(faceAngle);
			turnIssued = true;
		}
	}

	/**
	 * Where the worker would stand, and how it would hold its body, to reach the target with its body crouched
	 * `depth` deep; or why it cannot at that depth.
	 */
	function stanceAt(worker:HumanBody, bone:HumanBone, bellyBone:HumanBone, depth:Float, standing:Array<Float>,
			bellyStanding:Null<Array<Float>>, root:Array<Float>, kneeling:Bool = false):Stance {
		var drop = kneeling ? worker.kneelShift(bone, depth) : worker.crouchShift(bone, depth);
		var shoulder = [standing[0] + drop[0], standing[1] + drop[1], standing[2] + drop[2]];
		// Two hands face the point with the shoulders square, which turns the body by the idle pose's twist, so the shoulder is
		// planned where that puts it.
		if (bothHands && worker.standingTwist() != 0.0) {
			var square = -worker.standingTwist(), c = Math.cos(square), s = Math.sin(square);
			shoulder = [shoulder[0] * c - shoulder[1] * s, shoulder[0] * s + shoulder[1] * c, shoulder[2]];
		}
		var length = worker.description.upperArm + worker.description.forearm;
		// With two hands the stance is centred on the object, so each shoulder stays a little to the side of
		// its hand's grasp point. That sideways gap takes its share of the arm, leaving less for the reach
		// ahead and below; measuring only those would stand the worker further out than the arm reaches.
		var sideways = bothHands ? Math.abs(Math.abs(shoulder[1]) - worker.posture.handSpread) : 0.0;
		// The arm brings the wrist; the point being reached for is the palm's, a little beyond it.
		var comfortable = inPlane(worker.posture.comfort * length + worker.palmReach, sideways);
		var farthest = inPlane(worker.posture.stretch * length + worker.palmReach, sideways);
		var rise = target[2] - (root[14] + shoulder[2]);
		var stance:Stance = {failure: null, crouch: kneeling ? 0.0 : depth, kneel: kneeling ? depth : 0.0, lean: 0.0, hinge: 0.0, standDistance: 0.0, shortfall: 0.0, ux: 0.0, uy: 0.0,
			lateral: 0.0, comfortable: -rise <= comfortable};
		// A point is out of reach only beyond what the arm will stretch to, not beyond the comfortable reach.
		if (rise >= farthest) {
			stance.failure = "Target is above reachable height";
			return stance;
		}
		if (-rise >= farthest) {
			stance.failure = worker.canKneel() ? "Target is below the arm's reach even kneeling" :
				worker.canCrouch() ? "Target is below the arm's reach even crouched" : "Target is below standing arm reach; crouching is unsupported";
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
				var shift = kneeling ? worker.kneelShift(bellyBone, depth) : worker.crouchShift(bellyBone, depth);
				belly = [belly[0] + shift[0], belly[1] + shift[1], belly[2] + shift[2]];
			}
			var edge = edgeDistance(support, target, stance.ux, stance.uy);
			var required = edge + (belly == null ? 0.0 : belly[0]) + worker.posture.bellyFront + worker.posture.edgeGap;
			// A body that is down puts its knees and thighs at about the height of a low top, where they would meet the slab:
			// the legs have to stay behind the edge wherever they cross the slab's thickness.
			var slab = support;
			if (depth > 0.0 && slab != null) {
				var top = slab.center[2] + slab.halfExtents[2];
				var thickness = 2.0 * slab.halfExtents[2] + worker.posture.slabMargin;
				for (legBones in [[Pelvis, ShinL, FootL], [Pelvis, ShinR, FootR]]) {
					var chain:Array<Array<Float>> = [];
					for (legBone in legBones) {
						var base = worker.standingBone(legBone);
						if (base == null) break;
						var move = kneeling ? worker.kneelShift(legBone, depth) : worker.crouchShift(legBone, depth);
						chain.push([base[0] + move[0], base[1] + move[1], base[2] + move[2]]);
					}
					if (chain.length < 3) continue;
					for (segment in 0...2) for (part in 0...6) {
						var t = part / 6.0, from = chain[segment], to = chain[segment + 1];
						var z = root[14] + from[2] + (to[2] - from[2]) * t;
						if (z > top + worker.posture.slabMargin || z < top - thickness) continue;
						required = Math.max(required, edge + from[0] + (to[0] - from[0]) * t + worker.posture.legRadius + worker.posture.edgeGap);
					}
				}
			}
			if (required > stance.standDistance) {
				// Standing back from the edge leaves the shoulder short of the point: lean to make it up. The lean
				// carries the shoulder forward and also lowers it, both as measured, and the arm then stretches
				// past its comfortable reach, but no further from the shoulder where it ends up than the posture allows.
				var made = kneeling ? worker.leanFor(planLimb, required - stance.standDistance, 0.0, depth) : worker.leanFor(planLimb, required - stance.standDistance, depth);
				stance.lean = made.angle;
				var riseAfter = rise - made.drop;
				if (Math.abs(riseAfter) >= farthest) {
					stance.failure = riseAfter > 0.0 ? "Target is above reachable height" : "Target is below the arm's reach when leaning";
					return stance;
				}
				var reachMost = shoulder[0] + made.shift + Math.sqrt(farthest * farthest - riseAfter * riseAfter) - worker.posture.reachSlack;
				stance.standDistance = Math.max(stance.standDistance, Math.min(required, reachMost));
				stance.shortfall = Math.max(0.0, required - stance.standDistance);
				// A top too deep to reach across by leaning: bend at the hips as well, which carries the shoulder further.
				if (stance.shortfall > 0.01 && worker.posture.maxHinge > 0.0 && (depth <= 0.0 || worker.posture.hingeWithCrouch)) {
					var bent = worker.hingeFor(planLimb, stance.shortfall, kneeling ? 0.0 : depth, kneeling ? depth : 0.0, made.angle);
					stance.hinge = bent.angle;
					var riseBent = riseAfter - bent.drop;
					if (Math.abs(riseBent) < farthest) {
						var reachBent = shoulder[0] + made.shift + bent.shift + Math.sqrt(farthest * farthest - riseBent * riseBent) - worker.posture.reachSlack;
						stance.standDistance = Math.max(stance.standDistance, Math.min(required, reachBent));
						stance.shortfall = Math.max(0.0, required - stance.standDistance);
					}
				}
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
		var route = waiting;
		if (route != null && worker.downAmount() <= 1e-3) {
			waiting = null;
			worker.walker.continueAlong(route, speed);
		}
		if (!done && waiting == null && !turnIssued && !worker.walker.isWalking()) {
			worker.walker.face(faceAngle);
			turnIssued = true;
		}
		// The body lowers and leans once it stands where it will work and has turned to face the point, not on the
		// way there, and the reach waits until it has: a shoulder still on its way would put the point out of reach.
		if (!done && turnIssued && !postureIssued && !worker.walker.isTurning()) {
			postureIssued = true;
			worker.setHinge(hinge);
			worker.setCrouch(crouch);
			worker.setKneel(kneel);
		}
	}

	override public function isDone():Bool
		return done || (turnIssued && !worker.walker.isTurning() && postureIssued && worker.downReached() && worker.leanReached() && worker.walker.settled());
}
