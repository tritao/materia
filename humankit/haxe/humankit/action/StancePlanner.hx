package humankit.action;

import humankit.HumanBody;
import humankit.HumanLimb;
import humankit.HumanTargetBox;
import humankit.rig.HumanBone;

/**
 * Chooses where a worker stands, how it faces and how it holds its body (crouched or kneeling, leaned, bent at the hips) to
 * reach a point with one arm or two, optionally over a surface it has to keep its belly and knees clear of.
 *
 * Standing is tried first. A worker crouches only for a point lower than the arm comfortably reaches below the shoulder:
 * crouching brings the shoulder down, not forward over a surface, so it does nothing for a point at table height. For a
 * lower one, ever deeper crouches are tried and the first that is easy is taken; failing that, the least hard of those
 * tried. A point lower than a crouch reaches is taken from a knee instead, if that is clearly the easier stance. How hard
 * a stance is has one measure (`discomfort`), so every comparison below uses the same one.
 */
class StancePlanner {
	final worker:HumanBody;
	final target:Array<Float>;
	final limb:HumanLimb;
	final bothHands:Bool;
	final support:Null<HumanTargetBox>;
	// What the planning works from, found once in `plan`.
	var bone:HumanBone = UpperArmR;
	var planLimb:HumanLimb = ArmR;
	var standingShoulder:Array<Float> = [0.0, 0.0, 0.0];
	var bellyBone:HumanBone = Pelvis;
	var standingBelly:Null<Array<Float>> = null;
	var root:Array<Float> = [];

	public function new(worker:HumanBody, target:Array<Float>, limb:HumanLimb, bothHands:Bool, ?support:HumanTargetBox) {
		this.worker = worker;
		this.target = target;
		this.limb = limb;
		this.bothHands = bothHands;
		this.support = support;
	}

	/** The stance to take, or one whose `failure` says why there is none. */
	public function plan():Stance {
		bone = limb == ArmL ? UpperArmL : UpperArmR;
		var shoulder = worker.standingBone(bone);
		if (shoulder == null) return failed("The rig lacks an arm");
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
		standingShoulder = shoulder;
		planLimb = bone == UpperArmL ? ArmL : ArmR;
		bellyBone = worker.standingBone(Spine) != null ? Spine : Pelvis;
		standingBelly = worker.standingBone(bellyBone);
		root = worker.rootTransform();

		var levels = worker.canCrouch() ? Std.int(Math.max(1.0, worker.posture.crouchLevels)) : 1;
		var chosen:Null<Stance> = null;
		var failure:Null<String> = null;
		// Whether the point is within comfortable reach below the shoulder standing: then no way down helps, and a surface that
		// is merely deep is a matter of bending over it, not of kneeling.
		var reachesStanding = false;
		for (step in 0...levels) {
			var depth = levels > 1 ? step / (levels - 1) : 0.0;
			var stance = stanceAt(depth, false);
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
			if (easy(stance)) {
				chosen = stance;
				break;
			}
		}
		// A point lower than a crouch reaches comfortably is taken from a knee instead, if that is comfortable and the
		// crouch was not (or reached nothing).
		if (worker.canKneel() && (chosen == null || (!easy(chosen) && !reachesStanding))) {
			var kneeling:Null<Stance> = null;
			for (step in 1...levels) {
				var stance = stanceAt(step / (levels - 1), true);
				if (stance.failure != null) continue;
				if (kneeling == null || discomfort(stance) < discomfort(kneeling) - 0.01)
					kneeling = stance;
				if (easy(stance)) {
					kneeling = stance;
					break;
				}
			}
			// A crouch is kept unless the kneel is clearly the easier way: people crouch for a bench and kneel for the floor.
			if (kneeling != null && (chosen == null || discomfort(kneeling) < discomfort(chosen) - 0.25)) chosen = kneeling;
		}
		return chosen != null ? chosen : failed(failure == null ? "Target is out of reach" : failure);
	}

	/**
	 * How hard a stance is: how far short of the edge it leaves the worker, how far past comfortable reach, how near the
	 * arm's limit it stretches, how far past a comfortable lean, and how far it bends at the hips.
	 */
	function discomfort(stance:Stance):Float {
		var posture = worker.posture;
		return stance.shortfall * 4.0 + (stance.comfortable ? 0.0 : 0.15) + (stance.tight ? posture.tightDiscomfort : 0.0) +
			Math.max(0.0, stance.lean - posture.comfortLean) + stance.hinge * posture.hingeDiscomfort;
	}

	/** Whether a stance is one a worker would take without being pushed to it: at the edge, not leaning past comfort, in easy reach. */
	function easy(stance:Stance):Bool
		return stance.shortfall <= 0.01 && stance.lean <= worker.posture.comfortLean && stance.comfortable;

	/**
	 * Where the worker would stand, and how it would hold its body, to reach the target with its body down `depth`, crouched or
	 * (with `kneeling`) kneeling; or why it cannot at that depth.
	 */
	function stanceAt(depth:Float, kneeling:Bool):Stance {
		var drop = kneeling ? worker.kneelShift(bone, depth) : worker.crouchShift(bone, depth);
		var shoulder = [standingShoulder[0] + drop[0], standingShoulder[1] + drop[1], standingShoulder[2] + drop[2]];
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
		var stance:Stance = {failure: null, crouch: kneeling ? 0.0 : depth, kneel: kneeling ? depth : 0.0, lean: 0.0, hinge: 0.0,
			standDistance: 0.0, shortfall: 0.0, ux: 0.0, uy: 0.0, lateral: 0.0, comfortable: -rise <= comfortable, tight: false};
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
		faceTheSurface(stance);
		// Stand so the shoulder (model +X forward, +Y left of the root) sits
		// `ahead` metres behind the target along the facing direction.
		stance.lateral = bothHands ? 0.0 : shoulder[1];
		stance.standDistance = ahead + shoulder[0];
		var finalRise = rise;
		var surface = support;
		if (surface != null) {
			var required = standOffFor(surface, stance, depth, kneeling);
			if (required > stance.standDistance) {
				finalRise = makeUpByPitch(stance, required, shoulder, rise, farthest, depth, kneeling);
				if (stance.failure != null) return stance;
			}
		}
		// A reach that ends within a hand's width of the arm's limit is a stretch, whatever it took to get there.
		stance.tight = Math.abs(finalRise) > farthest - worker.posture.reachMargin;
		return stance;
	}

	/**
	 * A worker at a surface stands square to the edge it works at, not along the line it walked up: the nearest of the four
	 * directions the box's edges face, so the belly and the knees meet the edge head on and the stand-off is the true one.
	 * Without a surface it faces the point from where it stands.
	 */
	function faceTheSurface(stance:Stance):Void {
		var dx = target[0] - root[12], dy = target[1] - root[13];
		var distance = Math.sqrt(dx * dx + dy * dy);
		stance.ux = distance > 1e-8 ? dx / distance : root[0];
		stance.uy = distance > 1e-8 ? dy / distance : root[1];
		var faced = support;
		if (faced == null) return;
		var c = Math.cos(faced.yaw), s = Math.sin(faced.yaw);
		var best = -2.0, bx = stance.ux, by = stance.uy;
		for (candidate in [[c, s], [-c, -s], [-s, c], [s, -c]]) {
			var along = candidate[0] * stance.ux + candidate[1] * stance.uy;
			if (along > best) { best = along; bx = candidate[0]; by = candidate[1]; }
		}
		stance.ux = bx;
		stance.uy = by;
	}

	/**
	 * How far from the target the worker has to stand for the belly and, when the body is down, the knees and thighs to stay
	 * behind the surface's near edge.
	 */
	function standOffFor(surface:HumanTargetBox, stance:Stance, depth:Float, kneeling:Bool):Float {
		var belly = standingBelly;
		if (belly != null) {
			var shift = kneeling ? worker.kneelShift(bellyBone, depth) : worker.crouchShift(bellyBone, depth);
			belly = [belly[0] + shift[0], belly[1] + shift[1], belly[2] + shift[2]];
		}
		// The belly travels along a line parallel to the facing, a shoulder's width to the side of the target's: where an
		// edge is not square to the facing, that line meets it at a different distance from the target's own.
		var edge = edgeDistance(surface, [target[0] + stance.uy * stance.lateral, target[1] - stance.ux * stance.lateral], stance.ux, stance.uy);
		var required = edge + (belly == null ? 0.0 : belly[0]) + worker.posture.bellyFront + worker.posture.edgeGap;
		// A belly held above the surface's top (a worker kneeling at a low shelf) overhangs the edge instead of meeting it; the
		// knees and thighs, below, still have to stay behind.
		var top = surface.center[2] + surface.halfExtents[2];
		if (belly != null && root[14] + belly[2] - worker.posture.bellyHalfHeight - worker.posture.bellyOverhangMargin > top + worker.posture.slabMargin)
			required = 0.0;
		// A body that is down puts its knees and thighs at about the height of a low top, where they would meet the slab:
		// the legs have to stay behind the edge wherever they cross the slab's thickness.
		if (depth > 0.0) {
			var thickness = 2.0 * surface.halfExtents[2] + worker.posture.slabMargin;
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
		return required;
	}

	/**
	 * Standing back from the edge leaves the shoulder short of the point: lean to make it up, and bend at the hips as well for a
	 * top too deep to reach across by leaning. The lean carries the shoulder forward and also lowers it, both as measured, and
	 * the arm then stretches past its comfortable reach, but no further from the shoulder where it ends up than the posture
	 * allows. Sets the stance's lean, hinge, stand-off and shortfall; returns how far the point is above (+) or below (-) the
	 * shoulder where it ends up.
	 */
	function makeUpByPitch(stance:Stance, requiredAtFirst:Float, shoulder:Array<Float>, rise:Float, farthest:Float, depth:Float, kneeling:Bool):Float {
		var required = requiredAtFirst;
		var lean = function(gap:Float) return kneeling ? worker.leanFor(planLimb, gap, 0.0, depth, bellyBone) : worker.leanFor(planLimb, gap, depth, 0.0, bellyBone);
		var made = lean(required - stance.standDistance);
		// The lean carries the belly toward the edge along with the shoulder, so the worker stands that much further
		// back, and leans for the larger gap that leaves; the second round is the correction to the first.
		for (round in 0...2) {
			if (!(made.belly > 0.002)) break;
			required = requiredAtFirst + made.belly;
			made = lean(required - stance.standDistance);
		}
		required = requiredAtFirst + made.belly;
		stance.lean = made.angle;
		var riseAfter = rise - made.drop;
		if (Math.abs(riseAfter) >= farthest) {
			stance.failure = riseAfter > 0.0 ? "Target is above reachable height" : "Target is below the arm's reach when leaning";
			return riseAfter;
		}
		var reachMost = shoulder[0] + made.shift + Math.sqrt(farthest * farthest - riseAfter * riseAfter) - worker.posture.reachSlack;
		stance.standDistance = Math.max(stance.standDistance, Math.min(required, reachMost));
		stance.shortfall = Math.max(0.0, required - stance.standDistance);
		var finalRise = riseAfter;
		if (stance.shortfall > 0.01 && worker.posture.maxHinge > 0.0 && (depth <= 0.0 || worker.posture.hingeWithCrouch)) {
			var bent = worker.hingeFor(planLimb, stance.shortfall, kneeling ? 0.0 : depth, kneeling ? depth : 0.0, made.angle, bellyBone);
			stance.hinge = bent.angle;
			var riseBent = riseAfter - bent.drop;
			if (Math.abs(riseBent) < farthest) {
				var reachBent = shoulder[0] + made.shift + bent.shift + Math.sqrt(farthest * farthest - riseBent * riseBent) - worker.posture.reachSlack;
				stance.standDistance = Math.max(stance.standDistance, Math.min(required, reachBent));
				stance.shortfall = Math.max(0.0, required - stance.standDistance);
				finalRise = riseBent;
			}
		}
		return finalRise;
	}

	static function failed(message:String):Stance
		return {failure: message, crouch: 0.0, kneel: 0.0, lean: 0.0, hinge: 0.0, standDistance: 0.0, shortfall: 0.0, ux: 0.0, uy: 0.0,
			lateral: 0.0, comfortable: false, tight: false};

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
}
