package humankit;

/**
 * Every tuning number behind how a worker holds and moves its body, in one place. The values are in
 * metres and radians, tuned on the bundled worker rig (`REFERENCE_STATURE` tall); a body built without a
 * posture scales the lengths to its own stature (`forStature`), and one built with a modified copy carries
 * itself as that copy says. Nothing here is a fact about a rig's bones, which HumanKit reads from
 * the skeleton or measures instead.
 */
class HumanPosture {
	/** The height of the bundled worker rig the lengths below were tuned on, as its bounds measure it, in metres. */
	public static inline var REFERENCE_STATURE = 1.834;

	// Reaching.

	/** The fraction of arm length (shoulder to wrist) a standing reach uses; the palm adds no reliable length. */
	public var comfort:Float = 0.8;
	/** Beyond this fraction of the arm's length a wrist goal is out of reach. */
	public var reachLimit:Float = 0.95;
	/**
	 * How much of the arm's length a worker will stretch to keep clear of a surface's edge, once the lean is
	 * spent: more than the comfortable reach, and short of the limit the solver refuses at.
	 */
	public var stretch:Float = 0.91;

	/**
	 * Reach held back from the stretch when a stance is planned, in metres: the shoulder is measured at one moment
	 * and the grasp point is not exactly the planned one, so a stance planned to the stretch would sometimes be a
	 * little past what the arm can reach.
	 */
	public var reachSlack:Float = 0.03;

	/** With two hands on one object, how far each hand's grasp point sits to its own side of the object's centre, in metres. */
	public var handSpread:Float = 0.08;

	// Carrying.

	/** Where a carried wrist sits relative to the chest bone, in model space (the left hand; the right mirrors Y). */
	public var carryOffset:Array<Float> = [0.24, 0.12, -0.22];
	/** Seconds a hand takes to settle into the carry pose from wherever it was. */
	public var carryEaseSeconds:Float = 0.3;

	// Fingers: a curl of 0 is open, 1 a fist.

	/** The curl of a resting hand, of one reaching for something, and of one holding it. */
	public var relaxedCurl:Float = 0.25;
	public var openCurl:Float = 0.05;
	public var gripCurl:Float = 0.5;
	/** How fast the fingers open and close, in full curls per second. */
	public var curlRate:Float = 5.0;
	/**
	 * A held object thinner than this (across the palm, in metres) is pinched: thumb and index close on it and
	 * the other fingers stay relaxed. The thumb closes this share of the fingers' curl.
	 */
	public var pinchBelow:Float = 0.025;
	public var thumbShare:Float = 0.6;
	/** How many curl steps are measured to find the curl that brings a fingertip to a held object's far side. */
	public var graspSteps:Int = 20;

	// Leaning over a surface.

	/** The furthest the upper body leans into a reach, in radians, and how fast it leans. */
	public var maxLean:Float = 0.7;
	/** The furthest the body bends forward at the hips, on top of the lean, in radians; used when the lean is not enough to reach across. */
	public var maxHinge:Float = 0.8;
	/** Whether a hip hinge may combine with a crouch or a kneel, for a top too deep to reach across from a crouch. */
	public var hingeWithCrouch:Bool = true;
	/** What a stance's hip hinge costs in the planner's comparison of stances, per radian: a hinge is the last resort, after a deeper crouch. */
	public var hingeDiscomfort:Float = 0.25;
	public var leanRate:Float = 0.9;
	/** How long an arm that is not reaching or carrying takes to settle into hanging (and back) while the body leans, and where it hangs under the shoulder. */
	public var hangSeconds:Float = 0.5;
	public var hangDrop:Float = 0.92;
	public var hangForward:Float = 0.02;
	public var hangOutward:Float = 0.03;
	/** How far the front of the belly sits ahead of the abdomen bone, and the gap left to a surface edge. */
	public var bellyFront:Float = 0.12;
	public var edgeGap:Float = 0.03;

	/**
	 * How fast, in metres per second, a wrist is allowed to go at the peak of the easing as a reach blends in from
	 * the animation or out to it, working from the straight-line distance it has to cover. The wrist follows an arc
	 * and the IK blend is not linear in position, so its real peak is about twice this.
	 */
	public var blendSpeed:Float = 1.0;

	// Feet.

	/** Whether a planted foot is held where it is in the world, on a character whose legs are IK chains. */
	public var lockFeet:Bool = true;
	/** The share of the pose that is the idle one above which the feet are held, and the span over which the hold comes in. */
	public var lockFrom:Float = 0.15;
	public var lockSpan:Float = 0.3;
	/** Seconds the hold takes to come in and go out. */
	public var lockSeconds:Float = 0.15;
	/** How far a held foot may be from where the pose wants it, in metres, before the hold lets go. */
	public var lockReach:Float = 0.3;

	// Crouching.

	/** How fast the body lowers into a crouch and rises out of it, in full crouches per second. */
	public var crouchRate:Float = 0.8;
	/** The radius of a leg, and how far above and below a surface's slab a leg still counts as meeting it (for a kneel). */
	public var legRadius:Float = 0.1;
	public var slabMargin:Float = 0.04;
	/** The same for a kneel, which swings an arm down with the body and is taken slower. */
	public var kneelRate:Float = 0.55;
	/** How many depths between standing and the full crouch the planner tries, counting both. */
	public var crouchLevels:Int = 11;
	/** The lean a worker will put up with before it bends its knees instead, in radians. */
	public var comfortLean:Float = 0.5;

	// Withdrawing from a placed part.

	/** How far the wrists pull back toward the body, and how far they lift. */
	public var withdraw:Float = 0.25;
	public var lift:Float = 0.03;
	/** The least a withdrawn wrist stays ahead of the chest. */
	public var minAhead:Float = 0.15;

	public function new() {}

	/** The tuning the bundled worker rig was set up with. */
	public static function standard():HumanPosture
		return new HumanPosture();

	/**
	 * The tuning for a body of `stature` metres: every length, and the speed the wrist may go at, grows with
	 * the body. Angles, fractions of the arm's length, curls and times do not depend on size and are kept.
	 * `edgeGap` is also kept: it is the clearance left to a surface, which is not a matter of the worker's size.
	 */
	public static function forStature(stature:Float):HumanPosture {
		if (!(stature > 0.0)) throw "A posture needs a positive stature";
		var posture = new HumanPosture(), k = stature / REFERENCE_STATURE;
		posture.handSpread *= k;
		posture.carryOffset = [for (value in posture.carryOffset) value * k];
		posture.hangForward *= k;
		posture.hangOutward *= k;
		posture.bellyFront *= k;
		posture.blendSpeed *= k;
		posture.withdraw *= k;
		posture.lift *= k;
		posture.minAhead *= k;
		return posture;
	}
}
