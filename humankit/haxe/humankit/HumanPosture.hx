package humankit;

/**
 * Every tuning number behind how a worker holds and moves its body, in one place. The values are in
 * metres and radians, tuned on the bundled 1.7 m worker rig; build a body with a modified copy to
 * change how it carries itself. Nothing here is a fact about a rig's bones, which HumanKit reads from
 * the skeleton or measures instead.
 */
class HumanPosture {
	// Reaching.

	/** The fraction of arm length (shoulder to wrist) a standing reach uses; the palm adds no reliable length. */
	public var comfort:Float = 0.8;
	/** Beyond this fraction of the arm's length a wrist goal is out of reach. */
	public var reachLimit:Float = 0.95;

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
	public var leanRate:Float = 0.9;
	/** Lean at which an arm that is not reaching or carrying is held fully hanging, and where it hangs under the shoulder. */
	public var hangLean:Float = 0.25;
	public var hangDrop:Float = 0.92;
	public var hangForward:Float = 0.02;
	public var hangOutward:Float = 0.03;
	/** How far the front of the belly sits ahead of the abdomen bone, and the gap left to a surface edge. */
	public var bellyFront:Float = 0.12;
	public var edgeGap:Float = 0.03;

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
}
