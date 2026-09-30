package humankit;

/**
 * Reaches the palms to a grasp point, closes the hands there, then establishes
 * a carry pose. target is where the palm centre (HumanBody.gripPoint) goes;
 * the hands only close if they got within 2 cm of it.
 */
class Pick extends HumanActionBase {
	static inline var TOLERANCE = 0.02;

	public final target:Array<Float>;
	public final hands:Array<HumanLimb>;
	public final ramp:Float;
	/** Becomes true for one full-weight tick at the grasp point. */
	public var grip(default, null):Bool = false;
	/** Largest palm distance from its grasp point when the hands closed, in metres. */
	public var pickError(default, null):Float = 0.0;
	var elapsed:Float = 0.0;
	var fullTick:Bool = false;
	var from:Array<Array<Float>> = [];
	var guesses:Array<Array<Float>> = [];

	public function new(target:Array<Float>, hands:Array<HumanLimb>, ramp:Float = 0.35) {
		super();
		this.target = target.copy();
		this.hands = hands.copy();
		this.ramp = ramp;
	}

	override public function start(worker:HumanBody):Void {
		super.start(worker);
		if (target.length < 3 || hands.length == 0 || hands.length > 2 || ramp < 0.0) {
			fail("Pick needs a point, one or two hands, and a non-negative ramp");
			return;
		}
		for (hand in hands)
			if (hand != ArmL && hand != ArmR) { fail("Pick requires hands, not legs"); return; }
		worker.setCarry([]);
		from = [];
		guesses = [];
		for (index in 0...hands.length) {
			var wrist = worker.character.pose.bonePosition(hands[index] == ArmL ? HandL : HandR);
			if (wrist == null) { fail("Pick needs a hand bone"); return; }
			from.push(worker.toWorld(wrist));
			guesses.push(graspPoint(index));
		}
	}

	override public function advance(seconds:Float):Void {
		if (fullTick) {
			worker.setCarry(hands);
			// The grasp is made; stand up straight to carry.
			worker.setLean(0.0);
			done = true;
			return;
		}
		elapsed += seconds;
		var weight = ramp == 0.0 ? 1.0 : Math.min(1.0, elapsed / ramp);
		var goals:Array<Array<Float>> = [];
		var worst = 0.0;
		for (index in 0...hands.length) {
			var hand = hands[index];
			var solution = worker.solveReach(hand, graspPoint(index), () -> worker.gripPoint(hand),
				guesses[index], 0.3);
			if (solution.failure != null) {
				fail('Pick target ${solution.failure}');
				return;
			}
			guesses[index] = solution.goal;
			goals.push(solution.goal);
			worst = Math.max(worst, solution.error);
		}
		// Approach from where the wrists were, on the solved goals.
		for (index in 0...hands.length) {
			var start = from[index], goal = goals[index];
			worker.setReachWorld(hands[index], [for (axis in 0...3) start[axis] + (goal[axis] - start[axis]) * weight],
				1.0);
		}
		if (weight >= 1.0) {
			pickError = worst;
			if (worst > TOLERANCE) {
				fail('Pick target not reached (${worst} m from the palm)');
				return;
			}
			grip = true;
			worker.setGrip(true);
			fullTick = true;
		}
	}

	/** One hand's grasp point: the target, spread across the object for two hands. */
	function graspPoint(index:Int):Array<Float> {
		var root = worker.rootTransform();
		var side = hands[index] == ArmL ? 1.0 : -1.0;
		var spread = hands.length == 2 ? 0.08 : 0.0;
		return [target[0] + root[4] * side * spread, target[1] + root[5] * side * spread, target[2]];
	}
}
