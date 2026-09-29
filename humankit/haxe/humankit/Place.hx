package humankit;

/**
 * Places what the hands hold at target, then opens them and eases the IK out.
 * With a held-point provider (HumanBody.setHeldPoint, set by the simulation
 * layer while a hand holds an object), target is where the object's reference
 * point goes and the wrists are solved to put it there; without one, target is
 * where the palms go. The object keeps the orientation it was carried with.
 */
class Place extends HumanActionBase {
	static inline var TOLERANCE = 0.005;

	public final target:Array<Float>;
	public final hands:Array<HumanLimb>;
	public final ramp:Float;
	/** False once the part should be released by the simulation layer. */
	public var grip(default, null):Bool = true;
	/** True after the hands have reached the placing point. */
	public var atTarget(default, null):Bool = false;
	/** Distance from the placed point to target when the hands opened, in metres. */
	public var placementError(default, null):Float = 0.0;
	var stage:Int = 0;
	var elapsed:Float = 0.0;
	var stableTime:Float = 0.0;
	var from:Array<Array<Float>> = [];
	var guesses:Array<Array<Float>> = [];
	var goals:Array<Array<Float>> = [];
	var caps:Array<Float> = [];
	var previous:Array<Null<Array<Float>>> = [];

	public function new(target:Array<Float>, hands:Array<HumanLimb>, ramp:Float = 0.35) {
		super();
		this.target = target.copy();
		this.hands = hands.copy();
		this.ramp = ramp;
	}

	override public function start(worker:HumanBody):Void {
		super.start(worker);
		if (target.length < 3 || hands.length == 0 || ramp < 0.0) {
			fail("Place needs a point, hands, and a non-negative ramp");
			return;
		}
		from = [];
		guesses = [];
		caps = [];
		previous = [];
		for (index in 0...hands.length) {
			var hand = hands[index];
			var wrist = worker.character.pose.bonePosition(hand == ArmL ? HandL : HandR);
			if (wrist == null) { fail("Place needs a hand bone"); return; }
			var start = worker.toWorld(wrist);
			from.push(start);
			// First guess: carry the point's current offset from the wrist to the target.
			var point = placedPoint(hand);
			var goal = placeGoal(index);
			for (axis in 0...3) goal[axis] += start[axis] - point[axis];
			guesses.push(goal);
			caps.push(distance(start, point) + 0.1);
			previous.push(null);
		}
		goals = [for (guess in guesses) guess.copy()];
		worker.setCarry([]);
	}

	override public function advance(seconds:Float):Void {
		if (stage == 2) {
			elapsed += seconds;
			var weight = ramp == 0.0 ? 0.0 : Math.max(0.0, 1.0 - elapsed / ramp);
			for (index in 0...hands.length)
				worker.setReachWorld(hands[index], goals[index], weight);
			if (weight <= 0.0) {
				for (hand in hands) worker.clearReach(hand);
				done = true;
			}
			return;
		}
		elapsed += seconds;
		var worst = 0.0;
		for (index in 0...hands.length) {
			var hand = hands[index];
			var solution = worker.solveReach(hand, placeGoal(index), () -> placedPoint(hand), guesses[index],
				caps[index]);
			if (solution.failure != null) {
				fail('Place target ${solution.failure}');
				return;
			}
			guesses[index] = solution.goal;
			goals[index] = solution.goal;
			worst = Math.max(worst, solution.error);
		}
		var weight = stage == 1 || ramp == 0.0 ? 1.0 : Math.min(1.0, elapsed / ramp);
		for (index in 0...hands.length) {
			var start = from[index], goal = goals[index];
			worker.setReachWorld(hands[index], [for (axis in 0...3) start[axis] + (goal[axis] - start[axis]) * weight],
				1.0);
		}
		if (stage == 0) {
			if (weight >= 1.0) {
				atTarget = true;
				stage = 1;
				elapsed = 0.0;
			}
			return;
		}
		// Settle: open the hands once the placed point is on target and still,
		// or after half a second at the best reachable pose.
		placementError = worst;
		var speed = 0.0;
		for (index in 0...hands.length) {
			var point = placedPoint(hands[index]);
			var last = previous[index];
			if (last != null && seconds > 0.0) speed = Math.max(speed, distance(last, point) / seconds);
			previous[index] = point;
		}
		stableTime = worst < TOLERANCE && speed < 0.01 ? stableTime + seconds : 0.0;
		if (stableTime >= 0.15 || elapsed >= 0.5) {
			grip = false;
			worker.setGrip(false);
			stage = 2;
			elapsed = 0.0;
		}
	}

	/** The point Place puts on target: the held object's point, else the palm. */
	function placedPoint(hand:HumanLimb):Array<Float> {
		var held = worker.heldPoint(hand);
		return held != null ? held : worker.gripPoint(hand);
	}

	/** One hand's target, spread across the object for two hands. */
	function placeGoal(index:Int):Array<Float> {
		var root = worker.rootTransform();
		var side = hands[index] == ArmL ? 1.0 : -1.0;
		var spread = hands.length == 2 && worker.heldPoint(hands[index]) == null ? 0.08 : 0.0;
		return [target[0] + root[4] * side * spread, target[1] + root[5] * side * spread, target[2]];
	}

	static function distance(a:Array<Float>, b:Array<Float>):Float
		return Math.sqrt(Math.pow(a[0] - b[0], 2) + Math.pow(a[1] - b[1], 2) + Math.pow(a[2] - b[2], 2));
}
