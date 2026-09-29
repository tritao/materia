package humankit;

/** Reaches a place point, opens the hands there, then eases IK out. */
class Place extends HumanActionBase {
	public final target:Array<Float>;
	public final hands:Array<HumanLimb>;
	public final ramp:Float;
	/** False once the part should be released by the simulation layer. */
	public var grip(default, null):Bool = true;
	var stage:Int = 0;
	var elapsed:Float = 0.0;

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
		worker.setCarry([]);
		setWeight(0.0);
	}

	override public function advance(seconds:Float):Void {
		if (stage == 0) {
			elapsed += seconds;
			var weight = ramp == 0.0 ? 1.0 : Math.min(1.0, elapsed / ramp);
			setWeight(weight);
			if (weight >= 1.0) {
				grip = false;
				worker.setGrip(false);
				stage = 1;
			}
		} else if (stage == 1) {
			stage = 2;
			elapsed = 0.0;
		} else {
			elapsed += seconds;
			var weight = ramp == 0.0 ? 0.0 : Math.max(0.0, 1.0 - elapsed / ramp);
			setWeight(weight);
			if (weight <= 0.0) {
				for (hand in hands) worker.clearReach(hand);
				done = true;
			}
		}
	}

	function setWeight(weight:Float):Void {
		var root = worker.rootTransform();
		for (hand in hands) {
			var side = hand == ArmL ? 1.0 : -1.0;
			var spread = hands.length == 2 ? 0.08 : 0.0;
			worker.setReachWorld(hand, [target[0] + root[4] * side * spread,
				target[1] + root[5] * side * spread, target[2]], weight);
		}
	}
}
