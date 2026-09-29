package humankit;

/** Reaches a part, closes the selected hands, then establishes a carry pose. */
class Pick extends HumanActionBase {
	public final target:Array<Float>;
	public final hands:Array<HumanLimb>;
	public final ramp:Float;
	/** Becomes true for one full-weight tick at the grasp point. */
	public var grip(default, null):Bool = false;
	var elapsed:Float = 0.0;
	var fullTick:Bool = false;

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
		setWeight(0.0);
	}

	override public function advance(seconds:Float):Void {
		if (fullTick) {
			worker.setCarry(hands);
			done = true;
			return;
		}
		elapsed += seconds;
		var weight = ramp == 0.0 ? 1.0 : Math.min(1.0, elapsed / ramp);
		setWeight(weight);
		if (weight >= 1.0) {
			grip = true;
			worker.setGrip(true);
			fullTick = true;
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
