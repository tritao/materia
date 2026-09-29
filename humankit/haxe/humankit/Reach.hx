package humankit;

/** Ramps one limb's IK toward a world point. */
class Reach extends HumanActionBase {
	public final limb:HumanLimb;
	public final target:Array<Float>;
	public final ramp:Float;
	final modelSpace:Bool;
	final pole:Null<Array<Float>>;
	var elapsed:Float = 0.0;

	public function new(limb:HumanLimb, target:Array<Float>, ramp:Float = 0.4,
			modelSpace:Bool = false, ?pole:Array<Float>) {
		super();
		this.limb = limb;
		this.target = target.copy();
		this.ramp = ramp;
		this.modelSpace = modelSpace;
		this.pole = pole == null ? null : pole.copy();
	}

	/** Keeps HumanReachTask's historical model-space target semantics. */
	public static function inModelSpace(limb:HumanLimb, target:Array<Float>, ramp:Float,
			?pole:Array<Float>):Reach
		return new Reach(limb, target, ramp, true, pole);

	override public function start(worker:HumanBody):Void {
		super.start(worker);
		if (target.length < 3 || ramp < 0.0) { fail("Reach needs a point and non-negative ramp"); return; }
		setWeight(0.0);
	}

	override public function advance(seconds:Float):Void {
		elapsed += seconds;
		var weight = ramp == 0.0 ? 1.0 : Math.min(1.0, elapsed / ramp);
		setWeight(weight);
		if (weight >= 1.0) done = true;
	}

	function setWeight(weight:Float):Void {
		if (modelSpace) worker.setReachModel(limb, target, weight, pole);
		else worker.setReachWorld(limb, target, weight, pole);
	}
}
