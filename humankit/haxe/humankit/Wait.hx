package humankit;

/** Keeps the current animation and IK pose for a fixed duration. */
class Wait extends HumanActionBase {
	public final seconds:Float;
	var elapsed:Float = 0.0;

	public function new(seconds:Float) {
		super();
		this.seconds = seconds;
	}

	override public function start(worker:HumanBody):Void {
		super.start(worker);
		if (seconds < 0.0) fail("Wait duration cannot be negative");
		else if (seconds == 0.0) done = true;
	}

	override public function advance(seconds:Float):Void {
		elapsed += seconds;
		if (elapsed >= this.seconds) done = true;
	}
}
