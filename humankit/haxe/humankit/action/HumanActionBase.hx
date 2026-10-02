package humankit.action;

import humankit.HumanBody;

/** Common completion and failure state for concrete actions. */
class HumanActionBase implements HumanAction {
	public var worker(default, null):HumanBody;
	var done:Bool = false;
	var error:Null<String> = null;

	public function new() {}

	public function start(worker:HumanBody):Void
		this.worker = worker;

	public function advance(seconds:Float):Void {}

	public function isDone():Bool
		return done;

	public function failure():Null<String>
		return error;

	/** Eases 0 to 1 with no jerk at either end. */
	static function smooth(t:Float):Float {
		var clamped = Math.max(0.0, Math.min(1.0, t));
		return clamped * clamped * (3.0 - 2.0 * clamped);
	}

	function fail(message:String):Void {
		error = message;
		done = true;
	}
}
