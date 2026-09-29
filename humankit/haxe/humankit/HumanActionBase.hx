package humankit;

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

	function fail(message:String):Void {
		error = message;
		done = true;
	}
}
