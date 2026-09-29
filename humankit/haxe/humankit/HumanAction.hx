package humankit;

/** One step of a worker job. Targets supplied by actions are world-space. */
interface HumanAction {
	public function start(worker:HumanBody):Void;
	public function advance(seconds:Float):Void;
	public function isDone():Bool;
	public function failure():Null<String>;
}
