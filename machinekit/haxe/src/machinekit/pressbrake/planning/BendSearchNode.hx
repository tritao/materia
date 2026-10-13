package machinekit.pressbrake.planning;

/** Search ownership is private to one planning run; the program only receives copied step arrays. */
class BendSearchNode {
	public final steps:Array<BendProgramStep>;
	public final completed:Array<Int>;
	public final seconds:Float;
	public final estimate:Float;
	public final serial:Int;
	final action:Int;
	final gripIndex:Int;
	public function new(steps:Array<BendProgramStep>,completed:Array<Int>,seconds:Float,estimate:Float,serial:Int,action:Int = -1,gripIndex:Int = -1) {
		this.steps = steps; this.completed = completed; this.seconds = seconds; this.estimate = estimate; this.serial = serial;
		this.action = action; this.gripIndex = gripIndex;
	}
	public function last():Null<BendProgramStep> return steps.length == 0 ? null : steps[steps.length-1];
	/** Ordinal action/grip keys preserve exact floats and arbitrary authored station names. */
	public function key():String return keyFor(completed.join(","),action,gripIndex);
	public static function keyFor(completed:String,action:Int,gripIndex:Int):String return completed+"/"+action+"/"+gripIndex;
	public static function compare(a:BendSearchNode,b:BendSearchNode):Int {
		if (a.estimate != b.estimate) return a.estimate < b.estimate ? -1 : 1;
		if (a.completed.length != b.completed.length) return a.completed.length > b.completed.length ? -1 : 1;
		if (a.seconds != b.seconds) return a.seconds < b.seconds ? -1 : 1;
		return a.serial < b.serial ? -1 : a.serial > b.serial ? 1 : 0;
	}
}
