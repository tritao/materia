package humankit.sim;

/** Safety sample from one completed simulation tick. Distances are metres. */
class HumanWorkerSignals {
	public final time:Float;
	public final zones:Array<String>;
	public final separation:Map<String, Float>;

	public function new(time:Float, zones:Array<String>, separation:Map<String, Float>) {
		this.time = time;
		this.zones = zones;
		this.separation = separation;
	}
}
