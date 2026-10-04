package robotkit.tool;

import haxe.Int64;

/** In-memory changer lock state driven by a digital output. */
class SimulatedChangerLock implements ChangerLock {
	public final history:Array<{locked:Bool, timestampNs:Int64}> = [];
	var locked = true;

	public function new() {}

	public function lock(timestampNs:Int64):Void {
		locked = true;
		history.push({locked: true, timestampNs: timestampNs});
	}

	public function unlock(timestampNs:Int64):Void {
		locked = false;
		history.push({locked: false, timestampNs: timestampNs});
	}

	public function isLocked():Bool return locked;
}
