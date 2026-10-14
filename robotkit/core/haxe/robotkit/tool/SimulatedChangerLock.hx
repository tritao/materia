package robotkit.tool;

import haxe.Int64;

/** In-memory changer lock state driven by a digital output. */
class SimulatedChangerLock implements ChangerLock {
	public final history:Array<{locked:Bool, timestampNs:Int64}> = [];
	var locked = true;
	var mountedToolId:Null<String>;
	final sourceClockId:String;
	var sourceTimestampNs:Int64 = Int64.ofInt(0);

	public function new(initialToolId:String, ?sourceClockId:String = "robotkit.monotonic") {
		if (initialToolId == null || initialToolId.length == 0)
			throw "Simulated changer needs its initially mounted tool ID";
		if (sourceClockId == null || sourceClockId.length == 0)
			throw "Simulated changer needs a source clock ID";
		mountedToolId = initialToolId;
		this.sourceClockId = sourceClockId;
	}

	public function lock(timestampNs:Int64):Void {
		locked = true;
		sourceTimestampNs = timestampNs;
		history.push({locked: true, timestampNs: timestampNs});
	}

	public function unlock(timestampNs:Int64):Void {
		locked = false;
		sourceTimestampNs = timestampNs;
		history.push({locked: false, timestampNs: timestampNs});
	}

	/** Simulation-only dock action; a physical adapter obtains identity from its sensors. */
	public function detachTool(timestampNs:Int64):Void {
		if (locked) throw "Cannot detach a tool while the simulated changer is locked";
		if (mountedToolId == null) throw "Simulated changer has no tool to detach";
		mountedToolId = null;
		sourceTimestampNs = timestampNs;
	}

	/** Simulation-only dock action; a physical adapter obtains identity from its sensors. */
	public function attachTool(toolId:String, timestampNs:Int64):Void {
		if (locked) throw "Cannot attach a tool while the simulated changer is locked";
		if (mountedToolId != null) throw 'Simulated changer still carries tool "$mountedToolId"';
		if (toolId == null || toolId.length == 0) throw "Simulated changer needs a mounted tool ID";
		mountedToolId = toolId;
		sourceTimestampNs = timestampNs;
	}

	public function observation():ChangerLockObservation return {
		locked: locked,
		toolId: mountedToolId,
		sourceClockId: sourceClockId,
		sourceTimestampNs: sourceTimestampNs
	};
}
