package machinekit.pressbrake;

/** Stations reserve their complete span on a bed; touching ends are permitted. */
class ToolLayout {
	public final bedLength:Float;
	final installed:Array<ToolStation>;

	public function new(bedLength:Float, stations:Array<ToolStation>) {
		if (!Math.isFinite(bedLength) || !(bedLength > 0) || stations == null) throw "Tool layout needs a positive bed length and stations";
		this.bedLength = bedLength;
		installed = stations.copy();
		for (i in 0...installed.length) {
			var station = installed[i];
			if (station == null) throw "Tool layout has a null station";
			if (station.end > bedLength) throw 'Tool station "${station.id}" extends past the bed';
			for (j in 0...i) {
				var other = installed[j];
				if (station.id == other.id) throw 'Duplicate tool station "${station.id}"';
				if (station.offset < other.end && other.offset < station.end)
					throw 'Tool stations "${station.id}" and "${other.id}" overlap';
			}
		}
	}

	public function stations():Array<ToolStation> return installed.copy();
}
