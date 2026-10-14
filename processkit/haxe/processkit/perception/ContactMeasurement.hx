package processkit.perception;

import haxe.Int64;
import robotkit.spatial.Vec3;

/** One fresh contact observation, with the clock and coordinate facts used by registration. */
class ContactMeasurement {
	public final sensorId:String;
	public final sensorFrame:String;
	public final sensorKind:String;
	public final coordinateFrame:String;
	public final unit:String;
	public final sequence:Int64;
	public final sourceTimestampNs:Int64;
	public final sourceClockId:String;
	public final jointTimestampNs:Int64;
	public final clockErrorNs:Int64;
	public final point:Vec3;
	public final uncertaintyMeters:Float;

	public function new(sensorId:String, sensorFrame:String, sensorKind:String, coordinateFrame:String,
		sequence:Int64, sourceTimestampNs:Int64, sourceClockId:String, jointTimestampNs:Int64,
		clockErrorNs:Int64, point:Vec3, uncertaintyMeters:Float) {
		if (sensorId == null || sensorId.length == 0 || sensorFrame == null || sensorFrame.length == 0 ||
			sensorKind == null || sensorKind.length == 0 || coordinateFrame == null || coordinateFrame.length == 0 ||
			sequence == null || Int64.compare(sequence, Int64.ofInt(0)) <= 0 || sourceTimestampNs == null ||
			Int64.compare(sourceTimestampNs, Int64.ofInt(0)) < 0 || sourceClockId == null || sourceClockId.length == 0 ||
			sourceClockId == "unspecified" ||
			jointTimestampNs == null || Int64.compare(jointTimestampNs, Int64.ofInt(0)) < 0 || clockErrorNs == null ||
			Int64.compare(clockErrorNs, Int64.ofInt(0)) < 0 || point == null || !Math.isFinite(point.x) ||
			!Math.isFinite(point.y) || !Math.isFinite(point.z) || !Math.isFinite(uncertaintyMeters) || uncertaintyMeters < 0)
			throw "Contact measurement needs an identified fresh source, metre-valued point, clock bound and uncertainty";
		this.sensorId = sensorId;
		this.sensorFrame = sensorFrame;
		this.sensorKind = sensorKind;
		this.coordinateFrame = coordinateFrame;
		this.unit = "m";
		this.sequence = sequence;
		this.sourceTimestampNs = sourceTimestampNs;
		this.sourceClockId = sourceClockId;
		this.jointTimestampNs = jointTimestampNs;
		this.clockErrorNs = clockErrorNs;
		this.point = point;
		this.uncertaintyMeters = uncertaintyMeters;
	}
}
