package robotkit.perception;

import haxe.Int64;
import robotkit.spatial.Vec3;

/**
 * An immutable set of 3D points expressed in one named frame, with the same
 * source/receive clock provenance as `SensorFrame` (ARCHITECTURE.md "Time
 * and command provenance"): `sourceTimestampNs` is the sensor/simulator
 * clock at acquisition, `receivedTimestampNs` is the local monotonic clock
 * when it was accepted (zero/unknown until stamped).
 */
class PointCloud {
  public final frameId:String;
  public final points:Array<Vec3>;
  public final sourceTimestampNs:Int64;
  public final receivedTimestampNs:Int64;
  public final sourceClockId:String;
  public final receivedClockId:String;

  public function new(frameId:String, points:Array<Vec3>, sourceTimestampNs:Int64,
      ?receivedTimestampNs:Int64, ?sourceClockId:String = "unspecified",
      ?receivedClockId:String = "robotkit.monotonic") {
    if (frameId == null || frameId.length == 0) throw "PointCloud requires a non-empty frame id";
    if (points == null) throw "PointCloud requires a points array";
    this.frameId = frameId;
    this.points = points.copy();
    this.sourceTimestampNs = sourceTimestampNs;
    this.receivedTimestampNs = receivedTimestampNs == null ? Int64.ofInt(0) : receivedTimestampNs;
    this.sourceClockId = sourceClockId;
    this.receivedClockId = receivedClockId;
  }

  public function size():Int return points.length;
}
