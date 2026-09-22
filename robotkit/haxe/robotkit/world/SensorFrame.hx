package robotkit.world;

import haxe.Int64;

/** One immutable sensor observation shared by simulated and remote robots. */
class SensorFrame {
  public final sensorId:String;
  public final kind:String;
  public final frameId:String;
  public final sequence:Int64;
  public final sourceTimestampNs:Int64;
  public final receivedTimestampNs:Int64;
  public final values:ImmutableFloatArray;

  public function new(sensorId:String, kind:String, frameId:String, sequence:Int64,
      sourceTimestampNs:Int64, values:Array<Float>, ?receivedTimestampNs:Int64) {
    this.sensorId = sensorId;
    this.kind = kind;
    this.frameId = frameId;
    this.sequence = sequence;
    this.sourceTimestampNs = sourceTimestampNs;
    this.receivedTimestampNs = receivedTimestampNs == null
      ? Int64.ofInt(0) : receivedTimestampNs;
    this.values = new ImmutableFloatArray(values);
  }
}
