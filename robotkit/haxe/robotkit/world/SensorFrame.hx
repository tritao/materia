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
  public final sourceClockId:String;
  public final receivedClockId:String;
  public final values:ImmutableFloatArray;
  public final linkId:String;
  public final mountPosition:ImmutableFloatArray;
  public final mountRotation:ImmutableFloatArray;
  /** Pixels for camera observations; immutable, so copies share it. */
  public final image:Null<CameraImage>;

  public function new(sensorId:String, kind:String, frameId:String, sequence:Int64,
      sourceTimestampNs:Int64, values:Array<Float>, ?receivedTimestampNs:Int64,
      ?linkId:String = "", ?mountPosition:Array<Float>, ?mountRotation:Array<Float>,
      ?sourceClockId:String = "unspecified", ?receivedClockId:String = "robotkit.monotonic",
      ?image:CameraImage) {
    this.sensorId = sensorId;
    this.kind = kind;
    this.frameId = frameId;
    this.sequence = sequence;
    this.sourceTimestampNs = sourceTimestampNs;
    this.receivedTimestampNs = receivedTimestampNs == null
      ? Int64.ofInt(0) : receivedTimestampNs;
    this.sourceClockId = sourceClockId;
    this.receivedClockId = receivedClockId;
    this.values = new ImmutableFloatArray(values);
    this.linkId = linkId;
    this.mountPosition = new ImmutableFloatArray(mountPosition == null ? [0.0, 0.0, 0.0] : mountPosition);
    this.mountRotation = new ImmutableFloatArray(mountRotation == null ? [0.0, 0.0, 0.0, 1.0] : mountRotation);
    this.image = image;
  }

  public function copy():SensorFrame return new SensorFrame(sensorId, kind, frameId, sequence,
    sourceTimestampNs, values.toArray(), receivedTimestampNs, linkId, mountPosition.toArray(),
    mountRotation.toArray(), sourceClockId, receivedClockId, image);
}
