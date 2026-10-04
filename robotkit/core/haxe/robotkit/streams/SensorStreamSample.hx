package robotkit.streams;

import haxe.Int64;
import robotkit.core.SensorFrame;

/** One immutable timestamped sample, independent of control snapshot cadence. */
class SensorStreamSample {
  public final streamId:String;
  public final kind:String;
  public final sequence:Int64;
  public final sourceTimestampNs:Int64;
  public final receivedTimestampNs:Int64;
  public final sourceClockId:String;
  public final receivedClockId:String;
  public final frame:Null<SensorFrame>;
  public final detection:Null<ImageDetectionObservation>;

  function new(id:String, kind:String, sequence:Int64, source:Int64, received:Int64,
      sourceClock:String, receivedClock:String, frame:Null<SensorFrame>, detection:Null<ImageDetectionObservation>) {
    this.streamId = id; this.kind = kind; this.sequence = sequence;
    sourceTimestampNs = source; receivedTimestampNs = received;
    sourceClockId = sourceClock; receivedClockId = receivedClock;
    this.frame = frame; this.detection = detection;
  }
  public static function sensor(value:SensorFrame):SensorStreamSample
    return new SensorStreamSample(value.sensorId, value.kind, value.sequence,
      value.sourceTimestampNs, value.receivedTimestampNs, value.sourceClockId,
      value.receivedClockId, value.copy(), null);
  public static function inference(value:ImageDetectionObservation):SensorStreamSample
    return new SensorStreamSample('inference/${value.pipelineId}', "inference", value.sequence,
      value.sourceTimestampNs, value.receivedTimestampNs, value.sourceClockId,
      value.receivedClockId, null, value);
  public static function smallControlFrame(value:SensorFrame):Bool
    return value.image == null && value.values.length <= 32 &&
      value.kind != "camera" && value.kind != "lidar" && value.kind != "depth" && value.kind != "point_cloud";
}
