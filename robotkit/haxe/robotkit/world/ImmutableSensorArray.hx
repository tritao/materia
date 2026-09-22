package robotkit.world;

/** Owning read-only collection of sensor frames. */
class ImmutableSensorArray {
  final values:Array<SensorFrame>;
  public var length(get, never):Int;

  public function new(source:Null<Array<SensorFrame>>) {
    values = [];
    if (source != null) {
      for (frame in source)
        values.push(new SensorFrame(frame.sensorId, frame.kind, frame.frameId,
          frame.sequence, frame.sourceTimestampNs, frame.values.toArray(),
          frame.receivedTimestampNs));
    }
  }

  public function get(index:Int):SensorFrame return values[index];
  public function toArray():Array<SensorFrame> {
    var result:Array<SensorFrame> = [];
    for (frame in values)
      result.push(new SensorFrame(frame.sensorId, frame.kind, frame.frameId,
        frame.sequence, frame.sourceTimestampNs, frame.values.toArray(),
        frame.receivedTimestampNs));
    return result;
  }

  inline function get_length():Int return values.length;
}
