package robotkit.world;

/** Owning read-only collection of sensor frames. */
class ImmutableSensorArray {
  final values:Array<SensorFrame>;
  public var length(get, never):Int;

  public function new(source:Null<Array<SensorFrame>>) {
    values = [];
    if (source != null) {
      for (frame in source)
        values.push(frame.copy());
    }
  }

  public function get(index:Int):SensorFrame return values[index];
  public function toArray():Array<SensorFrame> {
    var result:Array<SensorFrame> = [];
    for (frame in values)
      result.push(frame.copy());
    return result;
  }

  inline function get_length():Int return values.length;
}
