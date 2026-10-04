package robotkit.core;



/** Read-only collection of sensor frames, which are immutable and so shared, not copied. */
class ImmutableSensorArray {
  final values:Array<SensorFrame>;
  public var length(get, never):Int;

  public function new(source:Null<Array<SensorFrame>>) {
    values = source == null ? [] : source.copy();
  }

  public function get(index:Int):SensorFrame return values[index];
  public function toArray():Array<SensorFrame> return values.copy();

  inline function get_length():Int return values.length;
}
