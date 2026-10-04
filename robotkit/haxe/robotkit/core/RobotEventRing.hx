package robotkit.core;



import haxe.Int64;
import robotkit.perception.ImageDetectionObservation;

/** Owner-thread ring. A slow reader gets one gap marker before retained events. */
class RobotEventRing {
  final capacity:Int;
  final values:Array<RobotEvent> = [];
  var nextOrdinal:Int64 = Int64.ofInt(1);

  public function new(?capacity:Int = 256) {
    if (capacity < 1) throw "Robot event capacity must be positive";
    this.capacity = capacity;
  }

  public function publish(value:ImageDetectionObservation):RobotEvent {
    var event = RobotEvent.Observation(nextOrdinal, value);
    append(event);
    return event;
  }

  /** Reserve ordinals for observations lost before this boundary. */
  public function skip(count:Int):Void {
    if (count < 0) throw "Robot event gap cannot be negative";
    var next = robotkit.time.ClockMapping.checkedAdd(nextOrdinal, Int64.ofInt(count));
    if (next == null) throw "Robot event ordinal overflow";
    nextOrdinal = next;
  }


  /** Used by replay; stored ordinals must remain strictly increasing. */
  public function restore(event:RobotEvent):Void {
    var ordinal = ordinalOf(event);
    if (Int64.compare(ordinal, nextOrdinal) < 0)
      throw "Robot event ordinals must increase";
    append(event);
  }

  function append(event:RobotEvent):Void {
    values.push(event);
    if (values.length > capacity) values.shift();
    nextOrdinal = Int64.add(ordinalOf(event), Int64.ofInt(1));
  }

  public function events(afterOrdinal:Int64, max:Int):Array<RobotEvent> {
    if (max <= 0) return [];
    var result:Array<RobotEvent> = [];
    if (values.length == 0) return result;
    var first = ordinalOf(values[0]);
    var expected = Int64.add(afterOrdinal, Int64.ofInt(1));
    if (Int64.compare(expected, first) < 0) {
      var lost = Int64.sub(first, expected);
      result.push(RobotEvent.Overflow(Int64.sub(first, Int64.ofInt(1)), Int64.toInt(lost)));
    }
    var previous = Int64.sub(first, Int64.ofInt(1));
    for (event in values) {
      if (result.length >= max) break;
      var ordinal = ordinalOf(event);
      var missing = Int64.sub(ordinal, Int64.add(previous, Int64.ofInt(1)));
      if (Int64.compare(missing, Int64.ofInt(0)) > 0 &&
          switch event { case Overflow(_, _): false; case _: true; } &&
          Int64.compare(ordinal, afterOrdinal) > 0) {
        result.push(RobotEvent.Overflow(Int64.sub(ordinal, Int64.ofInt(1)), Int64.toInt(missing)));
      }
      if (result.length < max && Int64.compare(ordinal, afterOrdinal) > 0) result.push(event);
      previous = ordinal;
    }
    return result;
  }

  public static function ordinalOf(event:RobotEvent):Int64 return switch event {
    case Observation(ordinal, _): ordinal;
    case Overflow(ordinal, _): ordinal;
  };
}
