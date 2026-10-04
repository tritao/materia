package robotkit.core;



import haxe.Int64;
import robotkit.perception.ImageDetectionObservation;

/** Bounded, per-robot event stream. Ordinals are local to one robot. */
enum RobotEvent {
  Observation(ordinal:Int64, value:ImageDetectionObservation);
  Overflow(ordinal:Int64, count:Int);
}
