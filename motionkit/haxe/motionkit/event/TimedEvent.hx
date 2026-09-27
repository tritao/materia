package motionkit.event;

import haxe.Int64;

/** Immutable process event lowered onto the trajectory clock. */
class TimedEvent {
  public final timeNs:Int64;
  public final channel:String;
  public final value:EventValue;
  public final holdPolicy:HoldPolicy;

  public function new(timeNs:Int64, channel:String, value:EventValue,
      ?holdPolicy:HoldPolicy) {
    if (Int64.compare(timeNs, Int64.ofInt(0)) < 0)
      throw "Timed-event path time must be non-negative";
    if (channel == null || StringTools.trim(channel).length == 0)
      throw "Timed event needs a non-empty channel ID";
    EventValueTools.validate(value, "Timed-event value");
    this.timeNs = timeNs;
    this.channel = channel;
    this.value = value;
    this.holdPolicy = holdPolicy == null ? HoldPolicy.Keep : holdPolicy;
  }
}
