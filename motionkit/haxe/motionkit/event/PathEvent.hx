package motionkit.event;

/** Immutable process event attached to authored path progress. */
class PathEvent {
  public final distance:Float;
  public final channel:String;
  public final value:EventValue;
  public final leadSeconds:Float;
  public final holdPolicy:HoldPolicy;

  public function new(distance:Float, channel:String, value:EventValue,
      ?leadSeconds:Float = 0.0, ?holdPolicy:HoldPolicy) {
    if (!Math.isFinite(distance) || distance < 0.0)
      throw "Path-event distance must be finite and non-negative";
    if (channel == null || StringTools.trim(channel).length == 0)
      throw "Path event needs a non-empty channel ID";
    if (!Math.isFinite(leadSeconds) || leadSeconds < 0.0)
      throw "Path-event lead time must be finite and non-negative";
    EventValueTools.validate(value, "Path-event value");
    this.distance = distance;
    this.channel = channel;
    this.value = value;
    this.leadSeconds = leadSeconds;
    this.holdPolicy = holdPolicy == null ? HoldPolicy.Keep : holdPolicy;
  }
}
