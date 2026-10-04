package robotkit.execution;



import haxe.Int64;

/** Output change scheduled in plan-relative trajectory time. */
class ProcessTimedEvent {
  public final timeNs:Int64;
  public final channel:String;
  public final value:ProcessEventValue;
  public final holdPolicy:ProcessHoldPolicy;

  public function new(timeNs:Int64, channel:String, value:ProcessEventValue,
      ?holdPolicy:ProcessHoldPolicy) {
    if (timeNs == null || Int64.compare(timeNs, Int64.ofInt(0)) < 0)
      throw "Process event needs a non-negative time";
    ProcessEventCodec.validateId(channel);
    ProcessEventCodec.validateValue(value);
    this.timeNs = timeNs;
    this.channel = channel;
    this.value = value;
    this.holdPolicy = holdPolicy == null ? ProcessHoldPolicy.Keep : holdPolicy;
  }
}
