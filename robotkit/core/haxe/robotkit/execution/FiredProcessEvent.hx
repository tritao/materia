package robotkit.execution;



import haxe.Int64;

/** One runtime output change, including synthetic safety transitions. */
class FiredProcessEvent {
  public final planId:Int64;
  public final channel:String;
  public final value:ProcessEventValue;
  public final scheduledTimeNs:Int64;
  public final appliedOwnerTimeNs:Int64;
  public final cause:Int;

  public function new(planId:Int64, channel:String, value:ProcessEventValue,
      scheduledTimeNs:Int64, appliedOwnerTimeNs:Int64, cause:Int) {
    this.planId = planId;
    this.channel = channel;
    this.value = value;
    this.scheduledTimeNs = scheduledTimeNs;
    this.appliedOwnerTimeNs = appliedOwnerTimeNs;
    this.cause = cause;
  }
}
