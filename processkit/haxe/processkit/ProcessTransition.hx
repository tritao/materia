package processkit;

/** Logged process state and path distance at the transition. */
class ProcessTransition {
  public final state:ProcessRunState;
  public final distance:Float;
  public final reason:String;

  public function new(state:ProcessRunState, distance:Float, reason:String) {
    this.state = state;
    this.distance = distance;
    this.reason = reason;
  }
}
