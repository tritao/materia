package machinekit.operator;

/** Parameters for processkit's EnclosureGuard. Not a safety function; see CONTROL_CABINET_PLAN.md. */
class OperatorGuard {
  public final setupSpeedLimit:Float;
  public final timeoutSeconds:Float;
  public function new(setupSpeedLimit:Float, timeoutSeconds:Float) {
    if (!Math.isFinite(setupSpeedLimit) || setupSpeedLimit <= 0 || !Math.isFinite(timeoutSeconds) || timeoutSeconds <= 0)
      throw "Guard needs a positive setup speed limit and feedback timeout";
    this.setupSpeedLimit = setupSpeedLimit;
    this.timeoutSeconds = timeoutSeconds;
  }
}
