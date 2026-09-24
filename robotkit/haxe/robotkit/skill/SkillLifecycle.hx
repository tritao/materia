package robotkit.skill;

/** Small shared state holder; it owns no robot or controller lifecycle. */
class SkillLifecycle {
  var currentStatus:SkillStatus = Idle;
  var currentResult:Null<SkillResult> = null;

  public function new() {}

  public function begin():Void {
    currentStatus = Running;
    currentResult = null;
  }

  public function succeed(message:String):Void finish(Succeeded, message);
  public function cancel():Void finish(Cancelled, "cancelled");
  public function fail(message:String):Void finish(Failed(message), message);

  public function status():SkillStatus return currentStatus;
  public function result():Null<SkillResult> return currentResult;

  public function isRunning():Bool return switch currentStatus {
    case Running: true;
    case _: false;
  };

  function finish(value:SkillStatus, message:String):Void {
    currentStatus = value;
    currentResult = new SkillResult(value, message);
  }
}
