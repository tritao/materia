package robotkit.skill;

import robotkit.world.RobotSnapshot;

/** Advances at most one robot-local skill from the application's control loop. */
class SkillRunner {
  var currentSkill:Null<Skill> = null;
  var currentStatus:SkillStatus = Idle;
  var currentResult:Null<SkillResult> = null;

  public function new() {}

  /** Starts a skill if this runner has no skill in progress. */
  public function start(skill:Skill):SkillStatus {
    if (skill == null) throw "SkillRunner.start requires a skill";
    if (isRunning()) throw "SkillRunner already has an active skill";
    if (switch skill.status() { case Running: true; case _: false; })
      throw "SkillRunner cannot take ownership of an already running skill";

    currentSkill = skill;
    currentResult = null;
    currentStatus = Running;
    try {
      skill.start();
      acceptStatus(skill, skill.status());
    } catch (error:Dynamic) {
      failAndStop(skill, Std.string(error));
    }
    return currentStatus;
  }

  /** Updates the active skill once using one robot observation and elapsed time. */
  public function update(snapshot:RobotSnapshot,
      durationSeconds:Float):SkillStatus {
    var skill:Null<Skill> = currentSkill;
    if (skill == null || !isRunning()) return currentStatus;
    if (snapshot == null || !Math.isFinite(durationSeconds) || durationSeconds <= 0.0) {
      failAndStop(skill, "SkillRunner update requires a snapshot and positive finite duration");
      return currentStatus;
    }
    try {
      acceptStatus(skill, skill.update(snapshot, durationSeconds));
    } catch (error:Dynamic) {
      failAndStop(skill, Std.string(error));
    }
    return currentStatus;
  }

  /** Cancels the active skill; terminal and idle runners are unchanged. */
  public function cancel():Void {
    var skill:Null<Skill> = currentSkill;
    if (skill == null || !isRunning()) return;
    try {
      skill.cancel();
      var status = skill.status();
      switch status {
        case Running, Idle: finish(skill, Cancelled);
        case _: finish(skill, status);
      }
    } catch (error:Dynamic) {
      failAndStop(skill, Std.string(error));
    }
  }

  /** The skill currently running, or null when the runner is idle or terminal. */
  public function activeSkill():Null<Skill>
    return isRunning() ? currentSkill : null;

  public function status():SkillStatus return currentStatus;

  /** Terminal outcome of the most recently completed skill, if available. */
  public function result():Null<SkillResult> return currentResult;

  function acceptStatus(skill:Skill, status:SkillStatus):Void {
    switch status {
      case Running:
        currentStatus = Running;
      case Idle:
        failAndStop(skill, "Skill returned to idle while owned by SkillRunner");
      case _: finish(skill, status);
    }
  }

  function finish(skill:Skill, status:SkillStatus):Void {
    currentStatus = status;
    currentResult = skill.result();
    if (currentResult == null)
      currentResult = new SkillResult(status, messageFor(status));
    currentSkill = null;
  }

  function failAndStop(skill:Skill, message:String):Void {
    try {
      if (switch skill.status() { case Running: true; case _: false; })
        skill.cancel();
    } catch (_:Dynamic) {
      // Preserve the original failure as the runner's result.
    }
    currentStatus = Failed(message == null || message.length == 0
      ? "Skill failed"
      : message);
    currentResult = new SkillResult(currentStatus, messageFor(currentStatus));
    currentSkill = null;
  }

  function isRunning():Bool return switch currentStatus {
    case Running: true;
    case _: false;
  };

  static function messageFor(status:SkillStatus):String return switch status {
    case Succeeded: "completed";
    case Cancelled: "cancelled";
    case Failed(message): message;
    case Idle: "idle";
    case Running: "running";
  };
}
