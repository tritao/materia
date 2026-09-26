package robotkit.skill;

import robotkit.navigation.NavigationGoal;
import robotkit.navigation.Navigator;
import robotkit.navigation.NavigatorStatus;
import robotkit.perception.PerceptionSnapshot;
import robotkit.world.RobotSnapshot;

/** Plans, follows, and replans a route to a framed goal. */
class GoTo implements Skill {
  public final navigator:Navigator;
  public final goal:NavigationGoal;
  /**
   * Converts one robot observation into obstacles in the Navigator costmap
   * frame. The provider should update the Navigator's localization from the
   * same observation before returning.
   */
  public final observePerception:RobotSnapshot -> PerceptionSnapshot;
  /**
   * Longest continuous time, in update durations, that the Navigator may stay
   * Blocked before the skill fails with the blocked reason. The Navigator keeps
   * retrying its route plan within this window.
   */
  public final blockedTimeoutSeconds:Float;

  final lifecycle:SkillLifecycle = new SkillLifecycle();
  var blockedSeconds:Float = 0.0;

  public function new(navigator:Navigator, goal:NavigationGoal,
      observePerception:RobotSnapshot -> PerceptionSnapshot,
      ?blockedTimeoutSeconds:Float = 10.0) {
    if (navigator == null || goal == null || observePerception == null)
      throw "GoTo requires a Navigator, goal, and robot observation processor";
    if (!Math.isFinite(blockedTimeoutSeconds) || blockedTimeoutSeconds < 0.0)
      throw "GoTo blocked timeout must be finite and non-negative";
    this.navigator = navigator;
    this.goal = goal;
    this.observePerception = observePerception;
    this.blockedTimeoutSeconds = blockedTimeoutSeconds;
  }

  public function start():Void {
    lifecycle.begin();
    blockedSeconds = 0.0;
    try sync(navigator.navigateTo(goal), 0.0)
    catch (error:Dynamic) failAndStop(Std.string(error));
  }

  public function update(snapshot:RobotSnapshot,
      durationSeconds:Float):SkillStatus {
    if (!lifecycle.isRunning()) return lifecycle.status();
    if (snapshot == null || !Math.isFinite(durationSeconds) || durationSeconds <= 0.0) {
      failAndStop("GoTo update requires a robot snapshot and positive finite duration");
      return lifecycle.status();
    }
    try {
      var perception = observePerception(snapshot);
      if (perception == null) throw "GoTo observation processor returned no perception";
      sync(navigator.update(perception, durationSeconds), durationSeconds);
    } catch (error:Dynamic) {
      failAndStop(Std.string(error));
    }
    return lifecycle.status();
  }

  public function cancel():Void {
    if (!lifecycle.isRunning()) return;
    navigator.cancel();
    lifecycle.cancel();
  }

  public function status():SkillStatus return lifecycle.status();
  public function result():Null<SkillResult> return lifecycle.result();

  function sync(value:NavigatorStatus, elapsedSeconds:Float):Void switch value {
    case Idle: failAndStop("Navigator returned to idle");
    case Navigating: blockedSeconds = 0.0;
    case Blocked(reason):
      blockedSeconds += elapsedSeconds;
      if (blockedSeconds > blockedTimeoutSeconds)
        failAndStop('GoTo remained blocked beyond its timeout: $reason');
    case Succeeded: lifecycle.succeed("goal reached");
    case Cancelled: lifecycle.cancel();
    case Failed(message): failAndStop(message);
  }

  function failAndStop(message:String):Void {
    try navigator.cancel() catch (_:Dynamic) {}
    lifecycle.fail(message == null || message.length == 0 ? "GoTo failed" : message);
  }
}
