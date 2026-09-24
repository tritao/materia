package robotkit.skill;

import robotkit.navigation.Navigation;
import robotkit.navigation.NavigationGoal;
import robotkit.navigation.NavigationStatus;
import robotkit.navigation.Path;
import robotkit.world.RobotSnapshot;

/** Follows a supplied path and reports a terminal goal result. */
class GoTo implements Skill {
  public final navigation:Navigation;
  public final path:Path;
  public final goal:Null<NavigationGoal>;
  final lifecycle:SkillLifecycle = new SkillLifecycle();

  public function new(navigation:Navigation, path:Path, ?goal:NavigationGoal) {
    if (navigation == null || path == null)
      throw "GoTo requires navigation and a path";
    this.navigation = navigation;
    this.path = path;
    this.goal = goal;
  }

  public function start():Void {
    lifecycle.begin();
    try navigation.follow(path, goal) catch (error:Dynamic) lifecycle.fail(Std.string(error));
  }

  public function update(snapshot:RobotSnapshot, durationSeconds:Float):SkillStatus {
    if (!lifecycle.isRunning()) return lifecycle.status();
    try sync(navigation.updateObservation(snapshot, durationSeconds))
    catch (error:Dynamic) lifecycle.fail(Std.string(error));
    return lifecycle.status();
  }

  public function cancel():Void {
    if (!lifecycle.isRunning()) return;
    navigation.cancel();
    lifecycle.cancel();
  }

  public function status():SkillStatus return lifecycle.status();
  public function result():Null<SkillResult> return lifecycle.result();

  function sync(value:NavigationStatus):Void switch value {
    case Following:
    case Succeeded: lifecycle.succeed("goal reached");
    case Cancelled: lifecycle.cancel();
    case Failed(message): lifecycle.fail(message);
    case Idle: lifecycle.fail("navigation returned to idle");
  }
}
