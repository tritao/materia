package robotkit.skill;

import robotkit.navigation.NavigationGoal;
import robotkit.navigation.Navigator;
import robotkit.perception.DockingTarget;
import robotkit.perception.PerceptionSnapshot;
import robotkit.world.RobotSnapshot;

/** Plans and navigates to the approach pose of a detected docking target. */
class Dock implements Skill {
  public final navigator:Navigator;
  public final target:DockingTarget;
  public final observePerception:RobotSnapshot -> PerceptionSnapshot;
  public final minimumConfidence:Float;
  public final positionTolerance:Float;
  public final headingTolerance:Float;

  final lifecycle:SkillLifecycle = new SkillLifecycle();
  var approach:Null<GoTo> = null;

  public function new(navigator:Navigator, target:DockingTarget,
      observePerception:RobotSnapshot -> PerceptionSnapshot,
      ?minimumConfidence:Float = 0.5, ?positionTolerance:Float = 0.05,
      ?headingTolerance:Float = 0.05) {
    if (navigator == null || target == null || observePerception == null ||
        !Math.isFinite(minimumConfidence) || minimumConfidence < 0.0 ||
        minimumConfidence > 1.0 || !Math.isFinite(positionTolerance) ||
        positionTolerance <= 0.0 || !Math.isFinite(headingTolerance) ||
        headingTolerance <= 0.0)
      throw "Dock requires a Navigator, target, observation processor, confidence, and positive tolerances";
    this.navigator = navigator;
    this.target = target;
    this.observePerception = observePerception;
    this.minimumConfidence = minimumConfidence;
    this.positionTolerance = positionTolerance;
    this.headingTolerance = headingTolerance;
  }

  public function start():Void {
    lifecycle.begin();
    var estimate = navigator.navigation.localization.state();
    if (estimate == null) return lifecycle.fail("Dock has no localization state");
    if (target.detection.confidence < minimumConfidence)
      return lifecycle.fail("Docking target confidence is too low");
    if (estimate.referenceFrame != target.detection.frameId)
      return lifecycle.fail("Docking target and localization frames differ");
    var value = new GoTo(navigator,
      new NavigationGoal(target.approachPose, estimate.referenceFrame,
        positionTolerance, headingTolerance), observePerception);
    approach = value;
    value.start();
    sync(value.status());
  }

  public function update(snapshot:RobotSnapshot, durationSeconds:Float):SkillStatus {
    if (!lifecycle.isRunning()) return lifecycle.status();
    var value:Null<GoTo> = approach;
    if (value == null) {
      lifecycle.fail("Dock has no active navigation task");
      return lifecycle.status();
    }
    sync(value.update(snapshot, durationSeconds));
    return lifecycle.status();
  }

  public function cancel():Void {
    if (!lifecycle.isRunning()) return;
    var value:Null<GoTo> = approach;
    if (value != null) value.cancel();
    lifecycle.cancel();
  }

  public function status():SkillStatus return lifecycle.status();
  public function result():Null<SkillResult> return lifecycle.result();

  function sync(value:SkillStatus):Void switch value {
    case Idle: lifecycle.fail("Dock navigation returned to idle");
    case Running:
    case Succeeded: lifecycle.succeed("docking approach reached");
    case Cancelled: lifecycle.cancel();
    case Failed(message): lifecycle.fail(message);
  }
}
