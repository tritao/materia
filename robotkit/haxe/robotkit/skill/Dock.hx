package robotkit.skill;

import robotkit.navigation.Navigation;
import robotkit.navigation.NavigationGoal;
import robotkit.navigation.NavigationStatus;
import robotkit.navigation.Path;
import robotkit.perception.DockingTarget;
import robotkit.world.RobotSnapshot;

/** Navigates to the approach pose of a detected docking target. */
class Dock implements Skill {
  public final navigation:Navigation;
  public final target:DockingTarget;
  public final minimumConfidence:Float;
  public final positionTolerance:Float;
  public final headingTolerance:Float;
  final lifecycle:SkillLifecycle = new SkillLifecycle();

  public function new(navigation:Navigation, target:DockingTarget,
      ?minimumConfidence:Float = 0.5, ?positionTolerance:Float = 0.05,
      ?headingTolerance:Float = 0.05) {
    if (navigation == null || target == null || !Math.isFinite(minimumConfidence) ||
        minimumConfidence < 0.0 || minimumConfidence > 1.0 ||
        !Math.isFinite(positionTolerance) || positionTolerance <= 0.0 ||
        !Math.isFinite(headingTolerance) || headingTolerance <= 0.0)
      throw "Dock requires navigation, a target, confidence, and positive tolerances";
    this.navigation = navigation;
    this.target = target;
    this.minimumConfidence = minimumConfidence;
    this.positionTolerance = positionTolerance;
    this.headingTolerance = headingTolerance;
  }

  public function start():Void {
    lifecycle.begin();
    var estimate = navigation.localization.state();
    if (estimate == null) return lifecycle.fail("Dock has no localization state");
    if (target.detection.confidence < minimumConfidence)
      return lifecycle.fail("Docking target confidence is too low");
    if (estimate.referenceFrame != target.detection.frameId)
      return lifecycle.fail("Docking target and localization frames differ");
    try {
      var path = new Path([estimate.pose, target.approachPose], estimate.referenceFrame);
      navigation.follow(path, new NavigationGoal(target.approachPose, estimate.referenceFrame,
        positionTolerance, headingTolerance));
    } catch (error:Dynamic) {
      lifecycle.fail(Std.string(error));
    }
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
    case Succeeded: lifecycle.succeed("docking approach reached");
    case Cancelled: lifecycle.cancel();
    case Failed(message): lifecycle.fail(message);
    case Idle: lifecycle.fail("navigation returned to idle");
  }
}
