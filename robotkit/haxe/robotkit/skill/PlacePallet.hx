package robotkit.skill;

import robotkit.material.Forks;
import robotkit.material.LoadState;
import robotkit.material.Payload;
import robotkit.mobile.Pose2;
import robotkit.navigation.Navigation;
import robotkit.navigation.NavigationGoal;
import robotkit.navigation.NavigationStatus;
import robotkit.navigation.Path;
import robotkit.world.RobotSnapshot;

private enum PlaceStage {
  Approach;
  WaitForReleaseConfirmation;
}

/** Navigates to a placement pose, lowers the forks, and waits for empty-load confirmation. */
class PlacePallet implements Skill {
  public final navigation:Navigation;
  public final forks:Forks;
  public final payload:Payload;
  public final approachPose:Pose2;
  public final liftHeightMeters:Float;
  public final tiltRadians:Null<Float>;
  public final spreadMeters:Null<Float>;
  public final positionTolerance:Float;
  public final headingTolerance:Float;

  final lifecycle:SkillLifecycle = new SkillLifecycle();
  var stage:PlaceStage = Approach;

  public function new(navigation:Navigation, forks:Forks, payload:Payload,
      approachPose:Pose2, liftHeightMeters:Float,
      ?tiltRadians:Float, ?spreadMeters:Float,
      ?positionTolerance:Float = 0.08, ?headingTolerance:Float = 0.08) {
    if (navigation == null || forks == null || payload == null || approachPose == null ||
        !Math.isFinite(liftHeightMeters) || !Math.isFinite(positionTolerance) ||
        positionTolerance <= 0.0 || !Math.isFinite(headingTolerance) || headingTolerance <= 0.0)
      throw "PlacePallet requires navigation, forks, payload, and valid approach settings";
    this.navigation = navigation;
    this.forks = forks;
    this.payload = payload;
    this.approachPose = new Pose2(approachPose.x, approachPose.y, approachPose.yaw);
    this.liftHeightMeters = liftHeightMeters;
    this.tiltRadians = tiltRadians;
    this.spreadMeters = spreadMeters;
    this.positionTolerance = positionTolerance;
    this.headingTolerance = headingTolerance;
  }

  public function start():Void {
    lifecycle.begin();
    stage = Approach;
    if (!forks.loadState.secured || forks.loadState.payload != payload)
      return lifecycle.fail("PlacePallet requires the expected payload to be secured");
    var estimate = navigation.localization.state();
    if (estimate == null) return lifecycle.fail("PlacePallet has no localization state");
    var violation = forks.config.loadLimits.violation(payload, liftHeightMeters);
    if (violation != null) return lifecycle.fail(violation);
    try {
      var path = new Path([estimate.pose, approachPose], estimate.referenceFrame);
      navigation.follow(path, new NavigationGoal(approachPose, estimate.referenceFrame,
        positionTolerance, headingTolerance));
    } catch (error:Dynamic) {
      lifecycle.fail(Std.string(error));
    }
  }

  public function update(snapshot:RobotSnapshot, durationSeconds:Float):SkillStatus {
    if (!lifecycle.isRunning()) return lifecycle.status();
    switch stage {
      case Approach:
        try {
          switch navigation.updateObservation(snapshot, durationSeconds) {
            case Following:
            case Succeeded:
              forks.command(liftHeightMeters, tiltRadians, spreadMeters);
              stage = WaitForReleaseConfirmation;
            case Cancelled: lifecycle.cancel();
            case Failed(message): lifecycle.fail(message);
            case Idle: lifecycle.fail("navigation returned to idle");
          }
        } catch (error:Dynamic) {
          lifecycle.fail(Std.string(error));
        }
      case WaitForReleaseConfirmation:
        if (forks.loadState.observed && forks.loadState.payload == null &&
            !forks.loadState.secured)
          lifecycle.succeed("pallet release confirmed");
    }
    return lifecycle.status();
  }

  public function cancel():Void {
    if (!lifecycle.isRunning()) return;
    navigation.cancel();
    try forks.robot.stop(robotkit.world.StopMode.Normal) catch (_:Dynamic) {}
    lifecycle.cancel();
  }

  public function status():SkillStatus return lifecycle.status();
  public function result():Null<SkillResult> return lifecycle.result();
}
