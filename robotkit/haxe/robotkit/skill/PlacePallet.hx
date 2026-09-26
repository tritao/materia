package robotkit.skill;

import robotkit.material.Forks;
import robotkit.material.LoadState;
import robotkit.material.Payload;
import robotkit.mobile.Pose2;
import robotkit.navigation.NavigationGoal;
import robotkit.navigation.Navigator;
import robotkit.perception.PerceptionSnapshot;
import robotkit.world.RobotSnapshot;

private enum PlaceStage {
  Approach;
  WaitForReleaseConfirmation;
}

/** Plans to a placement pose, lowers the forks, and waits for empty-load confirmation. */
class PlacePallet implements Skill {
  public final navigator:Navigator;
  public final observePerception:RobotSnapshot -> PerceptionSnapshot;
  public final forks:Forks;
  public final payload:Payload;
  public final approachPose:Pose2;
  public final liftHeightMeters:Float;
  public final tiltRadians:Null<Float>;
  public final spreadMeters:Null<Float>;
  public final positionTolerance:Float;
  public final headingTolerance:Float;
  /** Forwarded to the approach GoTo; see GoTo.blockedTimeoutSeconds. */
  public final blockedTimeoutSeconds:Float;

  final lifecycle:SkillLifecycle = new SkillLifecycle();
  var stage:PlaceStage = Approach;
  var approach:Null<GoTo> = null;

  public function new(navigator:Navigator,
      observePerception:RobotSnapshot -> PerceptionSnapshot,
      forks:Forks, payload:Payload, approachPose:Pose2, liftHeightMeters:Float,
      ?tiltRadians:Float, ?spreadMeters:Float,
      ?positionTolerance:Float = 0.08, ?headingTolerance:Float = 0.08,
      ?blockedTimeoutSeconds:Float = 10.0) {
    if (navigator == null || observePerception == null || forks == null ||
        payload == null || approachPose == null || !Math.isFinite(liftHeightMeters) ||
        !Math.isFinite(positionTolerance) || positionTolerance <= 0.0 ||
        !Math.isFinite(headingTolerance) || headingTolerance <= 0.0)
      throw "PlacePallet requires navigation, perception, forks, payload, and valid approach settings";
    this.navigator = navigator;
    this.observePerception = observePerception;
    this.forks = forks;
    this.payload = payload;
    this.approachPose = new Pose2(approachPose.x, approachPose.y, approachPose.yaw);
    this.liftHeightMeters = liftHeightMeters;
    this.tiltRadians = tiltRadians;
    this.spreadMeters = spreadMeters;
    this.positionTolerance = positionTolerance;
    this.headingTolerance = headingTolerance;
    this.blockedTimeoutSeconds = blockedTimeoutSeconds;
  }

  public function start():Void {
    lifecycle.begin();
    stage = Approach;
    if (!forks.loadState.secured || forks.loadState.payload != payload)
      return lifecycle.fail("PlacePallet requires the expected payload to be secured");
    var estimate = navigator.navigation.localization.state();
    if (estimate == null) return lifecycle.fail("PlacePallet has no localization state");
    var violation = forks.config.loadLimits.violation(payload, liftHeightMeters);
    if (violation != null) return lifecycle.fail(violation);
    var value = new GoTo(navigator,
      new NavigationGoal(approachPose, estimate.referenceFrame,
        positionTolerance, headingTolerance), observePerception, blockedTimeoutSeconds);
    approach = value;
    value.start();
    syncApproach(value.status());
  }

  public function update(snapshot:RobotSnapshot, durationSeconds:Float):SkillStatus {
    if (!lifecycle.isRunning()) return lifecycle.status();
    switch stage {
      case Approach:
        var value:Null<GoTo> = approach;
        if (value == null) {
          lifecycle.fail("PlacePallet has no active navigation task");
          return lifecycle.status();
        }
        syncApproach(value.update(snapshot, durationSeconds));
      case WaitForReleaseConfirmation:
        if (forks.loadState.observed && forks.loadState.payload == null &&
            !forks.loadState.secured)
          lifecycle.succeed("pallet release confirmed");
    }
    return lifecycle.status();
  }

  public function cancel():Void {
    if (!lifecycle.isRunning()) return;
    var value:Null<GoTo> = approach;
    if (value != null) value.cancel();
    try forks.robot.stop(robotkit.world.StopMode.Normal) catch (_:Dynamic) {}
    lifecycle.cancel();
  }

  public function status():SkillStatus return lifecycle.status();
  public function result():Null<SkillResult> return lifecycle.result();

  function syncApproach(value:SkillStatus):Void switch value {
    case Idle: lifecycle.fail("PlacePallet navigation returned to idle");
    case Running:
    case Succeeded:
      try {
        forks.command(liftHeightMeters, tiltRadians, spreadMeters);
        stage = WaitForReleaseConfirmation;
      } catch (error:Dynamic) {
        lifecycle.fail(Std.string(error));
      }
    case Cancelled: lifecycle.cancel();
    case Failed(message): lifecycle.fail(message);
  }
}
