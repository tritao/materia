package robotkit.skill;

import robotkit.material.Forks;
import robotkit.material.LoadState;
import robotkit.material.Payload;
import robotkit.mobile.Pose2;
import robotkit.navigation.NavigationGoal;
import robotkit.navigation.Navigator;
import robotkit.perception.Pallet;
import robotkit.perception.PerceptionSnapshot;
import robotkit.world.RobotSnapshot;

private enum PickStage {
  Approach;
  WaitForLoadConfirmation;
}

/** Plans to a pallet approach, positions forks, and waits for secured-load confirmation. */
class PickPallet implements Skill {
  public final navigator:Navigator;
  public final observePerception:RobotSnapshot -> PerceptionSnapshot;
  public final forks:Forks;
  public final pallet:Pallet;
  public final payload:Payload;
  public final approachPose:Pose2;
  public final liftHeightMeters:Float;
  public final tiltRadians:Null<Float>;
  public final spreadMeters:Null<Float>;
  public final positionTolerance:Float;
  public final headingTolerance:Float;

  final lifecycle:SkillLifecycle = new SkillLifecycle();
  var stage:PickStage = Approach;
  var approach:Null<GoTo> = null;

  public function new(navigator:Navigator,
      observePerception:RobotSnapshot -> PerceptionSnapshot,
      forks:Forks, pallet:Pallet, payload:Payload, approachPose:Pose2,
      liftHeightMeters:Float, ?tiltRadians:Float, ?spreadMeters:Float,
      ?positionTolerance:Float = 0.08, ?headingTolerance:Float = 0.08) {
    if (navigator == null || observePerception == null || forks == null ||
        pallet == null || payload == null || approachPose == null ||
        !Math.isFinite(liftHeightMeters) || !Math.isFinite(positionTolerance) ||
        positionTolerance <= 0.0 || !Math.isFinite(headingTolerance) ||
        headingTolerance <= 0.0)
      throw "PickPallet requires navigation, perception, forks, pallet, payload, and valid approach settings";
    this.navigator = navigator;
    this.observePerception = observePerception;
    this.forks = forks;
    this.pallet = pallet;
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
    var estimate = navigator.navigation.localization.state();
    if (estimate == null) return lifecycle.fail("PickPallet has no localization state");
    if (pallet.detection.confidence < 0.5)
      return lifecycle.fail("Pallet detection confidence is too low");
    if (estimate.referenceFrame != pallet.detection.frameId)
      return lifecycle.fail("Pallet and localization frames differ");
    var violation = forks.config.loadLimits.violation(payload, liftHeightMeters);
    if (violation != null) return lifecycle.fail(violation);
    var value = new GoTo(navigator,
      new NavigationGoal(approachPose, estimate.referenceFrame,
        positionTolerance, headingTolerance), observePerception);
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
          lifecycle.fail("PickPallet has no active navigation task");
          return lifecycle.status();
        }
        syncApproach(value.update(snapshot, durationSeconds));
      case WaitForLoadConfirmation:
        if (forks.loadState.secured && forks.loadState.payload == payload)
          lifecycle.succeed("pallet load secured");
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
    case Idle: lifecycle.fail("PickPallet navigation returned to idle");
    case Running:
    case Succeeded:
      try {
        forks.command(liftHeightMeters, tiltRadians, spreadMeters);
        forks.setLoadState(LoadState.detected(payload));
        stage = WaitForLoadConfirmation;
      } catch (error:Dynamic) {
        lifecycle.fail(Std.string(error));
      }
    case Cancelled: lifecycle.cancel();
    case Failed(message): lifecycle.fail(message);
  }
}
