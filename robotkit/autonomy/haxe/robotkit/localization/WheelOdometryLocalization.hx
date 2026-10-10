package robotkit.localization;

import robotkit.mobile.DifferentialOdometry;
import robotkit.mobile.MobileBase;
import robotkit.mobile.Pose2;
import robotkit.core.RobotSnapshot;

/** Differential wheel odometry exposed as an `odom` to `base` localization service. */
class WheelOdometryLocalization implements Localization {
  public final base:MobileBase;
  public final referenceFrame:String;
  public final bodyFrame:String;
  public final variancePerMeter:Float;
  public final yawVariancePerRadian:Float;

  final odometry:DifferentialOdometry;
  var varianceX:Float = 0.0;
  var covarianceXY:Float = 0.0;
  var covarianceXYaw:Float = 0.0;
  var varianceY:Float = 0.0;
  var covarianceYYaw:Float = 0.0;
  var varianceYaw:Float = 0.0;
  var currentState:Null<LocalizationState> = null;

  public function new(base:MobileBase, ?referenceFrame:String = "odom",
      ?bodyFrame:String = "base", ?variancePerMeter:Float = 0.01,
      ?yawVariancePerRadian:Float = 0.02) {
    if (base == null) throw "Wheel odometry requires a MobileBase";
    var configuredOdometry = base.driveModel.createOdometry();
    if (configuredOdometry == null)
      throw "Wheel odometry localization requires a drive model with wheel odometry";
    if (!Math.isFinite(variancePerMeter) || variancePerMeter < 0.0 ||
        !Math.isFinite(yawVariancePerRadian) || yawVariancePerRadian < 0.0)
      throw "Wheel odometry noise rates must be finite and non-negative";
    this.base = base;
    this.referenceFrame = referenceFrame;
    this.bodyFrame = bodyFrame;
    this.variancePerMeter = variancePerMeter;
    this.yawVariancePerRadian = yawVariancePerRadian;
    odometry = configuredOdometry;
  }

  public function update(snapshot:RobotSnapshot):LocalizationState {
    var pose = odometry.update(snapshot);
    varianceX += variancePerMeter * Math.abs(odometry.lastDistance);
    varianceY += variancePerMeter * Math.abs(odometry.lastDistance);
    varianceYaw += yawVariancePerRadian * Math.abs(odometry.lastHeadingChange);
    currentState = new LocalizationState(snapshot.sourceSequence, pose,
      referenceFrame, bodyFrame,
      new PoseCovariance2(varianceX, covarianceXY, covarianceXYaw,
        varianceY, covarianceYYaw, varianceYaw),
      LocalizationQuality.Degraded, snapshot.sourceTimestampNs,
      snapshot.receivedTimestampNs, snapshot.sourceClockId, snapshot.receivedClockId);
    return currentState;
  }

  public function state():Null<LocalizationState> return currentState;

  /** Re-anchors wheel integration from a same-tick localization observation, retaining its uncertainty. */
  public function rebase(observation:LocalizationState, snapshot:RobotSnapshot):LocalizationState {
    if (observation == null || snapshot == null)
      throw "Wheel odometry rebase requires a localization observation and robot snapshot";
    if (observation.referenceFrame != referenceFrame || observation.bodyFrame != bodyFrame)
      throw "Wheel odometry rebase localization frames do not match its configured frames";
    if (observation.sequence != snapshot.sourceSequence)
      throw "Wheel odometry rebase requires a localization observation from the same snapshot";
    if (switch observation.quality { case Invalid: true; case _: false; })
      throw "Wheel odometry cannot rebase from an invalid localization observation";

    odometry.reset(observation.pose);
    varianceX = observation.covariance.xx;
    covarianceXY = observation.covariance.xy;
    covarianceXYaw = observation.covariance.xYaw;
    varianceY = observation.covariance.yy;
    covarianceYYaw = observation.covariance.yYaw;
    varianceYaw = observation.covariance.yawYaw;
    return update(snapshot);
  }

  public function reset(?pose:Pose2):Void {
    odometry.reset(pose);
    varianceX = 0.0;
    covarianceXY = 0.0;
    covarianceXYaw = 0.0;
    varianceY = 0.0;
    covarianceYYaw = 0.0;
    varianceYaw = 0.0;
    currentState = null;
  }
}
