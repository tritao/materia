package robotkit.localization;

import robotkit.mobile.HolonomicOdometry;
import robotkit.mobile.Pose2;
import robotkit.world.RobotSnapshot;

/**
 * Wheel odometry for a three-wheel omni/kiwi base exposed as an `odom` to
 * `base` localization service, mirroring `WheelOdometryLocalization`.
 * `HolonomicDrive.createOdometry()` returns `null` (a three-wheel omni base
 * has no two-wheel differential relationship to decode, and `DriveModel`'s
 * `createOdometry()` is typed to `DifferentialOdometry` specifically), so
 * this takes the wheel joints/geometry directly instead of going through
 * `MobileBase.driveModel` — the same information a caller already has from
 * authoring the `RobotDriveConfiguration.Holonomic` role.
 */
class HolonomicOdometryLocalization implements Localization {
  public final referenceFrame:String;
  public final bodyFrame:String;
  public final variancePerMeter:Float;
  public final yawVariancePerRadian:Float;

  final odometry:HolonomicOdometry;
  var varianceX:Float = 0.0;
  var varianceY:Float = 0.0;
  var varianceYaw:Float = 0.0;
  var currentState:Null<LocalizationState> = null;

  public function new(wheelJoints:Array<Int>, wheelRadius:Float, baseRadius:Float,
      ?referenceFrame:String = "odom", ?bodyFrame:String = "base",
      ?variancePerMeter:Float = 0.01, ?yawVariancePerRadian:Float = 0.02, ?initialPose:Pose2) {
    if (!Math.isFinite(variancePerMeter) || variancePerMeter < 0.0 ||
        !Math.isFinite(yawVariancePerRadian) || yawVariancePerRadian < 0.0)
      throw "Holonomic odometry noise rates must be finite and non-negative";
    this.referenceFrame = referenceFrame;
    this.bodyFrame = bodyFrame;
    this.variancePerMeter = variancePerMeter;
    this.yawVariancePerRadian = yawVariancePerRadian;
    odometry = new HolonomicOdometry(wheelJoints, wheelRadius, baseRadius, initialPose);
  }

  public function update(snapshot:RobotSnapshot):LocalizationState {
    var pose = odometry.update(snapshot);
    varianceX += variancePerMeter * Math.abs(odometry.lastDistance);
    varianceY += variancePerMeter * Math.abs(odometry.lastDistance);
    varianceYaw += yawVariancePerRadian * Math.abs(odometry.lastHeadingChange);
    currentState = new LocalizationState(snapshot.sourceSequence, pose,
      referenceFrame, bodyFrame,
      new PoseCovariance2(varianceX, 0.0, 0.0, varianceY, 0.0, varianceYaw),
      LocalizationQuality.Degraded, snapshot.sourceTimestampNs,
      snapshot.receivedTimestampNs, snapshot.sourceClockId, snapshot.receivedClockId);
    return currentState;
  }

  public function state():Null<LocalizationState> return currentState;

  public function reset(?pose:Pose2):Void {
    odometry.reset(pose);
    varianceX = 0.0;
    varianceY = 0.0;
    varianceYaw = 0.0;
    currentState = null;
  }
}
