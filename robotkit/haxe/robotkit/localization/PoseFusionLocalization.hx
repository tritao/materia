package robotkit.localization;

import haxe.Int64;
import robotkit.mobile.Pose2;
import robotkit.world.RobotSnapshot;

/** Fuses wheel odometry with covariance-weighted external planar pose observations. */
class PoseFusionLocalization implements Localization {
  public final odometry:Localization;
  public final frames:FrameTree2;
  public final referenceFrame:String;
  public final bodyFrame:String;

  var latestOdometry:Null<LocalizationState> = null;
  var currentState:Null<LocalizationState> = null;
  var referenceFromOdometry:Null<Pose2> = null;
  var correctionCovariance:PoseCovariance2 = new PoseCovariance2(1.0e6, 0.0, 0.0,
    1.0e6, 0.0, 1.0e6);
  var hasAbsoluteObservation = false;
  var absoluteQuality:LocalizationQuality = Degraded;

  public function new(odometry:Localization, frames:FrameTree2,
      ?referenceFrame:String = "map", ?bodyFrame:String = "base") {
    if (odometry == null || frames == null || referenceFrame == null ||
        referenceFrame.length == 0 || bodyFrame == null || bodyFrame.length == 0 ||
        referenceFrame == bodyFrame)
      throw "Pose fusion requires odometry, a frame tree, and distinct frame IDs";
    this.odometry = odometry;
    this.frames = frames;
    this.referenceFrame = referenceFrame;
    this.bodyFrame = bodyFrame;
  }

  public function update(snapshot:RobotSnapshot):LocalizationState {
    latestOdometry = odometry.update(snapshot);
    var odom:LocalizationState = cast latestOdometry;
    ensureReferenceFromOdometry(odom.referenceFrame);
    return publish(odom);
  }

  /** Fuses an external pose observation after transforming both frames explicitly. */
  public function fuse(observation:LocalizationState):LocalizationState {
    if (observation == null || observation.quality == LocalizationQuality.Invalid)
      throw "Pose fusion requires a valid external localization observation";
    if (latestOdometry == null)
      throw "Pose fusion requires an odometry update before an external observation";
    var odom:LocalizationState = cast latestOdometry;
    if (odom.bodyFrame != bodyFrame)
      throw 'Odometry body frame "${odom.bodyFrame}" does not match fusion body frame "$bodyFrame"';

    var referenceFromObservation = observation.referenceFrame == referenceFrame
      ? new Pose2()
      : frames.lookup(referenceFrame, observation.referenceFrame);
    var observationBodyInTarget = observation.bodyFrame == bodyFrame
      ? new Pose2()
      : frames.lookup(observation.bodyFrame, bodyFrame);
    var absoluteBodyPose = referenceFromObservation.compose(observation.pose)
      .compose(observationBodyInTarget);
    var odometryInverse = new Pose2().relativeTo(odom.pose);
    var measuredCorrection = absoluteBodyPose.compose(odometryInverse);
    var summedCovariance = addCovariance(observation.covariance, odom.covariance);
    // This first planar fusion pass weights x, y, and yaw independently.
    var measurementCovariance = new PoseCovariance2(summedCovariance.xx, 0.0, 0.0,
      summedCovariance.yy, 0.0, summedCovariance.yawYaw);

    if (referenceFromOdometry == null) {
      referenceFromOdometry = measuredCorrection;
      correctionCovariance = measurementCovariance;
    } else {
      var prior:Pose2 = cast referenceFromOdometry;
      var fusedX = fuseAxis(prior.x, correctionCovariance.xx,
        measuredCorrection.x, measurementCovariance.xx);
      var fusedY = fuseAxis(prior.y, correctionCovariance.yy,
        measuredCorrection.y, measurementCovariance.yy);
      var yawMeasurement = prior.yaw + Pose2.wrapAngle(measuredCorrection.yaw - prior.yaw);
      var fusedYaw = fuseAxis(prior.yaw, correctionCovariance.yawYaw,
        yawMeasurement, measurementCovariance.yawYaw);
      referenceFromOdometry = new Pose2(fusedX.value, fusedY.value, fusedYaw.value);
      correctionCovariance = new PoseCovariance2(fusedX.variance, 0.0, 0.0,
        fusedY.variance, 0.0, fusedYaw.variance);
    }
    hasAbsoluteObservation = true;
    absoluteQuality = observation.quality;
    return publish(odom);
  }

  public function state():Null<LocalizationState> return currentState;

  public function reset(?pose:Pose2):Void {
    odometry.reset(pose);
    latestOdometry = null;
    currentState = null;
    referenceFromOdometry = null;
    correctionCovariance = new PoseCovariance2(1.0e6, 0.0, 0.0,
      1.0e6, 0.0, 1.0e6);
    hasAbsoluteObservation = false;
    absoluteQuality = Degraded;
  }

  function ensureReferenceFromOdometry(odometryReference:String):Void {
    if (referenceFromOdometry != null) return;
    if (referenceFrame == odometryReference) {
      referenceFromOdometry = new Pose2();
      return;
    }
    try {
      referenceFromOdometry = frames.lookup(referenceFrame, odometryReference);
    } catch (_:Dynamic) {
      // Without a known transform, publish Invalid until an absolute pose arrives.
    }
  }

  function publish(odom:LocalizationState):LocalizationState {
    var correction:Null<Pose2> = referenceFromOdometry;
    var pose = correction == null ? new Pose2() : cast(correction, Pose2).compose(odom.pose);
    var quality = switch odom.quality {
      case Invalid: Invalid;
      case _:
        hasAbsoluteObservation && absoluteQuality == LocalizationQuality.Good
          ? Good
          : (correction == null ? Invalid : Degraded);
    };
    var covariance = new PoseCovariance2(
      odom.covariance.xx + correctionCovariance.xx,
      odom.covariance.xy + correctionCovariance.xy,
      odom.covariance.xYaw + correctionCovariance.xYaw,
      odom.covariance.yy + correctionCovariance.yy,
      odom.covariance.yYaw + correctionCovariance.yYaw,
      odom.covariance.yawYaw + correctionCovariance.yawYaw);
    currentState = new LocalizationState(odom.sequence, pose, referenceFrame,
      bodyFrame, covariance, quality, odom.sourceTimestampNs,
      odom.receivedTimestampNs, odom.sourceClockId, odom.receivedClockId);
    return currentState;
  }

  static function fuseAxis(prior:Float, priorVariance:Float,
      measurement:Float, measurementVariance:Float):FusedAxis {
    var sum = priorVariance + measurementVariance;
    if (sum <= 1e-12) return new FusedAxis(measurement, 0.0);
    var gain = priorVariance / sum;
    return new FusedAxis(prior + gain * (measurement - prior),
      priorVariance * measurementVariance / sum);
  }

  static function addCovariance(left:PoseCovariance2,
      right:PoseCovariance2):PoseCovariance2
    return new PoseCovariance2(left.xx + right.xx, left.xy + right.xy,
      left.xYaw + right.xYaw, left.yy + right.yy, left.yYaw + right.yYaw,
      left.yawYaw + right.yawYaw);
}

private class FusedAxis {
  public final value:Float;
  public final variance:Float;

  public function new(value:Float, variance:Float) {
    this.value = value;
    this.variance = variance;
  }
}
