package robotkit.localization;

import haxe.Int64;
import robotkit.model.RobotModel;
import robotkit.mobile.Pose2;
import robotkit.world.RobotSnapshot;
import robotkit.world.SensorFrame;

/**
 * Converts a dual-antenna GNSS pose fix into a local ENU frame.
 *
 * Sensor values are latitude degrees, longitude degrees, and ENU yaw radians,
 * as published for a `gnss_pose` sensor. The configured antenna pose is the
 * antenna origin expressed in the robot body frame. Altitude is intentionally
 * ignored by this planar localization service. A fix is used while it lies
 * within `maximumAgeNs` of the robot snapshot, measured on its source clock
 * when that matches the snapshot's, or else on the shared local receive clock;
 * a fix from an unrelated clock is ignored. Feed valid states to
 * `PoseFusionLocalization.fuse` to correct wheel odometry.
 */
class GnssPoseLocalization implements Localization {
  static inline var WGS84_A:Float = 6378137.0;
  static inline var WGS84_F:Float = 1.0 / 298.257223563;

  public final sensorId:String;
  public final referenceFrame:String;
  public final bodyFrame:String;
  public final sensorFrameId:Null<String>;
  public final originLatitudeDegrees:Float;
  public final originLongitudeDegrees:Float;
  public final antennaPoseInBody:Pose2;
  public final positionVarianceMetersSquared:Float;
  public final yawVarianceRadiansSquared:Float;
  public final maximumAgeNs:Int64;

  var originEcef:Array<Float>;
  var originSinLatitude:Float;
  var originCosLatitude:Float;
  var originSinLongitude:Float;
  var originCosLongitude:Float;
  var correction:Pose2 = new Pose2();
  var pendingReset:Null<Pose2> = null;
  var lastRawPose:Null<Pose2> = null;
  var currentState:Null<LocalizationState> = null;

  public function new(sensorId:String, referenceFrame:String, bodyFrame:String,
      originLatitudeDegrees:Float, originLongitudeDegrees:Float,
      antennaPoseInBody:Pose2, ?positionVarianceMetersSquared:Float = 0.25,
      ?yawVarianceRadiansSquared:Float = 0.04,
      ?maximumAgeNs:Float = 500000000.0, ?sensorFrameId:String) {
    if (sensorId == null || sensorId.length == 0 || referenceFrame == null ||
        referenceFrame.length == 0 || bodyFrame == null || bodyFrame.length == 0 ||
        referenceFrame == bodyFrame || !validLatitude(originLatitudeDegrees) ||
        !validLongitude(originLongitudeDegrees) || antennaPoseInBody == null ||
        !Math.isFinite(positionVarianceMetersSquared) ||
        positionVarianceMetersSquared < 0.0 ||
        !Math.isFinite(yawVarianceRadiansSquared) || yawVarianceRadiansSquared < 0.0 ||
        !Math.isFinite(maximumAgeNs) || maximumAgeNs < 0.0 || maximumAgeNs > 1.0e15 ||
        (sensorFrameId != null && sensorFrameId.length == 0))
      throw "GNSS pose localization configuration is invalid";
    this.sensorId = sensorId;
    this.referenceFrame = referenceFrame;
    this.bodyFrame = bodyFrame;
    this.sensorFrameId = sensorFrameId;
    this.originLatitudeDegrees = originLatitudeDegrees;
    this.originLongitudeDegrees = originLongitudeDegrees;
    this.antennaPoseInBody = new Pose2(antennaPoseInBody.x,
      antennaPoseInBody.y, antennaPoseInBody.yaw);
    this.positionVarianceMetersSquared = positionVarianceMetersSquared;
    this.yawVarianceRadiansSquared = yawVarianceRadiansSquared;
    this.maximumAgeNs = Int64.fromFloat(maximumAgeNs);

    var latitude = radians(originLatitudeDegrees);
    var longitude = radians(originLongitudeDegrees);
    originSinLatitude = Math.sin(latitude);
    originCosLatitude = Math.cos(latitude);
    originSinLongitude = Math.sin(longitude);
    originCosLongitude = Math.cos(longitude);
    originEcef = ecef(latitude, longitude);
  }

  /** Builds the GNSS source and antenna lever arm from authored robot frames. */
  public static function fromRobotModel(model:RobotModel, sensorId:String,
      bodyLinkId:String, referenceFrame:String, bodyFrame:String,
      originLatitudeDegrees:Float, originLongitudeDegrees:Float,
      ?positionVarianceMetersSquared:Float = 0.25,
      ?yawVarianceRadiansSquared:Float = 0.04,
      ?maximumAgeNs:Float = 500000000.0):GnssPoseLocalization {
    if (model == null || sensorId == null || sensorId.length == 0)
      throw "Model-driven GNSS configuration requires a robot model and sensor ID";
    var sensor:Null<robotkit.model.Sensor> = null;
    for (candidate in model.sensors) if (candidate != null && candidate.id == sensorId) {
      if (sensor != null) throw 'GNSS sensor ID "$sensorId" is ambiguous';
      sensor = candidate;
    }
    if (sensor == null) throw 'GNSS sensor "$sensorId" is missing from the robot model';
    if (sensor.kind != "gnss_pose")
      throw 'Sensor "$sensorId" must use the gnss_pose kind';
    var configuredSensor:robotkit.model.Sensor = cast sensor;
    var frameTree = FrameTree2.fromRobotModel(model, bodyLinkId);
    var sensorFrame = configuredSensor.frame == null ? bodyLinkId :
      configuredSensor.frame.id;
    var antennaPose:Pose2;
    try {
      antennaPose = frameTree.lookup(bodyLinkId, sensorFrame);
    } catch (error:Dynamic) {
      throw 'GNSS sensor "$sensorId" must be mounted on body link "$bodyLinkId"';
    }
    return new GnssPoseLocalization(sensorId, referenceFrame, bodyFrame,
      originLatitudeDegrees, originLongitudeDegrees, antennaPose,
      positionVarianceMetersSquared, yawVarianceRadiansSquared, maximumAgeNs,
      sensorFrame);
  }

  public function update(snapshot:RobotSnapshot):LocalizationState {
    if (snapshot == null) throw "GNSS pose localization requires a robot snapshot";
    var fix = latestFix(snapshot);
    if (fix == null) return invalidState(snapshot);

    var position = enu(fix.latitude, fix.longitude);
    var antennaPose = new Pose2(position.x, position.y, fix.yaw);
    var rawPose = removeAntennaOffset(antennaPose, antennaPoseInBody);
    if (pendingReset != null) {
      correction = cast pendingReset.compose(inverse(rawPose));
      pendingReset = null;
    }
    lastRawPose = rawPose;
    var pose = correction.compose(rawPose);
    var rawCovariance = bodyCovariance(rawPose, antennaPoseInBody,
      positionVarianceMetersSquared, yawVarianceRadiansSquared);
    var covariance = rotateCovariance(rawCovariance, correction.yaw);
    currentState = new LocalizationState(fix.sequence, pose, referenceFrame,
      bodyFrame, covariance, LocalizationQuality.Degraded, fix.sourceTimestampNs,
      fix.receivedTimestampNs, fix.sourceClockId, fix.receivedClockId);
    return currentState;
  }

  public function state():Null<LocalizationState> return currentState;

  public function reset(?pose:Pose2):Void {
    if (pose == null) {
      correction = new Pose2();
      pendingReset = null;
      lastRawPose = null;
    } else if (lastRawPose == null) {
      pendingReset = new Pose2(pose.x, pose.y, pose.yaw);
    } else {
      correction = pose.compose(inverse(cast lastRawPose));
      pendingReset = null;
    }
    currentState = null;
  }

  function latestFix(snapshot:RobotSnapshot):Null<GnssFix> {
    var selected:Null<SensorFrame> = null;
    for (frame in snapshot.sensors.toArray()) {
      if (frame == null || frame.sensorId != sensorId || frame.kind != "gnss_pose" ||
          (sensorFrameId != null && frame.frameId != sensorFrameId) ||
          frame.values.length < 3 || frame.sourceClockId == null ||
          frame.sourceClockId.length == 0 || frame.receivedClockId == null ||
          frame.receivedClockId.length == 0)
        continue;
      // A fix newer than the robot state (negative age) is as usable as an
      // older one; both are bounded by the same window.
      var age = sampleAgeNs(frame, snapshot);
      if (age == null) continue;
      var span:Int64 = cast age;
      if (Int64.compare(span, Int64.ofInt(0)) < 0) span = Int64.sub(Int64.ofInt(0), span);
      if (Int64.compare(span, maximumAgeNs) > 0) continue;
      if (selected == null || Int64.compare(frame.sourceTimestampNs,
          selected.sourceTimestampNs) > 0) selected = frame;
    }
    if (selected == null) return null;
    var latitude = selected.values.get(0);
    var longitude = selected.values.get(1);
    var yaw = selected.values.get(2);
    if (!validLatitude(latitude) || !validLongitude(longitude) || !Math.isFinite(yaw))
      return null;
    return new GnssFix(selected.sequence, selected.sourceTimestampNs,
      selected.receivedTimestampNs, selected.sourceClockId,
      selected.receivedClockId, latitude, longitude, yaw);
  }

  /**
   * Age of a fix at the snapshot, on the source clock when both share it, or
   * else on the shared local receive clock; null when neither is shared.
   */
  static function sampleAgeNs(frame:SensorFrame, snapshot:RobotSnapshot):Null<Int64> {
    if (frame.sourceClockId == snapshot.sourceClockId)
      return Int64.sub(snapshot.sourceTimestampNs, frame.sourceTimestampNs);
    if (frame.receivedClockId == snapshot.receivedClockId)
      return Int64.sub(snapshot.receivedTimestampNs, frame.receivedTimestampNs);
    return null;
  }

  function enu(latitudeDegrees:Float, longitudeDegrees:Float):Pose2 {
    var position = ecef(radians(latitudeDegrees), radians(longitudeDegrees));
    var dx = position[0] - originEcef[0];
    var dy = position[1] - originEcef[1];
    var dz = position[2] - originEcef[2];
    var east = -originSinLongitude * dx + originCosLongitude * dy;
    var north = -originSinLatitude * originCosLongitude * dx -
      originSinLatitude * originSinLongitude * dy + originCosLatitude * dz;
    return new Pose2(east, north);
  }

  static function ecef(latitude:Float, longitude:Float):Array<Float> {
    var eccentricitySquared = WGS84_F * (2.0 - WGS84_F);
    var sinLatitude = Math.sin(latitude);
    var cosLatitude = Math.cos(latitude);
    var normalRadius = WGS84_A / Math.pow(1.0 - eccentricitySquared *
      sinLatitude * sinLatitude, 0.5);
    return [normalRadius * cosLatitude * Math.cos(longitude),
      normalRadius * cosLatitude * Math.sin(longitude),
      normalRadius * (1.0 - eccentricitySquared) * sinLatitude];
  }

  static function removeAntennaOffset(antennaPose:Pose2,
      antennaPoseInBody:Pose2):Pose2 {
    var bodyYaw = Pose2.wrapAngle(antennaPose.yaw - antennaPoseInBody.yaw);
    var cosine = Math.cos(bodyYaw);
    var sine = Math.sin(bodyYaw);
    return new Pose2(antennaPose.x - cosine * antennaPoseInBody.x +
        sine * antennaPoseInBody.y,
      antennaPose.y - sine * antennaPoseInBody.x -
        cosine * antennaPoseInBody.y, bodyYaw);
  }

  static function bodyCovariance(bodyPose:Pose2, antennaPoseInBody:Pose2,
      positionVariance:Float, yawVariance:Float):PoseCovariance2 {
    var cosine = Math.cos(bodyPose.yaw);
    var sine = Math.sin(bodyPose.yaw);
    var dxYaw = sine * antennaPoseInBody.x + cosine * antennaPoseInBody.y;
    var dyYaw = -cosine * antennaPoseInBody.x + sine * antennaPoseInBody.y;
    return new PoseCovariance2(positionVariance + dxYaw * dxYaw * yawVariance,
      dxYaw * dyYaw * yawVariance, dxYaw * yawVariance,
      positionVariance + dyYaw * dyYaw * yawVariance,
      dyYaw * yawVariance, yawVariance);
  }

  static function rotateCovariance(covariance:PoseCovariance2,
      yaw:Float):PoseCovariance2 {
    var cosine = Math.cos(yaw);
    var sine = Math.sin(yaw);
    var xx = covariance.xx;
    var xy = covariance.xy;
    var xYaw = covariance.xYaw;
    var yy = covariance.yy;
    var yYaw = covariance.yYaw;
    return new PoseCovariance2(
      cosine * cosine * xx - 2.0 * cosine * sine * xy + sine * sine * yy,
      cosine * sine * (xx - yy) + (cosine * cosine - sine * sine) * xy,
      cosine * xYaw - sine * yYaw,
      sine * sine * xx + 2.0 * cosine * sine * xy + cosine * cosine * yy,
      sine * xYaw + cosine * yYaw, covariance.yawYaw);
  }

  static function inverse(pose:Pose2):Pose2 {
    var cosine = Math.cos(pose.yaw);
    var sine = Math.sin(pose.yaw);
    return new Pose2(-cosine * pose.x - sine * pose.y,
      sine * pose.x - cosine * pose.y, -pose.yaw);
  }

  function invalidState(snapshot:RobotSnapshot):LocalizationState {
    var pose = currentState == null ? new Pose2() : currentState.pose;
    currentState = new LocalizationState(snapshot.sourceSequence, pose,
      referenceFrame, bodyFrame,
      new PoseCovariance2(1.0e30, 0.0, 0.0, 1.0e30, 0.0, 1.0e30),
      LocalizationQuality.Invalid, snapshot.sourceTimestampNs,
      snapshot.receivedTimestampNs, snapshot.sourceClockId, snapshot.receivedClockId);
    return currentState;
  }

  static inline function radians(degrees:Float):Float return degrees * Math.PI / 180.0;
  static inline function validLatitude(value:Float):Bool
    return Math.isFinite(value) && value >= -90.0 && value <= 90.0;
  static inline function validLongitude(value:Float):Bool
    return Math.isFinite(value) && value >= -180.0 && value <= 180.0;
}

private class GnssFix {
  public final sequence:Int64;
  public final sourceTimestampNs:Int64;
  public final receivedTimestampNs:Int64;
  public final sourceClockId:String;
  public final receivedClockId:String;
  public final latitude:Float;
  public final longitude:Float;
  public final yaw:Float;

  public function new(sequence:Int64, sourceTimestampNs:Int64,
      receivedTimestampNs:Int64, sourceClockId:String, receivedClockId:String,
      latitude:Float, longitude:Float, yaw:Float) {
    this.sequence = sequence;
    this.sourceTimestampNs = sourceTimestampNs;
    this.receivedTimestampNs = receivedTimestampNs;
    this.sourceClockId = sourceClockId;
    this.receivedClockId = receivedClockId;
    this.latitude = latitude;
    this.longitude = longitude;
    this.yaw = yaw;
  }
}
