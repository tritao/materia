package robotkit.localization;

import haxe.Int64;
import robotkit.mobile.DifferentialOdometry;
import robotkit.mobile.MobileBase;
import robotkit.mobile.Pose2;
import robotkit.model.RobotModel;
import robotkit.world.RobotSnapshot;
import robotkit.world.SensorFrame;

/** Wheel displacement with heading increments corrected by the IMU gyro z rate. */
class WheelImuLocalization implements Localization {
  public final base:MobileBase;
  public final imuSensorId:String;
  public final sensorFrameId:Null<String>;
  public final referenceFrame:String;
  public final bodyFrame:String;
  public final fusionWeight:Float;
  public final imuYawVariance:Float;
  public final maximumAgeNs:Int64;

  final odometry:DifferentialOdometry;
  final mountRotation:Array<Float>;
  var pose:Pose2 = new Pose2();
  var previousTimestamp:Null<Int64> = null;
  var previousClock:Null<String> = null;
  var variancePosition:Float = 0.0;
  var varianceYaw:Float = 0.0;
  var currentState:Null<LocalizationState> = null;

  /** `mountRotation` is the sensor's xyzw orientation in the body frame. */
  public function new(base:MobileBase, imuSensorId:String,
      ?referenceFrame:String = "odom", ?bodyFrame:String = "base",
      ?fusionWeight:Float = 0.5, ?imuYawVariance:Float = 0.01,
      ?maximumAgeNs:Float = 200000000.0, ?sensorFrameId:String,
      ?mountRotation:Array<Float>) {
    if (base == null || imuSensorId == null || imuSensorId.length == 0 ||
        referenceFrame == null || referenceFrame.length == 0 || bodyFrame == null ||
        bodyFrame.length == 0 || referenceFrame == bodyFrame ||
        !Math.isFinite(fusionWeight) || fusionWeight < 0.0 || fusionWeight > 1.0 ||
        !Math.isFinite(imuYawVariance) || imuYawVariance < 0.0 ||
        !Math.isFinite(maximumAgeNs) || maximumAgeNs <= 0.0 || maximumAgeNs > 1.0e15 ||
        (sensorFrameId != null && sensorFrameId.length == 0))
      throw "Wheel and IMU localization configuration is invalid";
    var configuredOdometry = base.driveModel.createOdometry();
    if (configuredOdometry == null) throw "Wheel and IMU localization requires differential wheel odometry";
    var rotation = mountRotation == null ? [0.0, 0.0, 0.0, 1.0] : mountRotation;
    if (rotation.length != 4) throw "IMU mount rotation requires an xyzw quaternion";
    var norm = 0.0;
    for (component in rotation) {
      if (!Math.isFinite(component)) throw "IMU mount rotation must be finite";
      norm += component * component;
    }
    if (norm < 1e-12) throw "IMU mount rotation must be nonzero";
    this.base = base;
    this.imuSensorId = imuSensorId;
    this.sensorFrameId = sensorFrameId;
    this.referenceFrame = referenceFrame;
    this.bodyFrame = bodyFrame;
    this.fusionWeight = fusionWeight;
    this.imuYawVariance = imuYawVariance;
    this.maximumAgeNs = Int64.fromFloat(maximumAgeNs);
    this.odometry = configuredOdometry;
    this.mountRotation = [for (component in rotation) component / Math.sqrt(norm)];
  }

  /** Resolves the IMU and its mount from the authored model. */
  public static function fromRobotModel(base:MobileBase, model:RobotModel,
      imuSensorId:String, bodyLinkId:String, ?referenceFrame:String = "odom",
      ?bodyFrame:String = "base", ?fusionWeight:Float = 0.5,
      ?imuYawVariance:Float = 0.01, ?maximumAgeNs:Float = 200000000.0):WheelImuLocalization {
    if (model == null || imuSensorId == null || imuSensorId.length == 0)
      throw "Model-driven IMU localization requires a robot model and sensor ID";
    var sensor:Null<robotkit.model.Sensor> = null;
    for (candidate in model.sensors) if (candidate != null && candidate.id == imuSensorId) {
      if (sensor != null) throw 'IMU sensor ID "$imuSensorId" is ambiguous';
      sensor = candidate;
    }
    if (sensor == null || sensor.kind != "imu") throw 'IMU sensor "$imuSensorId" is missing or not an imu';
    var configured:robotkit.model.Sensor = cast sensor;
    var frameId = configured.frame == null ? bodyLinkId : configured.frame.id;
    if (configured.frame != null && configured.frame.link.id != bodyLinkId)
      throw 'IMU sensor "$imuSensorId" must be mounted on body link "$bodyLinkId"';
    return new WheelImuLocalization(base, imuSensorId, referenceFrame, bodyFrame,
      fusionWeight, imuYawVariance, maximumAgeNs, frameId,
      configured.frame == null ? null : configured.frame.rotation);
  }

  public function update(snapshot:RobotSnapshot):LocalizationState {
    if (snapshot == null) throw "Wheel and IMU localization requires a robot snapshot";
    odometry.update(snapshot);
    var clockChanged = previousClock != null && previousClock != snapshot.sourceClockId;
    var advancing = previousTimestamp != null && !clockChanged &&
      Int64.compare(snapshot.sourceTimestampNs, previousTimestamp) > 0;
    var wheelHeading = advancing ? odometry.lastHeadingChange : 0.0;
    var distance = advancing ? odometry.lastDistance : 0.0;
    var heading = wheelHeading;
    var imu = advancing ? latestImu(snapshot) : null;
    if (imu != null) {
      var intervalSeconds = Std.parseFloat(Int64.toStr(Int64.sub(snapshot.sourceTimestampNs,
        cast previousTimestamp))) * 1e-9;
      var gyroHeading = imu * intervalSeconds;
      heading += fusionWeight * Pose2.wrapAngle(gyroHeading - wheelHeading);
      varianceYaw += imuYawVariance * intervalSeconds * intervalSeconds;
    }
    pose = pose.integrateDisplacement(distance, heading);
    variancePosition += 0.01 * Math.abs(distance);
    varianceYaw += 0.02 * Math.abs(wheelHeading);
    if (!advancing && (previousTimestamp == null || clockChanged)) {
      previousTimestamp = snapshot.sourceTimestampNs;
      previousClock = snapshot.sourceClockId;
    } else if (advancing) previousTimestamp = snapshot.sourceTimestampNs;
    currentState = new LocalizationState(snapshot.sourceSequence, pose,
      referenceFrame, bodyFrame, new PoseCovariance2(variancePosition, 0.0, 0.0,
        variancePosition, 0.0, varianceYaw), LocalizationQuality.Degraded,
      snapshot.sourceTimestampNs, snapshot.receivedTimestampNs,
      snapshot.sourceClockId, snapshot.receivedClockId);
    return currentState;
  }

  public function state():Null<LocalizationState> return currentState;

  public function reset(?newPose:Pose2):Void {
    pose = newPose == null ? new Pose2() : new Pose2(newPose.x, newPose.y, newPose.yaw);
    odometry.reset();
    previousTimestamp = null;
    previousClock = null;
    variancePosition = 0.0;
    varianceYaw = 0.0;
    currentState = null;
  }

  function latestImu(snapshot:RobotSnapshot):Null<Float> {
    var selected:Null<SensorFrame> = null;
    for (frame in snapshot.sensors.toArray()) {
      if (frame == null || frame.sensorId != imuSensorId || frame.kind != "imu" ||
          (sensorFrameId != null && frame.frameId != sensorFrameId) ||
          frame.values.length < 3) continue;
      var age:Null<Int64> = null;
      if (frame.sourceClockId == snapshot.sourceClockId)
        age = Int64.sub(snapshot.sourceTimestampNs, frame.sourceTimestampNs);
      else if (frame.receivedClockId == snapshot.receivedClockId)
        age = Int64.sub(snapshot.receivedTimestampNs, frame.receivedTimestampNs);
      if (age == null) continue;
      var absoluteAge:Int64 = cast age;
      if (Int64.compare(absoluteAge, Int64.ofInt(0)) < 0)
        absoluteAge = Int64.sub(Int64.ofInt(0), absoluteAge);
      if (Int64.compare(absoluteAge, maximumAgeNs) > 0) continue;
      if (selected == null || Int64.compare(frame.sourceTimestampNs,
          selected.sourceTimestampNs) > 0) selected = frame;
    }
    if (selected == null) return null;
    var x = selected.values.get(0);
    var y = selected.values.get(1);
    var z = selected.values.get(2);
    if (!Math.isFinite(x) || !Math.isFinite(y) || !Math.isFinite(z)) return null;
    // Rotate the sensor-frame angular velocity into the body frame, then use z.
    var qx = mountRotation[0], qy = mountRotation[1];
    var qz = mountRotation[2], qw = mountRotation[3];
    return 2.0 * (qx * qz - qw * qy) * x +
      2.0 * (qy * qz + qw * qx) * y +
      (1.0 - 2.0 * (qx * qx + qy * qy)) * z;
  }
}
