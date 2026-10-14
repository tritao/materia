package processkit.screwdriving;

import haxe.Int64;
import motionkit.robot.CartesianAdmittance;
import motionkit.robot.ServoSession;
import robotkit.core.ForceTorqueMeasurement;
import robotkit.core.SensorFrame;
import robotkit.spatial.Transform3;
import robotkit.spatial.Vec3;
import robotkit.spatial.Wrench3;
import robotkit.time.ClockMappings;
import processkit.screwdriving.ScrewEntryController.ScrewEntryLimits;
import processkit.screwdriving.ScrewEntryController.ScrewEntryPhase;

/**
 * Runs a `ScrewEntryController` over a `ServoSession` on fresh force/torque samples. This side owns what the
 * controller does not: the robot and sensor clocks, sample freshness and ordering, the sensor-to-TCP transform, the
 * measured travel, force, torque and run-time hard stops, and bringing the arm to rest before reporting. The
 * controller is made at `start`, when the measured approach pose fixes how deep the CAD entry plane lies.
 */
class ForceGuidedScrewEntry implements ScrewEntrySearch {
  public final servo:ServoSession;
  public final limits:ScrewEntryLimits;
  final admittance:CartesianAdmittance;
  /** The CAD entry pose in the robot root: x and y span the hole plane, z advances into the hole. */
  public final entry:Transform3;
  public final depthTolerance:Float;
  public var controller(default, null):Null<ScrewEntryController> = null;
  /** The tool's compliant feed: its robot joint and its axis at the TCP; the measured tip includes its travel. */
  public final feed:Null<{joint:Int, axisAtTcp:Vec3}>;
  public final sensorId:String;
  public final sensorFrame:String;
  public final tcpFromSensor:Transform3;
  public final maximumAgeNs:Int64;
  public final maximumForceN:Float;
  public final maximumTorqueNm:Float;
  public final maximumTravelMeters:Float;
  public final maximumTravelRadians:Float;
  public final maximumRunSeconds:Float;
  /** Lateral reach of the registered region around the approach axis. */
  public final regionHalfU:Float;
  public final regionHalfV:Float;
  public var complete(default, null):Bool = false;
  public var failure(default, null):Null<String> = null;
  /** Peak measured force during this search, excluding later fastening loads. */
  public var peakForceN(default, null):Float = 0.0;
  public var lastMeasurement(default, null):Null<ForceTorqueMeasurement> = null;
  /** The measured TCP at rest after a seated entry. */
  public var measuredEntry(default, null):Null<Transform3> = null;
  final mappings:Null<ClockMappings>;
  var active = false;
  var stopping = false;
  var sourceClock = "";
  var sensorClock:Null<String> = null;
  var startNs:Int64 = Int64.ofInt(0);
  var previousUpdateNs:Int64 = Int64.ofInt(0);
  var previousSequence:Null<Int64> = null;
  var previousSampleNs:Null<Int64> = null;
  var previousControlNs:Null<Int64> = null;
  var commandSequence = 0;
  /** The measured TCP when the search started: the approach pose. */
  public var origin(default, null):Null<Transform3> = null;

  /** `tcp_T_sensor` from the two frames' poses in their shared link. */
  public static function sensorTransformAtTcp(linkFromSensor:Transform3, linkFromTcp:Transform3):Transform3 {
    if (linkFromSensor == null || linkFromTcp == null)
      throw "Force/torque conversion needs the mounted sensor and screwdriver TCP frames";
    return linkFromTcp.inverse().compose(linkFromSensor);
  }

  public function new(servo:ServoSession, limits:ScrewEntryLimits, admittance:CartesianAdmittance, entry:Transform3,
      depthTolerance:Float, sensorId:String, sensorFrame:String,
      tcpFromSensor:Transform3, maximumAgeNs:Int64, maximumForceN:Float, maximumTorqueNm:Float,
      maximumTravelMeters:Float, maximumTravelRadians:Float, maximumRunSeconds:Float, regionHalfU:Float,
      regionHalfV:Float, mappings:Null<ClockMappings>, ?feed:{joint:Int, axisAtTcp:Vec3}) {
    function positive(value:Float):Bool return Math.isFinite(value) && value > 0;
    if (servo == null || limits == null || admittance == null || entry == null || sensorId == null || sensorId.length == 0 || sensorFrame == null ||
        sensorFrame.length == 0 || tcpFromSensor == null || Int64.compare(maximumAgeNs, Int64.ofInt(0)) <= 0 ||
        !positive(maximumForceN) || maximumForceN <= limits.contactForceN || !positive(maximumTorqueNm) ||
        !positive(maximumTravelMeters) || !positive(maximumTravelRadians) || !positive(maximumRunSeconds) ||
        !positive(regionHalfU) || !positive(regionHalfV) ||
        limits.spiralRadius + limits.trackingTolerance > Math.min(regionHalfU, regionHalfV))
      throw "Force-guided screw entry needs explicit sensor, freshness, force, travel, time and region bounds";
    if (servo.robot.snapshot().trajectoryActive)
      throw "Force-guided screw entry cannot take over an active motion program";
    this.servo = servo;
    this.limits = limits;
    this.admittance = admittance;
    this.entry = entry;
    this.depthTolerance = depthTolerance;
    this.sensorId = sensorId;
    this.sensorFrame = sensorFrame;
    this.tcpFromSensor = tcpFromSensor;
    this.maximumAgeNs = maximumAgeNs;
    this.maximumForceN = maximumForceN;
    this.maximumTorqueNm = maximumTorqueNm;
    this.maximumTravelMeters = maximumTravelMeters;
    this.maximumTravelRadians = maximumTravelRadians;
    this.maximumRunSeconds = maximumRunSeconds;
    this.regionHalfU = regionHalfU;
    this.regionHalfV = regionHalfV;
    this.mappings = mappings;
    this.feed = feed;
  }

  public function start():Void {
    if (active || complete || failure != null) throw "Force-guided screw entry cannot be reused";
    var snapshot = servo.robot.snapshot();
    if (snapshot.sourceClockId == null || snapshot.sourceClockId.length == 0 || snapshot.sourceClockId == "unspecified")
      throw "Force-guided screw entry needs the robot's identified monotonic source clock";
    sourceClock = snapshot.sourceClockId;
    startNs = snapshot.sourceTimestampNs;
    previousUpdateNs = startNs;
    var start = tip(snapshot);
    var axisU = entry.transformVector(new Vec3(1, 0, 0)).normalized();
    var axisV = entry.transformVector(new Vec3(0, 1, 0)).normalized();
    var depth = entry.translation.sub(start.translation).dot(axisU.cross(axisV));
    controller = new ScrewEntryController(limits, axisU, axisV, depth, depthTolerance, admittance);
    origin = start;
    active = true;
  }

  public function running():Bool return active;
  public function isComplete():Bool return complete;
  public function failureReason():Null<String> return failure;
  public function entryPose():Null<Transform3> return measuredEntry;

  /** Read one robot snapshot and, on a fresh force/torque sample, run one control step. */
  public function update(screwHeld:Bool):Void {
    if (!active) return;
    var snapshot = servo.robot.snapshot();
    var now = snapshot.sourceTimestampNs;
    if (snapshot.sourceClockId != sourceClock) {
      // The joint observations no longer share the search's clock; stop with what the robot reports.
      requestStop("robot clock changed epoch during the screw entry");
      try servo.robot.stop(robotkit.core.StopMode.Normal) catch (_:Dynamic) {}
      if (atRest(snapshot)) active = false;
      return;
    }
    if (now < previousUpdateNs) { requestStop("robot clock moved backwards during the screw entry"); brake(snapshot); return; }
    if (now == previousUpdateNs) return;
    var dt = Int64.toFloat(Int64.sub(now, previousUpdateNs)) * 1e-9;
    previousUpdateNs = now;
    if (Int64.toFloat(Int64.sub(now, startNs)) * 1e-9 > maximumRunSeconds) {
      requestStop('screw entry exceeded its ${maximumRunSeconds} s run bound in ${controller == null ? null : controller.phase}');
      brake(snapshot);
      return;
    }
    if (stopping) { brake(snapshot); return; }
    if (!screwHeld) { requestStop("screwdriver lost the held screw during the entry search"); brake(snapshot); return; }
    try {
      var measurement = freshSample(snapshot, now);
      if (measurement == null) { servo.update(); return; }
      var tcp = tip(snapshot);
      var start = origin, control = controller;
      if (start == null || control == null) throw "screw entry lost its approach origin";
      var displacement = tcp.translation.sub(start.translation);
      var travel = displacement.norm();
      if (travel > maximumTravelMeters)
        throw 'screw entry exhausted its measured TCP travel: ${travel} m > ${maximumTravelMeters} m';
      var angle = tcp.rotation.angularDistance(start.rotation);
      if (angle > maximumTravelRadians)
        throw 'screw entry exhausted its measured TCP rotation: ${angle} rad > ${maximumTravelRadians} rad';
      var u = displacement.dot(control.axisU), v = displacement.dot(control.axisV);
      if (Math.abs(u) > regionHalfU || Math.abs(v) > regionHalfV)
        throw 'screw TCP left the registered search region at ${u} m, ${v} m';
      var wrench = new Transform3(Vec3.zero(), tcp.rotation).transformWrench(tcpFromSensor.transformWrench(measurement.wrench));
      var force = wrench.force.norm(), torque = wrench.torque.norm();
      if (force > peakForceN) peakForceN = force;
      if (force > maximumForceN) {
        var f = wrench.force, slide = feed;
        throw 'screw entry measured ${force} N, above its ${maximumForceN} N bound: ' +
          'u ${f.dot(control.axisU)} N, v ${f.dot(control.axisV)} N, n ${f.dot(control.axisN)} N at depth ' +
          '${displacement.dot(control.axisN)} m, feed ${slide == null ? 0.0 : snapshot.positions.get(slide.joint)} m';
      }
      if (torque > maximumTorqueNm) throw 'screw entry measured ${torque} N m, above its ${maximumTorqueNm} N m bound';
      // Integrate over the sensor's acquisition period, the time the measured wrench actually covers.
      var controlNs = measurement.robotTimestampNs;
      var previous = previousControlNs;
      var period = previous == null ? dt : Int64.toFloat(Int64.sub(controlNs, previous)) * 1e-9;
      previousControlNs = controlNs;
      var twist = control.step(displacement, wrench, period > 0 ? period : dt);
      switch (control.phase) {
        case Seated: requestStop(null); brake(snapshot); return;
        case Failed: requestStop(control.failure); brake(snapshot); return;
        case Approach, Search, Descend:
      }
      commandSequence++;
      var rejected = servo.command(twist, commandSequence, now + maximumAgeNs);
      if (rejected != null) throw 'screw entry servo command rejected: $rejected';
      servo.update();
    } catch (error:Dynamic) {
      requestStop(Std.string(error));
      brake(snapshot);
    }
  }

  public function cancel():Void if (active) {
    requestStop("screw entry cancelled");
    try servo.robot.stop(robotkit.core.StopMode.Normal) catch (_:Dynamic) {}
  }

  /** The newest sample if it is a new acquisition, null when the sensor has not sampled again since the last step. */
  function freshSample(snapshot:robotkit.core.RobotSnapshot, now:Int64):Null<ForceTorqueMeasurement> {
    var frame:Null<SensorFrame> = null;
    for (index in 0...snapshot.sensors.length) {
      var candidate = snapshot.sensors.get(index);
      if (candidate.sensorId == sensorId) frame = candidate;
    }
    var measurement = ForceTorqueMeasurement.fromFrame(frame, sensorId, sensorFrame, now, sourceClock, maximumAgeNs,
      mappings);
    if (sensorClock != null && measurement.sourceClockId != sensorClock)
      throw 'force/torque sensor "$sensorId" changed source clock during the screw entry';
    sensorClock = measurement.sourceClockId;
    var sequence = previousSequence;
    if (sequence != null) {
      if (measurement.sequence < sequence ||
          (measurement.sequence > sequence && measurement.sourceTimestampNs <= cast(previousSampleNs, Int64)))
        throw 'force/torque sensor "$sensorId" sequence or source time moved backwards';
      if (measurement.sequence == sequence) {
        var last = lastMeasurement;
        if (last == null || measurement.sourceTimestampNs != last.sourceTimestampNs || !sameWrench(measurement.wrench, last.wrench))
          throw 'force/torque sensor "$sensorId" repeated a sequence with different data';
        return null;
      }
    }
    previousSequence = measurement.sequence;
    previousSampleNs = measurement.sourceTimestampNs;
    lastMeasurement = measurement;
    return measurement;
  }

  function requestStop(reason:Null<String>):Void {
    if (reason != null && failure == null) failure = reason;
    stopping = true;
    servo.stop();
  }

  function atRest(snapshot:robotkit.core.RobotSnapshot):Bool {
    for (index in servo.manipulator.jointIndices()) if (Math.abs(snapshot.velocities.get(index)) > 1e-5) return false;
    return true;
  }

  function brake(snapshot:robotkit.core.RobotSnapshot):Void {
    var tick = servo.update();
    if (!tick.atRest || snapshot.trajectoryActive || !atRest(snapshot)) return;
    active = false;
    measuredEntry = tip(snapshot);
    var control = controller;
    if (failure == null && control != null && control.phase == Seated) complete = true;
    else if (failure == null) failure = "screw entry stopped before seating";
  }

  /** The measured bit tip: the arm's TCP, plus the feed's measured travel when the tool has one. */
  function tip(snapshot:robotkit.core.RobotSnapshot):Transform3 {
    var tcp = servo.manipulator.tcpPose([for (index in servo.manipulator.jointIndices()) snapshot.positions.get(index)]);
    var slide = feed;
    if (slide == null) return tcp;
    var travel = snapshot.positions.get(slide.joint);
    return new Transform3(tcp.translation.add(tcp.rotation.rotate(slide.axisAtTcp.scale(travel))), tcp.rotation);
  }

  static function sameWrench(left:Wrench3, right:Wrench3):Bool
    return left.force.x == right.force.x && left.force.y == right.force.y && left.force.z == right.force.z &&
      left.torque.x == right.torque.x && left.torque.y == right.torque.y && left.torque.z == right.torque.z;
}
