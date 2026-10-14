package processkit;

import motionkit.event.EventValue;
import motionkit.program.MotionOp;
import motionkit.program.MotionProgram;
import motionkit.robot.ManipulatorMotion;
import motionkit.robot.ServoSession;
import processkit.WelderProcessDevice.WelderChannels;
import robotkit.spatial.Transform3;
import robotkit.spatial.Vec3;
import robotkit.core.StopMode;
import robotkit.core.JointTarget;
import robotkit.core.RobotCommand;
import robotkit.manipulation.ClearanceChange;
import robotkit.manipulation.ClearanceScene;
import robotkit.manipulation.ClearanceViolation;
import haxe.Int64;

private enum ProbePhase { Idle; Approaching; HandoffCoarse; Coarse; Backoff; HandoffFine; Fine; HandoffBackoff; HandoffRetreat; Retreat; Done; Failed; }

/** A prepared nominal search pose and bounded arc-off sensing parameters, in metres and seconds. */
class ContactProbeRequest {
  public final approach:Transform3;
  public final direction:Vec3;
  public final distance:Float;
  public final coarseSpeed:Float;
  public final fineSpeed:Float;
  public final backoff:Float;
  public final contactOffset:Float;
  /** Calibrated sensor-threshold error bound in metres. */
  public var calibratedError(default, null):Float = 0.0;
  public final approachJoints:Null<Array<Float>>;
  /** Exact tool collision-body prefix and CAD member allowed to touch during sensing. */
  public final contactToolPrefix:Null<String>;
  public final contactTarget:Null<String>;

  public function new(approach:Transform3, direction:Vec3, distance:Float, coarseSpeed:Float = 0.01,
      fineSpeed:Float = 0.0005, backoff:Float = 0.002, contactOffset:Float = 0.0, ?approachJoints:Array<Float>,
      ?contactToolPrefix:String, ?contactTarget:String) {
    if (approach == null || direction == null || !Math.isFinite(direction.norm()) || direction.norm() < 1e-12 ||
        !Math.isFinite(distance) || !(distance > 0) || !Math.isFinite(coarseSpeed) || !(coarseSpeed > 0) ||
        !Math.isFinite(fineSpeed) || !(fineSpeed > 0) || fineSpeed > coarseSpeed || !Math.isFinite(backoff) ||
        !(backoff > contactOffset) || !Math.isFinite(contactOffset) || contactOffset < 0 || contactOffset > 0.002)
      throw "Contact probe needs a prepared pose, finite bounded search and calibrated refinement";
    if ((contactToolPrefix == null) != (contactTarget == null) ||
        contactToolPrefix != null && contactTarget != null &&
        (contactToolPrefix.length == 0 || contactTarget.length == 0))
      throw "Contact probe contact window needs both an exact tool prefix and CAD member";
    this.approach = approach; this.direction = direction.normalized(); this.distance = distance;
    this.coarseSpeed = coarseSpeed; this.fineSpeed = fineSpeed; this.backoff = backoff; this.contactOffset = contactOffset;
    calibratedError = 0.0;
    this.approachJoints = approachJoints == null ? null : approachJoints.copy();
    this.contactToolPrefix = contactToolPrefix; this.contactTarget = contactTarget;
  }

  /** Static clearance ends before the earliest possible calibrated touch; sensing owns the uncertain interval. */
  public function precontactDistance(nominalTravel:Float, normalTravel:Float):Float {
    if (!Math.isFinite(nominalTravel) || nominalTravel <= 0 || nominalTravel > distance + 1e-8 ||
        !Math.isFinite(normalTravel) || normalTravel < 0)
      throw "Probe pre-contact clearance needs a nominal plane inside its search and finite uncertainty";
    var travel = nominalTravel - normalTravel - contactOffset;
    if (!(travel > 0)) throw "Probe approach has no guaranteed pre-contact clearance interval";
    return travel;
  }

  /** Keep calibrated sensor and acquisition-age uncertainty within the total observation budget. */
  public function withPrecision(error:Float, calibratedError:Float, maxAgeSeconds:Float):ContactProbeRequest {
    if (!Math.isFinite(error) || !(error > 0) || !Math.isFinite(calibratedError) || calibratedError < 0 ||
        calibratedError >= error || !Math.isFinite(maxAgeSeconds) || !(maxAgeSeconds > 0))
      throw "Contact probe precision needs a total error above its calibrated sensor bound and a positive age bound";
    var speed = Math.min(fineSpeed, (error - calibratedError) / maxAgeSeconds);
    var retreat = Math.min(backoff, contactOffset + 2 * coarseSpeed * maxAgeSeconds + 2 * error);
    var result = new ContactProbeRequest(approach, direction, distance, coarseSpeed, speed, retreat, contactOffset,
      approachJoints, contactToolPrefix, contactTarget);
    result.calibratedError = calibratedError;
    return result;
  }

  /** Attach the exact CAD surface selected from the registered contact geometry. */
  public function withContactWindow(toolPrefix:String, target:String):ContactProbeRequest {
    var result = new ContactProbeRequest(approach, direction, distance, coarseSpeed, fineSpeed, backoff,
      contactOffset, approachJoints, toolPrefix, target);
    result.calibratedError = calibratedError;
    return result;
  }
}

/** One exclusive owner alternates checked motion programs with contact servo searches. */
class ContactProbeRunner {
  public static inline var SEARCH_AGE_SECONDS:Float = 0.02;
  public final motion:ManipulatorMotion;
  public final planner:ProbeMotionPlanner;
  public final channels:Null<WelderChannels>;
  public final sensor:String;
  public final sensorKind:String;
  public final coordinateFrame:String;
  public var contact(default, null):Null<Vec3> = null;
  public var contactMeasurement(default, null):Null<processkit.perception.ContactMeasurement> = null;
  public var failure(default, null):Null<String> = null;
  final makeServo:Void -> ServoSession;
  var phase:ProbePhase = Idle;
  var request:Null<ContactProbeRequest> = null;
  var search:Null<ContactSearchRunner> = null;
  var pendingContact:Null<Vec3> = null;
  var handoffAt:Int64 = Int64.ofInt(0);
  final mappings:Null<robotkit.time.ClockMappings>;
  final clearanceScene:Null<ClearanceScene>;
  final safeOutputs:Array<MotionOp>;
  var sourceClock:String = "";
  var approachGoal:Null<Array<Float>> = null;
  var contactWindow:Null<{a:String, b:String}> = null;

  public function new(motion:ManipulatorMotion, planner:ProbeMotionPlanner, channels:Null<WelderChannels>,
      sensor:String, makeServo:Void -> ServoSession, ?mappings:robotkit.time.ClockMappings,
      sensorKind:String = processkit.tool.WeldSensor.KIND, coordinateFrame:String = "root", ?safeOutputs:Array<MotionOp>) {
    if (motion == null || planner == null || motion.compiler != planner.compiler ||
        sensor == null || sensor.length == 0 || makeServo == null)
      throw "Contact probe needs one motion owner, its checked planner, an identified sensor and a fresh servo factory";
    if (sensorKind != processkit.tool.WeldSensor.KIND && sensorKind != "tool_contact")
      throw "Contact probe needs welding feedback or a digital tool-contact sensor";
    if (coordinateFrame == null || coordinateFrame.length == 0) throw "Contact probe needs a measured coordinate frame";
    var selectedSafe = safeOutputs == null ? [] : safeOutputs.copy();
    if (channels != null) {
      for (name in [channels.arc, channels.wireSpeed, channels.voltage])
        if (name == null || StringTools.trim(name).length == 0) throw "Contact probe needs three valid torch channels";
      if (channels.arc == channels.wireSpeed || channels.arc == channels.voltage || channels.wireSpeed == channels.voltage)
        throw "Contact probe torch channels must be distinct";
      if (selectedSafe.length == 0) selectedSafe = [MotionOp.SetOutput(channels.wireSpeed, EventValue.Analog(0.0)),
        MotionOp.SetOutput(channels.arc, EventValue.Digital(false))];
    }
    if (selectedSafe.length == 0) throw "Contact probe needs explicit safe process outputs";
    this.motion = motion; this.planner = planner; this.channels = channels; this.sensor = sensor; this.makeServo = makeServo;
    clearanceScene = planner.clearance != null && Std.isOfType(planner.clearance, ClearanceScene)
      ? cast planner.clearance : null;
    this.mappings = mappings; this.sensorKind = sensorKind; this.coordinateFrame = coordinateFrame; this.safeOutputs = selectedSafe;
  }

  public function running():Bool return phase != Idle && phase != Done && phase != Failed;
  public function completed():Bool return phase == Done;

  function positions():Array<Float> {
    var snapshot = motion.robot.snapshot();
    return [for (index in motion.jointIndices) snapshot.positions.get(index)];
  }

  public function start(request:ContactProbeRequest):Void {
    if (request == null || running() || motion.running) throw "Contact probe cannot start without an idle exclusive motion owner";
    this.request = request; contact = null; contactMeasurement = null; failure = null;
    try {
      sourceClock = motion.robot.snapshot().sourceClockId;
      motion.reset();
      var goal = request.approachJoints;
      var move = goal == null ? planner.approach(request.approach, positions()) : planner.approachJoints(goal, positions());
      var at = planner.arm.tcpPose(move.endJoints);
      if (at.translation.sub(request.approach.translation).norm() > planner.compiler.ikTolerance.position ||
          at.rotation.angularDistance(request.approach.rotation) > planner.compiler.ikTolerance.orientation)
        throw "Prepared probe joint goal does not match its TCP approach";
      if (!planner.corridorReachable(move.endJoints, request.direction, request.distance))
        throw "Probe sensing corridor leaves the executed IK branch";
      approachGoal = move.endJoints.copy();
      // These records precede every movement; stop policy also keeps them safe through servo ownership.
      motion.run(new MotionProgram(safeOutputs.concat(move.program.ops)));
      phase = Approaching;
    } catch (error:Dynamic) fail(Std.string(error));
  }

  function beginSearch(speed:Float, distance:Float):Void {
    var requested:ContactProbeRequest = cast request;
    var checkedSpeed = planner.sensingSpeed(positions(), requested.direction, distance, speed, requested.contactOffset, 2 * SEARCH_AGE_SECONDS);
    var servo = makeServo();
    if (servo == null || servo.robot != motion.robot || servo.manipulator != planner.arm) {
      if (servo != null) servo.dispose();
      throw "Contact probe servo factory returned a different robot or arm";
    }
    try {
      search = new ContactSearchRunner(servo, sensor, cast(request, ContactProbeRequest).direction, distance, checkedSpeed,
        SEARCH_AGE_SECONDS, 0.002, cast(request, ContactProbeRequest).contactOffset, mappings,
        sensorKind, coordinateFrame, cast(request, ContactProbeRequest).calibratedError,
        checkedSpeed * SEARCH_AGE_SECONDS);
    } catch (error:Dynamic) { servo.dispose(); throw error; }
  }

  function releaseSearch():Void {
    var active = search;
    if (active != null) active.servo.dispose();
    search = null;
  }

  /** Hold the measured pose and wait for a fresh source sample before transferring motion ownership. */
  function holdAndHandoff(next:ProbePhase):Void {
    var snapshot = motion.robot.snapshot();
    motion.robot.submit(RobotCommand.JointTargets([for (index in motion.jointIndices)
      JointTarget.position(index, snapshot.positions.get(index))], null));
    handoffAt = snapshot.sourceTimestampNs;
    phase = next;
  }

  public function update(dt:Float):Void {
    if (!running()) return;
    if (!Math.isFinite(dt) || !(dt > 0)) throw "Contact probe update needs a positive finite duration";
    try {
      if (motion.robot.snapshot().sourceClockId != sourceClock) throw "Contact probe joint clock changed epoch";
      var requested:ContactProbeRequest = cast request;
      switch phase {
        case Approaching | Backoff | Retreat:
          motion.update(dt);
          if (motion.failure != null) throw motion.failure;
          if (motion.completed) {
            switch phase {
              case Approaching: holdAndHandoff(HandoffCoarse);
              case Backoff: holdAndHandoff(HandoffFine);
              case Retreat: closeContactWindow(positions()); phase = Done;
              case _:
            }
          }
        case HandoffCoarse | HandoffFine:
          var snapshot = motion.robot.snapshot();
          if (snapshot.sourceTimestampNs <= handoffAt) return;
          if (snapshot.sourceTimestampNs - handoffAt > Int64.fromFloat(2e9))
            throw 'Contact probe position hold did not settle before ${phase == HandoffCoarse ? "coarse" : "fine"} sensing';
          var indices = motion.jointIndices;
          var joints = [for (index in indices) snapshot.positions.get(index)];
          var velocity = [for (index in indices) snapshot.velocities.get(index)];
          for (speed in velocity) if (Math.abs(speed) > 1e-5) return;
          if (planner.stoppingViolation(joints, velocity, SEARCH_AGE_SECONDS) != null) return;
          if (phase == HandoffFine) closeContactWindow(joints);
          motion.reset();
          if (phase == HandoffCoarse) {
            beginSearch(requested.coarseSpeed, requested.distance);
            phase = Coarse;
          } else {
            beginSearch(requested.fineSpeed, requested.backoff);
            phase = Fine;
          }
        case Coarse | Fine:
          var active:ContactSearchRunner = cast search;
          var snapshot = motion.robot.snapshot();
          var indices = motion.jointIndices;
          var joints = [for (index in indices) snapshot.positions.get(index)];
          var velocity = [for (index in indices) snapshot.velocities.get(index)];
          var hit = planner.stoppingViolation(joints, velocity, 0.02);
          if (hit != null && contactWindow == null && openContactWindow(hit, joints))
            hit = planner.stoppingViolation(joints, velocity, 0.02);
          if (hit != null)
            throw 'Contact probe predicted braking motion is obstructed: ${hit.a} against ${hit.b}, clearance ${hit.distance} m (required ${hit.required} m); '
              + 'contact target=${requested.contactTarget}, tool prefix=${requested.contactToolPrefix}, '
              + 'open window=${contactWindow == null ? "none" : contactWindow.a + "/" + contactWindow.b}; '
              + 'joints=${joints.join(",")}; velocity=${velocity.join(",")}';
          active.update();
          if (active.stopped) {
            if (!active.completed()) throw active.search.failure;
            var measured:Vec3 = cast active.search.contact;
            contactMeasurement = active.search.measurement;
            if (contactMeasurement == null) throw "Contact probe completed without its typed measurement record";
            releaseSearch();
            pendingContact = measured;
            // A stopped velocity target still owns the native joint. Change to a position hold and await its owner tick.
            holdAndHandoff(phase == Coarse ? HandoffBackoff : HandoffRetreat);
          }
        case HandoffBackoff | HandoffRetreat:
          var snapshot = motion.robot.snapshot();
          if (snapshot.sourceTimestampNs <= handoffAt) return;
          if (snapshot.sourceTimestampNs - handoffAt > Int64.fromFloat(2e9)) throw "Contact probe position hold did not settle";
          for (index in motion.jointIndices) if (Math.abs(snapshot.velocities.get(index)) > 1e-5) return;
          motion.reset();
          var q = positions();
          var current = planner.arm.tcpPose(q);
          var measured:Vec3 = cast pendingContact;
          if (phase == HandoffBackoff) {
            var withdrawn = new Transform3(measured.sub(requested.direction.scale(requested.backoff)), current.rotation);
            var move = planner.line(withdrawn, q, requested.coarseSpeed, true);
            motion.run(move.program); phase = Backoff;
          } else {
            contact = measured;
            var move = planner.retreat(requested.approach, q, requested.coarseSpeed, cast approachGoal);
            motion.run(move.program); phase = Retreat;
          }
        case Idle | Done | Failed:
      }
    } catch (error:Dynamic) fail(Std.string(error));
  }

  public function cancel():Void if (running()) fail("contact probe cancelled");

  function openContactWindow(hit:ClearanceViolation, joints:Array<Float>):Bool {
    var requested:ContactProbeRequest = cast request;
    var prefix = requested.contactToolPrefix, target = requested.contactTarget;
    if (prefix == null || target == null) return false;
    function isTool(name:String):Bool return name == prefix || StringTools.startsWith(name, prefix + "/");
    function isTarget(name:String):Bool return name == target || StringTools.startsWith(name, target + "/") ||
      StringTools.startsWith(name, target + "#");
    var tool = isTool(hit.a) ? hit.a : isTool(hit.b) ? hit.b : null;
    var work = isTarget(hit.a) ? hit.a : isTarget(hit.b) ? hit.b : null;
    if (tool == null || work == null) return false;
    if (clearanceScene == null)
      throw "Contact probe has a named CAD target but no mutable collision scene for its contact window";
    clearanceScene.apply(ClearanceChange.OpenWindow(tool, work, null), joints);
    contactWindow = {a: tool, b: work};
    return true;
  }

  function closeContactWindow(joints:Array<Float>):Void {
    var active = contactWindow;
    if (active == null) return;
    if (clearanceScene == null) throw "Contact probe lost the collision scene for its open contact window";
    clearanceScene.apply(ClearanceChange.CloseWindow(active.a, active.b), joints);
    contactWindow = null;
  }

  function fail(reason:String):Void {
    var active = search;
    if (active != null) active.cancel();
    try motion.robot.stop(StopMode.Normal) catch (_:Dynamic) {}
    try motion.abort() catch (_:Dynamic) {}
    var cleanup = "";
    try closeContactWindow(positions()) catch (error:Dynamic) cleanup = '; contact-window cleanup failed: ${Std.string(error)}';
    releaseSearch(); contact = null; failure = '$phase: $reason$cleanup'; phase = Failed;
  }
}
