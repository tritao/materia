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
import haxe.Int64;

private enum ProbePhase { Idle; Approaching; Coarse; Backoff; Fine; HandoffBackoff; HandoffRetreat; Retreat; Done; Failed; }

/** A prepared nominal search pose and bounded arc-off sensing parameters, in metres and seconds. */
class ContactProbeRequest {
  public final approach:Transform3;
  public final direction:Vec3;
  public final distance:Float;
  public final coarseSpeed:Float;
  public final fineSpeed:Float;
  public final backoff:Float;
  public final contactOffset:Float;
  public final approachJoints:Null<Array<Float>>;

  public function new(approach:Transform3, direction:Vec3, distance:Float, coarseSpeed:Float = 0.01,
      fineSpeed:Float = 0.0005, backoff:Float = 0.002, contactOffset:Float = 0.0, ?approachJoints:Array<Float>) {
    if (approach == null || direction == null || !Math.isFinite(direction.norm()) || direction.norm() < 1e-12 ||
        !Math.isFinite(distance) || !(distance > 0) || !Math.isFinite(coarseSpeed) || !(coarseSpeed > 0) ||
        !Math.isFinite(fineSpeed) || !(fineSpeed > 0) || fineSpeed > coarseSpeed || !Math.isFinite(backoff) ||
        !(backoff > contactOffset) || !Math.isFinite(contactOffset) || contactOffset < 0 || contactOffset > 0.002)
      throw "Contact probe needs a prepared pose, finite bounded search and calibrated refinement";
    this.approach = approach; this.direction = direction.normalized(); this.distance = distance;
    this.coarseSpeed = coarseSpeed; this.fineSpeed = fineSpeed; this.backoff = backoff; this.contactOffset = contactOffset;
    this.approachJoints = approachJoints == null ? null : approachJoints.copy();
  }
}

/** One exclusive owner alternates checked motion programs with contact servo searches. */
class ContactProbeRunner {
  public final motion:ManipulatorMotion;
  public final planner:ProbeMotionPlanner;
  public final channels:WelderChannels;
  public final sensor:String;
  public var contact(default, null):Null<Vec3> = null;
  public var failure(default, null):Null<String> = null;
  final makeServo:Void -> ServoSession;
  var phase:ProbePhase = Idle;
  var request:Null<ContactProbeRequest> = null;
  var search:Null<ContactSearchRunner> = null;
  var pendingContact:Null<Vec3> = null;
  var handoffAt:Int64 = Int64.ofInt(0);
  final mappings:Null<robotkit.time.ClockMappings>;
  var sourceClock:String = "";

  public function new(motion:ManipulatorMotion, planner:ProbeMotionPlanner, channels:WelderChannels,
      sensor:String, makeServo:Void -> ServoSession, ?mappings:robotkit.time.ClockMappings) {
    if (motion == null || planner == null || motion.compiler != planner.compiler || channels == null ||
        sensor == null || sensor.length == 0 || makeServo == null)
      throw "Contact probe needs one motion owner, its checked planner, torch channels and a fresh servo factory";
    for (name in [channels.arc, channels.wireSpeed, channels.voltage])
      if (name == null || StringTools.trim(name).length == 0) throw "Contact probe needs three valid torch channels";
    if (channels.arc == channels.wireSpeed || channels.arc == channels.voltage || channels.wireSpeed == channels.voltage)
      throw "Contact probe torch channels must be distinct";
    this.motion = motion; this.planner = planner; this.channels = channels; this.sensor = sensor; this.makeServo = makeServo;
    this.mappings = mappings;
  }

  public function running():Bool return phase != Idle && phase != Done && phase != Failed;
  public function completed():Bool return phase == Done;

  function positions():Array<Float> {
    var snapshot = motion.robot.snapshot();
    return [for (index in motion.jointIndices) snapshot.positions.get(index)];
  }

  public function start(request:ContactProbeRequest):Void {
    if (request == null || running() || motion.running) throw "Contact probe cannot start without an idle exclusive motion owner";
    this.request = request; contact = null; failure = null;
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
      // These records precede every movement; stop policy also keeps them safe through servo ownership.
      var ops:Array<MotionOp> = [MotionOp.SetOutput(channels.wireSpeed, EventValue.Analog(0.0)),
        MotionOp.SetOutput(channels.arc, EventValue.Digital(false))];
      motion.run(new MotionProgram(ops.concat(move.program.ops)));
      phase = Approaching;
    } catch (error:Dynamic) fail(Std.string(error));
  }

  function beginSearch(speed:Float, distance:Float):Void {
    var servo = makeServo();
    if (servo == null || servo.robot != motion.robot || servo.manipulator != planner.arm) {
      if (servo != null) servo.dispose();
      throw "Contact probe servo factory returned a different robot or arm";
    }
    try {
      search = new ContactSearchRunner(servo, sensor, cast(request, ContactProbeRequest).direction, distance, speed,
        0.02, 0.002, cast(request, ContactProbeRequest).contactOffset, mappings);
    } catch (error:Dynamic) { servo.dispose(); throw error; }
  }

  function releaseSearch():Void {
    var active = search;
    if (active != null) active.servo.dispose();
    search = null;
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
              case Approaching:
                beginSearch(requested.coarseSpeed, requested.distance); phase = Coarse;
              case Backoff:
                beginSearch(requested.fineSpeed, requested.backoff + requested.contactOffset); phase = Fine;
              case Retreat: phase = Done;
              case _:
            }
          }
        case Coarse | Fine:
          var active:ContactSearchRunner = cast search;
          var snapshot = motion.robot.snapshot();
          var indices = motion.jointIndices;
          if (!planner.stoppingClear([for (index in indices) snapshot.positions.get(index)],
            [for (index in indices) snapshot.velocities.get(index)], 0.02))
            throw "Contact probe predicted braking motion is obstructed";
          active.update();
          if (active.stopped) {
            if (!active.completed()) throw active.search.failure;
            var measured:Vec3 = cast active.search.contact;
            releaseSearch();
            pendingContact = measured;
            var snapshot = motion.robot.snapshot();
            // A stopped velocity target still owns the native joint. Change to a position hold and await its owner tick.
            motion.robot.submit(RobotCommand.JointTargets([for (index in motion.jointIndices)
              JointTarget.position(index, snapshot.positions.get(index))], null));
            handoffAt = snapshot.sourceTimestampNs;
            phase = phase == Coarse ? HandoffBackoff : HandoffRetreat;
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
            var move = planner.line(new Transform3(requested.approach.translation, current.rotation), q, requested.coarseSpeed, true);
            motion.run(move.program); phase = Retreat;
          }
        case Idle | Done | Failed:
      }
    } catch (error:Dynamic) fail(Std.string(error));
  }

  public function cancel():Void if (running()) fail("contact probe cancelled");
  function fail(reason:String):Void {
    var active = search;
    if (active != null) active.cancel();
    try motion.robot.stop(StopMode.Normal) catch (_:Dynamic) {}
    try motion.abort() catch (_:Dynamic) {}
    releaseSearch(); contact = null; failure = '$phase: $reason'; phase = Failed;
  }
}
