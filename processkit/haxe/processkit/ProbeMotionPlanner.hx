package processkit;

import haxe.Int64;
import motionkit.MotionOptions;
import motionkit.kinematics.Pose3;
import motionkit.kinematics.Twist6;
import motionkit.program.Blend;
import motionkit.program.MotionOp;
import motionkit.program.MotionProgram;
import motionkit.program.MoveTarget;
import motionkit.robot.ProgramCompiler;
import motionkit.robot.CompiledProgram;
import robotkit.manipulation.Manipulator;
import robotkit.manipulation.ArmClearance;
import robotkit.manipulation.ArmClearance.ClearanceViolation;
import robotkit.manipulation.JointRoute;
import robotkit.spatial.Transform3;
import robotkit.spatial.Vec3;
import robotkit.model.JointType;

/** A complete checked air move; its end state comes from the compiled trajectory. */
class CheckedProbeMove {
  public final program:MotionProgram;
  public final endJoints:Array<Float>;
  public function new(program:MotionProgram, endJoints:Array<Float>) {
    this.program = program; this.endJoints = endJoints.copy();
  }
}

/** Contact probing's approach/refinement/retreat checks, without weld deposition semantics. */
class ProbeMotionPlanner {
  public final arm:Manipulator;
  public final compiler:ProgramCompiler;
  public final clearance:Null<ArmClearance>;
  public final wireClearance:Null<ProbeWireClearance>;
  public final airPoseReserve:Float;
  final interval:Float;
  final jointStep:Float;

  public function new(arm:Manipulator, compiler:ProgramCompiler, ?clearance:ArmClearance, ?wireClearance:ProbeWireClearance) {
    if (arm == null || compiler == null || compiler.solver.jointCount() != arm.group.count() ||
        wireClearance != null && wireClearance.arm != arm)
      throw "Probe motion planning needs matching arm kinematics and compiler";
    this.arm = arm; this.compiler = compiler; this.clearance = clearance;
    this.wireClearance = wireClearance;
    var zero = [for (_ in 0...arm.group.count()) 0.0];
    airPoseReserve = compiler.ikTolerance.position + (wireClearance == null ? 0.0
      : wireClearance.extentFrom(zero, arm.tcpPose(zero).translation) * compiler.ikTolerance.orientation);
    var period = Math.POSITIVE_INFINITY, smallest = Math.POSITIVE_INFINITY;
    for (joint in 0...arm.group.count()) {
      var step = arm.robot.joints[arm.jointIndices()[joint]].type == JointType.Prismatic ? 0.001 : 0.02;
      period = Math.min(period, step / compiler.maxVelocity[joint]); smallest = Math.min(smallest, step);
    }
    interval = period; jointStep = smallest;
  }

  public function violation(q:Array<Float>, contact:Bool = false):Null<ClearanceViolation> {
    var hit = clearance == null ? null : clearance.violation(q, contact);
    return hit != null || wireClearance == null ? hit : wireClearance.violation(q, contact);
  }

  function sweep(from:Array<Float>, to:Array<Float>, contact:Bool):Null<ClearanceViolation> {
    var hit = clearance == null ? null : clearance.sweep(from, to, contact, jointStep);
    return hit != null || wireClearance == null ? hit : wireClearance.sweep(from, to, contact, jointStep);
  }

  static function pose(frame:Transform3):Pose3 {
    var t = frame.translation, r = frame.rotation;
    return new Pose3(t.x, t.y, t.z, r.x, r.y, r.z, r.w);
  }

  /** Check actual time-law samples and their short swept edges, not just the requested endpoints. */
  function inspect(program:MotionProgram, start:Array<Float>, contact:Bool):Array<Float> {
    var compiled:CompiledProgram = compiler.compile(program, start, Int64.ofInt(1));
    try {
      var last = start.copy();
      if (violation(last, contact) != null) throw "Probe motion starts in collision";
      for (block in compiled.blocks) for (trajectory in block.plans) {
        var samples = Std.int(Math.max(1.0, Math.ceil(trajectory.durationSeconds / interval)));
        for (sample in 0...samples + 1) {
          var q = trajectory.evaluate(trajectory.durationSeconds * sample / samples).positions;
          var hit = sweep(last, q, contact);
          if (hit != null) throw 'Probe motion collision: ${hit.a} against ${hit.b}, ${hit.distance} m < ${hit.required} m';
          last = q.copy();
        }
      }
      compiled.dispose(); return last;
    } catch (error:Dynamic) {
      compiled.dispose(); throw error;
    }
  }

  function edge(from:Array<Float>, to:Array<Float>):Bool {
    var moving = false;
    for (joint in 0...from.length) if (Math.abs(from[joint] - to[joint]) > 1e-12) moving = true;
    if (!moving) return violation(from) == null;
    if (sweep(from, to, false) != null) return false;
    try {
      inspect(new MotionProgram([MotionOp.MoveJ(MoveTarget.JointTarget(to), new MotionOptions(), Blend.ExactStop)]), from, false);
      return true;
    } catch (_:Dynamic) return false;
  }

  /** Bounded joint-space detours reach the requested prepared search pose on a proved IK branch. */
  public function approach(target:Transform3, start:Array<Float>, proposals:Int = 1024):CheckedProbeMove {
    if (target == null || start == null || start.length != arm.group.count() || proposals < 1)
      throw "Probe approach needs a target, joint start and positive search budget";
    return approachCandidates(goalsFor(target, start), start, proposals);
  }

  /** Prove continuation from observed joints before requesting any globally discovered branch. */
  public function observedApproach(target:Transform3, start:Array<Float>):CheckedProbeMove {
    if (target == null || start == null || start.length != arm.group.count())
      throw "Observed probe approach needs a target and matching joint start";
    var goal = compiler.solver.solvePose(pose(target), start, compiler.ikTolerance);
    if (goal == null) throw "Probe approach has no checked observed IK configuration";
    return observedApproachJoints(goal, start);
  }

  /** Prove an observed-branch solution without repeating IK or discovering a detour. */
  public function observedApproachJoints(goal:Array<Float>, start:Array<Float>):CheckedProbeMove {
    if (goal == null || start == null || goal.length != arm.group.count() || start.length != goal.length)
      throw "Observed probe joint approach needs matching configurations";
    if (!edge(start, goal)) throw "Probe approach has no checked observed IK configuration";
    var program = new MotionProgram([MotionOp.MoveJ(MoveTarget.JointTarget(goal), new MotionOptions(), Blend.ExactStop)]);
    return new CheckedProbeMove(program, inspect(program, start, false));
  }

  function goalsFor(target:Transform3, start:Array<Float>):Array<Array<Float>> {
    var requested = pose(target);
    var goals = compiler.solver.sampleCandidates(requested, 12, compiler.ikTolerance);
    var continued = compiler.solver.solvePose(requested, start, compiler.ikTolerance);
    if (continued != null) goals.push(continued);
    function cost(q:Array<Float>):Float {
      var result = 0.0;
      for (i in 0...q.length) result += Math.pow((q[i] - start[i]) / compiler.maxVelocity[i], 2);
      return result;
    }
    goals.sort((a, b) -> Reflect.compare(cost(a), cost(b)));
    return goals;
  }

  /** Preserve the configuration whose bounded sensing corridor was checked during preparation. */
  public function approachJoints(goal:Array<Float>, start:Array<Float>, proposals:Int = 1024):CheckedProbeMove {
    if (goal == null || start == null || goal.length != arm.group.count() || start.length != goal.length || proposals < 1)
      throw "Probe joint approach needs matching configurations and a positive search budget";
    return approachCandidates([goal.copy()], start, proposals);
  }

  function approachCandidates(goals:Array<Array<Float>>, start:Array<Float>, proposals:Int):CheckedProbeMove {
    var lower = [for (i in 0...arm.group.count()) arm.group.limitsOf(i).lower];
    var upper = [for (i in 0...arm.group.count()) arm.group.limitsOf(i).upper];
    var initialHit = violation(start);
    if (initialHit != null) throw 'Probe approach start is blocked: ${initialHit.a} against ${initialHit.b}, clearance ${initialHit.distance} m (required ${initialHit.required} m)';
    var reasons:Array<String> = [];
    for (goal in goals) {
      try {
        var goalHit = violation(goal);
        if (goalHit != null) throw 'Probe approach goal is blocked: ${goalHit.a} against ${goalHit.b}, clearance ${goalHit.distance} m (required ${goalHit.required} m)';
        var route = JointRoute.plan(start, goal, lower, upper, edge, proposals);
        var ops:Array<MotionOp> = [for (i in 1...route.length)
          MotionOp.MoveJ(MoveTarget.JointTarget(route[i]), new MotionOptions(), Blend.ExactStop)];
        var program = new MotionProgram(ops);
        var end = inspect(program, start, false);
        return new CheckedProbeMove(program, end);
      } catch (error:Dynamic) reasons.push(Std.string(error));
    }
    throw goals.length == 0 ? "Probe approach has no reachable IK configuration"
      : 'Probe approach exhausted its checked candidates: ${reasons.join("; ")}';
  }

  /** Continue from the actual checked joint goal through the whole bounded search ray. */
  public function corridorReachable(start:Array<Float>, direction:Vec3, distance:Float):Bool {
    if (start == null || start.length != arm.group.count() || direction == null || Math.abs(direction.norm() - 1) > 1e-8 ||
        !Math.isFinite(distance) || !(distance > 0)) throw "Probe corridor needs observed joints, a unit direction and finite distance";
    var from = arm.tcpPose(start);
    var q = start.copy();
    var steps = Std.int(Math.max(1.0, Math.ceil(distance / 0.001)));
    for (step in 1...steps + 1) {
      var at = from.translation.add(direction.scale(distance * step / steps));
      var next = compiler.solver.solvePose(pose(new Transform3(at, from.rotation)), q, compiler.ikTolerance);
      if (next == null) return false;
      q = next;
    }
    return true;
  }

  /** Bound wire travel during sensor age, a missed command deadline and joint braking using CAD lever lengths. */
  public function sensingSpeed(start:Array<Float>, direction:Vec3, distance:Float, requested:Float,
      contactOffset:Float, reactionSeconds:Float = 0.04):Float {
    if (start == null || start.length != arm.group.count() || direction == null || Math.abs(direction.norm() - 1) > 1e-8 ||
        !Math.isFinite(distance) || !(distance > 0) || !Math.isFinite(requested) || !(requested > 0) ||
        !Math.isFinite(contactOffset) || contactOffset < 0 || !Math.isFinite(reactionSeconds) || reactionSeconds < 0)
      throw "Probe speed needs a bounded ray, requested speed, calibration and deadline";
    if (contactOffset == 0) {
      if (wireClearance != null) throw "Wire sensing needs a positive calibrated stopping stand-off";
      return requested;
    }
    var reserve = compiler.ikTolerance.position + (wireClearance == null ? 0.0 : wireClearance.envelopeExcess);
    var budget = contactOffset - reserve;
    if (!(budget > 0)) throw "Probe calibration leaves no wire stopping reserve";
    var indices = arm.jointIndices();
    var zero = [for (_ in indices) 0.0];
    var children = [for (index in indices) arm.robot.joints[index].child.id];
    var poses = arm.linkPoses(zero, children);
    var origins = [for (i in 0...indices.length) poses[i].transformPoint(Vec3.fromArray(arm.robot.joints[indices[i]].childFramePosition))];
    var last = origins.length - 1;
    var distal = wireClearance == null ? arm.tcpPose(zero).translation.sub(origins[last]).norm()
      : wireClearance.extentFrom(zero, origins[last]);
    var levers = [for (_ in indices) 0.0];
    for (i in 0...indices.length) {
      var joint = indices.length - 1 - i;
      if (joint < last) distal += origins[joint + 1].sub(origins[joint]).norm();
      levers[joint] = arm.robot.joints[indices[joint]].type == JointType.Prismatic ? 1.0 : distal;
      if (arm.robot.joints[indices[joint]].type == JointType.Prismatic && joint > 0) {
        var travel = arm.group.limitsOf(joint);
        if (!Math.isFinite(travel.lower) || !Math.isFinite(travel.upper) || !(travel.lower < travel.upper))
          throw "Probe stopping reserve needs bounded distal prismatic travel";
        distal += Math.max(Math.abs(travel.lower), Math.abs(travel.upper));
      }
    }
    var from = arm.tcpPose(start), q = start.copy(), speed = requested;
    var steps = Std.int(Math.max(1.0, Math.ceil(distance / 0.001)));
    for (step in 0...steps + 1) {
      if (step > 0) {
        var at = from.translation.add(direction.scale(distance * step / steps));
        var next = compiler.solver.solvePose(pose(new Transform3(at, from.rotation)), q, compiler.ikTolerance);
        if (next == null) throw "Probe speed corridor leaves the executed IK branch";
        q = next;
      }
      var rates = compiler.solver.solveDifferential(q, new Twist6(direction.x, direction.y, direction.z, 0, 0, 0));
      if (rates == null) throw "Probe sensing ray has no finite differential motion";
      var hold = 0.0, brake = 0.0;
      for (joint in 0...indices.length) {
        var rate = rates[joint];
        if (!Math.isFinite(rate)) throw "Probe sensing ray has no finite differential motion";
        var acceleration = compiler.maxAcceleration[joint];
        var bound = arm.group.limitsOf(joint).maxAcceleration;
        if (bound != null && Math.isFinite(bound) && bound > 0) acceleration = Math.min(acceleration, bound);
        hold += levers[joint] * Math.abs(rate) * reactionSeconds;
        brake += levers[joint] * rate * rate / (2 * acceleration);
      }
      // Rationalized positive quadratic root avoids cancellation at low braking distances.
      var allowed = brake > 0 ? 2 * budget / (hold + Math.sqrt(hold * hold + 4 * brake * budget))
        : hold > 0 ? budget / hold : requested;
      speed = Math.min(speed, allowed);
    }
    if (!Math.isFinite(speed) || !(speed > 0)) throw "Probe sensing ray has no positive safe speed";
    return speed;
  }

  /** Reject an obstructed predicted per-joint deadline brake from the current measured motion. */
  public function stoppingClear(start:Array<Float>, velocity:Array<Float>, keepaliveSeconds:Float):Bool
    return stoppingViolation(start, velocity, keepaliveSeconds) == null;

  /** Preserve the named body or joint-bound finding when the deadline-brake proof fails. */
  public function stoppingViolation(start:Array<Float>, velocity:Array<Float>, keepaliveSeconds:Float):Null<ClearanceViolation> {
    if (start == null || velocity == null || start.length != arm.group.count() || velocity.length != start.length ||
        !Math.isFinite(keepaliveSeconds) || keepaliveSeconds < 0)
      throw "Probe stopping check needs joint observations and a finite nonnegative keepalive";
    var accelerations:Array<Float> = [];
    var horizon = keepaliveSeconds;
    for (joint in 0...start.length) {
      if (!Math.isFinite(start[joint]) || !Math.isFinite(velocity[joint])) throw "Probe stopping observations must be finite";
      var acceleration = compiler.maxAcceleration[joint];
      var bound = arm.group.limitsOf(joint).maxAcceleration;
      if (bound != null && Math.isFinite(bound) && bound > 0) acceleration = Math.min(acceleration, bound);
      accelerations.push(acceleration);
      horizon = Math.max(horizon, keepaliveSeconds + Math.abs(velocity[joint]) / acceleration);
    }
    var samples = Std.int(Math.max(1.0, Math.ceil(horizon / interval)));
    var last = start.copy();
    for (sample in 0...samples + 1) {
      var time = horizon * sample / samples;
      var hold = Math.min(time, keepaliveSeconds);
      var q:Array<Float> = [];
      for (joint in 0...start.length) {
        var v = velocity[joint], acceleration = accelerations[joint];
        var braking = Math.min(Math.max(0.0, time - keepaliveSeconds), Math.abs(v) / acceleration);
        var direction = v < 0 ? -1.0 : 1.0;
        var end = start[joint] + v * (hold + braking) - direction * 0.5 * acceleration * braking * braking;
        var limits = arm.group.limitsOf(joint);
        if (limits.lower < limits.upper && (end < limits.lower || end > limits.upper))
          return {a: arm.robot.joints[arm.jointIndices()[joint]].id, b: "joint travel bound",
            distance: Math.min(end - limits.lower, limits.upper - end), required: 0.0};
        q.push(end);
      }
      var hit = sweep(last, q, true);
      if (hit != null) return hit;
      last = q;
    }
    return null;
  }

  /** Withdraw without rotating near contact, then restore the original proved air configuration. */
  public function retreat(target:Transform3, start:Array<Float>, speed:Float, goal:Array<Float>):CheckedProbeMove {
    if (target == null || goal == null || goal.length != arm.group.count()) throw "Probe retreat needs its checked air goal";
    var withdrawn:CheckedProbeMove;
    try {
      withdrawn = line(new Transform3(target.translation, arm.tcpPose(start).rotation), start, speed, true);
    } catch (axialError:Dynamic) {
      // Servo contact can leave a measured pose from which Cartesian IK is marginal even though
      // the proved air joint goal remains reachable. A direct joint return is accepted only after
      // the complete trajectory is compiled and swept with the calibrated contact allowance.
      var program = new MotionProgram([MotionOp.MoveJ(MoveTarget.JointTarget(goal.copy()),
        new MotionOptions(), Blend.ExactStop)]);
      try {
        var checked = new CheckedProbeMove(program, inspect(program, start, true));
        var hit = violation(checked.endJoints);
        if (hit != null) throw 'Probe joint retreat ends blocked: ${hit.a} against ${hit.b}, ${hit.distance} m < ${hit.required} m';
        return checked;
      } catch (jointError:Dynamic) {
        throw 'Probe retreat cannot compile its axial withdrawal (${Std.string(axialError)}) or checked joint return (${Std.string(jointError)})';
      }
    }
    var correction = false;
    for (joint in 0...goal.length) if (Math.abs(withdrawn.endJoints[joint] - goal[joint]) > 1e-8) correction = true;
    var checked = withdrawn;
    if (correction) {
      var program = new MotionProgram(withdrawn.program.ops.concat([
        MotionOp.MoveJ(MoveTarget.JointTarget(goal.copy()), new MotionOptions(), Blend.ExactStop)]));
      checked = new CheckedProbeMove(program, inspect(program, start, true));
    }
    var hit = violation(checked.endJoints);
    if (hit != null) throw 'Probe retreat cannot restore air clearance: ${hit.a} against ${hit.b}, clearance ${hit.distance} m (required ${hit.required} m)';
    return checked;
  }

  /** Straight air motion leaves/refines an already measured contact, with the tool contact margin when requested. */
  public function line(target:Transform3, start:Array<Float>, speed:Float, contact:Bool = false):CheckedProbeMove {
    if (target == null || start == null || start.length != arm.group.count() || !Math.isFinite(speed) || !(speed > 0))
      throw "Probe line needs a target, joint start and positive finite speed";
    var program = new MotionProgram([MotionOp.MoveL(pose(target), compiler.frameId, speed, Blend.ExactStop)]);
    return new CheckedProbeMove(program, inspect(program, start, contact));
  }
}
