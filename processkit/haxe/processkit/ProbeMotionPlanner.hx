package processkit;

import haxe.Int64;
import motionkit.MotionOptions;
import motionkit.kinematics.Pose3;
import motionkit.program.Blend;
import motionkit.program.MotionOp;
import motionkit.program.MotionProgram;
import motionkit.program.MoveTarget;
import motionkit.robot.ProgramCompiler;
import motionkit.robot.CompiledProgram;
import robotkit.manipulation.Manipulator;
import robotkit.manipulation.ArmClearance;
import robotkit.manipulation.JointRoute;
import robotkit.spatial.Transform3;
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
  final interval:Float;
  final jointStep:Float;

  public function new(arm:Manipulator, compiler:ProgramCompiler, ?clearance:ArmClearance) {
    if (arm == null || compiler == null || compiler.solver.jointCount() != arm.group.count())
      throw "Probe motion planning needs matching arm kinematics and compiler";
    this.arm = arm; this.compiler = compiler; this.clearance = clearance;
    var period = Math.POSITIVE_INFINITY, smallest = Math.POSITIVE_INFINITY;
    for (joint in 0...arm.group.count()) {
      var step = arm.robot.joints[arm.jointIndices()[joint]].type == JointType.Prismatic ? 0.001 : 0.02;
      period = Math.min(period, step / compiler.maxVelocity[joint]); smallest = Math.min(smallest, step);
    }
    interval = period; jointStep = smallest;
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
      if (clearance != null && clearance.violation(last, contact) != null) throw "Probe motion starts in collision";
      for (block in compiled.blocks) for (trajectory in block.plans) {
        var samples = Std.int(Math.max(1.0, Math.ceil(trajectory.durationSeconds / interval)));
        for (sample in 0...samples + 1) {
          var q = trajectory.evaluate(trajectory.durationSeconds * sample / samples).positions;
          if (clearance != null) {
            var hit = clearance.sweep(last, q, contact, jointStep);
            if (hit != null) throw 'Probe motion collision: ${hit.a} against ${hit.b}, ${hit.distance} m < ${hit.required} m';
          }
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
    if (!moving) return clearance == null || clearance.violation(from) == null;
    if (clearance != null && clearance.sweep(from, to, false, jointStep) != null) return false;
    try {
      inspect(new MotionProgram([MotionOp.MoveJ(MoveTarget.JointTarget(to), new MotionOptions(), Blend.ExactStop)]), from, false);
      return true;
    } catch (_:Dynamic) return false;
  }

  /** Bounded joint-space detours reach the requested prepared search pose on a proved IK branch. */
  public function approach(target:Transform3, start:Array<Float>, proposals:Int = 1024):CheckedProbeMove {
    if (target == null || start == null || start.length != arm.group.count() || proposals < 1)
      throw "Probe approach needs a target, joint start and positive search budget";
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
    var lower = [for (i in 0...arm.group.count()) arm.group.limitsOf(i).lower];
    var upper = [for (i in 0...arm.group.count()) arm.group.limitsOf(i).upper];
    var reasons:Array<String> = [];
    for (goal in goals) {
      try {
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

  /** Reject an obstructed predicted per-joint deadline brake from the current measured motion. */
  public function stoppingClear(start:Array<Float>, velocity:Array<Float>, keepaliveSeconds:Float):Bool {
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
        if (limits.lower < limits.upper && (end < limits.lower || end > limits.upper)) return false;
        q.push(end);
      }
      if (clearance != null && clearance.sweep(last, q, true, jointStep) != null) return false;
      last = q;
    }
    return true;
  }

  /** Straight air motion leaves/refines an already measured contact, with the tool contact margin when requested. */
  public function line(target:Transform3, start:Array<Float>, speed:Float, contact:Bool = false):CheckedProbeMove {
    if (target == null || start == null || start.length != arm.group.count() || !Math.isFinite(speed) || !(speed > 0))
      throw "Probe line needs a target, joint start and positive finite speed";
    var program = new MotionProgram([MotionOp.MoveL(pose(target), compiler.frameId, speed, Blend.ExactStop)]);
    return new CheckedProbeMove(program, inspect(program, start, contact));
  }
}
