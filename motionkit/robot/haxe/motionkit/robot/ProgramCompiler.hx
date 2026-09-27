package motionkit.robot;

import haxe.Int64;
import motionkit.MotionOptions;
import motionkit.event.PathEvent;
import motionkit.event.TimedEvent;
import motionkit.kinematics.IkTolerance;
import motionkit.kinematics.KinematicsSolver;
import motionkit.kinematics.Pose3;
import motionkit.path.OrientationPolicy;
import motionkit.path.PoseArc;
import motionkit.path.PoseLine;
import motionkit.path.PoseMath;
import motionkit.path.PosePath;
import motionkit.path.PoseWaypoint;
import motionkit.planner.JointPathSamples;
import motionkit.planner.PathTimingBackend;
import motionkit.planner.PathTimingLimits;
import motionkit.planner.SimplePathTiming;
import motionkit.program.Blend;
import motionkit.program.MotionOp;
import motionkit.program.MotionProgram;
import motionkit.program.MoveTarget;
import motionkit.trajectory.ExecutionPlan;
import motionkit.trajectory.Trajectory;
import motionkit.trajectory.ValidationLimits;

/** Lowers an authored program to validated, exact-stop plans. */
class ProgramCompiler {
  public final solver:KinematicsSolver;
  public final limits:ValidationLimits;
  public final timing:PathTimingBackend;
  public final frameId:String;
  public final maxVelocity:Array<Float>;
  public final maxAcceleration:Array<Float>;
  public final maxJerk:Array<Float>;
  public final cartesianResolution:Float;
  public final maxJointJump:Float;
  public final positionTolerance:Float;
  public final orientationTolerance:Float;
  public final ikTolerance:IkTolerance;

  public function new(solver:KinematicsSolver, limits:ValidationLimits,
      frameId:String, maxVelocity:Array<Float>, maxAcceleration:Array<Float>,
      maxJerk:Array<Float>, ?timing:PathTimingBackend,
      ?cartesianResolution:Float = 0.01, ?maxJointJump:Float = 0.5,
      ?positionTolerance:Float = 0.005, ?orientationTolerance:Float = 0.02,
      ?ikTolerance:IkTolerance) {
    if (solver == null || limits == null || solver.jointCount() != limits.jointCount)
      throw "Program compiler needs matching kinematics and validation limits";
    if (frameId == null || StringTools.trim(frameId).length == 0)
      throw "Program compiler needs a frame ID";
    var count = limits.jointCount;
    for (vector in [maxVelocity, maxAcceleration, maxJerk]) {
      if (vector == null || vector.length != count)
        throw "Program compiler joint-limit counts must match";
      for (value in vector)
        if (!Math.isFinite(value) || value <= 0.0)
          throw "Program compiler joint limits must be finite and positive";
    }
    for (value in [cartesianResolution, maxJointJump, positionTolerance,
        orientationTolerance])
      if (!Math.isFinite(value) || value <= 0.0)
        throw "Program compiler resolution and tolerances must be finite and positive";
    this.solver = solver;
    this.limits = limits;
    this.frameId = frameId;
    this.maxVelocity = maxVelocity.copy();
    this.maxAcceleration = maxAcceleration.copy();
    this.maxJerk = maxJerk.copy();
    this.timing = timing == null ? new SimplePathTiming() : timing;
    this.cartesianResolution = cartesianResolution;
    this.maxJointJump = maxJointJump;
    this.positionTolerance = positionTolerance;
    this.orientationTolerance = orientationTolerance;
    this.ikTolerance = ikTolerance == null ? new IkTolerance() : ikTolerance;
  }

  public function compile(program:MotionProgram, initialQ:Array<Float>,
      firstPlanId:Int64):CompiledProgram {
    if (program == null || initialQ == null || initialQ.length != solver.jointCount())
      throw "Program compiler needs a program and complete start position";
    for (value in initialQ) if (!Math.isFinite(value)) throw "Non-finite program start position";
    var q = initialQ.copy();
    var nextId = firstPlanId;
    var blocks:Array<ProgramBlock> = [];
    var plans:Array<ExecutionPlan> = [];
    var indices:Array<Int> = [];
    var notes:Array<String> = [];
    var pending:Null<PendingMotion> = null;
    var currentIndex = -1;
    try {
      for (index in 0...program.ops.length) {
        currentIndex = index;
        var op = program.ops[index];
        switch op {
          case MoveJ(target, options, blend):
            if (pending != null) {
              plans.push(finish(pending, nextId)); indices.push(pending.opIndex);
              nextId = Int64.add(nextId, Int64.ofInt(1));
            }
            noteBlend(blend, index, notes);
            var goal = resolveTarget(target, q, index);
            checkJointPosition(goal, index, null);
            var velocity = effective(maxVelocity, options.maxVelocity);
            var acceleration = effective(maxAcceleration, options.maxAcceleration);
            var jerk = effective(maxJerk, options.maxJerk);
            var generated = Trajectory.generateStateToState(q, zeros(), zeros(), goal,
              velocity, acceleration, jerk);
            pending = new PendingMotion(index, q, goal, generated, [], null, null);
            q = goal;
          case MoveL(pose, requestedFrame, feed, blend):
            if (pending != null) {
              plans.push(finish(pending, nextId)); indices.push(pending.opIndex);
              nextId = Int64.add(nextId, Int64.ofInt(1));
            }
            noteBlend(blend, index, notes);
            requireFrame(requestedFrame, index);
            var start = new PoseWaypoint(solver.forward(q), positionTolerance,
              orientationTolerance);
            var end = new PoseWaypoint(pose, positionTolerance, orientationTolerance);
            pending = lowerPath(new PosePath(frameId, [new PoseLine(start, end,
              OrientationPolicy.Interpolated, 0.1, feed)]), q, feed, [], index);
            q = pending.endQ.copy();
          case MoveC(via, endPose, requestedFrame, feed, blend):
            if (pending != null) {
              plans.push(finish(pending, nextId)); indices.push(pending.opIndex);
              nextId = Int64.add(nextId, Int64.ofInt(1));
            }
            noteBlend(blend, index, notes);
            requireFrame(requestedFrame, index);
            pending = lowerPath(new PosePath(frameId, [new PoseArc(
              new PoseWaypoint(solver.forward(q), positionTolerance, orientationTolerance),
              new PoseWaypoint(via, positionTolerance, orientationTolerance),
              new PoseWaypoint(endPose, positionTolerance, orientationTolerance),
              OrientationPolicy.Interpolated, feed)]), q, feed, [], index);
            q = pending.endQ.copy();
          case FollowPath(path, requestedFrame, feed, events):
            if (pending != null) {
              plans.push(finish(pending, nextId)); indices.push(pending.opIndex);
              nextId = Int64.add(nextId, Int64.ofInt(1));
            }
            requireFrame(requestedFrame, index);
            if (path.frameId != frameId)
              throw 'Motion program op $index path frame does not match $frameId';
            pending = lowerPath(path, q, feed, events, index);
            q = pending.endQ.copy();
          case SetOutput(channel, value):
            if (pending == null)
              throw 'Motion program op $index SetOutput needs preceding motion';
            pending.events.push(new TimedEvent(
              Trajectory.nanoseconds(pending.trajectory.durationSeconds()), channel, value));
          case Dwell(seconds):
            if (pending != null) {
              plans.push(finish(pending, nextId)); indices.push(pending.opIndex);
              nextId = Int64.add(nextId, Int64.ofInt(1)); pending = null;
            }
            blocks.push(new ProgramBlock(plans, indices, ProgramBarrier.Dwell(seconds)));
            plans = []; indices = [];
          case WaitInput(channel, predicate, timeoutSeconds):
            if (pending != null) {
              plans.push(finish(pending, nextId)); indices.push(pending.opIndex);
              nextId = Int64.add(nextId, Int64.ofInt(1)); pending = null;
            }
            blocks.push(new ProgramBlock(plans, indices,
              ProgramBarrier.WaitInput(channel, predicate, timeoutSeconds)));
            plans = []; indices = [];
        }
      }
      if (pending != null) {
        plans.push(finish(pending, nextId)); indices.push(pending.opIndex);
      }
      if (plans.length > 0) blocks.push(new ProgramBlock(plans, indices, null));
      return new CompiledProgram(blocks, notes);
    } catch (error:Dynamic) {
      if (pending != null) pending.trajectory.dispose();
      for (plan in plans) plan.dispose();
      for (block in blocks) for (plan in block.plans) plan.dispose();
      var message = Std.string(error);
      if (StringTools.startsWith(message, "Motion program op ")) throw error;
      throw 'Motion program op $currentIndex: $message';
    }
  }

  function finish(pending:PendingMotion, id:Int64):ExecutionPlan {
    pending.events.sort(function(a, b) return Int64.compare(a.timeNs, b.timeNs));
    var plan:Null<ExecutionPlan> = null;
    try {
      plan = ExecutionPlan.create(pending.trajectory, limits, id, pending.startQ,
        zeros(), zeros(), tolerances(), tolerances(), tolerances(), pending.events);
      if (pending.path != null) checkTaskSpace(plan, pending.path, pending.distances,
        pending.times, pending.opIndex);
      pending.trajectory.dispose();
      return plan;
    } catch (error:Dynamic) {
      if (plan != null) plan.dispose();
      throw 'Motion program op ${pending.opIndex}: $error';
    }
  }

  function resolveTarget(target:MoveTarget, start:Array<Float>, index:Int):Array<Float> {
    return switch target {
      case JointTarget(joints):
        if (joints.length != solver.jointCount())
          throw 'Motion program op $index joint count mismatch';
        joints.copy();
      case PoseTarget(pose, requestedFrame, _):
        requireFrame(requestedFrame, index);
        var candidates = solver.sampleCandidates(pose, 32, ikTolerance);
        if (candidates.length == 0) throw 'Motion program op $index unreachable pose';
        var best:Null<Array<Float>> = null;
        var bestCost = Math.POSITIVE_INFINITY;
        for (candidate in candidates) {
          if (candidate == null || candidate.length != start.length) continue;
          var cost = 0.0;
          for (joint in 0...start.length) {
            var distance = (candidate[joint] - start[joint]) / maxVelocity[joint];
            cost += distance * distance;
          }
          if (cost < bestCost) { bestCost = cost; best = candidate; }
        }
        if (best == null) throw 'Motion program op $index unreachable pose';
        best.copy();
    };
  }

  function lowerPath(path:PosePath, startQ:Array<Float>, feed:Float,
      authoredEvents:Array<PathEvent>, index:Int):PendingMotion {
    if (path.length() <= 0.0) throw 'Motion program op $index has zero path length';
    var count = Std.int(Math.ceil(path.length() / cartesianResolution));
    if (count > 10000) throw 'Motion program op $index exceeds Cartesian sample budget';
    var distances:Array<Float> = [];
    var positions:Array<Array<Float>> = [];
    var caps:Array<Float> = [];
    var previous = startQ.copy();
    for (sample in 0...(count + 1)) {
      var distance = path.length() * sample / count;
      var desired = path.waypointAt(distance).pose;
      var solved = sample == 0 ? startQ.copy() : solver.solvePose(desired, previous, ikTolerance);
      if (solved == null || solved.length != startQ.length)
        throw 'Motion program op $index unreachable pose at path distance $distance';
      checkJointPosition(solved, index, distance);
      if (sample > 0) {
        var jump = 0.0;
        for (joint in 0...startQ.length)
          jump = Math.max(jump, Math.abs(solved[joint] - previous[joint]));
        if (jump > maxJointJump)
          throw 'Motion program op $index IK discontinuity at path distance $distance';
        caps.push(Math.min(feed, primitiveSpeedAt(path,
          (distance + distances[sample-1]) * 0.5)));
      }
      distances.push(distance);
      positions.push(solved.copy());
      previous = solved;
    }
    var first:Array<Array<Float>> = [];
    var second:Array<Array<Float>> = [];
    for (sample in 0...positions.length) {
      var left = sample == 0 ? 0 : sample - 1;
      var right = sample == count ? count : sample + 1;
      var ds = distances[right] - distances[left];
      first.push([for (joint in 0...startQ.length)
        (positions[right][joint] - positions[left][joint]) / ds]);
      second.push([for (_ in 0...startQ.length) 0.0]);
    }
    var timed = timing.time(new JointPathSamples(distances, positions, first, second),
      new PathTimingLimits(maxVelocity, maxAcceleration, caps));
    var events:Array<TimedEvent> = [];
    for (event in authoredEvents) {
      var seconds = Math.max(0.0,
        timed.distanceToTime(event.distance) - event.leadSeconds);
      events.push(new TimedEvent(Trajectory.nanoseconds(seconds), event.channel,
        event.value, event.holdPolicy));
    }
    return new PendingMotion(index, startQ, positions[count], timed.trajectory, events,
      path, distances, [for (distance in distances) timed.distanceToTime(distance)]);
  }

  function checkTaskSpace(plan:ExecutionPlan, path:PosePath,
      distances:Array<Float>, times:Array<Float>, index:Int):Void {
    var worst = 0.0, worstTime = 0.0, tolerance = positionTolerance;
    var failure:Null<String> = null;
    for (sample in 0...(distances.length * 2 - 1)) {
      var left = Std.int(sample / 2);
      var distance = sample % 2 == 0 ? distances[left] :
        (distances[left] + distances[left+1]) * 0.5;
      var time = sample % 2 == 0 ? times[left] : (times[left] + times[left+1]) * 0.5;
      var desired = path.waypointAt(distance);
      var actual = solver.forward(plan.evaluate(time).positions);
      var error = PoseMath.distance(actual, desired.pose);
      var angle = orientationError(actual, desired.pose,
        path.orientationPolicyAt(distance));
      if (error > worst) { worst = error; worstTime = time; tolerance = desired.positionTolerance; }
      if (error > desired.positionTolerance + 1e-9 ||
          angle > desired.orientationTolerance + 1e-9)
        failure = 'Motion program op $index task-space tolerance exceeded at path distance $distance';
    }
    var resolutionSeconds = 0.0;
    for (sample in 1...times.length)
      resolutionSeconds = Math.max(resolutionSeconds, times[sample] - times[sample-1]);
    plan.report.setTaskSpace(failure == null ? MotionKitNativeConstants.MK_CHECK_PASSED :
      MotionKitNativeConstants.MK_CHECK_FAILED, worst, worstTime, tolerance,
      Trajectory.nanoseconds(resolutionSeconds * 0.5));
    if (failure != null) throw failure;
  }

  function requireFrame(requested:String, index:Int):Void {
    if (requested != frameId)
      throw 'Motion program op $index frame "$requested" does not match "$frameId"';
  }
  function checkJointPosition(q:Array<Float>, index:Int,
      distance:Null<Float>):Void {
    var nativeLimits = limits.nativeValue();
    for (joint in 0...q.length)
      if (nativeLimits.get_position_claimed(joint) != 0 &&
          (q[joint] < nativeLimits.get_position_lower(joint) - 1e-9 ||
           q[joint] > nativeLimits.get_position_upper(joint) + 1e-9)) {
        var location = distance == null ? "" : ' at path distance $distance';
        throw 'Motion program op $index joint limit $joint$location';
      }
  }
  static function primitiveSpeedAt(path:PosePath, distance:Float):Float {
    var start = 0.0;
    for (primitive in path.primitives) {
      start += primitive.length();
      if (distance <= start) return primitive.speedLimit();
    }
    return path.primitives[path.primitives.length-1].speedLimit();
  }
  static function orientationError(actual:Pose3, desired:Pose3,
      policy:OrientationPolicy):Float {
    return switch policy {
      case Fixed | Interpolated: PoseMath.angle(actual, desired);
      case FreeAboutTool:
        axisAngle(toolAxis(actual), toolAxis(desired));
      case Cone(axis, halfAngle):
        if (axis == null || axis.length != 3)
          throw "Invalid orientation cone axis";
        var norm = Math.sqrt(axis[0]*axis[0] + axis[1]*axis[1] + axis[2]*axis[2]);
        if (!Math.isFinite(norm) || norm <= 0.0)
          throw "Invalid orientation cone axis";
        Math.max(0.0, axisAngle(toolAxis(actual),
          [axis[0]/norm, axis[1]/norm, axis[2]/norm]) - halfAngle);
    };
  }
  static function toolAxis(pose:Pose3):Array<Float>
    return [2.0*(pose.qx*pose.qz + pose.qw*pose.qy),
      2.0*(pose.qy*pose.qz - pose.qw*pose.qx),
      1.0 - 2.0*(pose.qx*pose.qx + pose.qy*pose.qy)];
  static function axisAngle(a:Array<Float>, b:Array<Float>):Float {
    var dot = a[0]*b[0] + a[1]*b[1] + a[2]*b[2];
    return Math.acos(Math.max(-1.0, Math.min(1.0, dot)));
  }
  function zeros():Array<Float> return [for (_ in 0...solver.jointCount()) 0.0];
  function tolerances():Array<Float> return [for (_ in 0...solver.jointCount()) 0.02];
  static function effective(machine:Array<Float>, requested:Float):Array<Float>
    return [for (limit in machine) requested > 0.0 ? Math.min(limit, requested) : limit];
  static function noteBlend(blend:Blend, index:Int, notes:Array<String>):Void {
    switch blend {
      case ToleranceBlend(_): notes.push('Motion program op $index tolerance blend uses exact stop');
      case ExactStop:
    }
  }
}

private class PendingMotion {
  public final opIndex:Int;
  public final startQ:Array<Float>;
  public final endQ:Array<Float>;
  public final trajectory:Trajectory;
  public final events:Array<TimedEvent>;
  public final path:Null<PosePath>;
  public final distances:Array<Float>;
  public final times:Array<Float>;

  public function new(opIndex:Int, startQ:Array<Float>, endQ:Array<Float>,
      trajectory:Trajectory, events:Array<TimedEvent>, path:Null<PosePath>,
      distances:Null<Array<Float>>, ?times:Array<Float>) {
    this.opIndex = opIndex; this.startQ = startQ.copy(); this.endQ = endQ.copy();
    this.trajectory = trajectory; this.events = events;
    this.path = path;
    this.distances = distances == null ? [] : distances;
    this.times = times == null ? [] : times;
  }
}
