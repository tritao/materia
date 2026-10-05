package motionkit.robot;

import haxe.Int64;
import motionkit.kinematics.Twist6;
import motionkit.path.PosePrimitive;
import motionkit.path.PoseDerivatives;
import robotkit.model.JointCoupling;
import motionkit.MotionOptions;
import motionkit.event.PathEvent;
import motionkit.event.TimedEvent;
import motionkit.event.EventValue;
import motionkit.kinematics.IkTolerance;
import motionkit.kinematics.KinematicsSolver;
import motionkit.kinematics.PathRequest;
import motionkit.kinematics.RedundantPathSolver;
import motionkit.kinematics.Pose3;
import motionkit.path.OrientationPolicy;
import motionkit.path.CornerBlender;
import motionkit.path.ArcSegment;
import motionkit.path.GeometricPath;
import motionkit.path.LineSegment;
import motionkit.path.PathPoint;
import motionkit.path.PathPrimitive;
import motionkit.path.PoseArc;
import motionkit.path.PoseLine;
import motionkit.path.PoseMath;
import motionkit.path.PosePath;
import motionkit.path.PoseWaypoint;
import motionkit.planner.JointPathSamples;
import motionkit.planner.PathTimingBackend;
import motionkit.planner.PathTimingLimits;
import motionkit.planner.ToppraPathTiming;
import motionkit.program.Blend;
import motionkit.program.MotionOp;
import motionkit.program.MotionProgram;
import motionkit.program.MoveTarget;
import motionkit.trajectory.ExecutionPlan;
import motionkit.trajectory.Trajectory;
import motionkit.trajectory.ValidationLimits;

/** A joint-space sample of a path: where, on which primitive, and the primitive arriving there. */
private typedef PathSample = {
  var distance:Float;
  var primitive:PosePrimitive;
  var local:Float;
  var arriving:Null<{primitive:PosePrimitive, local:Float}>;
}

/** Lowers an authored program to validated, exact-stop plans. */
class ProgramCompiler {
  public final solver:KinematicsSolver;
  public final limits:ValidationLimits;
  public final timing:PathTimingBackend;
  public final frameId:String;
  public final maxVelocity:Array<Float>;
  public final maxAcceleration:Array<Float>;
  public final maxJerk:Array<Float>;
  /** Planning defaults exposed to callers alongside the drive check's physical assumptions. */
  public var planningAssumptions:Array<String> = [];
  public final startTolerances:StartTolerances;
  public final cartesianResolution:Float;
  /** The speed scale of the program being compiled. */

  public final maxJointJump:Float;
  public final perJointMaxJump:Array<Float>;
  final couplingIndices:Array<{leader:Int, follower:Int, ratio:Float, offset:Float}>;
  /** The coupled joints, each after the coupled joints among its leaders: a follower is the sum of its terms. */
  final projectionOrder:Array<Int>;
  public final positionTolerance:Float;
  public final orientationTolerance:Float;
  public final ikTolerance:IkTolerance;
  public final configurationSelector:Null<PathConfigurationSelector>;
  final jointIds:Null<Array<String>>;
  final couplings:Null<Array<JointCoupling>>;
  final controllerPeriodSeconds:Float;
  /**
   * The plan check every compiled program goes through, or null for none, for simulation and device.
   * Its findings are on `ExecutionPlan.checked`. Direct MotionSystem moves and live ServoSession
   * plan chunks run their checks at their submission boundaries.
   */
  public var planCheck:Null<PlanCheck> = null;
  /** Linear motor sums for machines whose axis velocities use the single-axis envelope. */
  public var motorSpace:Null<MotorSpaceConstraints> = null;

  /**
   * This compiler for a planning thread, on a fork of its solver: the worker never shares solver
   * state with the caller, who keeps using the same kinematics while plans execute.
   */
  public function forWorker():ProgramCompiler {
    var forked = solver.fork();
    if (forked == solver) return this;
    var worker = new ProgramCompiler(forked, limits, frameId, maxVelocity, maxAcceleration, maxJerk,
      startTolerances, timing, cartesianResolution, maxJointJump, positionTolerance,
      orientationTolerance, ikTolerance,
      configurationSelector == null ? null : configurationSelector.withSolver(forked),
      perJointMaxJump, jointIds, couplings, controllerPeriodSeconds);
    // The worker plans one program in order, so it remembers which way each axis last moved.
    worker.planningAssumptions = planningAssumptions.copy();
    if (planCheck != null) worker.planCheck = planCheck.fork();
    worker.motorSpace = motorSpace;
    return worker;
  }

  public function new(solver:KinematicsSolver, limits:ValidationLimits,
      frameId:String, maxVelocity:Array<Float>, maxAcceleration:Array<Float>,
      maxJerk:Array<Float>, startTolerances:StartTolerances, ?timing:PathTimingBackend,
      ?cartesianResolution:Float = 0.01, ?maxJointJump:Float = 0.5,
      ?positionTolerance:Float = 0.005, ?orientationTolerance:Float = 0.02,
      ?ikTolerance:IkTolerance, ?configurationSelector:PathConfigurationSelector,
      ?perJointMaxJump:Array<Float>, ?jointIds:Array<String>,
      ?couplings:Array<JointCoupling>, ?controllerPeriodSeconds:Float = 0.01) {
    if (solver == null || limits == null || solver.jointCount() != limits.jointCount)
      throw "Program compiler needs matching kinematics and validation limits";
    if (frameId == null || StringTools.trim(frameId).length == 0)
      throw "Program compiler needs a frame ID";
    if (!Math.isFinite(controllerPeriodSeconds) || controllerPeriodSeconds <= 0.0)
      throw "Program compiler controller period must be finite and positive";
    this.controllerPeriodSeconds = controllerPeriodSeconds;
    var count = limits.jointCount;
    if (startTolerances == null) throw "Program compiler start tolerances are required";
    startTolerances.validate(count);
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
    this.startTolerances = new StartTolerances(startTolerances.position,
      startTolerances.velocity, startTolerances.acceleration);
    this.timing = timing == null ? new ToppraPathTiming() : timing;
    this.cartesianResolution = cartesianResolution;
    this.maxJointJump = maxJointJump;
    if (perJointMaxJump != null && perJointMaxJump.length != count)
      throw "Program compiler joint jump limit count must match joints";
    this.perJointMaxJump = perJointMaxJump == null ?
      [for (_ in 0...count) maxJointJump] : perJointMaxJump.copy();
    for (jump in this.perJointMaxJump)
      if (!Math.isFinite(jump) || jump <= 0.0)
        throw "Program compiler joint jump limits must be finite and positive";
    couplingIndices = [];
    projectionOrder = [];
    if (couplings != null) {
      if (jointIds == null || jointIds.length != count)
        throw "Program compiler couplings need joint IDs in solver order";
      var pending:Array<{leader:Int, follower:Int, ratio:Float, offset:Float}> = [];
      for (coupling in couplings) {
        var leader = jointIds.indexOf(coupling.leader);
        var follower = jointIds.indexOf(coupling.follower);
        if (leader < 0 || follower < 0)
          throw 'Program compiler coupling "${coupling.id}" references an unknown joint';
        pending.push({leader: leader, follower: follower,
          ratio: coupling.ratio, offset: coupling.offset});
      }
      while (pending.length > 0) {
        var next = -1;
        for (index in 0...pending.length) {
          var depends = false;
          for (other in pending)
            if (other.follower == pending[index].leader) depends = true;
          if (!depends) { next = index; break; }
        }
        if (next < 0) throw "Program compiler joint couplings contain a cycle";
        couplingIndices.push(pending.splice(next, 1)[0]);
      }
      // A follower is complete once its last term is placed, which is after every leader is.
      var remaining = [for (_ in 0...count) 0];
      for (term in couplingIndices) remaining[term.follower]++;
      for (term in couplingIndices) if (--remaining[term.follower] == 0) projectionOrder.push(term.follower);
    }
    this.jointIds = jointIds == null ? null : jointIds.copy();
    this.couplings = couplings == null ? null : couplings.copy();
    this.positionTolerance = positionTolerance;
    this.orientationTolerance = orientationTolerance;
    this.ikTolerance = ikTolerance == null ? new IkTolerance() : ikTolerance;
    if (configurationSelector != null && configurationSelector.solver != solver)
      throw "Program compiler selector must use its kinematics solver";
    // An explicit selector forces the generic sampled search; otherwise each solver searches its own way.
    this.configurationSelector = configurationSelector;
  }

  /**
    Plans `program` from `initialQ`. With `firstOp`, ops before it are taken
    as done and the rest is planned, keeping their op indices; `speedScale`
    scales every path's speed, within the joint limits as always.
  **/
  public function compile(program:MotionProgram, initialQ:Array<Float>,
      firstPlanId:Int64, ?firstOp:Int = 0, ?speedScale:Float = 1.0):CompiledProgram {
    var blocks = new ProgramBlocks();
    var compilation = begin(program, initialQ, firstPlanId, firstOp, speedScale, blocks);
    try {
      while (compilation.step()) {}
    } catch (error:Dynamic) {
      compilation.dispose();
      blocks.dispose();
      throw error;
    }
    return new CompiledProgram(blocks.finishedBlocks(), blocks.notes);
  }

  /** Starts planning `program` one op at a time into `sink`; see `ProgramCompilation`. */
  public function begin(program:MotionProgram, initialQ:Array<Float>, firstPlanId:Int64,
      firstOp:Int, speedScale:Float, sink:ProgramSink):ProgramCompilation
    return new ProgramCompilation(this, program, initialQ, firstPlanId, firstOp, speedScale, sink);

  /** Finishes the pending motion as a plan and delivers it. */
  function retire(c:ProgramCompilation):Void {
    var pending = c.pending;
    if (pending == null) return;
    c.sink.plan(finish(pending, c.nextId), pending.opIndex,
      pathLength(pending), pending.opDistances(), pending.times.copy());
    c.nextId = Int64.add(c.nextId, Int64.ofInt(1));
    c.pending = null;
  }

  /** Ends the current block at a barrier. */
  function barrier(c:ProgramCompilation, barrier:ProgramBarrier):Void {
    retire(c);
    if (c.leadingOutputs.length > 0) {
      // An output before a wait must run before the wait can observe its input.
      var trajectory = Trajectory.fromSegments([{
        timeFromStartNs: Int64.ofInt(0),
        durationNs: Trajectory.nanoseconds(controllerPeriodSeconds),
        coefficients: [for (position in c.q) [position, 0.0]]
      }]);
      var pending = new PendingMotion(c.currentIndex, c.q, c.q, trajectory, [], null, null);
      attachLeadingOutputs(pending, c.leadingOutputs);
      c.pending = pending;
      retire(c);
    }
    c.sink.barrier(barrier);
  }

  /** Plans the next op of `c`; at the end, finishes its last plan and block. Returns false once done. */
  @:allow(motionkit.robot.ProgramCompilation)
  function advance(c:ProgramCompilation):Bool {
    if (c.done) return false;
    var program = c.program;
    var speedScale = c.speedScale;
    try {
      if (c.sectionIndex < c.sections.length) {
        lowerSection(c);
        return true;
      }
      if (c.cursor >= program.ops.length) {
        retire(c);
        if (c.leadingOutputs.length > 0)
          throw 'Motion program op ${c.currentIndex} has output changes without following motion';
        c.done = true;
        c.sink.finish(c.nextId);
        return false;
      }
      var index = c.cursor++;
      if (c.skipNext) { c.skipNext = false; return true; }
      c.currentIndex = index;
      var q = c.q;
      switch program.ops[index] {
        case MoveJ(target, options, blend):
          retire(c);
          noteBlend(blend, index, c.sink);
          var goal = resolveTarget(target, q, index);
          checkJointPosition(goal, index, null);
          var velocity = effective(maxVelocity, options.maxVelocity);
          var acceleration = effective(maxAcceleration, options.maxAcceleration);
          var jerk = effective(maxJerk, options.maxJerk);
          var motors = motorSpace;
          var generated = motors == null ? Trajectory.generateStateToState(q, zeros(), zeros(), goal,
            velocity, acceleration, jerk) : motors.move(q, goal, velocity, acceleration, jerk);
          var pending = new PendingMotion(index, q, goal, generated, [], null, null);
          c.pending = pending;
          attachLeadingOutputs(pending, c.leadingOutputs);
          c.q = goal;
        case MoveL(pose, requestedFrame, feed, blend, freedom):
          retire(c);
          requireFrame(requestedFrame, index);
          requireFixedOrientation(solver.forward(q), [pose], freedom, index);
          var blended:Null<PosePath> = null;
          var blendTolerance = 0.0;
          var blendNextPose:Null<Pose3> = null;
          switch blend {
            case ToleranceBlend(metres):
              blendTolerance = metres;
              if (index + 1 < program.ops.length) switch program.ops[index + 1] {
                case MoveL(nextPose, nextFrame, nextFeed, ExactStop, nextFreedom):
                  if (nextFrame == frameId && ToolFreedom.isFull(freedom) && ToolFreedom.isFull(nextFreedom)) {
                    blended = blendLinear(solver.forward(q), pose, nextPose,
                      metres, feed, nextFeed);
                    blendNextPose = nextPose;
                  }
                case _:
              }
            case ExactStop:
          }
          var pending:PendingMotion;
          if (blended != null) {
            var nextFeed = switch program.ops[index + 1] {
              case MoveL(_, _, speed, _, _): speed;
              case _: feed;
            };
            pending = lowerPath(speedScale, blended, q, Math.max(feed, nextFeed), [], index,
              [solver.forward(q), pose, cast blendNextPose], blendTolerance);
            c.skipNext = true;
            c.sink.note('Motion program ops $index and ${index + 1} tolerance blended');
          } else {
            noteBlend(blend, index, c.sink);
            var start = new PoseWaypoint(solver.forward(q), positionTolerance,
              orientationTolerance);
            var end = new PoseWaypoint(pose, positionTolerance, orientationTolerance);
            var policy = freedom == null ? OrientationPolicy.Interpolated : freedom;
            // Recovery can return to the same tip within floating-point roundoff.
            // Keep a controller tick so leading outputs and op completion still run.
            if (Math.isFinite(feed) && feed > 0 && PoseMath.distance(start.pose, pose) <= 1e-10 &&
                orientationError(start.pose, pose, policy) <= 1e-7) {
              var hold = Trajectory.fromSegments([{
                timeFromStartNs: Int64.ofInt(0),
                durationNs: Trajectory.nanoseconds(controllerPeriodSeconds),
                coefficients: [for (position in q) [position, 0.0]]
              }]);
              pending = new PendingMotion(index, q, q, hold, [], null, null);
            } else {
              pending = lowerPath(speedScale, new PosePath(frameId, [new PoseLine(start, end,
                policy, 0.1, feed)]), q, feed, [], index);
            }
          }
          c.pending = pending;
          c.q = pending.endQ.copy();
          attachLeadingOutputs(pending, c.leadingOutputs);
        case MoveC(via, endPose, requestedFrame, feed, blend, freedom):
          retire(c);
          noteBlend(blend, index, c.sink);
          requireFrame(requestedFrame, index);
          requireFixedOrientation(solver.forward(q), [via, endPose], freedom, index);
          var pending = lowerPath(speedScale, new PosePath(frameId, [new PoseArc(
            new PoseWaypoint(solver.forward(q), positionTolerance, orientationTolerance),
            new PoseWaypoint(via, positionTolerance, orientationTolerance),
            new PoseWaypoint(endPose, positionTolerance, orientationTolerance),
            freedom == null ? OrientationPolicy.Interpolated : freedom, feed)]), q, feed, [], index);
          c.pending = pending;
          c.q = pending.endQ.copy();
          attachLeadingOutputs(pending, c.leadingOutputs);
        case FollowPath(path, requestedFrame, feed, events):
          requireFrame(requestedFrame, index);
          if (path.frameId != frameId)
            throw 'Motion program op $index path frame does not match $frameId';
          // Following a sharp corner exactly means stopping there, so each
          // stretch between corners is its own plan of this op, one a step.
          c.sections = cornerSections(path);
          c.sectionIndex = 0;
          c.sectionFeed = feed;
          c.sectionEvents = events;
          lowerSection(c);
        case SetOutput(channel, value):
          var pending = c.pending;
          if (pending == null) c.leadingOutputs.push({channel:channel, value:value});
          else pending.events.push(new TimedEvent(
            Trajectory.nanoseconds(pending.trajectory.durationSeconds()), channel, value));
        case Dwell(seconds):
          barrier(c, ProgramBarrier.Dwell(seconds));
        case WaitInput(channel, predicate, timeoutSeconds):
          barrier(c, ProgramBarrier.WaitInput(channel, predicate, timeoutSeconds));
      }
      return true;
    } catch (error:Dynamic) {
      var message = Std.string(error);
      if (StringTools.startsWith(message, "Motion program op ")) throw error;
      throw 'Motion program op ${c.currentIndex}: $message';
    }
  }

  /** Plans the next stretch of the current path op. */
  function lowerSection(c:ProgramCompilation):Void {
    retire(c);
    var k = c.sectionIndex++;
    var section = c.sections[k], last = k == c.sections.length - 1;
    var end = section.offset + section.path.length();
    var pending = lowerPath(c.speedScale, section.path, c.q, c.sectionFeed, [for (event in c.sectionEvents)
      if (event.distance >= section.offset && (last || event.distance < end))
        new PathEvent(event.distance - section.offset, event.channel, event.value,
          event.leadSeconds, event.holdPolicy)], c.currentIndex);
    pending.distanceOffset = section.offset;
    c.pending = pending;
    c.q = pending.endQ.copy();
    if (k == 0) attachLeadingOutputs(pending, c.leadingOutputs);
    if (last) {
      c.sections = [];
      c.sectionIndex = 0;
      c.sectionEvents = [];
    }
  }

  static function pathLength(pending:PendingMotion):Float
    return pending.path == null ? 0.0 : pending.distanceOffset + pending.path.length();

  static function attachLeadingOutputs(pending:PendingMotion,
      leading:Array<{channel:String, value:EventValue}>):Void {
    for (output in leading)
      pending.events.push(new TimedEvent(Int64.ofInt(0), output.channel, output.value));
    while (leading.length > 0) leading.pop();
  }

  function finish(pending:PendingMotion, id:Int64):ExecutionPlan {
    pending.events.sort(function(a, b) return Int64.compare(a.timeNs, b.timeNs));
    var plan:Null<ExecutionPlan> = null;
    var projected:Null<Trajectory> = null;
    try {
      var motors = motorSpace;
      if (motors != null) motors.validate(pending.trajectory);
      if (couplingIndices.length > 0) projected = projectCouplings(pending.trajectory);
      plan = ExecutionPlan.create(projected == null ? pending.trajectory : projected,
        pending.path == null ? limits : limits.withoutJerk(), id, pending.startQ,
        zeros(), zeros(), startTolerances.position, startTolerances.velocity,
        startTolerances.acceleration, pending.events);
      if (pending.path != null) checkTaskSpace(plan, pending.path, pending.checkDistances,
        pending.checkTimes, pending.opIndex, pending.authoredPolyline,
        pending.blendTolerance, pending.taskSampleDistances);
      var check = planCheck;
      if (check != null && check.checks()) {
        var result = check.check(plan, pending.opIndex, pending.feed);
        result.locate(pending.times, pending.opDistances());
        plan.checked = result;
        if (check.options.rejects && result.diagnostics.length > 0)
          throw 'plan check: ${[for (diagnostic in result.diagnostics) diagnostic.describe()].join("; ")}';
      }
      pending.trajectory.dispose();
      if (projected != null) projected.dispose();
      return plan;
    } catch (error:Dynamic) {
      if (plan != null) plan.dispose();
      if (projected != null) projected.dispose();
      throw 'Motion program op ${pending.opIndex}: $error';
    }
  }

  /** Keep the follower polynomial exact after independent Hermite lowering. */
  function projectCouplings(source:Trajectory):Trajectory {
    var segments = source.segments();
    for (segment in segments) for (follower in projectionOrder)
      for (degree in 0...segment.coefficients[follower].length) {
        var sum = 0.0;
        for (coupling in couplingIndices) if (coupling.follower == follower)
          sum += coupling.ratio * segment.coefficients[coupling.leader][degree] +
            (degree == 0 ? coupling.offset : 0.0);
        segment.coefficients[follower][degree] = sum;
      }
    return Trajectory.fromSegments(segments);
  }

  function resolveTarget(target:MoveTarget, start:Array<Float>, index:Int):Array<Float> {
    return switch target {
      case JointTarget(joints):
        if (joints.length != solver.jointCount())
          throw 'Motion program op $index joint count mismatch';
        joints.copy();
      case PoseTarget(pose, requestedFrame, _):
        requireFrame(requestedFrame, index);
        var candidates = solver.sampleCandidates(pose, 32, ikTolerance, null);
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

  /** C3's planar corner geometry, retained as a typed pose path for IK. */
  function blendLinear(start:Pose3, corner:Pose3, end:Pose3,
      tolerance:Float, firstFeed:Float, secondFeed:Float):Null<PosePath> {
    if (PoseMath.distance(start, corner) <= 1e-8 ||
        PoseMath.distance(corner, end) <= 1e-8 ||
        Math.abs(start.z - corner.z) > 1e-8 ||
        Math.abs(corner.z - end.z) > 1e-8 ||
        PoseMath.angle(start, corner) > 1e-8 ||
        PoseMath.angle(corner, end) > 1e-8) return null;
    var geometry = CornerBlender.blend(GeometricPath.lines([
      new PathPoint(start.x, start.y, start.z),
      new PathPoint(corner.x, corner.y, corner.z),
      new PathPoint(end.x, end.y, end.z)]), tolerance, Math.PI * 5.0 / 6.0);
    if (geometry.path.primitives.length != 3 || geometry.diagnostics.length != 0)
      return null;
    var first = geometry.path.primitives[0];
    var arc = geometry.path.primitives[1];
    var last = geometry.path.primitives[2];
    var firstStart = poseWaypoint(first.pointAt(0.0), start);
    var firstEnd = poseWaypoint(first.pointAt(first.length()), start);
    var arcMiddle = poseWaypoint(arc.pointAt(arc.length() * 0.5), start);
    var arcEnd = poseWaypoint(arc.pointAt(arc.length()), start);
    var lastEnd = poseWaypoint(last.pointAt(last.length()), start);
    return new PosePath(frameId, [
      new PoseLine(firstStart, firstEnd, OrientationPolicy.Fixed, 0.1, firstFeed),
      new PoseArc(firstEnd, arcMiddle, arcEnd, OrientationPolicy.Fixed,
        Math.min(firstFeed, secondFeed)),
      new PoseLine(arcEnd, lastEnd, OrientationPolicy.Fixed, 0.1, secondFeed)
    ]);
  }

  function poseWaypoint(point:PathPoint, orientation:Pose3):PoseWaypoint
    return new PoseWaypoint(new Pose3(point.x, point.y, point.z,
      orientation.qx, orientation.qy, orientation.qz, orientation.qw),
      positionTolerance, orientationTolerance);

  function lowerPath(speedScale:Float, path:PosePath, startQ:Array<Float>, feed:Float,
      authoredEvents:Array<PathEvent>, index:Int,
      ?authoredPolyline:Array<Pose3>, ?blendTolerance:Float = 0.0):PendingMotion {
    if (path.length() <= 0.0) throw 'Motion program op $index has zero path length';
    var samples = pathSamples(path);
    var count = samples.length - 1;
    if (count > 10000) throw 'Motion program op $index exceeds Cartesian sample budget';
    var distances = [for (sample in samples) sample.distance];
    var positions:Array<Array<Float>> = [];
    var caps:Array<Float> = [];
    var previous = startQ.copy();
    var pathPoses = [for (sample in samples) sample.primitive.waypointAt(sample.local).pose];
    var freedoms = [for (sample in samples) sample.primitive.orientationPolicy()];
    var fullOrientation = true;
    for (freedom in freedoms) if (!ToolFreedom.isFull(freedom)) fullOrientation = false;
    var selected:Array<Null<Array<Float>>> = [];
    var redundancyRates:Null<Array<Array<Float>>> = null;
    if (configurationSelector != null && fullOrientation)
      for (q in configurationSelector.selectPoses(distances, pathPoses, startQ, ikTolerance)) selected.push(q);
    else {
      var request = new PathRequest(distances, pathPoses, startQ, ikTolerance, perJointMaxJump, maxVelocity, 48, freedoms);
      // A redundant solver also reports how its redundancy changes along the path it chose, exactly.
      if (Std.isOfType(solver, RedundantPathSolver)) {
        var redundant:RedundantPathSolver = cast solver;
        var solved = redundant.solvePathWithRates(request);
        selected = solved.configurations;
        redundancyRates = solved.redundancyRates;
      } else selected = solver.solvePath(request);
    }
    for (sample in 0...(count + 1)) {
      var distance = distances[sample];
      var solved = selected[sample];
      if (solved == null || solved.length != startQ.length)
        throw 'Motion program op $index unreachable pose at path distance $distance${solverFailure()}';
      checkJointPosition(solved, index, distance);
      if (sample > 0) {
        for (joint in 0...startQ.length)
          if (Math.abs(solved[joint] - previous[joint]) > perJointMaxJump[joint])
            throw 'Motion program op $index IK discontinuity at path distance $distance';
        caps.push(speedScale * Math.min(feed, primitiveSpeedAt(path,
          (distance + distances[sample-1]) * 0.5)));
      }
      positions.push(solved.copy());
      previous = solved;
    }
    // Joint derivatives come from each primitive's own geometry through the
    // differential kinematics, so they are exact whatever the sample spacing.
    var first:Array<Array<Float>> = [];
    var second:Array<Array<Float>> = [];
    var secondBefore:Array<Array<Float>> = [];
    for (k in 0...samples.length) {
      var sample = samples[k], q = positions[k];
      var leaving = sample.primitive.derivativesAt(sample.local);
      var redundancy = redundancyRates == null ? null : redundancyRates[k];
      var rate = jointRate(q, leaving.linear, leaving.angular, redundancy, freedoms[k]);
      if (rate == null) throw 'Motion program op $index has no joint velocity along the path at distance ${sample.distance}${solverFailure()}';
      first.push(rate);
      second.push(jointCurvature(q, rate, leaving, redundancy, freedoms[k]));
      var arriving = sample.arriving;
      if (arriving == null) secondBefore.push(second[k]);
      else {
        var before = arriving.primitive.derivativesAt(arriving.local);
        var pose = solver.forward(q);
        var beforeAngular = ToolFreedom.requiredAngular(pose, before.angular, arriving.primitive.orientationPolicy());
        var leavingAngular = ToolFreedom.requiredAngular(pose, leaving.angular, sample.primitive.orientationPolicy());
        var turn = 0.0;
        for (axis in 0...3) turn = Math.max(turn, Math.max(
          Math.abs(before.linear[axis] - leaving.linear[axis]),
          Math.abs(beforeAngular[axis] - leavingAngular[axis])));
        if (turn > 1e-6)
          throw 'Motion program op $index turns a corner at path distance ${sample.distance}: blend it or stop there';
        secondBefore.push(jointCurvature(q, rate, before, redundancy, arriving.primitive.orientationPolicy()));
      }
    }
    var jointPath = new JointPathSamples(distances, positions, first, second, secondBefore);
    var timingLimits = new PathTimingLimits(maxVelocity, maxAcceleration, caps);
    var motors = motorSpace;
    var timed = motors == null ? timing.time(jointPath, timingLimits) : motors.time(timing, jointPath, timingLimits);
    try {
      var events:Array<TimedEvent> = [];
      for (event in authoredEvents) {
        var seconds = Math.max(0.0,
          timed.distanceToTime(event.distance) - event.leadSeconds);
        events.push(new TimedEvent(Trajectory.nanoseconds(seconds), event.channel,
          event.value, event.holdPolicy));
      }
      var timeMap = [for (distance in distances) timed.distanceToTime(distance)];
      // The task-space check inspects the path between samples too. Time the inspected distances exactly:
      // interpolating between sample times is wrong where the path speed changes fast, e.g. braking to rest
      // over the last sample interval, where distance goes with the square of time.
      var checkSamples = authoredPolyline == null ? distances.length * 2 - 1 :
        Std.int(Math.max(distances.length * 2 - 1, Math.ceil(path.length() / (blendTolerance / 8.0)) + 1));
      var checkDistances = [for (sample in 0...checkSamples) path.length() * sample / (checkSamples - 1)];
      var checkTimes = [for (distance in checkDistances) timed.distanceToTime(distance)];
      var timeSteps = Std.int(Math.max(1,
        Math.ceil(timed.trajectory.durationSeconds() / 0.001)));
      var taskSampleDistances:Array<Float> = [];
      if (timed.hasDirectInverse()) {
        // The time law inverts in closed form: one native call covers every 1 ms sample, where a bisection
        // per sample cost 24 round trips each.
        var duration = timed.trajectory.durationSeconds();
        taskSampleDistances = timed.timesToDistances([for (sample in 0...(timeSteps + 1)) duration * sample / timeSteps]);
        taskSampleDistances[0] = 0.0;
        taskSampleDistances[timeSteps] = path.length();
      } else {
        var interval = 0;
        for (sample in 0...(timeSteps + 1)) {
          var time = timed.trajectory.durationSeconds() * sample / timeSteps;
          while (interval + 1 < timeMap.length - 1 && timeMap[interval + 1] < time)
            interval++;
          var lower = distances[interval];
          var upper = distances[interval + 1];
          if (sample == 0) taskSampleDistances.push(0.0);
          else if (sample == timeSteps) taskSampleDistances.push(path.length());
          else {
            for (_ in 0...24) {
              var middle = (lower + upper) * 0.5;
              if (timed.distanceToTime(middle) < time) lower = middle;
              else upper = middle;
            }
            taskSampleDistances.push((lower + upper) * 0.5);
          }
        }
      }
      timed.releaseDistanceMap();
      var made = new PendingMotion(index, startQ, positions[count], timed.trajectory, events,
        path, distances, timeMap, authoredPolyline, blendTolerance,
        taskSampleDistances, checkDistances, checkTimes);
      made.feed = feed;
      return made;
    } catch (error:Dynamic) {
      timed.releaseDistanceMap();
      timed.trajectory.dispose();
      throw error;
    }
  }

  /** `checkDistances` along the path, each at its exact `checkTimes`, then the trajectory clock at 1 ms. */
  function checkTaskSpace(plan:ExecutionPlan, path:PosePath,
      checkDistances:Array<Float>, checkTimes:Array<Float>, index:Int,
      authoredPolyline:Null<Array<Pose3>>, blendTolerance:Float,
      taskSampleDistances:Array<Float>):Void {
    var worst = 0.0, worstTime = 0.0;
    var tolerance = authoredPolyline == null && path.authoredGeometry == null ?
      positionTolerance : authoredPolyline == null ? path.blendTolerance : blendTolerance;
    var failure:Null<String> = null;
    function inspect(distance:Float, time:Float):Void {
      var desired = path.waypointAt(distance);
      var actual = solver.forward(plan.evaluate(time).positions);
      var error = authoredPolyline != null ?
        Math.min(distanceToSegment(actual, authoredPolyline[0], authoredPolyline[1]),
          distanceToSegment(actual, authoredPolyline[1], authoredPolyline[2])) :
        path.authoredGeometry != null ?
          distanceToGeometry(actual, path.authoredGeometry) :
          PoseMath.distance(actual, desired.pose);
      var allowed = authoredPolyline != null ? blendTolerance :
        path.authoredGeometry != null ? path.blendTolerance :
          desired.positionTolerance;
      var angle = orientationError(actual, desired.pose,
        path.orientationPolicyAt(distance));
      if (error > worst) { worst = error; worstTime = time; tolerance = allowed; }
      if (error > allowed + 1e-9 ||
          angle > desired.orientationTolerance + 1e-9)
        failure = 'Motion program op $index task-space tolerance exceeded at path distance $distance (position $error / $allowed, orientation $angle / ${desired.orientationTolerance})';
    }
    for (sample in 0...checkDistances.length) inspect(checkDistances[sample], checkTimes[sample]);
    // Cover the trajectory clock as well as the authored path geometry.
    var timeSteps = taskSampleDistances.length - 1;
    for (sample in 0...(timeSteps + 1)) {
      var time = plan.durationSeconds * sample / timeSteps;
      inspect(taskSampleDistances[sample], time);
    }
    plan.report.setTaskSpace(failure == null ? TrajectoryCoreConstants.MK_CHECK_PASSED :
      TrajectoryCoreConstants.MK_CHECK_FAILED, worst, worstTime, tolerance,
      Trajectory.nanoseconds(plan.durationSeconds / timeSteps));
    if (failure != null) throw failure;
  }

  static function distanceToSegment(point:Pose3, start:Pose3, end:Pose3):Float
    return pointToSegmentDistance(point.x, point.y, point.z, start.x, start.y, start.z,
      end.x, end.y, end.z);

  /** Distance uses coordinates directly; authored polylines are checked at every trajectory sample. */
  static function pointToSegmentDistance(px:Float, py:Float, pz:Float,
      sx:Float, sy:Float, sz:Float, ex:Float, ey:Float, ez:Float):Float {
    var dx = ex - sx, dy = ey - sy, dz = ez - sz;
    var lengthSquared = dx * dx + dy * dy + dz * dz;
    var fraction = lengthSquared <= 0.0 ? 0.0 : Math.max(0.0, Math.min(1.0,
      ((px - sx) * dx + (py - sy) * dy +
        (pz - sz) * dz) / lengthSquared));
    var x = px - sx - fraction * dx;
    var y = py - sy - fraction * dy;
    var z = pz - sz - fraction * dz;
    return Math.sqrt(x * x + y * y + z * z);
  }

  static function distanceToGeometry(point:Pose3, geometry:GeometricPath):Float {
    var best = Math.POSITIVE_INFINITY;
    for (primitive in geometry.primitives) {
      var candidate = if (Std.isOfType(primitive, LineSegment)) {
        var line:LineSegment = cast primitive;
        pointToSegmentDistance(point.x, point.y, point.z,
          line.start.x, line.start.y, line.start.z, line.end.x, line.end.y, line.end.z);
      } else if (Std.isOfType(primitive, ArcSegment)) {
        var arc:ArcSegment = cast primitive;
        var angle = Math.atan2(point.y - arc.center.y,
          point.x - arc.center.x);
        var delta = angle - arc.startAngle;
        if (arc.sweepAngle >= 0.0) {
          while (delta < 0.0) delta += 2.0 * Math.PI;
        } else {
          while (delta > 0.0) delta -= 2.0 * Math.PI;
        }
        var fraction = arc.sweepAngle == 0.0 ? 0.0 :
          Math.max(0.0, Math.min(1.0, delta / arc.sweepAngle));
        var at = arc.pointAt(arc.length() * fraction);
        Math.sqrt((point.x - at.x) * (point.x - at.x) +
          (point.y - at.y) * (point.y - at.y) +
          (point.z - at.z) * (point.z - at.z));
      } else if (Std.isOfType(primitive, motionkit.path.CircularSegment)) {
        var circular:motionkit.path.CircularSegment = cast primitive;
        circular.distanceTo(new motionkit.path.PathPoint(point.x, point.y, point.z));
      } else throw "Unsupported authored path primitive";
      best = Math.min(best, candidate);
    }
    return best;
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
  /**
    `path` split at its sharp corners, where one primitive's end tangent is
    not the next one's start tangent, with each section's distance along it.
  **/
  static function cornerSections(path:PosePath):Array<{path:PosePath, offset:Float}> {
    var sections:Array<{path:PosePath, offset:Float}> = [];
    var start = 0, offset = 0.0, sectionOffset = 0.0;
    for (k in 0...path.primitives.length) {
      var primitive = path.primitives[k];
      if (k > start) {
        var before = path.primitives[k - 1];
        var arriving = before.derivativesAt(before.length()), leaving = primitive.derivativesAt(0.0);
        var beforeAngular = ToolFreedom.requiredAngular(before.endWaypoint().pose, arriving.angular, before.orientationPolicy());
        var leavingAngular = ToolFreedom.requiredAngular(primitive.startWaypoint().pose, leaving.angular, primitive.orientationPolicy());
        var turn = 0.0;
        for (axis in 0...3) turn = Math.max(turn, Math.max(
          Math.abs(arriving.linear[axis] - leaving.linear[axis]),
          Math.abs(beforeAngular[axis] - leavingAngular[axis])));
        if (turn > 1e-6) {
          sections.push({path: new PosePath(path.frameId, path.primitives.slice(start, k)), offset: sectionOffset});
          start = k;
          sectionOffset = offset;
        }
      }
      offset += primitive.length();
    }
    sections.push({path: new PosePath(path.frameId, path.primitives.slice(start)), offset: sectionOffset});
    if (sections.length == 1) return [{path: path, offset: 0.0}];
    var authored = path.authoredGeometry;
    if (authored != null) for (section in sections)
      section.path.withAuthoredGeometry(authored, path.blendTolerance);
    return sections;
  }

  /**
    Where to sample a path in joint space: every primitive boundary, so that
    each span lies on one primitive, and inside each primitive at most
    `cartesianResolution` apart and a quarter radian of turning apart. A
    boundary sample also names the primitive arriving at it.
  **/
  function pathSamples(path:PosePath):Array<PathSample> {
    var first = path.primitives[0];
    var samples:Array<PathSample> = [{distance: 0.0, primitive: first, local: 0.0, arriving: null}];
    var start = 0.0;
    for (k in 0...path.primitives.length) {
      var primitive = path.primitives[k], length = primitive.length();
      var curvature = 0.0;
      for (at in [0.0, 0.5 * length, length]) {
        var second = primitive.derivativesAt(at);
        curvature = Math.max(curvature, Math.sqrt(
          second.linearSecond[0] * second.linearSecond[0] +
          second.linearSecond[1] * second.linearSecond[1] +
          second.linearSecond[2] * second.linearSecond[2]));
      }
      var pieces = Std.int(Math.max(1, Math.max(Math.ceil(length / cartesianResolution),
        Math.ceil(length * curvature / 0.25))));
      for (piece in 1...(pieces + 1)) {
        var last = piece == pieces;
        // Exactly the end: `length * pieces / pieces` can round one ulp past it.
        var local = last ? length : length * piece / pieces;
        var next = last && k + 1 < path.primitives.length ? path.primitives[k + 1] : null;
        samples.push(next == null ?
          {distance: start + local, primitive: primitive, local: local, arriving: null} :
          {distance: start + length, primitive: next, local: 0.0,
            arriving: {primitive: primitive, local: length}});
      }
      start += length;
    }
    samples[samples.length - 1].distance = path.length();
    return samples;
  }

  function requireFixedOrientation(start:Pose3, targets:Array<Pose3>, freedom:Null<OrientationPolicy>, index:Int):Void {
    switch freedom {
      case Fixed:
        for (target in targets) {
          var residual = PoseMath.angle(start, target);
          if (residual > orientationTolerance)
            throw 'Motion program op $index fixed orientation differs from its start: position residual 0 m; orientation residual $residual rad';
        }
      default:
    }
  }

  function solverFailure():String {
    if (Std.isOfType(solver, ManipulatorKinematics)) {
      var numeric:ManipulatorKinematics = cast solver;
      return numeric.lastFailure == null ? "" : ': ${numeric.lastFailure}';
    }
    return "";
  }

  /** dq/ds for a pose moving at `linear` and `angular` per metre of path, or null at a singularity. */
  function jointRate(q:Array<Float>, linear:Array<Float>, angular:Array<Float>,
      ?redundancyRate:Array<Float>, ?freedom:OrientationPolicy):Null<Array<Float>>
    return solver.solveDifferential(q, new Twist6(linear[0], linear[1], linear[2],
      angular[0], angular[1], angular[2]), redundancyRate, freedom);

  /**
    d²q/ds²: how dq/ds changes along the path, from the pose's second
    derivative and the kinematics' change between nearby configurations.
    Exact for a Cartesian machine, whose kinematics do not change.
  **/
  function jointCurvature(q:Array<Float>, rate:Array<Float>, pose:PoseDerivatives,
      ?redundancyRate:Array<Float>, ?freedom:OrientationPolicy):Array<Float> {
    var step = 1e-6;
    function at(sign:Float):Null<Array<Float>>
      return jointRate([for (joint in 0...q.length) q[joint] + sign * step * rate[joint]],
        [for (axis in 0...3) pose.linear[axis] + sign * step * pose.linearSecond[axis]],
        [for (axis in 0...3) pose.angular[axis] + sign * step * pose.angularSecond[axis]], redundancyRate, freedom);
    var ahead = at(1.0), behind = at(-1.0);
    if (ahead == null || behind == null) return [for (_ in q) 0.0];
    return [for (joint in 0...q.length) (ahead[joint] - behind[joint]) / (2.0 * step)];
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
    return ToolFreedom.orientationError(actual, desired, policy);
  }
  function zeros():Array<Float> return [for (_ in 0...solver.jointCount()) 0.0];
  static function effective(machine:Array<Float>, requested:Float):Array<Float>
    return [for (limit in machine) requested > 0.0 ? Math.min(limit, requested) : limit];
  static function noteBlend(blend:Blend, index:Int, sink:ProgramSink):Void {
    switch blend {
      case ToleranceBlend(_): sink.note('Motion program op $index tolerance blend uses exact stop');
      case ExactStop:
    }
  }
}

/** A planned motion waiting for the ops that may still add to it (internal to compilation). */
class PendingMotion {
  public final opIndex:Int;
  public final startQ:Array<Float>;
  public final endQ:Array<Float>;
  public final trajectory:Trajectory;
  public final events:Array<TimedEvent>;
  public final path:Null<PosePath>;
  public final distances:Array<Float>;
  public final times:Array<Float>;
  public final taskSampleDistances:Array<Float>;
  /** Path distances the task-space check inspects, and their exact times. */
  public final checkDistances:Array<Float>;
  public final checkTimes:Array<Float>;
  public final authoredPolyline:Null<Array<Pose3>>;
  public final blendTolerance:Float;
  /** Where this motion starts along its op's path, when the op is split at corners. */
  public var distanceOffset:Float = 0.0;
  /** The programmed speed of a path move in m/s, 0 for a joint move. */
  public var feed:Float = 0.0;

  public function new(opIndex:Int, startQ:Array<Float>, endQ:Array<Float>,
      trajectory:Trajectory, events:Array<TimedEvent>, path:Null<PosePath>,
      distances:Null<Array<Float>>, ?times:Array<Float>,
      ?authoredPolyline:Array<Pose3>, ?blendTolerance:Float = 0.0,
      ?taskSampleDistances:Array<Float>, ?checkDistances:Array<Float>, ?checkTimes:Array<Float>) {
    this.opIndex = opIndex; this.startQ = startQ.copy(); this.endQ = endQ.copy();
    this.trajectory = trajectory; this.events = events;
    this.path = path;
    this.distances = distances == null ? [] : distances;
    this.times = times == null ? [] : times;
    this.taskSampleDistances = taskSampleDistances == null ? [] : taskSampleDistances;
    this.checkDistances = checkDistances == null ? [] : checkDistances;
    this.checkTimes = checkTimes == null ? [] : checkTimes;
    this.authoredPolyline = authoredPolyline == null ? null : authoredPolyline.copy();
    this.blendTolerance = blendTolerance;
  }

  /** Path distances along the whole op. */
  public function opDistances():Array<Float>
    return [for (distance in distances) distanceOffset + distance];
}
