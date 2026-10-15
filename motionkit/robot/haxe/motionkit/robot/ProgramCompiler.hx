package motionkit.robot;

import haxe.Int64;
import TrajectoryCore;
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
import motionkit.trajectory.InputShaper;
import motionkit.trajectory.ShaperImpulses;
import motionkit.robot.TimedMotionTransforms;
import motionkit.robot.TimedMotionTransforms.TimedMotion;
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
  /** The couplings as one projection (a follower is the sum of its terms, each after the joints it follows), or null for none. */
  final projection:Null<CouplingProjection>;
  public final positionTolerance:Float;
  public final orientationTolerance:Float;
  public final ikTolerance:IkTolerance;
  public final jointPathPlanner:Null<JointPathPlanner>;
  final jointIds:Null<Array<String>>;
  final couplings:Null<Array<JointCoupling>>;
  final controllerPeriodSeconds:Float;
  var configurationConstraint:Null<ConfigurationConstraint> = null;
  /**
   * The plan check every compiled program goes through, or null for none, for simulation and device.
   * Its findings are on `ExecutionPlan.checked`. Direct MotionSystem moves and live ServoSession
   * plan chunks run their checks at their submission boundaries.
   */
  public var planCheck:Null<PlanCheck> = null;
  /** Linear motor sums for machines whose axis velocities use the single-axis envelope. */
  public var motorSpace:Null<MotorSpaceConstraints> = null;
  /**
    When positive, runs of a lowered path's segments are merged into one quintic each, up to this long, where the
    quintic follows them to a few microseconds of motion (`MERGE_TIMING`) in position, a percent of the speed limit
    and a tenth of the acceleration limit. A printer's curves lower to thousands of segments a second, which would
    fill the robot's queue and a device's link; the default of zero leaves a path as it was lowered.
  **/
  public var mergeSegmentsSeconds:Float = 0.0;
  /**
    How many plans were built from the lowered path because their merged form broke a limit, and why the last one did. A
    plan that was not merged (merging off, a joint move) is not a refusal.
  **/
  public var mergeRefusals:Int = 0;
  public var lastMergeRefusal:String = "";
  /** The position error a merged segment may have, as seconds of motion at the joint's speed limit. */
  public static inline var MERGE_TIMING:Float = 10e-6;
  /**
    The impulses each joint's motion is convolved with (see `shapeJoints`), in joint order; null shapes nothing. Their
    amplitudes are positive and add up to one, so shaping blends delayed copies of a plan's motion and cannot raise a
    velocity or an acceleration past the limits the plan was timed for.
  **/
  var shaping:Null<Array<ShaperImpulses>> = null;
  /** How far the shaped motion lags the lowered one on average, seconds: every joint is lined up to this. */
  var shapingDelay:Float = 0.0;
  /** The shaped plan's extra length: the longest impulse delay, seconds. */
  var shapingTail:Float = 0.0;
  /** Bound on how far shaping can move the tool from the lowered motion delayed by `shapingDelay`, metres. */
  var shapingDeviation:Float = 0.0;
  /** Seconds a path's distance-driven joint (an extruder) runs ahead of the flow it feeds; 0 is off. */
  public var pressureAdvance:Float = 0.0;
  /** The window, seconds, over which the driven joint's velocity is averaged for the advance. */
  public var pressureAdvanceWindow:Float = 0.04;
  /** Pure, worker-safe event scheduling on the actual timed path section.
   * Distances/events use whole-operation coordinates; times are section-local. */
  public var pathEventSchedule:Null<(Int,Float,Bool,Array<Float>,Array<Float>,Array<PathEvent>)->Array<PathEvent>> = null;

  /** Process sections reject a time-law retry or stretch after refinement. */
  public var requireFeasiblePath:Null<Int->Bool> = null;

  /**
   * Changes to the cell during the program, per op (grasps, places, contact windows; COLLISION.md
   * CL-D9). Validation replays them in time order through the planner's clearance world, which must
   * then be a `ClearanceScene`.
   */
  public var clearanceEvents:Array<ClearanceEvent> = [];
  /** How deep the conservative clearance check of a joint or generated move bisects (CL4b). */
  public var clearanceDepthLimit:Int = 16;
  /** Maximum task rotation per selection sample, in radians; clearance is certified separately. */
  public var angularResolution:Float = 0.25;
  /** Plans the path of a joint move whose options ask for it (`MotionOptions.planned`; COLLISION.md CL7). */
  public var motionPlanner:Null<motionkit.planner.MotionPlanner> = null;
  /**
   * Certifies the curved motion of planned joint moves joined by `Blend.FlyBy`; it must be the
   * space `motionPlanner` checks its edges in. Without it a fly-by stops at its target.
   */
  public var flyByCertificate:Null<motionkit.planner.BoxClearanceSpace> = null;

  /**
    Shapes the motion of the joints that have a shaper (`shapers`, one entry per joint in joint order, null for a joint
    that is not shaped): each path's motion is convolved with the shaper's impulses, which keeps the carriage from
    ringing at the machine's resonance. A joint without a shaper (the other axis, the extruder) is delayed by the same
    average, so everything stays in step, and the path's events with it. Joints that follow others are shaped through
    their leaders. Shaping is only meaningful where a joint's motion is the tool's (Cartesian axes): the task-space
    check allows the shaped tool the spread that the impulses can open between the motion and its delayed copy.
  **/
  public function shapeJoints(shapers:Null<Array<Null<InputShaper>>>):Void {
    if (shapers == null) {
      shaping = null; shapingDelay = 0.0; shapingTail = 0.0; shapingDeviation = 0.0;
      return;
    }
    if (shapers.length != maxVelocity.length) throw "Input shaping needs a shaper or null for each joint";
    var reference = 0.0, any = false;
    for (shaper in shapers) if (shaper != null) { any = true; reference = Math.max(reference, shaper.impulses.meanDelay()); }
    if (!any) { shapeJoints(null); return; }
    // The deviation is the tool's, in metres: each shaped joint is one Cartesian axis of an exact affine machine.
    var map = solver.affineMap();
    if (map == null) throw "Input shaping needs a machine whose joints are an exact affine map of the tool (Cartesian axes)";
    var impulses:Array<ShaperImpulses> = [];
    var tail = 0.0, squares = 0.0;
    for (joint in 0...shapers.length) {
      var shaper = shapers[joint];
      var own = shaper == null ? ShaperImpulses.delay(0.0) : shaper.impulses;
      var lined = own.later(reference - own.meanDelay());
      impulses.push(lined);
      tail = Math.max(tail, lined.tail());
      // Taylor: |sum a_i x(t - T_i) - x(t - d)| <= A / 2 * sum a_i (T_i - d)^2, with d the mean delay and A the
      // largest acceleration (the velocity term cancels because d is the mean).
      if (shaper != null) {
        var sum = 0.0;
        for (index in 0...lined.delays.length) sum += lined.amplitudes[index] * Math.pow(lined.delays[index] - reference, 2);
        // The joint's travel per metre of its axis; a shaped joint that moves with more than one axis (a CoreXY motor)
        // is shaped through its axes instead.
        var perMetre = 0.0, axes = 0;
        for (column in map.columns) if (column[joint] != 0.0) { perMetre = Math.abs(column[joint]); axes++; }
        if (axes != 1) throw 'Joint $joint is shaped but is not one Cartesian axis: shape the axes it follows';
        var metres = 0.5 * sum * maxAcceleration[joint] / perMetre;
        // The axes' deviations are independent components of the tool's.
        squares += metres * metres;
      }
    }
    shaping = impulses; shapingDelay = reference; shapingTail = tail; shapingDeviation = Math.sqrt(squares);
  }

  /**
    Gives `into` (a compiler of the same machine: a worker's, or one with another joint path planner) every setting of
    this one that is not a constructor argument, so the two cannot plan differently. Every such setting is copied here
    and only here: a setting left out of one factory once planned every print on the planner thread without pressure
    advance.
  **/
  function copySettings(into:ProgramCompiler):Void {
    into.planningAssumptions = planningAssumptions.copy();
    into.planCheck = planCheck == null ? null : planCheck.fork();
    into.motorSpace = motorSpace;
    into.mergeSegmentsSeconds = mergeSegmentsSeconds;
    into.shaping = shaping; into.shapingDelay = shapingDelay; into.shapingTail = shapingTail; into.shapingDeviation = shapingDeviation;
    into.pressureAdvance = pressureAdvance;
    into.pressureAdvanceWindow = pressureAdvanceWindow;
    into.pathEventSchedule = pathEventSchedule;
    into.requireFeasiblePath = requireFeasiblePath;
    into.clearanceEvents = clearanceEvents.copy();
    into.clearanceDepthLimit = clearanceDepthLimit;
    into.motionPlanner = motionPlanner;
    into.flyByCertificate = flyByCertificate;
    into.angularResolution = angularResolution;
  }

  /** Preserve compiler limits/checks while timing an already selected route. */
  public function withJointPathPlanner(planner:JointPathPlanner):ProgramCompiler {
    if(planner==null)throw "A replacement joint path planner is required";
    var result=new ProgramCompiler(solver,limits,frameId,maxVelocity,maxAcceleration,maxJerk,
      startTolerances,timing,cartesianResolution,maxJointJump,positionTolerance,orientationTolerance,
      ikTolerance,planner,perJointMaxJump,jointIds,couplings,controllerPeriodSeconds);
    copySettings(result);
    result.configurationConstraint=configurationConstraint;
    return result;
  }

  /**
   * This compiler for a planning thread, on a fork of its solver: the worker never shares solver
   * state with the caller, who keeps using the same kinematics while plans execute.
   */
  public function forWorker():ProgramCompiler {
    var forked = solver.fork();
    // The solver may be immutable, but planner diagnostics and compilation assumptions are worker-owned.
    var worker = new ProgramCompiler(forked, limits, frameId, maxVelocity, maxAcceleration, maxJerk,
      startTolerances, timing, cartesianResolution, maxJointJump, positionTolerance,
      orientationTolerance, ikTolerance,
      jointPathPlanner == null ? null : jointPathPlanner.withSolver(forked),
      perJointMaxJump, jointIds, couplings, controllerPeriodSeconds);
    // The worker plans one program in order, so it remembers which way each axis last moved (`planningAssumptions`).
    copySettings(worker);
    worker.configurationConstraint = configurationConstraint == null ? null :
      new ConfigurationConstraint(forked,configurationConstraint.configuration);
    return worker;
  }

  public function new(solver:KinematicsSolver, limits:ValidationLimits,
      frameId:String, maxVelocity:Array<Float>, maxAcceleration:Array<Float>,
      maxJerk:Array<Float>, startTolerances:StartTolerances, ?timing:PathTimingBackend,
      ?cartesianResolution:Float = 0.01, ?maxJointJump:Float = 0.5,
      ?positionTolerance:Float = 0.005, ?orientationTolerance:Float = 0.02,
      ?ikTolerance:IkTolerance, ?jointPathPlanner:JointPathPlanner,
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
    var projected:Null<CouplingProjection> = null;
    if (couplings != null) {
      if (jointIds == null || jointIds.length != count)
        throw "Program compiler couplings need joint IDs in solver order";
      var terms:Array<{leader:Int, follower:Int, ratio:Float, offset:Float}> = [];
      for (coupling in couplings) {
        var leader = jointIds.indexOf(coupling.leader);
        var follower = jointIds.indexOf(coupling.follower);
        if (leader < 0 || follower < 0)
          throw 'Program compiler coupling "${coupling.id}" references an unknown joint';
        terms.push({leader: leader, follower: follower, ratio: coupling.ratio, offset: coupling.offset});
      }
      projected = CouplingProjection.of(terms, count);
      if (projected.isEmpty()) projected = null;
    }
    projection = projected;
    this.jointIds = jointIds == null ? null : jointIds.copy();
    this.couplings = couplings == null ? null : couplings.copy();
    this.positionTolerance = positionTolerance;
    this.orientationTolerance = orientationTolerance;
    this.ikTolerance = ikTolerance == null ? new IkTolerance() : ikTolerance;
    this.jointPathPlanner = jointPathPlanner != null ? jointPathPlanner :
      defaultJointPathPlanner(solver);
  }

  static function defaultJointPathPlanner(solver:KinematicsSolver):Null<JointPathPlanner> {
    if (Std.isOfType(solver, AxisKinematics)) return new AxisJointPathPlanner(cast solver);
    if (Std.isOfType(solver, EaikKinematics)) {
      var opw:EaikKinematics = cast solver;
      // External axes require the process builder's explicit positioning rule.
      if (opw.manipulator.external.indexOf(true) < 0 && opw.manipulator.workFrame == null)
        return new StructuredJointPathPlanner(opw.manipulator);
    }
    if (Std.isOfType(solver, ManipulatorKinematics)) {
      var adapter:ManipulatorKinematics = cast solver;
      if (adapter.jointCount() == 6 && adapter.manipulator.external.indexOf(true) < 0 &&
          adapter.manipulator.workFrame == null)
        return new StructuredJointPathPlanner(adapter.manipulator,null,null,null,8,false,
          null,null,null,0,adapter);
    }
    return null;
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
      firstOp:Int, speedScale:Float, sink:ProgramSink):ProgramCompilation {
    if(program==null || initialQ==null || initialQ.length!=solver.jointCount())
      throw "Program compiler needs a program and complete start position";
    for(value in initialQ)if(!Math.isFinite(value))throw "Non-finite program start position";
    resetClearanceScene();
    var compiler=this;
    if(program.configuration!=null){
      var pin:motionkit.kinematics.SixAxisConfiguration=cast program.configuration;
      if(jointPathPlanner==null)throw "Configuration-pinned program requires a joint path planner";
      compiler=withJointPathPlanner(jointPathPlanner.withConfiguration(pin));
      compiler.configurationConstraint=new ConfigurationConstraint(solver,pin);
    }
    if(compiler.configurationConstraint!=null)compiler.configurationConstraint.require(initialQ);
    return new ProgramCompilation(compiler, program, initialQ, firstPlanId, firstOp, speedScale, sink);
  }

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

  /**
    A copy of `trajectory` in which runs of segments are one quintic each within `share` of the merge tolerances (the
    followers not yet projected). Merges applied one after the other add their errors, so together they share one budget.
  **/
  function mergeRuns(trajectory:Trajectory, share:Float):Trajectory {
    var count = maxVelocity.length;
    return trajectory.mergeSegments(
      [for (joint in 0...count) share * MERGE_TIMING * maxVelocity[joint]],
      [for (joint in 0...count) share * 0.01 * maxVelocity[joint]],
      [for (joint in 0...count) share * 0.1 * maxAcceleration[joint]],
      maxVelocity.slice(0, count), maxAcceleration.slice(0, count), positionBounds(count, false), positionBounds(count, true),
      mergeSegmentsSeconds);
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
      c.sectionCurves = null;
      if (c.cursor >= program.ops.length) {
        retire(c);
        if (c.leadingOutputs.length > 0)
          throw 'Motion program op ${c.currentIndex} has output changes without following motion';
        c.done = true;
        c.sink.finish(c.nextId);
        return false;
      }
      var index = c.cursor++;
      if (c.skip > 0) { c.skip--; return true; }
      c.currentIndex = index;
      var q = c.q;
      switch program.ops[index] {
        case MoveJ(target, options, blend):
          retire(c);
          var chainEnd = flyByChainEnd(program, index);
          if (chainEnd > index) {
            var pending = flyBy(program, index, chainEnd, q);
            c.pending = pending;
            attachLeadingOutputs(pending, c.leadingOutputs);
            c.q = pending.endQ.copy();
            c.skip = chainEnd - index;
            c.sink.note('Motion program ops $index to $chainEnd fly by their joint targets where certified clear');
            return true;
          }
          noteBlend(blend, index, c.sink);
          var goal = resolveTarget(target, q, index);
          checkJointPosition(goal, index, null);
          var velocity = effective(maxVelocity, options.maxVelocity);
          var acceleration = effective(maxAcceleration, options.maxAcceleration);
          var jerk = effective(maxJerk, options.maxJerk);
          // The limits of a joint offset are its driven joint's: what moves with it is held in proportion.
          switch target {
            case JointOffset(driven, delta):
              for (joint in 0...goal.length) {
                var share = Math.abs(goal[joint] - q[joint]) / Math.abs(delta);
                if (share <= 0.0) continue;
                if (options.maxVelocity > 0.0) velocity[joint] = Math.min(maxVelocity[joint], options.maxVelocity * share);
                if (options.maxAcceleration > 0.0) acceleration[joint] = Math.min(maxAcceleration[joint], options.maxAcceleration * share);
                if (options.maxJerk > 0.0) jerk[joint] = Math.min(maxJerk[joint], options.maxJerk * share);
              }
            case _:
          }
          var motors = motorSpace;
          var generated = options.planned ? plannedMove(q, goal, velocity, acceleration, jerk, index)
            : motors == null ? Trajectory.generateStateToState(q, zeros(), zeros(), goal,
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
            // Program validation admits a fly-by only between planned joint moves.
            case ExactStop, FlyBy:
          }
          var pending:PendingMotion;
          if (blended != null) {
            var nextFeed = switch program.ops[index + 1] {
              case MoveL(_, _, speed, _, _): speed;
              case _: feed;
            };
            pending = lowerPath(speedScale, blended, q, Math.max(feed, nextFeed), [], index,
              [solver.forward(q), pose, cast blendNextPose], blendTolerance);
            c.skip = 1;
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
          c.sections = timingSections(path);
          c.sectionCurves = null;
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
  /** The stopped joint transition used for free path entry/exit checks.
   * Caller owns and must dispose the returned trajectory. */
  public function generateEntry(from:Array<Float>,to:Array<Float>):Trajectory {
    if(from==null || to==null || from.length!=solver.jointCount() || to.length!=from.length)
      throw "Generated entry requires aligned joint configurations";
    for(q in [from,to])for(value in q)if(!Math.isFinite(value))
      throw "Generated entry requires finite joint positions";
    var moved = false;
    for (joint in 0...from.length) if (Math.abs(from[joint]-to[joint]) > 1e-7) moved = true;
    if (!moved) return Trajectory.fromSegments([{
      timeFromStartNs:Int64.ofInt(0),durationNs:Trajectory.nanoseconds(controllerPeriodSeconds),
      coefficients:[for (position in from) [position,0.0]]
    }]);
    var motors = motorSpace;
    var generated = motors == null ? Trajectory.generateStateToState(from,zeros(),zeros(),to,
      maxVelocity,maxAcceleration,maxJerk) : motors.move(from,to,maxVelocity,maxAcceleration,maxJerk);
    if (projection != null) {
      try { var projected = projectCouplings(generated); generated.dispose(); return projected; }
      catch (error:Dynamic) { generated.dispose(); throw error; }
    }
    return generated;
  }

  function lowerSection(c:ProgramCompilation):Void {
    retire(c);
    var k = c.sectionIndex++;
    var section = c.sections[k], last = k == c.sections.length - 1;
    var end = section.offset + section.path.length();
    var curve:Null<JointPathSamples> = null;
    var pathStart = c.q;
    var entryTrajectory:Null<Trajectory> = null;
    var exitTrajectory:Null<Trajectory> = null;
    var retreat = last && jointPathPlanner != null ? jointPathPlanner.retreatTarget() : null;
    function releaseTransitions():Void {
      if (entryTrajectory != null) { entryTrajectory.dispose(); entryTrajectory = null; }
      if (exitTrajectory != null) { exitTrajectory.dispose(); exitTrajectory = null; }
    }
    try {
      if(k==0 && c.sections.length>1 && Std.isOfType(jointPathPlanner,StructuredJointPathPlanner)) {
        var structured:StructuredJointPathPlanner=cast jointPathPlanner;
        var distances:Array<Float> = [],poses:Array<Pose3> = [],freedoms:Array<OrientationPolicy> = [],plannedOffset=0.0;
        for(part in c.sections){
          var samples=pathSamples(part.path);
          // The outgoing section owns the shared ladder knot; its incoming
          // endpoint is checked by the planner and refined independently.
          if(distances.length>0){distances.pop();poses.pop();freedoms.pop();}
          for(sample in samples){
            distances.push(plannedOffset+sample.distance);
            poses.push(sample.primitive.waypointAt(sample.local).pose);
            freedoms.push(sample.primitive.orientationPolicy());
          }
          plannedOffset+=part.path.length();
        }
        if(distances.length>MAX_SECTION_SAMPLES+1)throw 'Motion program op ${c.currentIndex} exceeds Cartesian sample budget';
        var destination=structured.retreatTarget();
        c.sectionCurves=structured.planSections([for(part in c.sections)part.path],new PathRequest(
          distances,poses,c.q,ikTolerance,perJointMaxJump,maxVelocity,48,freedoms),
          !structured.allowsFreeStart(),(from,to)->{
            if(entryTrajectory!=null){entryTrajectory.dispose();entryTrajectory=null;}
            entryTrajectory=generateEntry(from,to);
            return structured.checkMotion(entryTrajectory);
          },destination==null?null:from->{
            var generated=generateEntry(from,destination);
            try {var failure=structured.checkMotion(generated);generated.dispose();return failure;}
            catch(error:Dynamic){generated.dispose();throw error;}
          });
        // Subtracting a global prefix can change a local distance by an ulp.
        // Retain the compiler's original local grid for exact timing alignment.
        for(part in 0...c.sections.length){
          var curve=c.sectionCurves[part],local=[for(sample in pathSamples(c.sections[part].path))sample.distance];
          if(local.length!=curve.s.length)throw "Global section refinement changed its sample count";
          for(sample in 0...local.length)if(Math.abs(local[sample]-curve.s[sample])>1e-12)
            throw "Global section refinement changed its local sample grid";
          c.sectionCurves[part]=new JointPathSamples(local,curve.q,curve.qPrime,curve.qDoublePrime,curve.qDoublePrimeBefore);
          c.sectionCurves[part].clearanceProof=curve.clearanceProof;
        }
      }
      if(c.sectionCurves!=null){
        curve=c.sectionCurves[k];pathStart=curve.q[0];
        if(k>0)for(joint in 0...c.q.length)if(Math.abs(pathStart[joint]-c.q[joint])>1e-7)
          throw "Globally selected timing sections disagree at their shared configuration";
      }
      if (c.sectionCurves == null && jointPathPlanner != null && (k == 0 && jointPathPlanner.allowsFreeStart() || retreat != null)) {
        var freeStart = k == 0 && jointPathPlanner.allowsFreeStart();
        var samples = pathSamples(section.path);
        if (samples.length > MAX_SECTION_SAMPLES + 1) throw 'Motion program op ${c.currentIndex} exceeds Cartesian sample budget';
        curve = jointPathPlanner.plan(section.path,new PathRequest(
          [for (sample in samples) sample.distance],
          [for (sample in samples) sample.primitive.waypointAt(sample.local).pose],
          c.q,ikTolerance,perJointMaxJump,maxVelocity,48,
          [for (sample in samples) sample.primitive.orientationPolicy()]),!freeStart,(from,to) -> {
            if (entryTrajectory != null) { entryTrajectory.dispose(); entryTrajectory = null; }
            entryTrajectory = generateEntry(from,to);
            return jointPathPlanner.checkMotion(entryTrajectory);
          },retreat == null ? null : from -> {
            if (exitTrajectory != null) { exitTrajectory.dispose(); exitTrajectory = null; }
            exitTrajectory = generateEntry(from,retreat);
            return jointPathPlanner.checkMotion(exitTrajectory);
          });
        pathStart = curve.q[0];
      }
    } catch (error:Dynamic) { releaseTransitions(); throw error; }
    var pending:PendingMotion;
    try {
      pending = lowerPath(c.speedScale, section.path, pathStart, c.sectionFeed, [for (event in c.sectionEvents)
      if (event.distance >= section.offset && (last || event.distance < end))
        new PathEvent(event.distance - section.offset, event.channel, event.value,
          event.leadSeconds, event.holdPolicy)], c.currentIndex,null,0.0,curve,section.offset,last);
    } catch (error:Dynamic) { releaseTransitions(); throw error; }
    try {
      var moved = false;
      for (joint in 0...c.q.length) if (Math.abs(pathStart[joint]-c.q[joint]) > 1e-7) moved = true;
      if (moved) {
        checkJointPosition(pathStart,c.currentIndex,null);
        var generated = entryTrajectory == null ? generateEntry(c.q,pathStart) : entryTrajectory;
        entryTrajectory = null; // PendingMotion now owns the accepted trajectory.
        c.pending = new PendingMotion(c.currentIndex,c.q,pathStart,generated,[],null,null);
        // Explicit preceding outputs retain program order; distance events stay on the path.
        attachLeadingOutputs(c.pending,c.leadingOutputs);
        retire(c);
      }
    } catch (error:Dynamic) {
      releaseTransitions();
      pending.trajectory.dispose(); throw error;
    }
    if (entryTrajectory != null) { entryTrajectory.dispose(); entryTrajectory = null; }
    pending.distanceOffset = section.offset;
    c.pending = pending;
    c.q = pending.endQ.copy();
    if (k == 0) attachLeadingOutputs(pending, c.leadingOutputs);
    if (retreat != null) {
      try {
        var moved = false;
        for (joint in 0...c.q.length) if (Math.abs(retreat[joint]-c.q[joint]) > 1e-7) moved = true;
        if (moved) {
          checkJointPosition(retreat,c.currentIndex,null);
          if (exitTrajectory == null) exitTrajectory = generateEntry(c.q,retreat);
          retire(c);
          c.pending = new PendingMotion(c.currentIndex,c.q,retreat,exitTrajectory,[],null,null);
          exitTrajectory = null;
          c.q = retreat.copy();
        }
      } catch (error:Dynamic) { releaseTransitions(); throw error; }
    }
    releaseTransitions();
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
    var profiling = Sys.getEnv("PROCESS_PATH_PROFILE") == "1";
    var profileBegan = Sys.time();
    var profileRows:Array<Dynamic> = [];
    var mark = function(stage:String) {
      var now = Sys.time();
      if (profiling) profileRows.push({stage:stage, seconds:now-profileBegan});
      profileBegan = now;
    };
    var plan:Null<ExecutionPlan> = null;
    var motion:Null<TimedMotion> = null;
    try {
      var motors = motorSpace;
      if (motors != null) motors.validate(pending.trajectory);
      var transforms = TimedMotionTransforms.apply({pressureAdvance: pressureAdvance, pressureAdvanceWindow: pressureAdvanceWindow,
        shaping: shaping, shapingDelay: shapingDelay, mergeSeconds: mergeSegmentsSeconds, mergeRuns: mergeRuns}, projection, pending, mark);
      motion = transforms;
      var timedMotion = transforms.executed, shaped = transforms.shaped;
      if(pending.geometryCurve!=null && pending.geometryClearance && Sys.getEnv("PROCESS_PATH_VERIFY_CLEARANCE")=="1"){
        var curve:JointPathSamples=cast pending.geometryCurve;
        if(Std.isOfType(curve.clearanceProof,JointCurveClearance)){var proof:JointCurveClearance=cast curve.clearanceProof;
          proof.auditTrajectory(curve,pending.trajectory,pending.taskSampleDistances);}
      }
      // With changes to the cell in this op, the conservative check below replays them instead.
      if (jointPathPlanner != null && (!pending.geometryClearance || Sys.getEnv("PROCESS_PATH_VERIFY_CLEARANCE")=="1") &&
          !hasClearanceEvents(pending.opIndex)) {
        // What is checked is what is executed: the shaped motion, when there is shaping.
        var failure = jointPathPlanner.checkMotion(timedMotion);
        if (failure != null)
          throw 'compiled trajectory clearance (${failure.a}, ${failure.b}): ${failure.distance} < ${failure.required}';
      }
      mark("trajectoryClearance");
      if(configurationConstraint!=null)configurationConstraint.checkTrajectory(timedMotion,controllerPeriodSeconds);
      mark("configurationCheck");
      var planLimits = pending.path == null ? limits : limits.withoutJerk();
      var mergedPlan:Null<ExecutionPlan> = null;
      // A merged path follows what it replaces closely but not exactly: should it break a limit anyway, it is a
      // refusal, and the path goes unmerged (`transforms.executed`: as lowered, or shaped from the pre-shape merge).
      if (transforms.merged != null) {
        try {
          mergedPlan = ExecutionPlan.create(transforms.merged, planLimits, id, pending.startQ, zeros(), zeros(),
            startTolerances.position, startTolerances.velocity, startTolerances.acceleration, pending.events);
        } catch (error:Dynamic) {
          mergeRefusals++;
          lastMergeRefusal = Std.string(error);
          transforms.dropMerged();
        }
      }
      var completedPlan:ExecutionPlan = mergedPlan != null ? mergedPlan : ExecutionPlan.create(timedMotion,
        planLimits, id, pending.startQ,
        zeros(), zeros(), startTolerances.position, startTolerances.velocity,
        startTolerances.acceleration, pending.events);
      plan=completedPlan;
      mark("executionPlanCreation");
      var taskProof:Null<JointCurveTask> = null;
      if(pending.path!=null && pending.geometryRefined && pending.geometryCurve!=null &&
          pending.authoredPolyline==null && Std.isOfType(timing,ToppraPathTiming) &&
          projection==null && motorSpace==null){
        var arm:Null<robotkit.manipulation.KinematicGroup> = null;
        if(Std.isOfType(solver,ManipulatorKinematics))arm=cast(solver,ManipulatorKinematics).manipulator;
        else if(Std.isOfType(solver,EaikKinematics))arm=cast(solver,EaikKinematics).manipulator;
        if(arm!=null){var nativeTiming:ToppraPathTiming=cast timing;
          taskProof=JointCurveTask.prove(arm,pending.path,pending.geometryCurve,nativeTiming.loweringTolerance);}
      }
      mark("taskSpaceCertificate");
      var auditTask=Sys.getEnv("PROCESS_PATH_VERIFY_TASK")=="1";
      if(pending.path!=null && (taskProof==null || auditTask))checkTaskSpace(completedPlan,pending.path,pending.checkDistances,
        pending.checkTimes,pending.opIndex,pending.authoredPolyline,pending.blendTolerance,pending.taskSampleDistances,taskProof,
        shaped ? shapingDelay : 0.0, shaped ? shapingTail : 0.0, shaped ? shapingDeviation : 0.0);
      if(taskProof!=null)completedPlan.report.setTaskSpaceBound(taskProof.positionBound,taskProof.positionTolerance);
      if(profiling && pending.path!=null)Sys.println("PROCESS_PATH_TASK_CERTIFICATE "+haxe.Json.stringify({
        op:pending.opIndex,certified:taskProof!=null,audited:auditTask && taskProof!=null,
        queries:taskProof==null?0:taskProof.queries,positionBound:taskProof==null?null:taskProof.positionBound,
        orientationBound:taskProof==null?null:taskProof.orientationBound}));
      mark("taskSpaceCheck");
      recordClearance(completedPlan, pending, timedMotion);
      mark("clearanceReport");
      var check = planCheck;
      if (check != null && check.checks()) {
        var result = check.check(completedPlan, pending.opIndex, pending.feed);
        result.locate(pending.times, pending.opDistances());
        completedPlan.checked = result;
        if (check.options.rejects && result.diagnostics.length > 0)
          throw 'plan check: ${[for (diagnostic in result.diagnostics) diagnostic.describe()].join("; ")}';
      }
      mark("planCheck");
      if (profiling) Sys.println("PROCESS_PATH_COMPILE_CHECKS " + haxe.Json.stringify({op:pending.opIndex, stages:profileRows}));
      pending.trajectory.dispose();
      transforms.dispose();
      return completedPlan;
    } catch (error:Dynamic) {
      if (plan != null) plan.dispose();
      if (motion != null) motion.dispose();
      throw 'Motion program op ${pending.opIndex}: $error';
    }
  }

  /** The lower (or upper) position bound each joint is claimed to keep to, unbounded where none is claimed. */
  function positionBounds(count:Int, upper:Bool):Array<Float> {
    var claims = limits.nativeValue();
    return [for (joint in 0...count) claims.get_position_claimed(joint) == 0 ? (upper ? 1e300 : -1e300) :
      (upper ? claims.get_position_upper(joint) : claims.get_position_lower(joint))];
  }

  /**
   * A planned joint move (COLLISION.md CL7): the motion planner's waypoints,
   * each edge timed rest to rest with phase synchronization, so the timed path
   * runs along the straight edges that were checked. Validation (CL4b) then
   * checks the whole motion again.
   */
  function plannedMove(from:Array<Float>, to:Array<Float>, velocity:Array<Float>, acceleration:Array<Float>,
      jerk:Array<Float>, index:Int):Trajectory {
    var planner = motionPlanner;
    if (planner == null) throw 'Motion program op $index asks for a planned move, but the compiler has no motion planner';
    if (motorSpace != null) throw 'Motion program op $index: planned moves in motor space are not supported';
    var plan = planner.plan(from, to);
    var path = plan.waypoints;
    if (path == null) throw 'Motion program op $index has no planned path: ${plan.failure}';
    var timing = new JointPathTiming([for (joint in 0...velocity.length)
      {velocity: velocity[joint], acceleration: acceleration[joint], jerk: jerk[joint]}]);
    return timing.trajectory(path);
  }

  /** The last op of the planned joint moves joined by fly-bys from `index`, or `index` when none can fly by. */
  function flyByChainEnd(program:MotionProgram, index:Int):Int {
    if (motionPlanner == null || flyByCertificate == null || motorSpace != null) return index;
    var end = index;
    while (end + 1 < program.ops.length) {
      var joins = switch program.ops[end] {
        case MoveJ(JointTarget(_), options, FlyBy): options.planned;
        case _: false;
      }
      if (!joins) break;
      var next = switch program.ops[end + 1] {
        case MoveJ(JointTarget(_), options, _): options.planned;
        case _: false;
      }
      if (!next) break;
      end++;
    }
    return end;
  }

  /**
   * Planned joint moves `first` to `last` as one motion: each edge planned and certified as for
   * a single planned move, the joined path timed through its waypoints by
   * `JointPathTiming.blended`, which certifies the curved motion near each waypoint with
   * `flyByCertificate` or stops there. Uses the strictest limits of the joined moves.
   */
  function flyBy(program:MotionProgram, first:Int, last:Int, start:Array<Float>):PendingMotion {
    var planner = motionPlanner, space = flyByCertificate;
    if (planner == null || space == null) throw 'Motion program op $first fly-by needs a motion planner and its space';
    var path = [start.copy()], at = start;
    var velocity = maxVelocity.copy(), acceleration = maxAcceleration.copy(), jerk = maxJerk.copy();
    for (index in first...last + 1) switch program.ops[index] {
      case MoveJ(target, options, _):
        var goal = resolveTarget(target, at, index);
        checkJointPosition(goal, index, null);
        var v = effective(maxVelocity, options.maxVelocity), a = effective(maxAcceleration, options.maxAcceleration),
          j = effective(maxJerk, options.maxJerk);
        for (joint in 0...velocity.length) {
          velocity[joint] = Math.min(velocity[joint], v[joint]);
          acceleration[joint] = Math.min(acceleration[joint], a[joint]);
          jerk[joint] = Math.min(jerk[joint], j[joint]);
        }
        var plan = planner.plan(at, goal);
        var waypoints = plan.waypoints;
        if (waypoints == null) throw 'Motion program op $index has no planned path: ${plan.failure}';
        for (waypoint in 1...waypoints.length) path.push(waypoints[waypoint]);
        at = goal;
      case _: throw 'Motion program op $index cannot fly by';
    }
    var timing = new JointPathTiming([for (joint in 0...velocity.length)
      {velocity: velocity[joint], acceleration: acceleration[joint], jerk: jerk[joint]}]);
    return new PendingMotion(first, start, at, timing.blended(path, space), [], null, null);
  }

  function hasClearanceEvents(op:Int):Bool {
    for (event in clearanceEvents) if (event.op == op) return true;
    return false;
  }

  function resetClearanceScene():Void {
    if (clearanceEvents.length == 0 || jointPathPlanner == null) return;
    var world = jointPathPlanner.clearanceWorld();
    if (world != null && Std.isOfType(world, robotkit.manipulation.ClearanceScene)) {
      var scene:robotkit.manipulation.ClearanceScene = cast world;
      scene.reset();
    }
  }

  /**
   * Records the clearance check in the plan's report (COLLISION.md CL4b). A refined process path
   * certified by `JointCurveClearance` is a bound. Any other motion (a joint move, a generated
   * entry, exit or hold) gets `TrajectoryClearanceProof`: a bound when it closes every interval,
   * else sampled (the sampled check above passed) with the first open interval named; a real
   * violation it finds between samples is recorded and throws. Without a clearance world the check
   * stays unchecked.
   */
  function recordClearance(plan:ExecutionPlan, pending:PendingMotion, motion:Trajectory):Void {
    var planner = jointPathPlanner;
    if (planner == null) return;
    var world = planner.clearanceWorld();
    if (world == null) return;
    var duration = motion.durationSeconds();
    if (pending.geometryClearance) {
      var closest:Null<robotkit.manipulation.ClearanceViolation> = null, when = 0.0;
      for (i in 0...8) {
        var t = duration * (i + 0.5) / 8, q = motion.evaluate(t).positions;
        var found = world.closest(q, false);
        if (found != null && (closest == null || found.distance < (cast closest : robotkit.manipulation.ClearanceViolation).distance)) {
          closest = found;
          when = t;
        }
      }
      setClearance(plan, TrajectoryCoreConstants.MK_CHECK_PASSED, TrajectoryCoreConstants.MK_CHECK_METHOD_BOUND, closest, when,
        0.0, duration);
      return;
    }
    var events = [for (event in clearanceEvents) if (event.op == pending.opIndex) event];
    var proof = planner.proveMotion(motion, velocityBounds(motion), events, clearanceDepthLimit);
    if (proof == null) return;
    var failure = proof.failure;
    if (failure != null) {
      setClearance(plan, TrajectoryCoreConstants.MK_CHECK_FAILED, TrajectoryCoreConstants.MK_CHECK_METHOD_BOUND, failure,
        proof.failureTime, proof.failureTime, proof.failureTime);
      throw 'trajectory clearance bound (${failure.a}, ${failure.b}): ${failure.distance} < ${failure.required} at ${proof.failureTime} s';
    }
    if (proof.closed)
      setClearance(plan, TrajectoryCoreConstants.MK_CHECK_PASSED, TrajectoryCoreConstants.MK_CHECK_METHOD_BOUND, proof.closest,
        proof.closestTime, 0.0, duration);
    else
      setClearance(plan, TrajectoryCoreConstants.MK_CHECK_PASSED, TrajectoryCoreConstants.MK_CHECK_METHOD_SAMPLED, proof.closest,
        proof.closestTime, proof.openStart, proof.openEnd, Int64.ofInt(10000000));
  }

  static function setClearance(plan:ExecutionPlan, status:Int, method:Int, pair:Null<robotkit.manipulation.ClearanceViolation>,
      time:Float, start:Float, end:Float, ?resolutionNs:Int64):Void {
    var resolution = resolutionNs == null ? Int64.ofInt(0) : resolutionNs;
    if (pair == null) plan.report.setCollision(status, method, 0.0, 0.0, time, start, end, resolution, "", "");
    else plan.report.setCollision(status, method, pair.distance, pair.required, time, start, end, resolution, pair.a, pair.b);
  }

  /**
   * Each joint's largest speed over the motion, bounded from its polynomials: on a segment of
   * duration d, |p'(t)| <= sum over k of k |c_k| d^(k-1).
   */
  static function velocityBounds(motion:Trajectory):Array<Float> {
    var bounds:Array<Float> = [];
    for (segment in motion.segments()) {
      var d = Int64.toFloat(segment.durationNs) / 1e9;
      for (joint in 0...segment.coefficients.length) {
        var c = segment.coefficients[joint], speed = 0.0, power = 1.0;
        for (k in 1...c.length) {
          speed += k * Math.abs(c[k]) * power;
          power *= d;
        }
        if (joint >= bounds.length) bounds.push(speed) else bounds[joint] = Math.max(bounds[joint], speed);
      }
    }
    return bounds;
  }

  /** Keep the follower polynomial exact after independent Hermite lowering. */
  function projectCouplings(source:Trajectory):Trajectory
    return projection.trajectory(source);

  /** Sets every coupled joint to the sum of its terms, in dependency order. */
  function completeCoupled(q:Array<Float>):Void projection.complete(q);

  function resolveTarget(target:MoveTarget, start:Array<Float>, index:Int):Array<Float> {
    return switch target {
      case JointTarget(joints):
        if (joints.length != solver.jointCount())
          throw 'Motion program op $index joint count mismatch';
        joints.copy();
      case JointOffset(joint, delta):
        if (joint >= start.length) throw 'Motion program op $index joint offset names a joint the machine does not have';
        var moved = start.copy();
        moved[joint] += delta;
        completeCoupled(moved);
        moved;
      case PoseTarget(pose, requestedFrame, _):
        requireFrame(requestedFrame, index);
        var candidates:Array<Array<Float>>;
        if (configurationConstraint != null) {
          candidates = configurationConstraint.candidates(pose, start);
        } else {
          // Preserve the locally reached branch before enumerating alternatives.
          var seeded = solver.solvePose(pose, start, ikTolerance, null);
          if (seeded != null) return seeded;
          candidates = solver.sampleCandidates(pose, 32, ikTolerance, null);
        }
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
      ?authoredPolyline:Array<Pose3>, ?blendTolerance:Float = 0.0,
      ?preplannedCurve:JointPathSamples, eventOffset:Float=0.0, eventLast:Bool=true):PendingMotion {
    if (path.length() <= 0.0) throw 'Motion program op $index has zero path length';
    if (path.drive != null && (jointPathPlanner == null || !jointPathPlanner.appliesDistanceDrive()))
      throw 'Motion program op $index drives a joint by path distance, which this machine\'s joint path planner does not apply';
    var samples = pathSamples(path);
    var exactMap = solver.affineMap();
    var count = samples.length - 1;
    if (count > (exactMap == null ? MAX_SECTION_SAMPLES : MAX_AFFINE_SECTION_SAMPLES))
      throw 'Motion program op $index exceeds Cartesian sample budget';
    var distances = [for (sample in samples) sample.distance];
    var positions:Array<Array<Float>> = [];
    var caps:Array<Float> = [];
    var previous = startQ.copy();
    var pathPoses = [for (sample in samples) sample.primitive.waypointAt(sample.local).pose];
    var freedoms = [for (sample in samples) sample.primitive.orientationPolicy()];
    var selected:Array<Null<Array<Float>>> = [];
    var redundancyRates:Null<Array<Array<Float>>> = null;
    var refined:Null<JointPathSamples> = null;
    if(jointPathPlanner!=null){
      refined=preplannedCurve != null ? preplannedCurve :
        jointPathPlanner.plan(path,new PathRequest(distances,pathPoses,startQ,ikTolerance,perJointMaxJump,maxVelocity,48,freedoms),true);
      if(refined==null || refined.jointCount!=startQ.length || refined.s.length!=distances.length)
        throw 'Motion program op $index joint planner returned incompatible samples';
      for(sample in 0...distances.length)if(refined.s[sample]!=distances[sample])
        throw 'Motion program op $index joint planner changed path distance at sample $sample';
      for(joint in 0...startQ.length)if(Math.abs(refined.q[0][joint]-startQ[joint])>1e-7)
        throw 'Motion program op $index joint planner changed the pinned start';
      for(q in refined.q)selected.push(q);
    } else {
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
      checkJointPosition(solved, index, distance, startQ);
      if (sample > 0) {
        // An exact affine map has one solution per pose: no branch to flip, and a long piece is one span.
        if (exactMap == null) for (joint in 0...startQ.length)
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
    if(refined==null)for (k in 0...samples.length) {
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
    var jointPath = refined==null ? new JointPathSamples(distances, positions, first, second, secondBefore) : refined;
    var geometryClearance=false;
    if(jointPathPlanner!=null && Std.isOfType(timing,ToppraPathTiming) && projection==null){
      var nativeTiming:ToppraPathTiming=cast timing;
      geometryClearance=jointPathPlanner.checkPathClearance(jointPath,nativeTiming.loweringTolerance);
    }
    var timingLimits = new PathTimingLimits(maxVelocity, maxAcceleration, caps,0,0,requireFeasiblePath!=null && requireFeasiblePath(index));
    var motors = motorSpace;
    var timingBegan = Sys.time();
    var timed = motors == null ? timing.time(jointPath, timingLimits) : motors.time(timing, jointPath, timingLimits);
    if (Sys.getEnv("PROCESS_PATH_PROFILE") == "1") Sys.println("PROCESS_PATH_TIME_LAW " + haxe.Json.stringify({op:index, seconds:Sys.time()-timingBegan}));
    try {
      var events:Array<TimedEvent> = [];
      var timeMap = [for (distance in distances) timed.distanceToTime(distance)];
      var schedule=pathEventSchedule;
      if(schedule!=null){
        var scheduled=schedule(index,eventOffset,eventLast,[for(distance in distances)distance+eventOffset],
          timeMap.copy(),[for(event in authoredEvents)new PathEvent(event.distance+eventOffset,
            event.channel,event.value,event.leadSeconds,event.holdPolicy)]);
        if(scheduled==null)throw "Path event schedule must return events";
        authoredEvents=[];
        for(event in scheduled){
          if(event==null || event.distance<eventOffset || event.distance>eventOffset+path.length())
            throw "Scheduled event lies outside its timed path section";
          authoredEvents.push(new PathEvent(Math.min(path.length(),Math.max(0.0,event.distance-eventOffset)),
            event.channel,event.value,event.leadSeconds,event.holdPolicy));
        }
      }
      for (event in authoredEvents) {
        var seconds = Math.max(0.0,
          timed.distanceToTime(event.distance) - event.leadSeconds);
        events.push(new TimedEvent(Trajectory.nanoseconds(seconds), event.channel,
          event.value, event.holdPolicy));
      }
      // The task-space check inspects the path between samples too. Time the inspected distances exactly:
      // interpolating between sample times is wrong where the path speed changes fast, e.g. braking to rest
      // over the last sample interval, where distance goes with the square of time.
      var checkSamples = authoredPolyline == null ? distances.length * 2 - 1 :
        Std.int(Math.max(distances.length * 2 - 1, Math.ceil(path.length() / (blendTolerance / 8.0)) + 1));
      var checkDistances = [for (sample in 0...checkSamples) path.length() * sample / (checkSamples - 1)];
      var checkTimes = [for (distance in checkDistances) timed.distanceToTime(distance)];
      // Axis machines map a path to joints exactly, so the executed position can differ from the path's only by
      // the lowering tolerance, which the lowering itself bounds: the clock need not be sampled every millisecond
      // (a long print has millions), the path's own samples above are checked.
      var timeSteps = solver.affineMap() != null ? 1 : Std.int(Math.max(1,
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
      made.geometryClearance=geometryClearance;made.geometryCurve=jointPath;made.geometryRefined=refined!=null;
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
      taskSampleDistances:Array<Float>,?certificate:JointCurveTask, ?lag:Float = 0.0, ?tail:Float = 0.0, ?spread:Float = 0.0):Void {
    var worst = 0.0, worstTime = 0.0,worstOrientation=0.0;
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
      var allowed = spread + (authoredPolyline != null ? blendTolerance :
        path.authoredGeometry != null ? path.blendTolerance :
          desired.positionTolerance);
      var angle = orientationError(actual, desired.pose,
        path.orientationPolicyAt(distance));
      worstOrientation=Math.max(worstOrientation,angle);
      if(certificate!=null && (error>certificate.positionBound+1e-9 || angle>certificate.orientationBound+1e-9))
        throw 'Task certificate audit exceeded its continuous bound at op $index: $error / ${certificate.positionBound}, $angle / ${certificate.orientationBound}';
      if (error > worst) { worst = error; worstTime = time; tolerance = allowed; }
      if (error > allowed + 1e-9 ||
          angle > desired.orientationTolerance + 1e-9)
        failure = 'Motion program op $index task-space tolerance exceeded at path distance $distance (position $error / $allowed, orientation $angle / ${desired.orientationTolerance})';
    }
    for (sample in 0...checkDistances.length) inspect(checkDistances[sample], checkTimes[sample]);
    // Cover the trajectory clock as well as the authored path geometry.
    var timeSteps = taskSampleDistances.length - 1;
    for (sample in 0...(timeSteps + 1)) {
      // The lowered motion's clock: the shaped plan runs `lag` behind it and `tail` longer.
      var time = lag + (plan.durationSeconds - tail) * sample / timeSteps;
      inspect(taskSampleDistances[sample], time);
    }
    plan.report.setTaskSpace(failure == null ? TrajectoryCoreConstants.MK_CHECK_PASSED :
      TrajectoryCoreConstants.MK_CHECK_FAILED, worst, worstTime, tolerance,
      Trajectory.nanoseconds(plan.durationSeconds / timeSteps));
    if(certificate!=null)Sys.println("PROCESS_PATH_TASK_AUDIT "+haxe.Json.stringify({op:index,
      samples:checkDistances.length+timeSteps+1,positionError:worst,positionBound:certificate.positionBound,
      orientationError:worstOrientation,orientationBound:certificate.orientationBound}));
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

  static function distanceToGeometry(point:Pose3, geometry:GeometricPath):Float
    return geometry.distanceTo(new motionkit.path.PathPoint(point.x, point.y, point.z));

  function requireFrame(requested:String, index:Int):Void {
    if (requested != frameId)
      throw 'Motion program op $index frame "$requested" does not match "$frameId"';
  }
  /**
    A joint may sit at its `start` position even just outside its bound, within the start tolerance
    (a stepper homed onto a limit comes to rest a step or two past it): the validator's rule, ValidationLimits.admittedBounds.
  **/
  function checkJointPosition(q:Array<Float>, index:Int,
      distance:Null<Float>, ?start:Array<Float>):Void {
    var nativeLimits = limits.nativeValue();
    for (joint in 0...q.length)
      if (nativeLimits.get_position_claimed(joint) != 0) {
        var lower = nativeLimits.get_position_lower(joint), upper = nativeLimits.get_position_upper(joint);
        // Only a position outside the claimed bounds can need the start-at-limit rule (a call into the validator).
        if (start != null && (q[joint] < lower - 1e-9 || q[joint] > upper + 1e-9)) {
          var bounds = limits.admittedBounds(joint, start[joint], startTolerances.position[joint]);
          lower = bounds.lower; upper = bounds.upper;
        }
        if (q[joint] < lower - 1e-9 || q[joint] > upper + 1e-9) {
          var location = distance == null ? "" : ' at path distance $distance';
          throw 'Motion program op $index joint limit $joint$location';
        }
      }
  }
  /**
    Most samples one timed section holds. An arm's samples each cost an inverse-kinematics search, so its budget is
    small; on a machine whose joints are an exact affine map a straight piece is one sample, and what a section costs is
    native memory (a sample is 2 KB).
  **/
  public static inline final MAX_SECTION_SAMPLES = 10000;
  public static inline final MAX_AFFINE_SECTION_SAMPLES = 100000;

  /**
    `path` split at its sharp corners, where one primitive's end tangent is not the next one's start tangent (authored
    task-velocity discontinuities, each timed from rest to rest), with each section's distance along it.
  **/
  public static function timingSections(path:PosePath):Array<{path:PosePath, offset:Float}> {
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
        // A path rests only where it turns: a long smooth one is timed in windows that carry their boundary speeds.
        if (turn > 1e-6) {
          sections.push({path: path.sub(start, k), offset: sectionOffset});
          start = k;
          sectionOffset = offset;
        }
      }
      offset += primitive.length();
    }
    sections.push({path: path.sub(start, path.primitives.length), offset: sectionOffset});
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
    // On a machine whose joints are an exact affine map of the pose a straight piece needs no samples inside it.
    var exact = solver.affineMap() != null;
    var samples:Array<PathSample> = [{distance: 0.0, primitive: first, local: 0.0, arriving: null}];
    var start = 0.0;
    // A distance-driven joint changes its rate's slope at its stations: they are sampled too.
    var stations = path.drive == null ? [] : path.drive.stations();
    var stationAt = 0;
    for (k in 0...path.primitives.length) {
      var primitive = path.primitives[k], length = primitive.length();
      var pieces = motionkit.path.PoseSampling.pieces(primitive,cartesianResolution,angularResolution,exact);
      var locals = [for (piece in 1...pieces) length * piece / pieces];
      while (stationAt < stations.length && stations[stationAt] <= start + 1e-12) stationAt++;
      var extra = stationAt;
      while (extra < stations.length && stations[extra] < start + length - 1e-12) { locals.push(stations[extra] - start); extra++; }
      if (extra > stationAt) locals.sort(Reflect.compare);
      // Exactly the end: `length * pieces / pieces` can round one ulp past it.
      locals.push(length);
      var previous = 0.0;
      for (index in 0...locals.length) {
        var local = locals[index], last = index == locals.length - 1;
        if (!last && local - previous <= 1e-12) continue;
        previous = local;
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

  static function primitiveSpeedAt(path:PosePath, distance:Float):Float
    return path.primitives[path.indexAt(distance)].speedLimit();
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
      case FlyBy: sink.note('Motion program op $index fly-by uses exact stop');
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
  public var geometryClearance:Bool = false;
  public var geometryRefined:Bool = false;
  public var geometryCurve:Null<JointPathSamples> = null;

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
