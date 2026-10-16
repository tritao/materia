package motionkit.robot;

import haxe.Int64;
import TrajectoryCore;
import motionkit.kinematics.KinematicsSolver;
import motionkit.kinematics.Pose3;
import motionkit.path.GeometricPath;
import motionkit.path.PoseMath;
import motionkit.path.PosePath;
import motionkit.planner.JointPathSamples;
import motionkit.planner.PathTimingBackend;
import motionkit.planner.ToppraPathTiming;
import motionkit.robot.ProgramCompiler.PendingMotion;
import motionkit.robot.TimedMotionTransforms.TimedMotion;
import motionkit.robot.TrajectoryClearanceProof.ClearanceEvent;
import motionkit.trajectory.ExecutionPlan;
import motionkit.trajectory.Trajectory;
import motionkit.trajectory.ValidationLimits;

/** Typed settings borrowed for synchronous certification; the stage owns none of these inputs. */
typedef PlanCertificationContext = {
  var solver:KinematicsSolver;
  var limits:ValidationLimits;
  var timing:PathTimingBackend;
  var projection:Null<CouplingProjection>;
  var motorSpace:Null<MotorSpaceConstraints>;
  var jointPathPlanner:Null<JointPathPlanner>;
  var configurationConstraint:Null<ConfigurationConstraint>;
  var controllerPeriodSeconds:Float;
  var startTolerances:StartTolerances;
  var positionTolerance:Float;
  var clearanceEvents:Array<ClearanceEvent>;
  var clearanceDepthLimit:Int;
  var planCheck:Null<PlanCheck>;
  var shapingDelay:Float;
  var shapingTail:Float;
  var shapingDeviation:Float;
  /** Record a rejected merge even if a later certification check fails. */
  var mergeRefused:String->Void;
}

/** Certifies transformed motion and owns any execution plan until all checks succeed. */
class PlanCertification {
  final context:PlanCertificationContext;

  public function new(context:PlanCertificationContext) this.context = context;

  /** Borrows pending/transformed trajectories; returns the caller-owned certified plan. */
  public function certify(pending:PendingMotion, transforms:TimedMotion, id:Int64,
      mark:String->Void, profiling:Bool):ExecutionPlan {
    var timedMotion = transforms.executed, shaped = transforms.shaped;
    var plan:Null<ExecutionPlan> = null;
    try {
      if(pending.geometryCurve!=null && pending.geometryClearance && Sys.getEnv("PROCESS_PATH_VERIFY_CLEARANCE")=="1"){
        var curve:JointPathSamples=cast pending.geometryCurve;
        if(Std.isOfType(curve.clearanceProof,JointCurveClearance)){var proof:JointCurveClearance=cast curve.clearanceProof;
          proof.auditTrajectory(curve,pending.trajectory,pending.taskSampleDistances);}
      }
      // With changes to the cell in this op, the conservative check below replays them instead.
      if (context.jointPathPlanner != null && (!pending.geometryClearance || Sys.getEnv("PROCESS_PATH_VERIFY_CLEARANCE")=="1") &&
          !hasClearanceEvents(pending.opIndex)) {
        // What is checked is what is executed: the shaped motion, when there is shaping.
        var failure = context.jointPathPlanner.checkMotion(timedMotion);
        if (failure != null)
          throw 'compiled trajectory clearance (${failure.a}, ${failure.b}): ${failure.distance} < ${failure.required}';
      }
      mark("trajectoryClearance");
      if(context.configurationConstraint!=null)context.configurationConstraint.checkTrajectory(timedMotion,context.controllerPeriodSeconds);
      mark("configurationCheck");
      var planLimits = pending.path == null ? context.limits : context.limits.withoutJerk();
      var mergedPlan:Null<ExecutionPlan> = null;
      // A merged path follows what it replaces closely but not exactly: should it break a limit anyway, it is a
      // refusal, and the path goes unmerged (`transforms.executed`: as lowered, or shaped from the pre-shape merge).
      if (transforms.merged != null) {
        try {
          mergedPlan = ExecutionPlan.create(transforms.merged, planLimits, id, pending.startQ, pending.startVelocity, pending.startAcceleration,
            context.startTolerances.position, context.startTolerances.velocity, context.startTolerances.acceleration, pending.events);
        } catch (error:Dynamic) {
          context.mergeRefused(Std.string(error));
          transforms.dropMerged();
        }
      }
      var completedPlan:ExecutionPlan = mergedPlan != null ? mergedPlan : ExecutionPlan.create(timedMotion,
        planLimits, id, pending.startQ,
        pending.startVelocity, pending.startAcceleration, context.startTolerances.position, context.startTolerances.velocity,
        context.startTolerances.acceleration, pending.events);
      completedPlan.endsAtRest = !pending.continues;
      plan=completedPlan;
      mark("executionPlanCreation");
      var taskProof:Null<JointCurveTask> = null;
      if(pending.path!=null && pending.geometryRefined && pending.geometryCurve!=null &&
          pending.authoredPolyline==null && Std.isOfType(context.timing,ToppraPathTiming) &&
          context.projection==null && context.motorSpace==null){
        var arm:Null<robotkit.manipulation.KinematicGroup> = null;
        if(Std.isOfType(context.solver,ManipulatorKinematics))arm=cast(context.solver,ManipulatorKinematics).manipulator;
        else if(Std.isOfType(context.solver,EaikKinematics))arm=cast(context.solver,EaikKinematics).manipulator;
        if(arm!=null){var nativeTiming:ToppraPathTiming=cast context.timing;
          taskProof=JointCurveTask.prove(arm,pending.path,pending.geometryCurve,nativeTiming.loweringTolerance);}
      }
      mark("taskSpaceCertificate");
      var auditTask=Sys.getEnv("PROCESS_PATH_VERIFY_TASK")=="1";
      if(pending.path!=null && (taskProof==null || auditTask))checkTaskSpace(completedPlan,pending.path,pending.checkDistances,
        pending.checkTimes,pending.opIndex,pending.authoredPolyline,pending.blendTolerance,pending.taskSampleDistances,taskProof,
        shaped && !pending.filtered ? context.shapingDelay : 0.0, shaped && !pending.filtered ? context.shapingTail : 0.0, shaped ? context.shapingDeviation : 0.0);
      if(taskProof!=null)completedPlan.report.setTaskSpaceBound(taskProof.positionBound,taskProof.positionTolerance);
      if(profiling && pending.path!=null)Sys.println("PROCESS_PATH_TASK_CERTIFICATE "+haxe.Json.stringify({
        op:pending.opIndex,certified:taskProof!=null,audited:auditTask && taskProof!=null,
        queries:taskProof==null?0:taskProof.queries,positionBound:taskProof==null?null:taskProof.positionBound,
        orientationBound:taskProof==null?null:taskProof.orientationBound}));
      mark("taskSpaceCheck");
      recordClearance(completedPlan, pending, timedMotion);
      mark("clearanceReport");
      var check = context.planCheck;
      if (check != null && check.checks()) {
        var result = check.check(completedPlan, pending.opIndex, pending.feed);
        result.locate(pending.times, pending.opDistances());
        completedPlan.checked = result;
        if (check.options.rejects && result.diagnostics.length > 0)
          throw 'plan check: ${[for (diagnostic in result.diagnostics) diagnostic.describe()].join("; ")}';
      }
      mark("planCheck");
      return completedPlan;
    } catch (error:Dynamic) {
      if (plan != null) plan.dispose();
      throw error;
    }
  }

  function hasClearanceEvents(op:Int):Bool {
    for (event in context.clearanceEvents) if (event.op == op) return true;
    return false;
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
    var planner = context.jointPathPlanner;
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
    var events = [for (event in context.clearanceEvents) if (event.op == pending.opIndex) event];
    var proof = planner.proveMotion(motion, velocityBounds(motion), events, context.clearanceDepthLimit);
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

  /** `checkDistances` along the path, each at its exact `checkTimes`, then the trajectory clock at 1 ms. */
  function checkTaskSpace(plan:ExecutionPlan, path:PosePath,
      checkDistances:Array<Float>, checkTimes:Array<Float>, index:Int,
      authoredPolyline:Null<Array<Pose3>>, blendTolerance:Float,
      taskSampleDistances:Array<Float>,?certificate:JointCurveTask, ?lag:Float = 0.0, ?tail:Float = 0.0, ?spread:Float = 0.0):Void {
    var worst = 0.0, worstTime = 0.0,worstOrientation=0.0;
    var tolerance = authoredPolyline == null && path.authoredGeometry == null ?
      context.positionTolerance : authoredPolyline == null ? path.blendTolerance : blendTolerance;
    var failure:Null<String> = null;
    function inspect(distance:Float, time:Float):Void {
      var desired = path.waypointAt(distance);
      var actual = context.solver.forward(plan.evaluate(time).positions);
      var authoredBound:Null<Float> = null;
      if (authoredPolyline==null && path.authoredGeometry!=null && path.geometryDeviationBound!=null) {
        // Triangle inequality: the authored corner certificate plus the error
        // against this exact path point bounds distance to the authored path.
        // An inconclusive bound retains the original nearest-geometry query.
        var scale=Math.max(1.0,Math.max(Math.max(Math.abs(actual.x),Math.abs(actual.y)),Math.abs(actual.z)));
        var deviation:Float=cast path.geometryDeviationBound;
        var bound=PoseMath.distance(actual,desired.pose)+deviation+64*2.220446049250313e-16*scale;
        if (bound<=path.blendTolerance) authoredBound=bound;
      }
      var error = authoredBound!=null ? authoredBound : authoredPolyline != null ?
        Math.min(distanceToSegment(actual, authoredPolyline[0], authoredPolyline[1]),
          distanceToSegment(actual, authoredPolyline[1], authoredPolyline[2])) :
        path.authoredGeometry != null ?
          distanceToGeometry(actual, path.authoredGeometry) :
          PoseMath.distance(actual, desired.pose);
      var allowed = spread + (authoredPolyline != null ? blendTolerance :
        path.authoredGeometry != null ? path.blendTolerance :
          desired.positionTolerance);
      var angle = ToolFreedom.orientationError(actual, desired.pose,
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

  function zeros():Array<Float> return [for (_ in 0...context.solver.jointCount()) 0.0];
}
