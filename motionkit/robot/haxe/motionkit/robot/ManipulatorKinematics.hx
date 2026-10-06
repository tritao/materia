package motionkit.robot;

import motionkit.path.OrientationPolicy;
import motionkit.path.PoseMath;

import kinematicskit.LinearAlgebra;
import motionkit.kinematics.IkTolerance;
import motionkit.kinematics.KinematicsSolver;
import motionkit.kinematics.PathRequest;
import motionkit.kinematics.PathSolution;
import motionkit.kinematics.RedundantPathSolver;
import motionkit.kinematics.Pose3;
import motionkit.kinematics.Twist6;
import robotkit.manipulation.IKResult;
import robotkit.manipulation.IkOptions;
import robotkit.manipulation.KinematicGroup;
import robotkit.spatial.Quat;
import robotkit.spatial.Transform3;
import robotkit.spatial.Vec3;

/**
 * KinematicsSolver adapter over RobotKit's deterministic DLS manipulator.
 *
 * A redundant arm (`Manipulator.redundant`, e.g. 7-axis) is handled through
 * its swivel angle: `solvePose` keeps the seed's swivel as a preference, so
 * the elbow does not drift with each solve; `sampleCandidates` sweeps the
 * swivel around the circle on each IK branch it finds; and `solvePath`
 * chooses the swivel (or a cell's external-axis values) along a whole path
 * with `RedundancyResolver`.
 */
class ManipulatorKinematics implements RedundantPathSolver {
  /** The group solved: an arm (`Manipulator`), or an arm with external axes and a work frame. */
  public final manipulator:KinematicGroup;
  /**
   * A posture the solves draw the arm towards (`q` order): with external
   * axes, the arm stays comfortable and they bring the work to it.
   */
  public var preferredPosture:Null<Array<Float>> = null;
  /** Soft tool orientation preference for reduced tasks; hard rows always win. */
  public var preferredOrientation:Null<Pose3> = null;
  /** Prefer each authored orientation softly while preserving the hard task freedom. */
  public var preferTargetOrientation:Bool = false;
  public final differentialDamping:Float;
  /** Residuals from the last failed solve, for planner diagnostics. */
  public var lastFailure:Null<String> = null;
  /** True when the hard tool task leaves no joint-space null direction. */
  function fullyConstrained(q:Array<Float>, freedom:OrientationPolicy):Bool {
    switch freedom {
      case Free | Cone(_, _): return false;
      default:
    }
    var n = jointCount();
    var projection = ToolFreedom.twistRows(forward(q), freedom);
    if (projection.length < n) return false;
    var jacobian = manipulator.tcpJacobian(q);
    var reduced:Array<Float> = [];
    for (row in projection) for (joint in 0...n) {
      var value = 0.0;
      for (axis in 0...6) value += row[axis] * jacobian[axis*n + joint];
      reduced.push(value);
    }
    // Normalize columns before the rank test so metres and radians do not
    // choose the numerical threshold for each other.
    for (joint in 0...n) {
      var norm = 0.0;
      for (row in 0...projection.length) norm += reduced[row*n + joint] * reduced[row*n + joint];
      if (!(norm > 1e-20)) return false;
      norm = Math.sqrt(norm);
      for (row in 0...projection.length) reduced[row*n + joint] /= norm;
    }
    var normal = [for (_ in 0...n*n) 0.0], rhs = [for (_ in 0...n) 0.0];
    LinearAlgebra.normalEquationsDense(reduced, projection.length, n,
      [for (_ in 0...projection.length) 0.0], normal, rhs);
    return LinearAlgebra.solveInPlace(normal, rhs, n, [for (_ in 0...n) 0.0], 1e-10);
  }

  /** How the group's redundancy is named; null for a group without any. */
  final parameterization:Null<RedundancyParameterization>;
  final reachBound:Null<RevoluteReachBound>;

  public function new(manipulator:KinematicGroup, ?differentialDamping:Float = 1e-6) {
    if (manipulator == null) throw "Manipulator kinematics requires a manipulator";
    if (!Math.isFinite(differentialDamping) || differentialDamping <= 0.0)
      throw "Differential IK damping must be finite and positive";
    this.manipulator = manipulator;
    this.differentialDamping = differentialDamping;
    reachBound = RevoluteReachBound.of(manipulator);
    parameterization = manipulator.redundant() ? new SwivelParameterization(manipulator)
      : manipulator.external.indexOf(true) >= 0 ? new ExternalAxesParameterization(manipulator) : null;
  }

  /** The same kinematics with its own posture preference; the group is shared, as it is safe to. */
  public function fork():KinematicsSolver {
    var copy = new ManipulatorKinematics(manipulator, differentialDamping);
    copy.preferredPosture = preferredPosture == null ? null : preferredPosture.copy();
    copy.preferredOrientation = preferredOrientation;
    copy.preferTargetOrientation = preferTargetOrientation;
    return copy;
  }

  public function jointCount():Int return manipulator.dofCount();

  public function forward(q:Array<Float>):Pose3 return fromTransform(manipulator.tcpPose(q));

  public function solvePose(target:Pose3, seed:Array<Float>,
      tolerance:IkTolerance, ?freedom:OrientationPolicy):Null<Array<Float>> {
    requireTolerance(tolerance);
    lastFailure = null;
    var task = ToolFreedom.of(target, freedom, tolerance.orientation);
    if (seed != null && seed.length != jointCount())
      throw 'Kinematic group requires ${jointCount()} values, got ${seed.length}';
    var toolAxisFixed = switch freedom { case FreeAboutTool: true; default: false; };
    if (reachBound != null && reachBound.excludes(toTransform(target), tolerance.position,
        tolerance.orientation, ToolFreedom.isFull(freedom), toolAxisFixed)) {
      lastFailure = "Tool pose is outside the revolute chain reach bound";
      return null;
    }
    var preference = preferredOrientation;
    if (!ToolFreedom.isFull(freedom) && preference != null) {
      var preferred = PoseMath.angle(target, preference) <= tolerance.orientation ? target
        : new Pose3(target.x, target.y, target.z, preference.qx, preference.qy, preference.qz, preference.qw);
      if (ToolFreedom.orientationError(preferred, target, freedom) <= tolerance.orientation) {
        // Zero preference error is optimal. If it is unreachable, solve only the hard rows.
        var exact = solvePose(preferred, seed, tolerance);
        if (exact != null) return exact;
      }
    }
    if (manipulator.redundant() && seed != null) {
      var swivel = manipulator.swivelAngle(seed);
      if (Math.isFinite(swivel)) {
        var kept = solveAtSwivel(toTransform(target), seed, swivel, false, tolerance, freedom);
        if (kept.converged) return kept.q.copy();
      }
    }
    var result = manipulator.solve(toTransform(task.target), seed, options(tolerance, freedom, target));
    if (!result.converged)
      lastFailure = 'tool position residual ${result.positionError} m; orientation residual ${result.orientationError} rad';
    return result.converged ? result.q.copy() : null;
  }

  public function sampleCandidates(target:Pose3, maxCount:Int,
      tolerance:IkTolerance, ?freedom:OrientationPolicy):Array<Array<Float>> {
    requireTolerance(tolerance);
    if (target == null) throw "Candidate sampling requires a target pose";
    if (maxCount < 0) throw "Candidate count must be non-negative";
    var candidates:Array<Array<Float>> = [];
    if (maxCount == 0) return candidates;

    var combinationCount = 1;
    for (_ in 0...jointCount()) {
      if (combinationCount > 4096 / 3) { combinationCount = 4096; break; }
      combinationCount *= 3;
    }
    // A redundant arm finds a few IK branches this way, then sweeps each around its swivel.
    var redundant = manipulator.redundant();
    var branchLimit = redundant ? Std.int(Math.max(1, Math.min(4, maxCount))) : maxCount;
    var attemptLimit = Math.min(combinationCount, Math.max(32, maxCount * 32));
    for (attempt in 0...Std.int(attemptLimit)) {
      var code = attempt;
      var shift = attempt % 3;
      var seed:Array<Float> = [];
      for (joint in 0...jointCount()) {
        var level = code % 3;
        code = Std.int(code / 3);
        // A bijective ternary shear preserves the full lattice while giving
        // high-index joints all levels in each early three-attempt block.
        if (joint > 0) level = (level + shift) % 3;
        var limits = manipulator.group.limitsOf(joint);
        if (limits.lower < limits.upper) {
          var fraction = level == 0 ? 0.5 : (level == 1 ? 0.25 : 0.75);
          seed.push(limits.lower + (limits.upper - limits.lower) * fraction);
        } else {
          seed.push(level == 0 ? 0.0 : (level == 1 ? -Math.PI : Math.PI));
        }
      }
      var solved = solvePose(target, seed, tolerance, freedom);
      if (solved == null || containsNear(candidates, solved, tolerance.candidateSeparation))
        continue;
      candidates.push(solved);
      if (candidates.length >= branchLimit) break;
    }
    // The coarse lattice can miss a branch close to a bounded axis's end.
    // Probe each joint near both ends with the other joints centred. This
    // also covers high-index joints when the product lattice is budgeted.
    for (joint in 0...jointCount()) for (fraction in [0.05, 0.95]) {
      if (candidates.length >= branchLimit) break;
      var seed = [for (i in 0...jointCount()) {
        var limits = manipulator.group.limitsOf(i);
        limits.lower < limits.upper ? limits.lower + (limits.upper - limits.lower) * (i == joint ? fraction : 0.5)
          : (i == joint ? (fraction < 0.5 ? -Math.PI : Math.PI) : 0.0);
      }];
      var solved = solvePose(target, seed, tolerance, freedom);
      if (solved != null && !containsNear(candidates, solved, tolerance.candidateSeparation)) candidates.push(solved);
    }
    return redundant ? sweepSwivel(target, candidates, maxCount, tolerance, freedom) : candidates;
  }

  /** Each branch solved at evenly spaced swivel angles, stepping from its own, each solve seeded by the last. */
  function sweepSwivel(target:Pose3, branches:Array<Array<Float>>, maxCount:Int,
      tolerance:IkTolerance, ?freedom:OrientationPolicy):Array<Array<Float>> {
    var candidates = branches.copy();
    if (branches.length == 0) return candidates;
    var steps = Std.int(Math.max(4, Math.floor(maxCount / branches.length)));
    var goal = toTransform(target);
    for (branch in branches) {
      var start = manipulator.swivelAngle(branch);
      if (!Math.isFinite(start)) continue;
      for (direction in [1.0, -1.0]) {
        var seed = branch;
        for (step in 1...Std.int(steps / 2) + 1) {
          if (candidates.length >= maxCount) return candidates;
          var angle = start + direction * 2.0 * Math.PI * step / steps;
          var solved = solveAtSwivel(goal, seed, angle, true, tolerance, freedom);
          if (!solved.converged) break;
          seed = solved.q;
          if (!containsNear(candidates, solved.q, tolerance.candidateSeparation)) candidates.push(solved.q.copy());
        }
      }
    }
    return candidates;
  }

  /**
   * The path: a redundant arm (7-axis) chooses its swivel along it, a group
   * with external axes their values, both through `RedundancyResolver`; a
   * plain arm follows point by point, each sample seeded by the last.
   */
  public function solvePath(request:PathRequest):Array<Null<Array<Float>>>
    return solvePathWithRates(request).configurations;

  /** The path and, for a redundant group, the exact rate of its redundancy along it (see `RedundancyResolver`). */
  public function solvePathWithRates(request:PathRequest):PathSolution {
    var named = parameterization;
    var freedom = request.freedoms[0];
    var uniform = true;
    for (policy in request.freedoms) if (policy != freedom) uniform = false;
    if (named == null || (uniform && fullyConstrained(request.startQ, freedom)))
      return new PathSolution(request.followPointByPoint(this));
    named.preferringOrientation(preferredOrientation, preferTargetOrientation);
    named.preferringPosture(preferredPosture);
    return new RedundancyResolver(named).solve(this, request);
  }

  /** How the group's redundancy is named, if it has any: its swivel, else its external axes. */
  public function redundancy():Null<RedundancyParameterization> return parameterization;


  public function solveDifferential(q:Array<Float>, twist:Twist6,
      ?redundancyRate:Array<Float>, ?freedom:OrientationPolicy):Null<Array<Float>> {
    if (q == null || q.length != jointCount())
      throw 'Differential IK requires ${jointCount()} joint values';
    if (twist == null) throw "Differential IK requires a tool twist";
    var n = jointCount();
    var jacobian = manipulator.tcpJacobian(q);
    var columns = [for (joint in 0...n) joint];
    if (!ToolFreedom.isFull(freedom)) {
      if (preferredOrientation != null) {
        var asked = twist.toArray();
        var projected = [asked[0], asked[1], asked[2], 0.0, 0.0, 0.0];
        for (row in ToolFreedom.twistRows(forward(q), freedom)) {
          var value = 0.0;
          for (axis in 3...6) value += row[axis]*asked[axis];
          for (axis in 3...6) projected[axis] += row[axis]*value;
        }
        // Keep roundoff-sized spin untouched so existing full-orientation paths are identical.
        var freeRate = 0.0;
        for (axis in 3...6) freeRate = Math.max(freeRate, Math.abs(projected[axis] - asked[axis]));
        if (freeRate <= 1e-12) projected = asked;
        var preferredTwist = new Twist6(projected[0], projected[1], projected[2], projected[3], projected[4], projected[5]);
        var exact = solveDifferential(q, preferredTwist, redundancyRate);
        if (fitsRows(jacobian, 6, n, projected, exact)) return exact;
      }
      return reducedDifferential(q, twist, redundancyRate, freedom, jacobian);
    }
    if (n <= 6) {
      var result = LinearAlgebra.dampedStep(jacobian, 6, n, columns, twist.toArray(), differentialDamping);
      return n < 6 && !fitsRows(jacobian, 6, n, twist.toArray(), result) ? null : result;
    }
    var rows = 6, target = twist.toArray();
    // The redundancy moves exactly at the asked rate: its values' rows join the tool's, G q̇ = ṗ, as long
    // as they leave a solvable system. A group with redundancy left over takes the smallest rate.
    var named = redundancyRate == null || parameterization == null ? null : parameterization.valuesJacobian(q);
    if (named != null && 6 + parameterization.dimension() <= n) {
      var extra = parameterization.dimension();
      var asked:Array<Float> = cast redundancyRate;
      if (asked.length != extra) throw 'Differential IK redundancy rate needs $extra values';
      for (k in 0...extra) {
        for (joint in 0...n) jacobian.push(named[k * n + joint]);
        target.push(asked[k]);
      }
      rows += extra;
    }
    // A redundant group's JᵀJ is rank-deficient: solve in row space instead.
    return LinearAlgebra.dampedRowStep(jacobian, rows, n, columns, target, differentialDamping);
  }

  function reducedDifferential(q:Array<Float>, twist:Twist6, redundancyRate:Null<Array<Float>>,
      freedom:OrientationPolicy, jacobian:Array<Float>):Null<Array<Float>> {
    var n = jointCount();
    var projection = ToolFreedom.twistRows(forward(q), freedom);
    var asked = twist.toArray();
    switch freedom {
      case Cone(_, _): for (i in 3...6) asked[i] = 0.0;
      default:
    }
    var reduced:Array<Float> = [], target:Array<Float> = [];
    for (row in projection) {
      var value = 0.0;
      for (axis in 0...6) value += row[axis]*asked[axis];
      target.push(value);
      for (joint in 0...n) {
        var coefficient = 0.0;
        for (axis in 0...6) coefficient += row[axis]*jacobian[axis*n + joint];
        reduced.push(coefficient);
      }
    }
    var rows = projection.length;
    var named = redundancyRate == null || parameterization == null ? null : parameterization.valuesJacobian(q);
    if (named != null && rows + parameterization.dimension() <= n) {
      var extra = parameterization.dimension();
      var requested:Array<Float> = cast redundancyRate;
      if (requested.length != extra) throw 'Differential IK redundancy rate needs $extra values';
      for (k in 0...extra) {
        for (joint in 0...n) reduced.push(named[k*n + joint]);
        target.push(requested[k]);
      }
      rows += extra;
    }
    var columns = [for (joint in 0...n) joint];
    var result = rows >= n ? LinearAlgebra.dampedStep(reduced, rows, n, columns, target, differentialDamping)
      : LinearAlgebra.dampedRowStep(reduced, rows, n, columns, target, differentialDamping);
    return fitsRows(reduced, rows, n, target, result) ? result : null;
  }

  function fitsRows(jacobian:Array<Float>, rows:Int, n:Int, target:Array<Float>, result:Null<Array<Float>>):Bool {
    if (result == null) return false;
    var position = 0.0, orientation = 0.0;
    for (row in 0...rows) {
      var achieved = 0.0;
      for (joint in 0...n) achieved += jacobian[row*n + joint]*result[joint];
      var residual = Math.abs(achieved - target[row]);
      if (row < 3) position = Math.max(position, residual);
      else orientation = Math.max(orientation, residual);
    }
    if (position <= 1e-6 && orientation <= 1e-6) return true;
    lastFailure = 'tool position velocity residual $position m/s; orientation velocity residual $orientation rad/s';
    return false;
  }

  function options(tolerance:IkTolerance, ?freedom:OrientationPolicy, ?target:Pose3):IkOptions {
    var result = target == null ? new IkOptions(tolerance.position, tolerance.orientation, tolerance.maxIterations, tolerance.damping)
      : ToolFreedom.of(target, freedom, tolerance.orientation).options(tolerance,
          preferredOrientation == null && preferTargetOrientation ? target : preferredOrientation);
    var posture = preferredPosture;
    if (posture != null) result.preferring(posture);
    return result;
  }

  function solveAtSwivel(goal:Transform3, seed:Array<Float>, angle:Float, exact:Bool, tolerance:IkTolerance,
      ?freedom:OrientationPolicy):IKResult {
    if (ToolFreedom.isFull(freedom))
      return manipulator.solve(goal, seed, options(tolerance).atSwivel(angle, exact));
    var target = fromTransform(goal);
    var task = ToolFreedom.of(target, freedom, tolerance.orientation);
    return manipulator.solve(toTransform(task.target), seed, options(tolerance, freedom, target).atSwivel(angle, exact));
  }

  static function requireTolerance(tolerance:IkTolerance):Void {
    if (tolerance == null) throw "IK tolerance is required";
  }

  static function fromTransform(value:Transform3):Pose3
    return new Pose3(value.translation.x, value.translation.y, value.translation.z,
      value.rotation.x, value.rotation.y, value.rotation.z, value.rotation.w);

  static function toTransform(value:Pose3):Transform3 {
    if (value == null) throw "IK target pose is required";
    return new Transform3(new Vec3(value.x, value.y, value.z),
      new Quat(value.qx, value.qy, value.qz, value.qw));
  }

  static function containsNear(candidates:Array<Array<Float>>, value:Array<Float>,
      separation:Float):Bool {
    for (candidate in candidates) {
      var squaredDistance = 0.0;
      for (joint in 0...value.length) {
        var difference = value[joint] - candidate[joint];
        squaredDistance += difference * difference;
      }
      if (Math.sqrt(squaredDistance) < separation) return true;
    }
    return false;
  }
}
