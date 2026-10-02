package motionkit.robot;

import kinematicskit.LinearAlgebra;
import motionkit.kinematics.IkTolerance;
import motionkit.kinematics.KinematicsSolver;
import motionkit.kinematics.PathRequest;
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
class ManipulatorKinematics implements KinematicsSolver {
  /** The group solved: an arm (`Manipulator`), or an arm with external axes and a work frame. */
  public final manipulator:KinematicGroup;
  /**
   * A posture the solves draw the arm towards (`q` order): with external
   * axes, the arm stays comfortable and they bring the work to it.
   */
  public var preferredPosture:Null<Array<Float>> = null;
  public final differentialDamping:Float;
  /** How the group's redundancy is named; null for a group without any. */
  final parameterization:Null<RedundancyParameterization>;

  public function new(manipulator:KinematicGroup, ?differentialDamping:Float = 1e-6) {
    if (manipulator == null) throw "Manipulator kinematics requires a manipulator";
    if (!Math.isFinite(differentialDamping) || differentialDamping <= 0.0)
      throw "Differential IK damping must be finite and positive";
    this.manipulator = manipulator;
    this.differentialDamping = differentialDamping;
    parameterization = manipulator.redundant() ? new SwivelParameterization(manipulator)
      : manipulator.external.indexOf(true) >= 0 ? new ExternalAxesParameterization(manipulator) : null;
  }

  /** The same kinematics with its own posture preference; the group is shared, as it is safe to. */
  public function fork():KinematicsSolver {
    var copy = new ManipulatorKinematics(manipulator, differentialDamping);
    copy.preferredPosture = preferredPosture == null ? null : preferredPosture.copy();
    return copy;
  }

  public function jointCount():Int return manipulator.dofCount();

  public function forward(q:Array<Float>):Pose3 return fromTransform(manipulator.tcpPose(q));

  public function solvePose(target:Pose3, seed:Array<Float>,
      tolerance:IkTolerance):Null<Array<Float>> {
    requireTolerance(tolerance);
    if (manipulator.redundant() && seed != null) {
      var swivel = manipulator.swivelAngle(seed);
      if (Math.isFinite(swivel)) {
        var kept = solveAtSwivel(toTransform(target), seed, swivel, false, tolerance);
        if (kept.converged) return kept.q.copy();
      }
    }
    var result = manipulator.solve(toTransform(target), seed, options(tolerance));
    return result.converged ? result.q.copy() : null;
  }

  public function sampleCandidates(target:Pose3, maxCount:Int,
      tolerance:IkTolerance):Array<Array<Float>> {
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
      var seed:Array<Float> = [];
      for (joint in 0...jointCount()) {
        var level = code % 3;
        code = Std.int(code / 3);
        var limits = manipulator.group.limitsOf(joint);
        if (limits.lower < limits.upper) {
          var fraction = level == 0 ? 0.5 : (level == 1 ? 0.25 : 0.75);
          seed.push(limits.lower + (limits.upper - limits.lower) * fraction);
        } else {
          seed.push(level == 0 ? 0.0 : (level == 1 ? -Math.PI : Math.PI));
        }
      }
      var solved = solvePose(target, seed, tolerance);
      if (solved == null || containsNear(candidates, solved, tolerance.candidateSeparation))
        continue;
      candidates.push(solved);
      if (candidates.length >= branchLimit) break;
    }
    return redundant ? sweepSwivel(target, candidates, maxCount, tolerance) : candidates;
  }

  /** Each branch solved at evenly spaced swivel angles, stepping from its own, each solve seeded by the last. */
  function sweepSwivel(target:Pose3, branches:Array<Array<Float>>, maxCount:Int,
      tolerance:IkTolerance):Array<Array<Float>> {
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
          var solved = solveAtSwivel(goal, seed, angle, true, tolerance);
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
  public function solvePath(request:PathRequest):Array<Null<Array<Float>>> {
    var named = parameterization;
    if (named == null) return request.followPointByPoint(this);
    return new RedundancyResolver(named).solvePath(this, request);
  }

  /** How the group's redundancy is named, if it has any: its swivel, else its external axes. */
  public function redundancy():Null<RedundancyParameterization> return parameterization;


  public function solveDifferential(q:Array<Float>, twist:Twist6,
      ?preferredRate:Array<Float>):Null<Array<Float>> {
    if (q == null || q.length != jointCount())
      throw 'Differential IK requires ${jointCount()} joint values';
    if (twist == null) throw "Differential IK requires a tool twist";
    var n = jointCount();
    var jacobian = manipulator.tcpJacobian(q);
    var columns = [for (joint in 0...n) joint];
    if (n <= 6) return LinearAlgebra.dampedStep(jacobian, 6, n, columns, twist.toArray(), differentialDamping);
    var preferred = preferredRate;
    if (preferred != null && preferred.length != n) throw 'Differential IK preferred rate needs $n values';
    var rows = 6, target = twist.toArray();
    // The redundancy moves exactly as the preferred rate moves it: its values' rows join the
    // tool's, G q̇ = G p, as long as they leave a solvable system.
    var named = preferred == null || parameterization == null ? null : parameterization.valuesJacobian(q);
    if (named != null && 6 + parameterization.dimension() <= n) {
      var extra = parameterization.dimension();
      for (k in 0...extra) {
        var rate = 0.0;
        for (joint in 0...n) {
          jacobian.push(named[k * n + joint]);
          rate += named[k * n + joint] * preferred[joint];
        }
        target.push(rate);
      }
      rows += extra;
    }
    // A redundant group's JᵀJ is rank-deficient: solve in row space instead, for
    // q̇ = p + A⁺(b - A p), the rate nearest the preferred one p that meets every row.
    if (preferred != null)
      for (row in 0...rows) {
        var made = 0.0;
        for (joint in 0...n) made += jacobian[row * n + joint] * preferred[joint];
        target[row] -= made;
      }
    var step = LinearAlgebra.dampedRowStep(jacobian, rows, n, columns, target, differentialDamping);
    if (step == null || preferred == null) return step;
    return [for (joint in 0...n) step[joint] + preferred[joint]];
  }

  function options(tolerance:IkTolerance):IkOptions {
    var result = new IkOptions(tolerance.position, tolerance.orientation, tolerance.maxIterations, tolerance.damping);
    var posture = preferredPosture;
    if (posture != null) result.preferring(posture);
    return result;
  }

  function solveAtSwivel(goal:Transform3, seed:Array<Float>, angle:Float, exact:Bool, tolerance:IkTolerance):IKResult
    return manipulator.solve(goal, seed, options(tolerance).atSwivel(angle, exact));

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
