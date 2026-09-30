package motionkit.robot;

import kinematicskit.LinearAlgebra;
import motionkit.kinematics.IkTolerance;
import motionkit.kinematics.KinematicsSolver;
import motionkit.kinematics.Pose3;
import motionkit.kinematics.Twist6;
import robotkit.manipulation.Manipulator;
import robotkit.spatial.Quat;
import robotkit.spatial.Transform3;
import robotkit.spatial.Vec3;

/**
 * KinematicsSolver adapter over RobotKit's deterministic DLS manipulator.
 *
 * A redundant arm (`Manipulator.redundant`, e.g. 7-axis) is handled through
 * its swivel angle: `solvePose` keeps the seed's swivel as a preference, so
 * the elbow does not drift with each solve; `sampleCandidates` sweeps the
 * swivel around the circle on each IK branch it finds; `continueCandidates`
 * grows a path's candidates sample by sample; and `refinePath` smooths the
 * swivel along a chosen path and re-solves it exactly there.
 */
class ManipulatorKinematics implements KinematicsSolver {
  public final manipulator:Manipulator;
  public final differentialDamping:Float;

  public function new(manipulator:Manipulator, ?differentialDamping:Float = 1e-6) {
    if (manipulator == null) throw "Manipulator kinematics requires a manipulator";
    if (!Math.isFinite(differentialDamping) || differentialDamping <= 0.0)
      throw "Differential IK damping must be finite and positive";
    this.manipulator = manipulator;
    this.differentialDamping = differentialDamping;
  }

  public function jointCount():Int return manipulator.dofCount();

  public function forward(q:Array<Float>):Pose3 return fromTransform(manipulator.tcpPose(q));

  public function solvePose(target:Pose3, seed:Array<Float>,
      tolerance:IkTolerance):Null<Array<Float>> {
    requireTolerance(tolerance);
    if (manipulator.redundant() && seed != null) {
      var swivel = manipulator.swivelAngle(seed);
      if (Math.isFinite(swivel)) {
        var kept = manipulator.solveIkAtSwivel(toTransform(target), seed, swivel, true, tolerance.position,
          tolerance.orientation, 1e-4, tolerance.maxIterations, tolerance.damping);
        if (kept.converged) return kept.q.copy();
      }
    }
    var result = manipulator.solveIkForTcp(toTransform(target), seed,
      tolerance.position, tolerance.orientation, tolerance.maxIterations,
      tolerance.damping);
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
          var solved = manipulator.solveIkAtSwivel(goal, seed, angle, false, tolerance.position,
            tolerance.orientation, 1e-4, tolerance.maxIterations, tolerance.damping);
          if (!solved.converged) break;
          seed = solved.q;
          if (!containsNear(candidates, solved.q, tolerance.candidateSeparation)) candidates.push(solved.q.copy());
        }
      }
    }
    return candidates;
  }

  /**
   * A redundant arm's candidates at the next path sample, grown from the
   * previous sample's: each continues at its own swivel, and at `step`
   * radians either side, so the swivel can drift gradually along the path.
   * Where a joint limit blocks its own swivel, a candidate continues with
   * the swivel as a preference instead, letting the limit bend it. One
   * candidate is kept per `step`-wide swivel bin (the one continuing the
   * lowest-numbered previous candidate), at most `maxCount`, nearest the
   * previous swivels first. Every continuous swivel path at that resolution
   * is then in the graph the path search walks.
   */
  public function continueCandidates(target:Pose3, previous:Array<Array<Float>>, maxCount:Int, step:Float,
      tolerance:IkTolerance):Array<Array<Float>> {
    requireTolerance(tolerance);
    if (!manipulator.redundant()) throw "Continuing candidates needs a redundant arm";
    if (!(step > 0.0)) throw "Swivel step must be positive";
    var goal = toTransform(target);
    var bins = new Map<Int, Bool>();
    var found:Array<{q:Array<Float>, order:Int}> = [];
    for (order in [0, 1, -1]) for (seed in previous) {
      var swivel = manipulator.swivelAngle(seed);
      if (!Math.isFinite(swivel)) continue;
      var angle = swivel + order * step;
      var solved = manipulator.solveIkAtSwivel(goal, seed, angle, false, tolerance.position, tolerance.orientation,
        1e-4, tolerance.maxIterations, tolerance.damping);
      if (!solved.converged && order == 0)
        solved = manipulator.solveIkAtSwivel(goal, seed, angle, true, tolerance.position, tolerance.orientation,
          1e-4, tolerance.maxIterations, tolerance.damping);
      if (!solved.converged) continue;
      var reached = manipulator.swivelAngle(solved.q);
      if (!Math.isFinite(reached)) continue;
      var bin = Math.round(reached / step);
      if (bins.exists(bin) || containsNear([for (entry in found) entry.q], solved.q, tolerance.candidateSeparation))
        continue;
      bins.set(bin, true);
      found.push({q: solved.q.copy(), order: order == 0 ? 0 : 1});
    }
    // Continuations at their own swivel first, then the neighbours, up to the cap.
    var kept = [for (entry in found) if (entry.order == 0) entry.q];
    for (entry in found) if (entry.order != 0 && kept.length < maxCount) kept.push(entry.q);
    return kept.length > maxCount ? kept.slice(0, maxCount) : kept;
  }

  /**
   * Smooths the swivel of a chosen joint path and re-solves every sample
   * exactly at the smoothed angle, each seeded by the previous one. The
   * swivel is averaged over `radius` samples either side (a Gaussian of
   * half that width), with the first sample pinned. Returns null, leaving the
   * path as chosen, where the swivel is undefined, a sample does not solve,
   * or a joint would move more than `maxJump` between samples.
   */
  public function refinePath(poses:Array<Pose3>, chosen:Array<Array<Float>>, radius:Int, maxJump:Array<Float>,
      tolerance:IkTolerance):Null<Array<Array<Float>>> {
    requireTolerance(tolerance);
    if (!manipulator.redundant() || chosen.length < 3 || radius < 1) return null;
    var swivel:Array<Float> = [];
    for (q in chosen) {
      var angle = manipulator.swivelAngle(q);
      if (!Math.isFinite(angle)) return null;
      // Unwrapped, so smoothing never averages across the ±π seam.
      if (swivel.length > 0) {
        var last = swivel[swivel.length - 1];
        while (angle - last > Math.PI) angle -= 2.0 * Math.PI;
        while (angle - last < -Math.PI) angle += 2.0 * Math.PI;
      }
      swivel.push(angle);
    }
    var sigma = radius / 2.0;
    var smoothed = [swivel[0]];
    for (i in 1...swivel.length) {
      var sum = 0.0, weights = 0.0;
      for (j in Std.int(Math.max(0, i - radius))...Std.int(Math.min(swivel.length, i + radius + 1))) {
        var w = Math.exp(-0.5 * Math.pow((j - i) / sigma, 2));
        sum += w * swivel[j];
        weights += w;
      }
      smoothed.push(sum / weights);
    }
    var refined = [chosen[0].copy()];
    for (i in 1...chosen.length) {
      var seed = refined[i - 1];
      var solved = manipulator.solveIkAtSwivel(toTransform(poses[i]), seed, smoothed[i], false, tolerance.position,
        tolerance.orientation, 1e-4, tolerance.maxIterations, tolerance.damping);
      if (!solved.converged) return null;
      for (joint in 0...solved.q.length) if (Math.abs(solved.q[joint] - seed[joint]) > maxJump[joint]) return null;
      refined.push(solved.q.copy());
    }
    return refined;
  }

  public function solveDifferential(q:Array<Float>, twist:Twist6):Null<Array<Float>> {
    if (q == null || q.length != jointCount())
      throw 'Differential IK requires ${jointCount()} joint values';
    if (twist == null) throw "Differential IK requires a tool twist";
    var n = jointCount();
    var jacobian = manipulator.tcpJacobian(q);
    return LinearAlgebra.dampedStep(jacobian, 6, n, [for (joint in 0...n) joint], twist.toArray(),
      differentialDamping);
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
