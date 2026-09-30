package motionkit.robot;

import kinematicskit.LinearAlgebra;
import motionkit.kinematics.IkTolerance;
import motionkit.kinematics.KinematicsSolver;
import motionkit.kinematics.Pose3;
import motionkit.kinematics.Twist6;
import robotkit.manipulation.CoordinatedGroup;
import robotkit.spatial.Quat;
import robotkit.spatial.Transform3;
import robotkit.spatial.Vec3;

/**
 * KinematicsSolver over an arm with external axes (RobotKit
 * `CoordinatedGroup`): poses are the tool centre point in the work frame,
 * joint values cover every group joint. `ProgramCompiler` over it plans
 * paths on the workpiece while a positioner and a rail move with the arm,
 * all timed together in one plan. Each solve starts from the previous
 * sample and damps the external axes, so they move smoothly and only as
 * much as the arm needs.
 */
class CoordinatedKinematics implements KinematicsSolver {
  public final coordinated:CoordinatedGroup;
  public final differentialDamping:Float;

  public function new(coordinated:CoordinatedGroup, ?differentialDamping:Float = 1e-6) {
    if (coordinated == null) throw "Coordinated kinematics requires a coordinated group";
    if (!Math.isFinite(differentialDamping) || differentialDamping <= 0.0)
      throw "Differential IK damping must be finite and positive";
    this.coordinated = coordinated;
    this.differentialDamping = differentialDamping;
  }

  public function jointCount():Int return coordinated.dofCount();

  public function forward(q:Array<Float>):Pose3 {
    var value = coordinated.toolInWork(q);
    return new Pose3(value.translation.x, value.translation.y, value.translation.z, value.rotation.x,
      value.rotation.y, value.rotation.z, value.rotation.w);
  }

  public function solvePose(target:Pose3, seed:Array<Float>, tolerance:IkTolerance):Null<Array<Float>> {
    if (tolerance == null) throw "IK tolerance is required";
    var result = coordinated.solveIk(toTransform(target), seed, tolerance.position, tolerance.orientation,
      Std.int(Math.max(200, tolerance.maxIterations)), tolerance.damping);
    return result.converged ? result.q.copy() : null;
  }

  /** Solutions from seeds spread over the joint ranges (mid, ¼, ¾ of each), without near-duplicates. */
  public function sampleCandidates(target:Pose3, maxCount:Int, tolerance:IkTolerance):Array<Array<Float>> {
    if (tolerance == null) throw "IK tolerance is required";
    if (target == null) throw "Candidate sampling requires a target pose";
    var candidates:Array<Array<Float>> = [];
    var n = jointCount();
    for (attempt in 0...Std.int(Math.max(32, maxCount * 8))) {
      if (candidates.length >= maxCount) break;
      var code = attempt;
      var seed:Array<Float> = [];
      for (joint in 0...n) {
        var level = code % 3;
        code = Std.int(code / 3);
        var limits = coordinated.group.limitsOf(joint);
        var fraction = level == 0 ? 0.5 : (level == 1 ? 0.25 : 0.75);
        seed.push(limits.lower < limits.upper ? limits.lower + (limits.upper - limits.lower) * fraction
          : (level == 0 ? 0.0 : (level == 1 ? -Math.PI : Math.PI)));
      }
      var solved = solvePose(target, seed, tolerance);
      if (solved == null) continue;
      var near = false;
      for (candidate in candidates) {
        var squared = 0.0;
        for (joint in 0...n) squared += Math.pow(solved[joint] - candidate[joint], 2);
        if (Math.sqrt(squared) < tolerance.candidateSeparation) near = true;
      }
      if (!near) candidates.push(solved);
    }
    return candidates;
  }

  /** Joint velocities for a tool twist relative to the workpiece, in the work frame. */
  public function solveDifferential(q:Array<Float>, twist:Twist6):Null<Array<Float>> {
    if (q == null || q.length != jointCount()) throw 'Differential IK requires ${jointCount()} joint values';
    if (twist == null) throw "Differential IK requires a tool twist";
    var n = jointCount();
    return LinearAlgebra.dampedStep(coordinated.relativeJacobian(q), 6, n, [for (joint in 0...n) joint],
      twist.toArray(), differentialDamping);
  }

  static function toTransform(value:Pose3):Transform3 {
    if (value == null) throw "IK target pose is required";
    return new Transform3(new Vec3(value.x, value.y, value.z), new Quat(value.qx, value.qy, value.qz, value.qw));
  }
}
