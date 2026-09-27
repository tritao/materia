package motionkit.robot;

import motionkit.kinematics.IkTolerance;
import motionkit.kinematics.KinematicsSolver;
import motionkit.kinematics.Pose3;
import motionkit.kinematics.Twist6;
import robotkit.manipulation.Manipulator;
import robotkit.spatial.Quat;
import robotkit.spatial.Transform3;
import robotkit.spatial.Vec3;

/** KinematicsSolver adapter over RobotKit's deterministic DLS manipulator. */
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

  public function jointCount():Int return manipulator.chain.dofCount();

  public function forward(q:Array<Float>):Pose3 return fromTransform(manipulator.tcpPose(q));

  public function solvePose(target:Pose3, seed:Array<Float>,
      tolerance:IkTolerance):Null<Array<Float>> {
    requireTolerance(tolerance);
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
      if (candidates.length >= maxCount) break;
    }
    return candidates;
  }

  public function solveDifferential(q:Array<Float>, twist:Twist6):Null<Array<Float>> {
    if (q == null || q.length != jointCount())
      throw 'Differential IK requires ${jointCount()} joint values';
    if (twist == null) throw "Differential IK requires a tool twist";
    var jacobian = manipulator.chain.jacobian(q);

    // The chain Jacobian is at the flange. Shift its linear rows to the TCP.
    var flange = manipulator.forwardKinematics(q);
    var tcpOffset = flange.rotation.rotate(manipulator.flangeTTcp.translation);
    for (joint in 0...jointCount()) {
      var angular = new Vec3(jacobian[3][joint], jacobian[4][joint], jacobian[5][joint]);
      var shiftedLinear = angular.cross(tcpOffset);
      jacobian[0][joint] += shiftedLinear.x;
      jacobian[1][joint] += shiftedLinear.y;
      jacobian[2][joint] += shiftedLinear.z;
    }

    var n = jointCount();
    var normal:Array<Array<Float>> = [];
    for (row in 0...n) {
      var values:Array<Float> = [];
      for (column in 0...n) {
        var value = 0.0;
        for (axis in 0...6) value += jacobian[axis][row] * jacobian[axis][column];
        if (row == column) value += differentialDamping * differentialDamping;
        values.push(value);
      }
      normal.push(values);
    }
    var requested = twist.toArray();
    var rhs:Array<Float> = [];
    for (joint in 0...n) {
      var value = 0.0;
      for (axis in 0...6) value += jacobian[axis][joint] * requested[axis];
      rhs.push(value);
    }
    return solveLinear(normal, rhs);
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

  static function solveLinear(matrix:Array<Array<Float>>,
      right:Array<Float>):Null<Array<Float>> {
    var n = right.length;
    var values:Array<Array<Float>> = [for (row in matrix) row.copy()];
    var result = right.copy();
    for (column in 0...n) {
      var pivot = column;
      var pivotMagnitude = Math.abs(values[column][column]);
      for (row in (column + 1)...n) {
        var magnitude = Math.abs(values[row][column]);
        if (magnitude > pivotMagnitude) { pivot = row; pivotMagnitude = magnitude; }
      }
      if (!Math.isFinite(pivotMagnitude) || pivotMagnitude < 1e-15) return null;
      if (pivot != column) {
        var rowSwap = values[column]; values[column] = values[pivot]; values[pivot] = rowSwap;
        var valueSwap = result[column]; result[column] = result[pivot]; result[pivot] = valueSwap;
      }
      for (row in (column + 1)...n) {
        var factor = values[row][column] / values[column][column];
        for (entry in column...n) values[row][entry] -= factor * values[column][entry];
        result[row] -= factor * result[column];
      }
    }
    var solution:Array<Float> = [for (_ in 0...n) 0.0];
    var row = n - 1;
    while (row >= 0) {
      var value = result[row];
      for (column in (row + 1)...n) value -= values[row][column] * solution[column];
      solution[row] = value / values[row][row];
      if (!Math.isFinite(solution[row])) return null;
      row--;
    }
    return solution;
  }
}
