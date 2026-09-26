package robotkit.manipulation;

import robotkit.spatial.Vec3;
import robotkit.spatial.Quat;
import robotkit.spatial.Transform3;

/**
 * Damped-least-squares (Levenberg-Marquardt style) numerical IK. Joint
 * limits are clamped every iteration; non-convergence is reported in the
 * result rather than thrown.
 */
class InverseKinematics {
  public static function solve(chain:KinematicChain, group:JointGroup, target:Transform3,
      seed:Array<Float>, ?positionTolerance:Float = 1e-4, ?orientationTolerance:Float = 1e-3,
      ?maxIterations:Int = 100, ?damping:Float = 0.02):IKResult {
    if (chain == null || group == null || target == null)
      throw "Inverse kinematics requires a chain, joint group, and target";
    if (chain.dofCount() != group.count())
      throw "Inverse kinematics requires the chain and joint group to share the same degrees of freedom";
    var n = chain.dofCount();
    var q = group.clamp(seed == null ? [for (_ in 0...n) 0.0] : seed);

    var positionError = 0.0;
    var orientationError = 0.0;
    var iterationsUsed = 0;
    for (iteration in 0...maxIterations) {
      iterationsUsed = iteration;
      var current = chain.forwardKinematics(q);
      var positionErrorVec = target.translation.sub(current.translation);
      var orientationErrorVec = orientationError3(current.rotation, target.rotation);
      positionError = positionErrorVec.norm();
      orientationError = orientationErrorVec.norm();
      if (positionError <= positionTolerance && orientationError <= orientationTolerance)
        return new IKResult(true, q, positionError, orientationError, iteration);

      var jacobian = chain.jacobian(q);
      var error = [positionErrorVec.x, positionErrorVec.y, positionErrorVec.z,
        orientationErrorVec.x, orientationErrorVec.y, orientationErrorVec.z];
      var jacobianT = transpose(jacobian, n);
      var normalMatrix = multiply(jacobianT, n, 6, jacobian, 6, n);
      for (i in 0...n) normalMatrix[i][i] += damping * damping;
      var rhs = multiplyVector(jacobianT, n, 6, error);
      var delta = solveSymmetric(normalMatrix, rhs, n);
      for (i in 0...n) q[i] += delta[i];
      q = group.clamp(q);
    }

    var final_ = chain.forwardKinematics(q);
    positionError = target.translation.sub(final_.translation).norm();
    orientationError = orientationError3(final_.rotation, target.rotation).norm();
    return new IKResult(false, q, positionError, orientationError, maxIterations);
  }

  /** Exact axis-angle rotation vector taking `from` to `to` (log map of to * from^-1). */
  static function orientationError3(from:Quat, to:Quat):Vec3 {
    var error = to.multiply(from.conjugate());
    if (error.w < 0.0) error = new Quat(-error.x, -error.y, -error.z, -error.w);
    var sinHalf = Math.sqrt(error.x * error.x + error.y * error.y + error.z * error.z);
    if (sinHalf < 1e-9) return Vec3.zero();
    var angle = 2.0 * Math.atan2(sinHalf, error.w);
    return new Vec3(error.x / sinHalf, error.y / sinHalf, error.z / sinHalf).scale(angle);
  }

  static function zeroMatrix(rows:Int, cols:Int):Array<Array<Float>> {
    var result:Array<Array<Float>> = [];
    for (_ in 0...rows) {
      var row:Array<Float> = [];
      for (_ in 0...cols) row.push(0.0);
      result.push(row);
    }
    return result;
  }

  static function transpose(m:Array<Array<Float>>, cols:Int):Array<Array<Float>> {
    var rows = m.length;
    var result = zeroMatrix(cols, rows);
    for (r in 0...rows) for (c in 0...cols) result[c][r] = m[r][c];
    return result;
  }

  static function multiply(a:Array<Array<Float>>, aRows:Int, aCols:Int,
      b:Array<Array<Float>>, bRows:Int, bCols:Int):Array<Array<Float>> {
    if (aCols != bRows) throw "Matrix dimensions do not agree for multiplication";
    var result = zeroMatrix(aRows, bCols);
    for (r in 0...aRows) for (c in 0...bCols) {
      var sum = 0.0;
      for (k in 0...aCols) sum += a[r][k] * b[k][c];
      result[r][c] = sum;
    }
    return result;
  }

  static function multiplyVector(a:Array<Array<Float>>, aRows:Int, aCols:Int, v:Array<Float>):Array<Float> {
    var result:Array<Float> = [for (_ in 0...aRows) 0.0];
    for (r in 0...aRows) {
      var sum = 0.0;
      for (k in 0...aCols) sum += a[r][k] * v[k];
      result[r] = sum;
    }
    return result;
  }

  /** Gaussian elimination with partial pivoting; `m` is square and (with damping) always non-singular. */
  static function solveSymmetric(m:Array<Array<Float>>, rhs:Array<Float>, n:Int):Array<Float> {
    var a:Array<Array<Float>> = [];
    for (row in m) a.push(row.copy());
    var b = rhs.copy();
    for (col in 0...n) {
      var pivotRow = col;
      var pivotValue = Math.abs(a[col][col]);
      for (row in (col + 1)...n) if (Math.abs(a[row][col]) > pivotValue) {
        pivotRow = row;
        pivotValue = Math.abs(a[row][col]);
      }
      if (pivotValue < 1e-15) throw "Singular system in inverse kinematics normal equations";
      if (pivotRow != col) {
        var swapRow = a[col]; a[col] = a[pivotRow]; a[pivotRow] = swapRow;
        var swapValue = b[col]; b[col] = b[pivotRow]; b[pivotRow] = swapValue;
      }
      for (row in (col + 1)...n) {
        var factor = a[row][col] / a[col][col];
        if (factor == 0.0) continue;
        for (k in col...n) a[row][k] -= factor * a[col][k];
        b[row] -= factor * b[col];
      }
    }
    var x:Array<Float> = [for (_ in 0...n) 0.0];
    var row = n - 1;
    while (row >= 0) {
      var sum = b[row];
      for (k in (row + 1)...n) sum -= a[row][k] * x[k];
      x[row] = sum / a[row][row];
      row--;
    }
    return x;
  }
}
