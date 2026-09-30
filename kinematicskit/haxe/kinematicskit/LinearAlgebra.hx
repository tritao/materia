package kinematicskit;

/**
 * Dense helpers on row-major flat matrices, sized for kinematics (tens of
 * rows and columns). Summation order is fixed and documented because
 * migrated solvers must reproduce their previous answers.
 */
class LinearAlgebra {
  /**
   * `A = Jᵀ J` over the selected `columns` of the `rows` x `stride` matrix
   * `jacobian`, and `b = Jᵀ e`. Each entry sums over rows in order, starting
   * from zero. `A` is written row-major, `columns.length` square.
   */
  public static function normalEquations(jacobian:Array<Float>, rows:Int, stride:Int, columns:Array<Int>,
      residual:Array<Float>, normal:Array<Float>, rhs:Array<Float>):Void {
    var n = columns.length;
    for (i in 0...n) {
      var ci = columns[i];
      for (j in 0...n) {
        var cj = columns[j];
        var sum = 0.0;
        for (k in 0...rows) sum += jacobian[k * stride + ci] * jacobian[k * stride + cj];
        normal[i * n + j] = sum;
      }
      var sum = 0.0;
      for (k in 0...rows) sum += jacobian[k * stride + ci] * residual[k];
      rhs[i] = sum;
    }
  }

  /** `A = Jᵀ J` and `b = Jᵀ e` for a dense `rows` x `width` matrix; same summation order as `normalEquations`. */
  public static function normalEquationsDense(jacobian:Array<Float>, rows:Int, width:Int, residual:Array<Float>,
      normal:Array<Float>, rhs:Array<Float>):Void {
    for (i in 0...width) {
      for (j in 0...width) {
        var sum = 0.0;
        for (k in 0...rows) sum += jacobian[k * width + i] * jacobian[k * width + j];
        normal[i * width + j] = sum;
      }
      var sum = 0.0;
      for (k in 0...rows) sum += jacobian[k * width + i] * residual[k];
      rhs[i] = sum;
    }
  }

  /**
   * As `solve`, but destroys `a` and `b` and writes the answer into `x`
   * instead of allocating. Returns false where `solve` returns null.
   */
  public static function solveInPlace(a:Array<Float>, b:Array<Float>, n:Int, x:Array<Float>,
      pivotTolerance:Float):Bool {
    for (col in 0...n) {
      var pivotRow = col;
      var pivotValue = Math.abs(a[col * n + col]);
      for (row in (col + 1)...n) if (Math.abs(a[row * n + col]) > pivotValue) {
        pivotRow = row;
        pivotValue = Math.abs(a[row * n + col]);
      }
      if (!Math.isFinite(pivotValue) || pivotValue < pivotTolerance) return false;
      if (pivotRow != col) {
        for (k in 0...n) {
          var swap = a[col * n + k]; a[col * n + k] = a[pivotRow * n + k]; a[pivotRow * n + k] = swap;
        }
        var swapValue = b[col]; b[col] = b[pivotRow]; b[pivotRow] = swapValue;
      }
      var pivot = a[col * n + col];
      for (row in (col + 1)...n) {
        var factor = a[row * n + col] / pivot;
        if (factor == 0.0) continue;
        for (k in col...n) a[row * n + k] -= factor * a[col * n + k];
        b[row] -= factor * b[col];
      }
    }
    var row = n - 1;
    while (row >= 0) {
      var sum = b[row];
      for (k in (row + 1)...n) sum -= a[row * n + k] * x[k];
      x[row] = sum / a[row * n + row];
      if (!Math.isFinite(x[row])) return false;
      row--;
    }
    return true;
  }

  /**
   * Solves `A x = b` (`n` square, row-major) by Gaussian elimination with
   * partial pivoting. `A` and `b` are left unchanged. Returns null when a
   * pivot is below `pivotTolerance` or not finite, or the result is not finite.
   */
  public static function solve(a:Array<Float>, b:Array<Float>, n:Int, ?pivotTolerance:Float = 1e-15):Null<Array<Float>> {
    var x = [for (_ in 0...n) 0.0];
    return solveInPlace(a.copy(), b.copy(), n, x, pivotTolerance) ? x : null;
  }


  /**
   * One damped least-squares step: `(JᵀJ + λ²I) Δ = Jᵀ e` over the selected
   * columns. Returns null when the system cannot be solved.
   */
  public static function dampedStep(jacobian:Array<Float>, rows:Int, stride:Int, columns:Array<Int>,
      residual:Array<Float>, damping:Float):Null<Array<Float>> {
    var n = columns.length;
    var normal = [for (_ in 0...n * n) 0.0];
    var rhs = [for (_ in 0...n) 0.0];
    normalEquations(jacobian, rows, stride, columns, residual, normal, rhs);
    for (i in 0...n) normal[i * n + i] += damping * damping;
    return solve(normal, rhs, n);
  }

  /** Numerical rank by Gauss-Jordan elimination, with pivots at or below `tolerance` x max |entry| treated as zero. */
  public static function rank(matrix:Array<Float>, rows:Int, cols:Int, tolerance:Float):Int {
    if (rows == 0 || cols == 0) return 0;
    var m = matrix.copy();
    var maxValue = 0.0;
    for (value in m) maxValue = Math.max(maxValue, Math.abs(value));
    if (maxValue == 0.0) return 0;
    var threshold = maxValue * tolerance;
    var row = 0;
    var col = 0;
    while (row < rows && col < cols) {
      var pivot = row;
      for (candidate in row...rows) if (Math.abs(m[candidate * cols + col]) > Math.abs(m[pivot * cols + col])) pivot = candidate;
      if (Math.abs(m[pivot * cols + col]) <= threshold) { col++; continue; }
      if (pivot != row) for (k in 0...cols) {
        var swap = m[row * cols + k]; m[row * cols + k] = m[pivot * cols + k]; m[pivot * cols + k] = swap;
      }
      var divisor = m[row * cols + col];
      for (k in col...cols) m[row * cols + k] /= divisor;
      for (candidate in 0...rows) if (candidate != row) {
        var factor = m[candidate * cols + col];
        for (k in col...cols) m[candidate * cols + k] -= factor * m[row * cols + k];
      }
      row++;
      col++;
    }
    return row;
  }

  /** Euclidean norm of the first `count` values (required, not optional: an optional Int is boxed per call). */
  public static function norm(values:Array<Float>, count:Int):Float {
    var sum = 0.0;
    for (i in 0...count) sum += values[i] * values[i];
    return Math.sqrt(sum);
  }
}
