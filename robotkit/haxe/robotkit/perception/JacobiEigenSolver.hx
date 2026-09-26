package robotkit.perception;

/**
 * The classic cyclic Jacobi eigenvalue algorithm for small symmetric
 * matrices (no external linear-algebra library, per the plan). Used by
 * `PlaneFit` for the 3x3 point covariance; kept general over N so it is not
 * accidentally coupled to that one call site.
 */
class JacobiEigenSolver {
  public static function solveSymmetric(matrix:Array<Array<Float>>, ?maxSweeps:Int = 100,
      ?tolerance:Float = 1e-12):EigenDecomposition {
    if (matrix == null || matrix.length == 0) throw "Jacobi eigensolver requires a non-empty matrix";
    var n = matrix.length;
    for (row in matrix) if (row == null || row.length != n) throw "Jacobi eigensolver requires a square matrix";

    var a:Array<Array<Float>> = [];
    for (i in 0...n) a.push(matrix[i].copy());

    var v:Array<Array<Float>> = [];
    for (i in 0...n) {
      var row:Array<Float> = [];
      for (j in 0...n) row.push(i == j ? 1.0 : 0.0);
      v.push(row);
    }

    for (_ in 0...maxSweeps) {
      var offDiagonal = 0.0;
      for (p in 0...n) for (q in (p + 1)...n) offDiagonal += a[p][q] * a[p][q];
      if (offDiagonal < tolerance) break;

      for (p in 0...n) for (q in (p + 1)...n) {
        if (Math.abs(a[p][q]) < 1e-300) continue;
        var theta = (a[q][q] - a[p][p]) / (2.0 * a[p][q]);
        var t = (theta >= 0.0 ? 1.0 : -1.0) / (Math.abs(theta) + Math.sqrt(theta * theta + 1.0));
        var c = 1.0 / Math.sqrt(t * t + 1.0);
        var s = t * c;
        var app = a[p][p], aqq = a[q][q], apq = a[p][q];
        a[p][p] = c * c * app - 2.0 * s * c * apq + s * s * aqq;
        a[q][q] = s * s * app + 2.0 * s * c * apq + c * c * aqq;
        a[p][q] = 0.0;
        a[q][p] = 0.0;
        for (i in 0...n) if (i != p && i != q) {
          var aip = a[i][p], aiq = a[i][q];
          a[i][p] = c * aip - s * aiq;
          a[p][i] = a[i][p];
          a[i][q] = s * aip + c * aiq;
          a[q][i] = a[i][q];
        }
        for (i in 0...n) {
          var vip = v[i][p], viq = v[i][q];
          v[i][p] = c * vip - s * viq;
          v[i][q] = s * vip + c * viq;
        }
      }
    }

    var eigenvalues:Array<Float> = [];
    for (i in 0...n) eigenvalues.push(a[i][i]);
    var eigenvectors:Array<Array<Float>> = [];
    for (i in 0...n) {
      var vector:Array<Float> = [];
      for (r in 0...n) vector.push(v[r][i]);
      eigenvectors.push(vector);
    }
    return new EigenDecomposition(eigenvalues, eigenvectors);
  }
}
