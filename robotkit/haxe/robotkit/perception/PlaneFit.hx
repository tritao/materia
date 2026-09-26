package robotkit.perception;

import robotkit.spatial.Vec3;

/**
 * Least-squares plane fitting (total least squares via the 3x3 point
 * covariance and `JacobiEigenSolver`) plus seeded RANSAC for outlier
 * rejection, per the plan: no external linear-algebra library.
 */
class PlaneFit {
  /**
   * Fits `normal . x = offset` by minimizing the sum of squared orthogonal
   * distances: the normal is the covariance matrix's smallest-eigenvalue
   * eigenvector, and the offset is `normal . centroid`. `referenceNormal`,
   * when given, orients the (otherwise sign-ambiguous) normal onto the same
   * side.
   */
  public static function fit(points:Array<Vec3>, ?referenceNormal:Vec3):PlaneEstimate {
    if (points == null || points.length < 3) throw "Plane fit requires at least three points";
    var centroid = centroidOf(points);
    var covariance = covarianceOf(points, centroid);
    var eigen = JacobiEigenSolver.solveSymmetric(covariance);

    var minIndex = 0;
    for (i in 1...eigen.eigenvalues.length) if (eigen.eigenvalues[i] < eigen.eigenvalues[minIndex]) minIndex = i;
    var raw = eigen.eigenvectors[minIndex];
    var normal = new Vec3(raw[0], raw[1], raw[2]).normalized();
    if (referenceNormal != null && normal.dot(referenceNormal) < 0.0) normal = normal.negate();

    var offset = normal.dot(centroid);
    var sumSquares = 0.0;
    for (point in points) {
      var residual = normal.dot(point) - offset;
      sumSquares += residual * residual;
    }
    var rms = Math.sqrt(sumSquares / points.length);
    return new PlaneEstimate(normal, offset, points.length, rms);
  }

  /**
   * Seeded RANSAC: repeatedly samples three points to build a candidate
   * plane, keeps the candidate with the most inliers within
   * `inlierThreshold`, then refits (`fit`) over just that consensus set for
   * the final estimate. Deterministic for a fixed `seed`.
   */
  public static function fitRansac(points:Array<Vec3>, seed:Int, ?maxIterations:Int = 500,
      ?inlierThreshold:Float = 0.01, ?referenceNormal:Vec3):PlaneEstimate {
    if (points == null || points.length < 3) throw "RANSAC plane fit requires at least three points";
    if (!Math.isFinite(inlierThreshold) || inlierThreshold <= 0.0)
      throw "RANSAC plane fit inlier threshold must be positive and finite";
    var rng = new SeededRandom(seed);
    var bestInliers:Array<Int> = [];

    var iteration = 0;
    while (iteration < maxIterations) {
      iteration++;
      var i0 = rng.nextInt(points.length);
      var i1 = rng.nextInt(points.length);
      var i2 = rng.nextInt(points.length);
      if (i0 == i1 || i1 == i2 || i0 == i2) continue;
      var p0 = points[i0], p1 = points[i1], p2 = points[i2];
      var raw = p1.sub(p0).cross(p2.sub(p0));
      var length = raw.norm();
      if (!Math.isFinite(length) || length < 1e-9) continue;
      var normal = raw.scale(1.0 / length);
      if (referenceNormal != null && normal.dot(referenceNormal) < 0.0) normal = normal.negate();
      var offset = normal.dot(p0);

      var inliers:Array<Int> = [];
      for (i in 0...points.length) {
        var residual = Math.abs(normal.dot(points[i]) - offset);
        if (residual <= inlierThreshold) inliers.push(i);
      }
      if (inliers.length > bestInliers.length) bestInliers = inliers;
    }

    if (bestInliers.length < 3) throw "RANSAC plane fit found no consensus plane";
    var inlierPoints:Array<Vec3> = [];
    for (index in bestInliers) inlierPoints.push(points[index]);
    return fit(inlierPoints, referenceNormal);
  }

  static function centroidOf(points:Array<Vec3>):Vec3 {
    var sum = Vec3.zero();
    for (point in points) sum = sum.add(point);
    return sum.scale(1.0 / points.length);
  }

  static function covarianceOf(points:Array<Vec3>, centroid:Vec3):Array<Array<Float>> {
    var xx = 0.0, xy = 0.0, xz = 0.0, yy = 0.0, yz = 0.0, zz = 0.0;
    for (point in points) {
      var d = point.sub(centroid);
      xx += d.x * d.x; xy += d.x * d.y; xz += d.x * d.z;
      yy += d.y * d.y; yz += d.y * d.z; zz += d.z * d.z;
    }
    var n = points.length;
    return [
      [xx / n, xy / n, xz / n],
      [xy / n, yy / n, yz / n],
      [xz / n, yz / n, zz / n]
    ];
  }
}
