package robotkit.perception;

import haxe.Int64;
import robotkit.spatial.Vec3;
import robotkit.spatial.Transform3;
import robotkit.work.WorkSurface;
import robotkit.work.Point2;

/**
 * Samples points from a "true" wall that differs from a design `WorkSurface`
 * by a small rigid offset/rotation (`surface_T_trueSurface`, e.g. +14 mm
 * along the surface's local +Z and a small rotation about local +Y
 * representing an installation yaw error) plus a smooth bow, with seeded
 * Gaussian-like noise. Openings (the design surface's exclusions) are
 * skipped, matching what a real scanner would see through a window/door.
 *
 * The bow term is re-centered to zero mean over the actual sampled points
 * before the rigid offset is applied, so `PlaneFit`/`SurfaceRegistration`
 * recovers the injected offset/rotation rather than a bow-biased plane; the
 * bow itself remains visible as a spatial pattern in a `DeviationMap`.
 */
class SimulatedSurfaceScanner {
  public static function scan(design:WorkSurface, surface_T_trueSurface:Transform3, bowAmplitude:Float,
      pointSpacing:Float, noiseStdDev:Float, seed:Int, sourceTimestampNs:Int64):PointCloud {
    if (design == null) throw "Simulated surface scan requires a design work surface";
    if (surface_T_trueSurface == null) throw "Simulated surface scan requires a surface_T_trueSurface transform";
    if (!Math.isFinite(bowAmplitude)) throw "Simulated surface scan bow amplitude must be finite";
    if (!Math.isFinite(pointSpacing) || pointSpacing <= 0.0) throw "Simulated surface scan point spacing must be positive and finite";
    if (!Math.isFinite(noiseStdDev) || noiseStdDev < 0.0) throw "Simulated surface scan noise stddev must be finite and non-negative";

    var bounds = design.boundary.bounds();
    var centerX = (bounds.minX + bounds.maxX) * 0.5;
    var centerY = (bounds.minY + bounds.maxY) * 0.5;
    var radius = Math.max(bounds.maxX - bounds.minX, bounds.maxY - bounds.minY) * 0.5;
    if (radius <= 0.0) radius = 1.0;

    var xs:Array<Float> = [];
    var ys:Array<Float> = [];
    var rawBow:Array<Float> = [];
    var y = bounds.minY;
    while (y <= bounds.maxY) {
      var x = bounds.minX;
      while (x <= bounds.maxX) {
        var candidate = new Point2(x, y);
        var inside = design.boundary.contains(candidate);
        if (inside) {
          for (exclusion in design.exclusions) if (exclusion.contains(candidate)) { inside = false; break; }
        }
        if (inside) {
          var dx = x - centerX, dy = y - centerY;
          var normalizedRadiusSq = (dx * dx + dy * dy) / (radius * radius);
          xs.push(x);
          ys.push(y);
          rawBow.push(bowAmplitude * (1.0 - normalizedRadiusSq));
        }
        x += pointSpacing;
      }
      y += pointSpacing;
    }
    if (xs.length == 0) throw "Simulated surface scan found no samples inside the design boundary";

    var meanBow = 0.0;
    for (value in rawBow) meanBow += value;
    meanBow /= rawBow.length;

    var rng = new SeededRandom(seed);
    var points:Array<Vec3> = [];
    for (i in 0...xs.length) {
      var localZ = (rawBow[i] - meanBow) + rng.nextGaussian() * noiseStdDev;
      points.push(surface_T_trueSurface.transformPoint(new Vec3(xs[i], ys[i], localZ)));
    }
    return new PointCloud(design.surfaceFrameId, points, sourceTimestampNs);
  }
}
