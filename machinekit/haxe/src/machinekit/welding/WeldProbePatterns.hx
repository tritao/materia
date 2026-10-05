package machinekit.welding;

import cadkit.modeling.Vector;
import machinekit.welding.WeldProbeGeometry.WeldProbeFace;

/** A CAD-derived stage of a 3–2–1 registration sequence, before arm-motion feasibility checks. */
class WeldProbeStage {
  public final face:WeldProbeFace;
  public final points:Array<Vector>;
  public final approach:Float;
  public final score:Float;
  public function new(face:WeldProbeFace, points:Array<Vector>, approach:Float, score:Float) {
    this.face = face; this.points = points.copy(); this.approach = approach; this.score = score;
  }
}

/** Select geometrically observable contact patterns from patches large enough for the current uncertainty. */
class WeldProbePatterns {
  public static function stages(geometry:WeldProbeGeometry, count:Int, previousNormals:Array<Vector>,
      uncertainty:WeldProbeUncertainty, clearance:Float = 3, divisions:Int = 33):Array<WeldProbeStage> {
    if (geometry == null || uncertainty == null || previousNormals == null || count < 1 || count > 3 ||
        previousNormals.length != 3 - count || !Math.isFinite(clearance) || !(clearance > 0))
      throw "Contact patterns need CAD geometry, uncertainty and a 3-2-1 sequence of independent normals";
    var normals = [for (normal in previousNormals) normal.normalized()];
    if (normals.length == 2 && normals[0].cross(normals[1]).length() < 1e-3)
      throw "The first two registration planes must be independent";
    var stages:Array<WeldProbeStage> = [];
    for (face in geometry.faces) if (face.target) {
      if (count == 2 && normals[0].cross(face.normal).length() < 1e-3) continue;
      if (count == 1 && Math.abs(normals[0].cross(normals[1]).dot(face.normal)) < 1e-3) continue;
      var candidates:Array<Vector> = [];
      var travel = 0.0;
      for (point in face.samples(divisions, clearance)) {
        var bounds = uncertainty.region(face, point);
        if (!face.containsRegion(point, bounds.halfU, bounds.halfV, clearance)) continue;
        var approach = bounds.normalTravel + clearance;
        if (!geometry.exposedRegion(face, point, bounds.halfU + approach * bounds.tiltU,
          bounds.halfV + approach * bounds.tiltV, approach, clearance)) continue;
        candidates.push(point); travel = Math.max(travel, approach);
      }
      if (candidates.length < count) continue;
      var points:Array<Vector> = [];
      var score = 0.0;
      if (count == 3) {
        // Farthest-point sweeps find a broad baseline without a quadratic scan of the whole lattice.
        var first = candidates[0], second = first;
        for (_ in 0...4) {
          var distance = -1.0;
          for (point in candidates) {
            var delta = point.subtract(first), square = delta.dot(delta);
            if (square > distance) { second = point; distance = square; }
          }
          var swap = first; first = second; second = swap;
        }
        var third = first, area = 0.0;
        for (point in candidates) {
          var value = Math.abs(second.subtract(first).cross(point.subtract(first)).dot(face.normal));
          if (value > area) { third = point; area = value; }
        }
        if (area < clearance * clearance) continue;
        points = [first, second, third]; score = area;
      } else if (count == 2) {
        // Separation along the planes' intersection observes rotation about the first plane's normal.
        var axis = normals[0].cross(face.normal).normalized();
        var first = candidates[0], second = first;
        for (point in candidates) {
          if (point.dot(axis) < first.dot(axis)) first = point;
          if (point.dot(axis) > second.dot(axis)) second = point;
        }
        var span = Math.abs(second.subtract(first).dot(axis));
        if (span < clearance) continue;
        points = [first, second]; score = span * span;
      } else {
        var chosen = candidates[0], distance = Math.POSITIVE_INFINITY;
        for (point in candidates) {
          var delta = point.subtract(face.centre), square = delta.dot(delta);
          if (square < distance) { chosen = point; distance = square; }
        }
        points = [chosen]; score = -distance;
      }
      stages.push(new WeldProbeStage(face, points, travel, score));
    }
    stages.sort((a, b) -> a.score == b.score ? Reflect.compare(a.face.name(), b.face.name()) : Reflect.compare(b.score, a.score));
    return stages;
  }
}
