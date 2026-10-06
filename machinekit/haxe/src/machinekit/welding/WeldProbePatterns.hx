package machinekit.welding;

import cadkit.modeling.Vector;
import machinekit.welding.WeldProbeGeometry.WeldProbeFace;
import machinekit.welding.WeldProbeParkingBounds.WeldProbeRegionBounds;

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
  /** Preserve lattice-order ties while rejecting unreachable candidates in geometric rank order. */
  static function reachableBest(candidates:Array<Vector>, rank:Vector->Float, face:WeldProbeFace,
      uncertainty:WeldProbeUncertainty, possible:Null<WeldProbeFace->Vector->WeldProbeRegionBounds->Bool>):Null<Vector> {
    while (candidates.length > 0) {
      var chosen = 0;
      for (index in 1...candidates.length)
        if (rank(candidates[index]) > rank(candidates[chosen])) chosen = index;
      var point = candidates[chosen];
      if (possible == null || possible(face, point, uncertainty.region(face, point))) return point;
      candidates.splice(chosen, 1);
    }
    return null;
  }

  public static function stages(geometry:WeldProbeGeometry, count:Int, previousNormals:Array<Vector>,
      uncertainty:WeldProbeUncertainty, clearance:Float = 3, divisions:Int = 33,
      ?possible:WeldProbeFace->Vector->WeldProbeRegionBounds->Bool, ?onlyFace:WeldProbeFace):Array<WeldProbeStage> {
    if (geometry == null || uncertainty == null || previousNormals == null || count < 1 || count > 3 ||
        previousNormals.length != 3 - count || !Math.isFinite(clearance) || !(clearance > 0))
      throw "Contact patterns need CAD geometry, uncertainty and a 3-2-1 sequence of independent normals";
    var normals = [for (normal in previousNormals) normal.normalized()];
    if (normals.length == 2 && normals[0].cross(normals[1]).length() < 1e-3)
      throw "The first two registration planes must be independent";
    var stages:Array<WeldProbeStage> = [];
    for (face in geometry.faces) if (face.target && (onlyFace == null || face == onlyFace)) {
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
        var initial = reachableBest(candidates, (_) -> 0.0, face, uncertainty, possible);
        if (initial == null) continue;
        var first:Vector = initial, second:Vector = initial;
        for (_ in 0...4) {
          var origin = first;
          second = cast reachableBest(candidates, (point) -> {
            var delta = point.subtract(origin); return delta.dot(delta);
          }, face, uncertainty, possible);
          var swap = first; first = second; second = swap;
        }
        var anchor = first, edge = second.subtract(first);
        var third:Vector = cast reachableBest(candidates,
          (point) -> Math.abs(edge.cross(point.subtract(anchor)).dot(face.normal)), face, uncertainty, possible);
        var area = Math.abs(edge.cross(third.subtract(anchor)).dot(face.normal));
        if (area < clearance * clearance) continue;
        points = [first, second, third]; score = area;
        travel = Math.max(uncertainty.region(face, first).normalTravel,
          Math.max(uncertainty.region(face, second).normalTravel, uncertainty.region(face, third).normalTravel)) + clearance;
      } else if (count == 2) {
        // Separation along the planes' intersection observes rotation about the first plane's normal.
        var axis = normals[0].cross(face.normal).normalized();
        var first = reachableBest(candidates, (point) -> -point.dot(axis), face, uncertainty, possible);
        if (first == null) continue;
        var second = reachableBest(candidates, (point) -> point.dot(axis), face, uncertainty, possible);
        if (second == null) continue;
        var span = Math.abs(second.subtract(first).dot(axis));
        if (span < clearance) continue;
        points = [first, second]; score = span * span;
        travel = Math.max(uncertainty.region(face, first).normalTravel, uncertainty.region(face, second).normalTravel) + clearance;
      } else {
        // Only one contact is needed: screen nearest points lazily instead of solving IK for the whole patch.
        var chosen:Null<Vector> = null, distance = Math.POSITIVE_INFINITY;
        while (candidates.length > 0) {
          var nearest = 0;
          distance = Math.POSITIVE_INFINITY;
          for (index in 0...candidates.length) {
            var delta = candidates[index].subtract(face.centre), square = delta.dot(delta);
            if (square < distance) { nearest = index; distance = square; }
          }
          var point = candidates[nearest];
          candidates.splice(nearest, 1);
          var bounds = uncertainty.region(face, point);
          if (possible != null && !possible(face, point, bounds)) continue;
          chosen = point; travel = bounds.normalTravel + clearance; break;
        }
        if (chosen == null) continue;
        points = [cast chosen]; score = -distance;
      }
      stages.push(new WeldProbeStage(face, points, travel, score));
    }
    stages.sort((a, b) -> a.score == b.score ? Reflect.compare(a.face.name(), b.face.name()) : Reflect.compare(b.score, a.score));
    return stages;
  }
}
