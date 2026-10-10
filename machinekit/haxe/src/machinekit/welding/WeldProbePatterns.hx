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
  /** True only when the optional reach screen accepted this one-contact point. */
  public final screenPassed:Bool;
  public function new(face:WeldProbeFace, points:Array<Vector>, approach:Float, score:Float, screenPassed:Bool = false) {
    this.face = face; this.points = points.copy(); this.approach = approach; this.score = score;
    this.screenPassed = screenPassed;
  }
}

/** Select geometrically observable contact patterns from patches large enough for the current uncertainty. */
class WeldProbePatterns {
  static inline final ALTERNATE_PAIR_COUNT = 4;
  /** Preserve lattice-order ties while rejecting unreachable candidates in geometric rank order. */
  static function reachableBest(candidates:Array<Vector>, rank:Vector->Float, face:WeldProbeFace,
      uncertainty:WeldProbeUncertainty, possible:Null<WeldProbeFace->Vector->WeldProbeRegionBounds->Bool>,
      screened:Array<Null<Bool>>):Null<Vector> {
    while (candidates.length > 0) {
      var chosen = 0;
      for (index in 1...candidates.length)
        if (rank(candidates[index]) > rank(candidates[chosen])) chosen = index;
      var point = candidates[chosen];
      var reachable = screened[chosen];
      if (reachable == null) {
        reachable = possible == null || possible(face, point, uncertainty.region(face, point));
        screened[chosen] = reachable;
      }
      if (reachable) return point;
      candidates.splice(chosen, 1);
      // Keep the parallel feasibility cache aligned with the candidate order.
      // Shift explicitly because Haxeon's nullable Array.splice result currently
      // produces invalid IR for Array<Null<Bool>>.
      for (index in chosen...screened.length - 1) screened[index] = screened[index + 1];
      screened.pop();
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
      // Farthest-point selection asks about the same rejected lattice points
      // several times. Screening may run robot IK, so cache each point's
      // feasibility for this face and resolution.
      var screened:Array<Null<Bool>> = [for (_ in candidates) null];
      var score = 0.0;
      if (count == 3) {
        // Farthest-point sweeps find a broad baseline without a quadratic scan of the whole lattice.
        var initial = reachableBest(candidates, (_) -> 0.0, face, uncertainty, possible, screened);
        if (initial == null) continue;
        var first:Vector = initial, second:Vector = initial;
        for (_ in 0...4) {
          var origin = first;
          second = cast reachableBest(candidates, (point) -> {
            var delta = point.subtract(origin); return delta.dot(delta);
          }, face, uncertainty, possible, screened);
          var swap = first; first = second; second = swap;
        }
        var anchor = first, edge = second.subtract(first);
        var third:Vector = cast reachableBest(candidates,
          (point) -> Math.abs(edge.cross(point.subtract(anchor)).dot(face.normal)), face, uncertainty, possible, screened);
        var area = Math.abs(edge.cross(third.subtract(anchor)).dot(face.normal));
        if (area < clearance * clearance) continue;
        points = [first, second, third]; score = area;
        travel = Math.max(uncertainty.region(face, first).normalTravel,
          Math.max(uncertainty.region(face, second).normalTravel, uncertainty.region(face, third).normalTravel)) + clearance;
      } else if (count == 2) {
        // Separation along the planes' intersection observes rotation about the first plane's normal.
        var axis = normals[0].cross(face.normal).normalized();
        var checkedPoints = new Map<String, Bool>();
        var pairScreen:Null<WeldProbeFace->Vector->WeldProbeRegionBounds->Bool> = null;
        if (possible != null) {
          var check:WeldProbeFace->Vector->WeldProbeRegionBounds->Bool = cast possible;
          pairScreen = (candidateFace, point, region) -> {
            var key = point.x + ":" + point.y + ":" + point.z;
            var cached = checkedPoints.get(key);
            if (cached != null) return cached;
            var reachable = check(candidateFace, point, region);
            checkedPoints.set(key, reachable);
            return reachable;
          };
        }
        function extremes(rank:Vector->Float):Array<Vector> {
          var remaining:Array<Vector> = [], remainingScreened:Array<Null<Bool>> = [];
          for (index in 0...candidates.length) {
            remaining.push(candidates[index]);
            remainingScreened.push(screened[index]);
          }
          var selected:Array<Vector> = [];
          for (_ in 0...2) {
            var point = reachableBest(remaining, rank, face, uncertainty, pairScreen, remainingScreened);
            if (point == null) break;
            selected.push(point);
            var next:Array<Vector> = [], nextScreened:Array<Null<Bool>> = [];
            for (index in 0...remaining.length) if (remaining[index].subtract(point).length() > 1e-8) {
              next.push(remaining[index]); nextScreened.push(remainingScreened[index]);
            }
            remaining = next; remainingScreened = nextScreened;
          }
          return selected;
        }
        var low = extremes((point) -> -point.dot(axis));
        var high = extremes((point) -> point.dot(axis));
        var pairs:Array<{first:Vector, second:Vector, span:Float}> = [];
        for (first in low) for (second in high) {
          var span = Math.abs(second.subtract(first).dot(axis));
          if (span < clearance) continue;
          var duplicate = false;
          for (pair in pairs) if (pair.first.subtract(first).length() < 1e-8 && pair.second.subtract(second).length() < 1e-8 ||
              pair.first.subtract(second).length() < 1e-8 && pair.second.subtract(first).length() < 1e-8) duplicate = true;
          if (!duplicate) pairs.push({first: first, second: second, span: span});
        }
        pairs.sort((a, b) -> Reflect.compare(b.span, a.span));
        var pairCount = 0;
        for (pair in pairs) {
          if (pairCount >= ALTERNATE_PAIR_COUNT) break;
          var approach = Math.max(uncertainty.region(face, pair.first).normalTravel,
            uncertainty.region(face, pair.second).normalTravel) + clearance;
          stages.push(new WeldProbeStage(face, [pair.first, pair.second], approach, pair.span * pair.span));
          pairCount++;
        }
        continue;
      } else {
        if (possible != null) {
          // A single contact has no observability benefit from a farthest-point
          // pattern. Screen a bounded spread, ordered toward the face centre;
          // the checked planner still proves each selected approach.
          var screen:WeldProbeFace->Vector->WeldProbeRegionBounds->Bool = cast possible;
          candidates.sort((a, b) -> {
            var da = a.subtract(face.centre), db = b.subtract(face.centre);
            var sa = da.dot(da), sb = db.dot(db);
            if (sa != sb) return Reflect.compare(sa, sb);
            if (a.x != b.x) return Reflect.compare(a.x, b.x);
            if (a.y != b.y) return Reflect.compare(a.y, b.y);
            return Reflect.compare(a.z, b.z);
          });
          var last = candidates.length - 1;
          var unique = new Map<Int, Bool>();
          for (sample in 0...8) {
            var index = Std.int(last * sample / 7);
            if (unique.exists(index)) continue;
            unique.set(index, true);
            var point = candidates[index];
            var bounds = uncertainty.region(face, point);
            var clear = screen(face, point, bounds);
            var square = point.subtract(face.centre).dot(point.subtract(face.centre));
            stages.push(new WeldProbeStage(face, [point], bounds.normalTravel + clearance,
              (clear ? 1000000.0 : 0.0) - square, clear));
          }
          continue;
        }
        // Geometry-only ranking uses the nearest patch point.
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
