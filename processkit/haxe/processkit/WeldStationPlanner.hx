package processkit;

import robotkit.mobile.Pose2;

/** A parking pose supplied by the CAD-derived candidate generator. */
class WeldStationCandidate {
  public final id:String;
  public final pose:Pose2;
  public function new(id:String, pose:Pose2) {
    if (id == null || id == "" || pose == null) throw "A welding station requires an id and pose";
    this.id = id;
    this.pose = pose;
  }
}

/** Stations in navigation order, with each required seam assigned exactly once. */
class WeldStationPlan {
  public final stations:Array<WeldStationCandidate>;
  public final seams:Array<Array<String>>;
  public final travelCost:Float;
  public function new(stations:Array<WeldStationCandidate>, seams:Array<Array<String>>, travelCost:Float) {
    this.stations = stations.copy();
    this.seams = [for (group in seams) group.copy()];
    this.travelCost = travelCost;
  }
}

/**
 * Exact minimum station cover over a finite candidate set, then minimum directed route cost.
 * Access must check the complete weld motion at its torch angles, including entry and exit.
 * Route cost comes from navigation; null means no route. This policy owns neither CAD nor IK.
 * The caller orders weld runs within each returned station using the existing weld planner.
 */
class WeldStationPlanner {
  public static function plan(required:Array<String>, candidates:Array<WeldStationCandidate>, start:Pose2,
      access:(WeldStationCandidate, String) -> Null<String>,
      route:(Pose2, Pose2) -> Null<Float>):WeldStationPlan {
    if (required == null || candidates == null || start == null || access == null || route == null)
      throw "Station planning requires seams, candidates, start, access and navigation";
    var names = new Map<String, Bool>();
    for (name in required) {
      if (name == null || name == "" || names.exists(name)) throw "Station seams must have unique nonempty names";
      names.set(name, true);
    }
    var ids = new Map<String, Bool>();
    for (candidate in candidates) {
      if (candidate == null || ids.exists(candidate.id)) throw "Station candidates must have unique ids";
      ids.set(candidate.id, true);
    }
    if (required.length == 0) return new WeldStationPlan([], [], 0);
    return new StationSearch(required, candidates, start, access, route).solve();
  }
}

private class StationSearch {
  final required:Array<String>;
  final candidates:Array<WeldStationCandidate>;
  final covers:Array<Array<Bool>> = [];
  final start:Pose2;
  final route:(Pose2, Pose2) -> Null<Float>;
  final costs:Map<String, Null<Float>> = new Map();
  var best:Array<Int> = [];
  var bestCost:Float = Math.POSITIVE_INFINITY;

  public function new(required:Array<String>, candidates:Array<WeldStationCandidate>, start:Pose2,
      access:(WeldStationCandidate, String) -> Null<String>, route:(Pose2, Pose2) -> Null<Float>) {
    this.required = required.copy();
    this.candidates = candidates.copy();
    this.start = start;
    this.route = route;
    var failures:Array<Array<String>> = [for (_ in required) []];
    for (candidate in candidates) {
      var covered:Array<Bool> = [];
      for (index in 0...required.length) {
        var problem = access(candidate, required[index]);
        covered.push(problem == null);
        if (problem != null) failures[index].push(candidate.id + ": " + problem);
      }
      covers.push(covered);
    }
    for (index in 0...required.length) {
      var reachable = false;
      for (coverage in covers) if (coverage[index]) reachable = true;
      if (!reachable) throw 'No welding station covers ${required[index]}: ${failures[index].join("; ")}';
    }
  }

  static function checkedCost(value:Null<Float>):Null<Float> {
    if (value != null && (!Math.isFinite(value) || value < 0))
      throw "Navigation route costs must be finite and nonnegative, or null for unreachable";
    return value;
  }

  function edgeCost(from:Int, to:Int):Null<Float> {
    var key = '$from>$to';
    if (!costs.exists(key)) costs.set(key, checkedCost(route(from < 0 ? start : candidates[from].pose, candidates[to].pose)));
    return costs.get(key);
  }

  public function solve():WeldStationPlan {
    for (size in 1...candidates.length + 1) {
      subsets([], 0, size);
      if (best.length > 0) {
        var assigned = new Map<String, Bool>();
        var groups:Array<Array<String>> = [];
        for (index in best) {
          var group:Array<String> = [];
          for (seam in 0...required.length) if (covers[index][seam] && !assigned.exists(required[seam])) {
            group.push(required[seam]);
            assigned.set(required[seam], true);
          }
          groups.push(group);
        }
        return new WeldStationPlan([for (index in best) candidates[index]], groups, bestCost);
      }
    }
    throw "Welding stations cover the seams but no navigation route visits a complete cover";
  }

  function subsets(selected:Array<Int>, next:Int, remaining:Int):Void {
    if (candidates.length - next < remaining) return;
    // Reject a branch as soon as its remaining candidates cannot cover an outstanding seam.
    for (seam in 0...required.length) {
      var possible = false;
      for (index in selected) if (covers[index][seam]) possible = true;
      if (remaining > 0) for (index in next...candidates.length) if (covers[index][seam]) possible = true;
      if (!possible) return;
    }
    if (remaining == 0) {
      visit(selected, [], 0);
      return;
    }
    for (index in next...candidates.length) {
      var extended = selected.copy();
      extended.push(index);
      subsets(extended, index + 1, remaining - 1);
    }
  }

  function visit(remaining:Array<Int>, order:Array<Int>, cost:Float):Void {
    if (cost >= bestCost) return;
    if (remaining.length == 0) {
      best = order.copy();
      bestCost = cost;
      return;
    }
    for (index in remaining) {
      var edge = edgeCost(order.length == 0 ? -1 : order[order.length - 1], index);
      if (edge == null) continue;
      var pending = remaining.copy();
      pending.remove(index);
      var extended = order.copy();
      extended.push(index);
      visit(pending, extended, cost + edge);
    }
  }
}
