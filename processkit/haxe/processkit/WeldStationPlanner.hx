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

/** A complete scheduled station group failed after its individual seam checks passed. */
class WeldStationPlanFailure {
  public final stationId:String;
  public final seams:Array<String>;
  public final reason:String;
  public function new(stationId:String, seams:Array<String>, reason:String) {
    if (stationId == null || stationId.length == 0 || seams == null || seams.length == 0 ||
        reason == null || reason.length == 0)
      throw "A station-plan failure needs a station, assigned seams and a reason";
    this.stationId = stationId;
    this.seams = seams.copy();
    this.reason = reason;
  }
}

/**
 * Exact minimum station cover over a finite candidate set, then minimum directed route cost.
 * Access must check the complete weld motion at its torch angles, including entry and exit.
 * Route cost comes from navigation; null means no route. This policy owns neither CAD nor IK.
 * The caller orders weld runs within each returned station using the existing weld planner.
 */
class WeldStationPlanner {
  /**
   * Use a conservative cheap screen first, then prove only the selected coverage edges.
   * The screen may reject only when it establishes that the motion is impossible.
   * A null screen result is provisional; it never substitutes for the full motion check.
   * Failed edges are removed and selection repeats. A successful optimistic minimum cover is
   * also a minimum proven cover: rejecting other provisional edges cannot improve its count/cost.
   */
  public static function verified(required:Array<String>, candidates:Array<WeldStationCandidate>, start:Pose2,
      screen:(WeldStationCandidate, String) -> Null<String>,
      verify:(WeldStationCandidate, String) -> Null<String>,
      route:(Pose2, Pose2) -> Null<Float>,
      ?verifyPlan:WeldStationPlan -> Null<WeldStationPlanFailure>):WeldStationPlan {
    if (screen == null || verify == null) throw "Verified station planning requires a screen and full motion check";
    return new StationVerification(screen, verify, verifyPlan).solve(required, candidates, start, route);
  }

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

private class StationVerification {
  final screen:(WeldStationCandidate, String) -> Null<String>;
  final verify:(WeldStationCandidate, String) -> Null<String>;
  final verifyPlan:Null<WeldStationPlan -> Null<WeldStationPlanFailure>>;
  final screened:Map<String, Map<String, Null<String>>> = new Map();
  final verified:Map<String, Map<String, Null<String>>> = new Map();

  public function new(screen:(WeldStationCandidate, String) -> Null<String>,
      verify:(WeldStationCandidate, String) -> Null<String>,
      verifyPlan:Null<WeldStationPlan -> Null<WeldStationPlanFailure>>) {
    this.screen = screen;
    this.verify = verify;
    this.verifyPlan = verifyPlan;
  }

  function results(table:Map<String, Map<String, Null<String>>>, id:String):Map<String, Null<String>> {
    var found = table.get(id);
    if (found != null) return found;
    var made:Map<String, Null<String>> = new Map();
    table.set(id, made);
    return made;
  }

  function access(station:WeldStationCandidate, seam:String):Null<String> {
    var full = results(verified, station.id);
    if (full.exists(seam)) return full.get(seam);
    var cheap = results(screened, station.id);
    if (!cheap.exists(seam)) cheap.set(seam, screen(station, seam));
    return cheap.get(seam);
  }

  public function solve(required:Array<String>, candidates:Array<WeldStationCandidate>, start:Pose2,
      route:(Pose2, Pose2) -> Null<Float>):WeldStationPlan {
    while (true) {
      var plan = WeldStationPlanner.plan(required, candidates, start, access, route);
      var complete = true;
      for (index in 0...plan.stations.length) {
        var station = plan.stations[index];
        var full = results(verified, station.id);
        for (seam in plan.seams[index]) {
          if (!full.exists(seam)) full.set(seam, verify(station, seam));
          if (full.get(seam) != null) complete = false;
        }
      }
      if (complete) {
        if (verifyPlan == null) return plan;
        var failure = verifyPlan(plan);
        if (failure == null) return plan;
        var stationIndex = -1;
        for (index in 0...plan.stations.length)
          if (plan.stations[index].id == failure.stationId) stationIndex = index;
        if (stationIndex < 0) throw "Station-plan verification rejected a station outside the proposed plan";
        var full = results(verified, failure.stationId);
        var rejected = 0;
        for (seam in failure.seams) {
          if (plan.seams[stationIndex].indexOf(seam) < 0)
            throw 'Station-plan verification rejected unassigned seam "$seam"';
          if (full.get(seam) == null) {
            full.set(seam, failure.reason);
            rejected++;
          }
        }
        if (rejected == 0) throw "Station-plan verification must reject at least one currently valid assigned seam";
      }
    }
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
      subsets(size);
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

  function subsets(size:Int):Void {
    var visited = new Map<String, Bool>();

    function search(selected:Array<Int>):Void {
      var keyParts = selected.copy();
      keyParts.sort((a, b) -> a - b);
      var key = keyParts.join(",");
      if (visited.exists(key)) return;
      visited.set(key, true);

      var complete = true;
      var uncovered = 0;
      var maximumGain = 0;
      var pivot:Array<Int> = null;
      var pivotCount = candidates.length + 1;
      for (seam in 0...required.length) {
        var alreadyCovered = false;
        for (index in selected) if (covers[index][seam]) { alreadyCovered = true; break; }
        if (alreadyCovered) continue;
        complete = false;
        uncovered++;
        var options:Array<Int> = [];
        for (index in 0...candidates.length) {
          if (selected.indexOf(index) < 0 && covers[index][seam]) options.push(index);
        }
        if (options.length == 0) return;
        if (options.length < pivotCount) { pivot = options; pivotCount = options.length; }
      }

      if (selected.length == size) {
        if (complete) visit(selected, [], 0);
        return;
      }
      if (candidates.length - selected.length < size - selected.length) return;

      if (complete) {
        // A routeable minimum cover can need a redundant station when smaller covers are
        // disconnected. Enumerate the fillers only after all seam coverage is established.
        for (index in 0...candidates.length) if (selected.indexOf(index) < 0) {
          var extended = selected.copy();
          extended.push(index);
          search(extended);
        }
        return;
      }

      for (index in 0...candidates.length) if (selected.indexOf(index) < 0) {
        var gain = 0;
        for (seam in 0...required.length) {
          var covered = false;
          for (chosen in selected) if (covers[chosen][seam]) { covered = true; break; }
          if (!covered && covers[index][seam]) gain++;
        }
        maximumGain = Std.int(Math.max(maximumGain, gain));
      }
      if (maximumGain == 0 || selected.length + Std.int(Math.ceil(uncovered / maximumGain)) > size) return;

      // Cover the seam with the fewest remaining choices first. The visited set removes
      // duplicate subsets reached through overlapping seam coverage.
      if (pivot == null) return;
      for (index in pivot) {
        var extended = selected.copy();
        extended.push(index);
        search(extended);
      }
    }

    search([]);
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
