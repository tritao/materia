package motionkit.robot;

import motionkit.kinematics.IkTolerance;
import motionkit.kinematics.KinematicsSolver;
import motionkit.kinematics.PathRequest;
import motionkit.kinematics.Pose3;

/** One configuration in the beam: its cheapest cost from the path's start, and the previous sample's candidate it came from. */
private class Candidate {
  public final q:Array<Float>;
  public final cost:Float;
  public final parent:Int;

  public function new(q:Array<Float>, cost:Float, parent:Int) {
    this.q = q;
    this.cost = cost;
    this.parent = parent;
  }
}

/**
 * Chooses a group's redundancy along a whole path (KINEMATICS.md KK-D19),
 * for any `RedundancyParameterization`:
 *
 * 1. A beam search over a redundancy lattice. Each of a sample's candidates
 *    continues to the next sample with its redundancy held, and moved one
 *    lattice step up or down in each value (the step is the
 *    parameterization's rate times the distance between the samples); where
 *    a limit blocks the held value, it continues with the redundancy as a
 *    preference. Every new configuration gets its cheapest cost from the
 *    start over all the previous candidates it can follow (no joint moving
 *    more than `maxJump`; each DOF's motion weighed by its cost factor over
 *    its speed), and the `maxCandidates` cheapest, one per lattice cell,
 *    go on. Cheap routes survive whichever way they move; the route is read
 *    back through each candidate's best predecessor.
 * 2. Smooth the route's redundancy (a Gaussian over about an eighth of the
 *    samples, the first pinned, the values reflected through each end so a
 *    steady trend stays straight there) and re-solve every sample exactly there, so
 *    the joints do not step between lattice cells. If a sample does not
 *    solve or a joint would jump, the route stands as found.
 */
class RedundancyResolver {
  public final parameterization:RedundancyParameterization;

  public function new(parameterization:RedundancyParameterization) {
    if (parameterization == null) throw "A redundancy resolver needs a parameterization";
    this.parameterization = parameterization;
  }

  /** The path for `request`. Throws, naming the sample, where no candidate continues. */
  public function solvePath(solver:KinematicsSolver, request:PathRequest):Array<Null<Array<Float>>> {
    var costs = parameterization.costFactors();
    var weights = [for (j in 0...request.velocity.length) costs[j] / request.velocity[j]];
    var rates = parameterization.ratesPerMetre();
    var layers:Array<Array<Candidate>> = [[new Candidate(request.startQ.copy(), 0.0, -1)]];
    for (index in 1...request.poses.length) {
      var spacing = request.distances[index] - request.distances[index - 1];
      var steps = [for (rate in rates) rate * spacing];
      var next = grow(request.poses[index], layers[index - 1], steps, weights, request);
      if (next.length == 0) throw 'No continuous configuration reaches path sample $index';
      layers.push(next);
    }
    // The cheapest end, traced back.
    var last = layers[layers.length - 1];
    var best = 0;
    for (k in 1...last.length) if (last[k].cost < last[best].cost) best = k;
    var chosen:Array<Array<Float>> = [];
    var at = best;
    var layer = layers.length - 1;
    while (layer >= 0) {
      chosen.push(layers[layer][at].q);
      at = layers[layer][at].parent;
      layer--;
    }
    chosen.reverse();
    var refined = refine(request, chosen);
    var result:Array<Null<Array<Float>>> = [];
    for (q in (refined != null ? refined : chosen)) result.push(q);
    return result;
  }

  function grow(target:Pose3, previous:Array<Candidate>, steps:Array<Float>, weights:Array<Float>,
      request:PathRequest):Array<Candidate> {
    var dimension = parameterization.dimension();
    var tolerance = request.tolerance;
    // Held first, then one step up and down along each value.
    var offsets:Array<Array<Float>> = [[for (_ in 0...dimension) 0.0]];
    for (d in 0...dimension) for (sign in [1.0, -1.0])
      offsets.push([for (k in 0...dimension) k == d ? sign : 0.0]);
    var found:Array<Candidate> = [];
    var cells = new Map<String, Int>();
    for (o in 0...offsets.length) for (seed in previous) {
      var values = parameterization.valuesAt(seed.q);
      if (values == null) continue;
      var goal = [for (d in 0...dimension) values[d] + offsets[o][d] * steps[d]];
      var solved = parameterization.solveAt(target, seed.q, goal, tolerance);
      if (solved == null && o == 0) solved = parameterization.solveNear(target, seed.q, tolerance);
      if (solved == null) continue;
      var reached = parameterization.valuesAt(solved);
      if (reached == null) continue;
      // The cheapest way here from any previous candidate.
      var cost = Math.POSITIVE_INFINITY, parent = -1;
      for (p in 0...previous.length) {
        var step = edge(previous[p].q, solved, weights, request.maxJump);
        if (previous[p].cost + step < cost) {
          cost = previous[p].cost + step;
          parent = p;
        }
      }
      if (parent < 0) continue;
      var key = [for (d in 0...dimension) Std.string(steps[d] > 0.0 ? Math.round(reached[d] / steps[d]) : 0)].join(",");
      var existing = cells.get(key);
      if (existing != null) {
        var index:Int = existing;
        if (cost < found[index].cost) found[index] = new Candidate(solved.copy(), cost, parent);
        continue;
      }
      cells.set(key, found.length);
      found.push(new Candidate(solved.copy(), cost, parent));
    }
    found.sort((a, b) -> a.cost < b.cost ? -1 : a.cost > b.cost ? 1 : 0);
    var kept:Array<Candidate> = [];
    for (candidate in found) {
      if (kept.length >= request.maxCandidates) break;
      var duplicate = false;
      for (other in kept) if (distance(other.q, candidate.q) < tolerance.candidateSeparation) duplicate = true;
      if (!duplicate) kept.push(candidate);
    }
    return kept;
  }

  /** Weighted joint motion between two configurations; infinite where a joint jumps more than allowed. */
  static function edge(from:Array<Float>, to:Array<Float>, weights:Array<Float>, maxJump:Array<Float>):Float {
    var cost = 0.0;
    for (joint in 0...from.length) {
      var move = Math.abs(to[joint] - from[joint]);
      if (move > maxJump[joint]) return Math.POSITIVE_INFINITY;
      cost += weights[joint] * move;
    }
    return cost;
  }

  static function distance(a:Array<Float>, b:Array<Float>):Float {
    var squared = 0.0;
    for (joint in 0...a.length) squared += Math.pow(a[joint] - b[joint], 2);
    return Math.sqrt(squared);
  }

  /** The chosen route with its redundancy smoothed and re-solved exactly, or null where that fails. */
  function refine(request:PathRequest, chosen:Array<Array<Float>>):Null<Array<Array<Float>>> {
    var count = chosen.length;
    if (count < 3) return null;
    var dimension = parameterization.dimension();
    var values:Array<Array<Float>> = [];
    for (q in chosen) {
      var current = parameterization.valuesAt(q);
      if (current == null) return null;
      var row = current.copy();
      // Unwrapped, so smoothing never averages across the ±π seam.
      if (values.length > 0) for (d in 0...dimension) if (parameterization.periodic(d)) {
        var last = values[values.length - 1][d];
        while (row[d] - last > Math.PI) row[d] -= 2.0 * Math.PI;
        while (row[d] - last < -Math.PI) row[d] += 2.0 * Math.PI;
      }
      values.push(row);
    }
    var radius = Std.int(Math.max(3, Math.round(count / 8)));
    var sigma = radius / 2.0;
    // Beyond each end the values are reflected through it (v(-k) = 2·v(0) - v(k)), so a window
    // cut short by an end does not bend a steady trend there.
    function at(j:Int, d:Int):Float {
      if (j < 0) return 2.0 * values[0][d] - values[-j][d];
      if (j >= count) return 2.0 * values[count - 1][d] - values[2 * (count - 1) - j][d];
      return values[j][d];
    }
    var smoothed = [values[0].copy()];
    for (i in 1...count) {
      var row = [for (_ in 0...dimension) 0.0], weights = 0.0;
      for (j in (i - radius)...(i + radius + 1)) {
        if (j < -(count - 1) || j > 2 * (count - 1)) continue;
        var w = Math.exp(-0.5 * Math.pow((j - i) / sigma, 2));
        for (d in 0...dimension) row[d] += w * at(j, d);
        weights += w;
      }
      smoothed.push([for (d in 0...dimension) row[d] / weights]);
    }
    var refined = [chosen[0].copy()];
    for (i in 1...count) {
      var seed = refined[i - 1];
      var solved = parameterization.solveAt(request.poses[i], seed, smoothed[i], request.tolerance);
      if (solved == null) return null;
      for (joint in 0...solved.length) if (Math.abs(solved[joint] - seed[joint]) > request.maxJump[joint]) return null;
      refined.push(solved.copy());
    }
    return refined;
  }
}
