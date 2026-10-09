package motionkit.planner;

/**
 * Bidirectional RRT-Connect in joint space, then shortcutting (COLLISION.md
 * CL-D7, CL7), over a `PlannerSpace`. Deterministic from its seed: the same
 * seed, space and endpoints give the same path.
 *
 * Each iteration samples a configuration, extends one tree toward it by at
 * most `step` (Euclidean, joint units), and tries to connect the other tree
 * to the new node: its nearest node's straight steps toward it are checked in
 * one batch, and the clear prefix is added; reaching the node joins the
 * trees. Trees store nodes flat and find neighbours by k-d tree once they
 * grow (`PlannerTree`). Shortcutting then tries `shortcutRounds` random
 * direct edges between waypoints, in batches.
 */
class RrtConnect implements MotionPlanner {
  public final space:PlannerSpace;
  public var seed:Int;
  public var maxIterations:Int = 5000;
  public var step:Float = 0.3;
  public var shortcutRounds:Int = 64;
  public var kdThreshold:Int = 64;
  var checkSeconds = 0.0;
  var edgeChecks = 0;

  public function new(space:PlannerSpace, seed:Int = 1) {
    if (space == null) throw "RRT-Connect needs a planner space";
    this.space = space;
    this.seed = seed;
  }

  public function plan(start:Array<Float>, goal:Array<Float>):MotionPlan {
    var began = Sys.time();
    checkSeconds = 0.0;
    edgeChecks = 0;
    var n = space.dimension();
    if (start == null || goal == null || start.length != n || goal.length != n)
      throw 'RRT-Connect needs a start and a goal of $n joints';
    var lower = space.lower(), upper = space.upper();
    for (q in [start, goal]) for (k in 0...n)
      if (!(q[k] >= lower[k] && q[k] <= upper[k])) return failed('${q == start ? "start" : "goal"} outside the joint bounds', 0, 0, began);
    var startProblem = timed(() -> space.invalid(start));
    if (startProblem != null) return failed('start in collision: $startProblem', 0, 0, began);
    var goalProblem = timed(() -> space.invalid(goal));
    if (goalProblem != null) return failed('goal in collision: $goalProblem', 0, 0, began);
    if (clear([start, goal])[0]) return done([start.copy(), goal.copy()], 0, 2, began);
    var rng = new PlannerRandom(seed);
    var a = new PlannerTree(n, kdThreshold), b = new PlannerTree(n, kdThreshold);
    a.add(start, -1);
    b.add(goal, -1);
    var aIsStart = true;
    for (iteration in 0...maxIterations) {
      var sample = [for (k in 0...n) lower[k] + (upper[k] - lower[k]) * rng.next()];
      var near = a.nearest(sample);
      var from = a.node(near);
      var to = steer(from, sample);
      if (clear([from, to])[0]) {
        var added = a.add(to, near);
        var reached = connect(b, to);
        if (reached >= 0) {
          var path = a.pathTo(added);
          var other = b.pathTo(reached);
          other.reverse();
          var joined = path.concat(other);
          if (!aIsStart) joined.reverse();
          return done(shortcut(joined, rng), iteration + 1, a.count + b.count, began);
        }
      }
      var swap = a;
      a = b;
      b = swap;
      aIsStart = !aIsStart;
    }
    return failed('no path found in $maxIterations iterations', maxIterations, a.count + b.count, began);
  }

  /** Grows `tree` from its node nearest `target` toward it in straight steps; returns the node reaching it, or -1. */
  function connect(tree:PlannerTree, target:Array<Float>):Int {
    var near = tree.nearest(target);
    var from = tree.node(near);
    var total = PlannerTree.distance(from, target);
    var steps = Std.int(Math.max(1, Math.ceil(total / step)));
    var points = [from];
    for (i in 1...steps + 1) {
      var t = i / steps;
      points.push([for (k in 0...from.length) from[k] + (target[k] - from[k]) * t]);
    }
    var flags = clear(points);
    var parent = near;
    for (i in 0...steps) {
      if (!flags[i]) return -1;
      parent = tree.add(points[i + 1], parent);
    }
    return parent;
  }

  /** Whether each consecutive edge of `points` is clear, in one batch. */
  function clear(points:Array<Array<Float>>):Array<Bool> {
    var edges:Array<Float> = [];
    for (i in 1...points.length) {
      for (value in points[i - 1]) edges.push(value);
      for (value in points[i]) edges.push(value);
    }
    edgeChecks += points.length - 1;
    var began = Sys.time();
    var flags = space.edgesClear(edges, points.length - 1);
    checkSeconds += Sys.time() - began;
    return flags;
  }

  function timed(check:Void->Null<String>):Null<String> {
    var began = Sys.time();
    var result = check();
    checkSeconds += Sys.time() - began;
    return result;
  }

  function steer(from:Array<Float>, to:Array<Float>):Array<Float> {
    var d = PlannerTree.distance(from, to);
    if (d <= step) return to.copy();
    var t = step / d;
    return [for (k in 0...from.length) from[k] + (to[k] - from[k]) * t];
  }

  /** Random direct edges between waypoints, eight per batch, replacing what they skip when clear. */
  function shortcut(path:Array<Array<Float>>, rng:PlannerRandom):Array<Array<Float>> {
    var rounds = 0;
    while (rounds < shortcutRounds && path.length > 2) {
      var tries:Array<Array<Int>> = [];
      for (_ in 0...8) {
        var i = Std.int(rng.next() * (path.length - 2));
        var j = i + 2 + Std.int(rng.next() * (path.length - i - 2));
        if (j >= path.length) j = path.length - 1;
        tries.push([i, j]);
      }
      rounds += tries.length;
      var edges:Array<Float> = [];
      for (t in tries) {
        for (value in path[t[0]]) edges.push(value);
        for (value in path[t[1]]) edges.push(value);
      }
      edgeChecks += tries.length;
      var began = Sys.time();
      var flags = space.edgesClear(edges, tries.length);
      checkSeconds += Sys.time() - began;
      // Apply the first clear one (later ones may index a changed path).
      for (k in 0...tries.length) if (flags[k]) {
        var i = tries[k][0], j = tries[k][1];
        path = path.slice(0, i + 1).concat(path.slice(j));
        break;
      }
    }
    return path;
  }

  function done(path:Array<Array<Float>>, iterations:Int, nodes:Int, began:Float):MotionPlan
    return new MotionPlan(path, "", iterations, nodes, edgeChecks, checkSeconds, Sys.time() - began);

  function failed(why:String, iterations:Int, nodes:Int, began:Float):MotionPlan
    return new MotionPlan(null, why, iterations, nodes, edgeChecks, checkSeconds, Sys.time() - began);
}

/** A seeded generator (xorshift32), so plans repeat exactly. */
class PlannerRandom {
  var state:Int;

  public function new(seed:Int) {
    state = seed == 0 ? 0x9e3779b9 : seed;
  }

  /** Uniform in [0, 1). */
  public function next():Float {
    var x = state;
    x ^= x << 13;
    x ^= x >>> 17;
    x ^= x << 5;
    state = x;
    return (x >>> 1) / 2147483648.0;
  }
}
