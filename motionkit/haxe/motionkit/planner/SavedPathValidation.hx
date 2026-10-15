package motionkit.planner;

/** A saved route is only a proposal: endpoints, bounds, points and whole edges are rechecked. */
class SavedPathValidation {
  public static function failure(space:PlannerSpace, path:Null<Array<Array<Float>>>, from:Array<Float>, to:Array<Float>):Null<String> {
    if (path == null || path.length < 2) return "missing complete route";
    var n = space.dimension(), low = space.lower(), high = space.upper();
    for (q in path) {
      if (q == null || q.length != n) return "wrong route dimension";
      for (i in 0...n) if (!Math.isFinite(q[i]) || q[i] < low[i] || q[i] > high[i]) return "route outside joint bounds";
    }
    if (from.length != n || to.length != n) return "wrong endpoint dimension";
    for (i in 0...n) if (path[0][i] != from[i] || path[path.length - 1][i] != to[i]) return "route endpoints changed";
    for (q in path) { var problem = space.invalid(q); if (problem != null) return problem; }
    var edges:Array<Float> = [];
    for (i in 1...path.length) {
      for (value in path[i - 1]) edges.push(value);
      for (value in path[i]) edges.push(value);
    }
    var checked = space.edgesClear(edges, path.length - 1);
    if (checked.length != path.length - 1) throw "Planner returned the wrong edge count";
    for (clear in checked) if (!clear) return "route edge is not certified";
    return null;
  }
}
