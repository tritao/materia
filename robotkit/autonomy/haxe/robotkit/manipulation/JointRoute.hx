package robotkit.manipulation;

private typedef RouteNode = {var q:Array<Float>; var parent:Int;}

/** Bounded, deterministic bidirectional joint-space search. The caller verifies directed motion edges. */
class JointRoute {
  static final PRIMES:Array<Int> = [2, 3, 5, 7, 11, 13, 17, 19, 23, 29, 31, 37];

  static function halton(index:Int, base:Int):Float {
    var value = 0.0, fraction = 1.0;
    while (index > 0) {
      fraction /= base;
      value += fraction * (index % base);
      index = Std.int(index / base);
    }
    return value;
  }

  static function distance(a:Array<Float>, b:Array<Float>, spans:Array<Float>):Float {
    var sum = 0.0;
    for (i in 0...a.length) { var d = (a[i] - b[i]) / spans[i]; sum += d * d; }
    return Math.sqrt(sum);
  }

  static function nearest(tree:Array<RouteNode>, q:Array<Float>, spans:Array<Float>):Int {
    var best = 0, cost = Math.POSITIVE_INFINITY;
    for (i in 0...tree.length) {
      var found = distance(tree[i].q, q, spans);
      if (found < cost) { cost = found; best = i; }
    }
    return best;
  }

  static function branch(tree:Array<RouteNode>, index:Int):Array<Array<Float>> {
    var result:Array<Array<Float>> = [];
    while (index >= 0) { result.push(tree[index].q.copy()); index = tree[index].parent; }
    result.reverse();
    return result;
  }

  /** Includes both endpoints. A failed bounded search is explicit, never evidence that no route exists. */
  public static function plan(from:Array<Float>, to:Array<Float>, lower:Array<Float>, upper:Array<Float>,
      clear:(Array<Float>, Array<Float>) -> Bool, iterations:Int = 1024, step:Float = 0.08):Array<Array<Float>> {
    if (from == null || to == null || lower == null || upper == null || clear == null || from.length == 0 ||
        from.length > PRIMES.length || to.length != from.length || lower.length != from.length || upper.length != from.length ||
        iterations < 1 || !(step > 0)) throw "Joint route needs matching finite bounds and a positive search budget";
    var spans:Array<Float> = [];
    for (i in 0...from.length) {
      if (!Math.isFinite(lower[i]) || !Math.isFinite(upper[i]) || !(upper[i] > lower[i]) ||
          !Math.isFinite(from[i]) || !Math.isFinite(to[i]) || from[i] < lower[i] || from[i] > upper[i] ||
          to[i] < lower[i] || to[i] > upper[i]) throw "Joint route endpoint is outside its finite bounds";
      spans.push(upper[i] - lower[i]);
    }
    if (!clear(from, from) || !clear(to, to)) throw "Joint route endpoint is blocked";
    if (clear(from, to)) return [from.copy(), to.copy()];
    var starts:Array<RouteNode> = [{q: from.copy(), parent: -1}];
    var goals:Array<RouteNode> = [{q: to.copy(), parent: -1}];
    for (iteration in 0...iterations) {
      var forward = iteration % 2 == 0;
      var tree = forward ? starts : goals, other = forward ? goals : starts;
      // Both trees see the complete low-discrepancy sequence; parity must not partition base-two samples.
      var sampleIndex = Std.int(iteration / 2) + 1;
      var sample = [for (i in 0...from.length) lower[i] + spans[i] * halton(sampleIndex, PRIMES[i])];
      var parent = nearest(tree, sample, spans);
      var length = distance(tree[parent].q, sample, spans);
      if (length < 1e-12) continue;
      var fraction = Math.min(1.0, step / length);
      var q = [for (i in 0...from.length) tree[parent].q[i] + (sample[i] - tree[parent].q[i]) * fraction];
      if (!(forward ? clear(tree[parent].q, q) : clear(q, tree[parent].q))) continue;
      tree.push({q: q, parent: parent});
      var connection = nearest(other, q, spans);
      if (!(forward ? clear(q, other[connection].q) : clear(other[connection].q, q))) continue;
      var first = branch(starts, forward ? starts.length - 1 : connection);
      var last = branch(goals, forward ? connection : goals.length - 1);
      last.reverse();
      var route = first.concat(last);
      // Shortcut only through checked directed edges; keep the exact endpoint configurations.
      var result = [route[0]], index = 0;
      while (index < route.length - 1) {
        var next = route.length - 1;
        while (next > index + 1 && !clear(route[index], route[next])) next--;
        result.push(route[next]); index = next;
      }
      return result;
    }
    throw 'Joint route search exhausted $iterations proposals (${starts.length} start nodes, ${goals.length} goal nodes)';
  }
}
