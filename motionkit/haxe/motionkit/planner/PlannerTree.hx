package motionkit.planner;

/**
 * A search tree with its nodes stored flat (one array of coordinates, one of
 * parents) and nearest-neighbour queries by a k-d tree over the nodes it was
 * last built from, plus a scan of those added since (COLLISION.md CL-D7). The
 * k-d tree is rebuilt whenever the tree doubles past `kdThreshold` nodes.
 */
class PlannerTree {
  public final dimension:Int;
  public final coordinates:Array<Float> = [];
  public final parents:Array<Int> = [];
  public var count(default, null):Int = 0;
  final kdThreshold:Int;
  /** k-d tree over nodes [0, built): node index per tree slot, split axis per slot, children by slot. */
  var built = 0;
  var kdNode:Array<Int> = [];
  var kdAxis:Array<Int> = [];
  var kdLeft:Array<Int> = [];
  var kdRight:Array<Int> = [];
  var kdRoot = -1;

  public function new(dimension:Int, kdThreshold:Int = 64) {
    this.dimension = dimension;
    this.kdThreshold = kdThreshold;
  }

  public function add(q:Array<Float>, parent:Int):Int {
    for (value in q) coordinates.push(value);
    parents.push(parent);
    count++;
    if (count >= kdThreshold && count >= 2 * built) rebuild();
    return count - 1;
  }

  public function node(index:Int):Array<Float> return coordinates.slice(index * dimension, (index + 1) * dimension);

  /** The path from the root to `index`, root first. */
  public function pathTo(index:Int):Array<Array<Float>> {
    var path:Array<Array<Float>> = [];
    var current = index;
    while (current >= 0) {
      path.push(node(current));
      current = parents[current];
    }
    path.reverse();
    return path;
  }

  public function nearest(q:Array<Float>):Int {
    var best = -1, bestDistance = Math.POSITIVE_INFINITY;
    if (kdRoot >= 0) {
      var found = searchKd(kdRoot, q, -1, Math.POSITIVE_INFINITY);
      best = found.index;
      bestDistance = found.distance;
    }
    for (index in built...count) {
      var d = squared(index, q);
      if (d < bestDistance) {
        bestDistance = d;
        best = index;
      }
    }
    return best;
  }

  function squared(index:Int, q:Array<Float>):Float {
    var sum = 0.0, o = index * dimension;
    for (k in 0...dimension) {
      var d = coordinates[o + k] - q[k];
      sum += d * d;
    }
    return sum;
  }

  function rebuild():Void {
    built = count;
    kdNode = [];
    kdAxis = [];
    kdLeft = [];
    kdRight = [];
    var indices = [for (i in 0...built) i];
    kdRoot = buildKd(indices, 0, indices.length, 0);
  }

  function buildKd(indices:Array<Int>, from:Int, to:Int, depth:Int):Int {
    if (from >= to) return -1;
    var axis = depth % dimension;
    var slice = indices.slice(from, to);
    slice.sort((a, b) -> {
      var x = coordinates[a * dimension + axis], y = coordinates[b * dimension + axis];
      return x < y ? -1 : x > y ? 1 : a - b;
    });
    for (i in 0...slice.length) indices[from + i] = slice[i];
    var middle = (from + to) >> 1;
    var slot = kdNode.length;
    kdNode.push(indices[middle]);
    kdAxis.push(axis);
    kdLeft.push(-1);
    kdRight.push(-1);
    kdLeft[slot] = buildKd(indices, from, middle, depth + 1);
    kdRight[slot] = buildKd(indices, middle + 1, to, depth + 1);
    return slot;
  }

  function searchKd(slot:Int, q:Array<Float>, best:Int, bestDistance:Float):{index:Int, distance:Float} {
    if (slot < 0) return {index: best, distance: bestDistance};
    var index = kdNode[slot];
    var d = squared(index, q);
    if (d < bestDistance) {
      best = index;
      bestDistance = d;
    }
    var axis = kdAxis[slot];
    var delta = q[axis] - coordinates[index * dimension + axis];
    var near = delta < 0 ? kdLeft[slot] : kdRight[slot], far = delta < 0 ? kdRight[slot] : kdLeft[slot];
    var found = searchKd(near, q, best, bestDistance);
    best = found.index;
    bestDistance = found.distance;
    if (delta * delta < bestDistance) {
      found = searchKd(far, q, best, bestDistance);
      best = found.index;
      bestDistance = found.distance;
    }
    return {index: best, distance: bestDistance};
  }

  public static function distance(a:Array<Float>, b:Array<Float>):Float {
    var sum = 0.0;
    for (k in 0...a.length) sum += (a[k] - b[k]) * (a[k] - b[k]);
    return Math.sqrt(sum);
  }
}
