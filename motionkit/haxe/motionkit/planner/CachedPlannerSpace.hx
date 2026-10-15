package motionkit.planner;

/** Bounded exact-query cache for one immutable scene and policy.
 * The owner must discard/clear it when geometry, roots, limits, rules, margins
 * or subdivision policy changes. Posing bodies during queries is permitted.
 */
class CachedPlannerSpace implements PlannerSpace {
  final inner:PlannerSpace;
  final capacity:Int;
  var points:Map<String,Null<String>> = [];
  var edges:Map<String,Bool> = [];
  var pointCount = 0;
  var edgeCount = 0;
  public var pointHits(default, null):Int = 0;
  public var edgeHits(default, null):Int = 0;

  public function new(inner:PlannerSpace, capacity:Int = 4096) {
    if (inner == null || capacity < 1) throw "Planner cache needs a space and positive capacity";
    this.inner = inner; this.capacity = capacity;
  }
  public function dimension():Int return inner.dimension();
  public function lower():Array<Float> return inner.lower();
  public function upper():Array<Float> return inner.upper();
  public function clear():Void { points = []; edges = []; pointCount = 0; edgeCount = 0; }
  public function invalid(q:Array<Float>):Null<String> {
    if (q == null || q.length != dimension()) throw "Cached planner point has the wrong dimension";
    var key = PlannerQueryKey.of(q);
    if (points.exists(key)) { pointHits++; return points.get(key); }
    var result = inner.invalid(q);
    if (pointCount >= capacity) { points = []; pointCount = 0; }
    points.set(key, result); pointCount++;
    return result;
  }
  public function edgesClear(values:Array<Float>, count:Int):Array<Bool> {
    var width = dimension() * 2;
    if (values == null || count < 0 || values.length != count * width) throw "Cached planner edges have the wrong dimension";
    var result = [for (_ in 0...count) false], missing:Array<Float> = [], keys:Array<String> = [];
    var pending:Map<String,Int> = [], slots:Array<Int> = [];
    for (i in 0...count) {
      var key = PlannerQueryKey.of(values, i * width, width);
      if (edges.exists(key)) { result[i] = edges.get(key); slots.push(-1); edgeHits++; continue; }
      var slot = pending.get(key);
      if (slot == null) {
        slot = keys.length; pending.set(key, slot); keys.push(key);
        for (k in 0...width) missing.push(values[i * width + k]);
      } else edgeHits++;
      slots.push(slot);
    }
    if (keys.length == 0) return result;
    var checked = inner.edgesClear(missing, keys.length);
    if (checked.length != keys.length) throw "Planner returned the wrong edge count";
    for (i in 0...count) if (slots[i] >= 0) result[i] = checked[slots[i]];
    for (i in 0...keys.length) {
      if (edgeCount >= capacity) { edges = []; edgeCount = 0; }
      edges.set(keys[i], checked[i]); edgeCount++;
    }
    return result;
  }
}
