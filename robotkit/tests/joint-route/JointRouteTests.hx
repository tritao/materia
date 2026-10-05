import robotkit.manipulation.JointRoute;

class JointRouteTests {
  static var assertions:Int = 0;
  static function check(value:Bool, message:String):Void {
    assertions++;
    if (!value) throw message;
  }
  static function clear(a:Array<Float>, b:Array<Float>):Bool {
    for (sample in 0...201) {
      var t = sample / 200;
      var x = a[0] + (b[0] - a[0]) * t, y = a[1] + (b[1] - a[1]) * t;
      if (Math.abs(x) < 0.2 && Math.abs(y) < 0.6) return false;
    }
    return true;
  }
  public static function main():Void {
    var from = [-0.8, 0.0], to = [0.8, 0.0];
    var route = JointRoute.plan(from, to, [-1.0, -1.0], [1.0, 1.0], clear);
    check(route.length > 2, "A blocked direct move requires a detour");
    check(route[0][0] == from[0] && route[route.length - 1][0] == to[0], "Exact endpoints survive search");
    for (i in 1...route.length) check(clear(route[i - 1], route[i]), "Every returned motion edge is checked");
    var again = JointRoute.plan(from, to, [-1.0, -1.0], [1.0, 1.0], clear);
    check(route.length == again.length, "Search is reproducible");
    for (i in 0...route.length) for (j in 0...2) check(route[i][j] == again[i][j], "Deterministic waypoints");
    var direct = JointRoute.plan([-0.8, 0.8], [0.8, 0.8], [-1.0, -1.0], [1.0, 1.0], clear);
    check(direct.length == 2, "A clear direct route needs no intermediate motion");
    var blocked = false;
    try JointRoute.plan([0.0, 0.0], to, [-1.0, -1.0], [1.0, 1.0], clear) catch (_:Dynamic) blocked = true;
    check(blocked, "A blocked start fails explicitly");
    var exhausted = false;
    try JointRoute.plan(from, to, [-1.0, -1.0], [1.0, 1.0],
      (a, b) -> a[0] == b[0] && a[1] == b[1], 8) catch (_:Dynamic) exhausted = true;
    check(exhausted, "Disconnected endpoints exhaust the bounded search explicitly");
    var directed = JointRoute.plan(from, to, [-1.0, -1.0], [1.0, 1.0],
      (a, b) -> b[0] >= a[0] && clear(a, b));
    for (i in 1...directed.length)
      check(directed[i][0] >= directed[i - 1][0] && clear(directed[i - 1], directed[i]), "Goal-tree edges respect execution direction");
    check(from[0] == -0.8 && to[0] == 0.8, "Input configurations are not mutated");
    Sys.println('Joint route: $assertions assertions passed');
  }
}
