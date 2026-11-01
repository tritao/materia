package gearkit;

/** Fixed-order Gauss-Legendre quadrature; nodes are found by Newton iteration on P_n. */
class Quadrature {
  static final ORDER = 16;
  static var nodes:Array<Float>;
  static var weights:Array<Float>;

  static function init():Void {
    if (nodes != null) return;
    var n = ORDER, xs:Array<Float> = [], ws:Array<Float> = [];
    for (i in 0...n) {
      var x = Math.cos(Math.PI * (i + 0.75) / (n + 0.5)), dp = 1.0;
      for (_ in 0...100) {
        var p0 = 1.0, p1 = x;
        for (k in 2...n + 1) {
          var p2 = ((2 * k - 1) * x * p1 - (k - 1) * p0) / k;
          p0 = p1;
          p1 = p2;
        }
        dp = n * (x * p1 - p0) / (x * x - 1);
        var dx = p1 / dp;
        x -= dx;
        if (Math.abs(dx) < 1e-16) break;
      }
      xs.push(x);
      ws.push(2 / ((1 - x * x) * dp * dp));
    }
    nodes = xs;
    weights = ws;
  }

  /** Integral of f over [a, b] on one panel (exact for polynomials up to degree 31). */
  public static function panel(f:Float -> Float, a:Float, b:Float):Float {
    init();
    var mid = (a + b) / 2, half = (b - a) / 2, sum = 0.0;
    for (i in 0...ORDER) sum += weights[i] * f(mid + half * nodes[i]);
    return sum * half;
  }

  /** Integral over [a, b], split at the given interior breakpoints and at panels no longer than `maxPanel`. */
  public static function integrate(f:Float -> Float, a:Float, b:Float, breaks:Array<Float>, maxPanel:Float):Float {
    if (b == a) return 0;
    if (b < a) return -integrate(f, b, a, breaks, maxPanel);
    var cuts = [a];
    for (x in breaks) if (x > a && x < b) cuts.push(x);
    cuts.sort((p, q) -> p < q ? -1 : p > q ? 1 : 0);
    cuts.push(b);
    var sum = 0.0;
    for (i in 0...cuts.length - 1) {
      var lo = cuts[i], hi = cuts[i + 1], pieces = Std.int(Math.max(1, Math.ceil((hi - lo) / maxPanel)));
      for (k in 0...pieces) sum += panel(f, lo + (hi - lo) * k / pieces, lo + (hi - lo) * (k + 1) / pieces);
    }
    return sum;
  }
}
