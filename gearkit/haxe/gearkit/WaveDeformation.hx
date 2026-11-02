package gearkit;

/** Inextensible-ring deformation of the flexspline neutral line (TOOTH_DESIGN_PLAN.md TD-D1),
    for a two-lobe cam. With `delta` the section angle from the major axis:
    radial `w`, tangential `v` with `w + dv/d(delta) = 0`, section tilt `mu = (v - dw/d(delta)) / rn`.
    `v` is the integral of `w` from the major axis, done numerically (Gauss-Legendre on a table),
    so every cam takes the same path; the cosine cam reproduces `v = -(w0/2) sin 2 delta`. */
class WaveDeformation {
  static final TABLE = 512;
  public final cam:Cam;
  /** Neutral-line radius, mm. */
  public final neutralRadius:Float;
  final nodes:Array<Float> = [];
  final step:Float;
  final breaks:Array<Float>;

  public function new(cam:Cam, neutralRadius:Float) {
    if (!Math.isFinite(neutralRadius) || neutralRadius <= cam.w0()) throw "neutral radius must exceed the cam amplitude";
    this.cam = cam;
    this.neutralRadius = neutralRadius;
    breaks = cam.breakpoints();
    step = Math.PI / 2 / TABLE;
    nodes.push(0.0);
    for (k in 0...TABLE) nodes.push(nodes[k] + Quadrature.integrate(x -> cam.w(x), k * step, (k + 1) * step, breaks, step));
  }

  /** Integral of w from 0 to x, x in [0, pi/2]. */
  function accumulated(x:Float):Float {
    var k = Std.int(Math.min(TABLE - 1, Math.floor(x / step)));
    return nodes[k] + Quadrature.integrate(x -> cam.w(x), k * step, x, breaks, step);
  }

  /** Residual of the closure condition: integral of w over a quarter turn. v closes iff this is 0. */
  public function closure():Float return nodes[TABLE];

  public function w(delta:Float):Float return cam.w(delta);
  public function dw(delta:Float):Float return cam.dw(delta);

  public function v(delta:Float):Float {
    var x = delta - Math.PI * Math.floor(delta / Math.PI);
    return x <= Math.PI / 2 ? -accumulated(x) : accumulated(Math.PI - x);
  }

  public function mu(delta:Float):Float return (v(delta) - dw(delta)) / neutralRadius;

  /** Relative length error of the deformed neutral line, exact arc length against 2 pi rn.
      Zero to first order by construction; the remainder is second order in w0 / rn. */
  public function lengthStrain(samples:Int = 4096):Float {
    var n = neutralRadius, sum = 0.0;
    for (i in 0...samples) {
      var d = 2 * Math.PI * (i + 0.5) / samples;
      var vv = v(d);
      // dP/d(delta) = (w' - v) e_r + (rn + w + v') e_phi, and w + v' = 0 leaves rn on e_phi.
      var dr = dw(d) - vv, dt = n;
      sum += Math.sqrt(dr * dr + dt * dt) * 2 * Math.PI / samples;
    }
    return sum / (2 * Math.PI * n) - 1;
  }
}
