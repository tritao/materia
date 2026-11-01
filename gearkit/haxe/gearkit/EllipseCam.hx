package gearkit;

/** Elliptical cam with semi-axes `neutralRadius +/- w0` before the mean radius is removed. */
class EllipseCam extends ShapedCam {
  final a:Float;
  final b:Float;

  public function new(neutralRadius:Float, w0:Float) {
    super("ellipse", w0, []);
    if (!Math.isFinite(neutralRadius) || neutralRadius <= w0) throw "neutral radius must exceed the cam amplitude";
    a = neutralRadius + w0;
    b = neutralRadius - w0;
    normalise();
  }

  override function rho(phi:Float):Float {
    var c = Math.cos(phi), s = Math.sin(phi);
    return a * b / Math.sqrt(b * b * c * c + a * a * s * s);
  }

  override function drho(phi:Float):Float {
    var c = Math.cos(phi), s = Math.sin(phi), q = b * b * c * c + a * a * s * s;
    return -a * b * (a * a - b * b) * Math.sin(2 * phi) / (2 * q * Math.sqrt(q));
  }
}
