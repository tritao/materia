package gearkit;

/** A cam given by a physical polar profile `rho(phi)` (ellipse, four-arc oval). The neutral line
    follows the profile minus its mean radius (the ring keeps its length), scaled so that
    `w(0) = w0` exactly. Subclasses supply `rho` and `drho` on [0, pi/2]. */
class ShapedCam implements Cam {
  final label:String;
  final amplitude:Float;
  final joins:Array<Float>;
  var mean:Float = 0;
  var scale:Float = 1;

  function new(label:String, w0:Float, joins:Array<Float>) {
    if (!Math.isFinite(w0) || w0 <= 0) throw "cam amplitude w0 must be finite and positive";
    this.label = label;
    this.amplitude = w0;
    this.joins = joins;
  }

  /** Subclasses call this once `rho` is usable. */
  function normalise():Void {
    mean = Quadrature.integrate(x -> rho(x), 0, Math.PI / 2, joins, 0.05) / (Math.PI / 2);
    scale = amplitude / (rho(0) - mean);
  }

  /** Polar radius of the physical profile for phi in [0, pi/2]. */
  function rho(phi:Float):Float throw "rho is supplied by the cam";
  function drho(phi:Float):Float throw "drho is supplied by the cam";

  /** Fold any angle into [0, pi/2]; sign is the derivative's sign under that fold. */
  static function fold(phi:Float):{x:Float, sign:Float} {
    var x = phi - Math.PI * Math.floor(phi / Math.PI);
    return x <= Math.PI / 2 ? {x: x, sign: 1.0} : {x: Math.PI - x, sign: -1.0};
  }

  public function name():String return label;
  public function w0():Float return amplitude;
  public function w(phi:Float):Float return scale * (rho(fold(phi).x) - mean);
  public function dw(phi:Float):Float {
    var f = fold(phi);
    return f.sign * scale * drho(f.x);
  }
  public function breakpoints():Array<Float> return joins.copy();
  /** Mean radius removed from the physical profile, mm. */
  public function meanRadius():Float return mean;
}
