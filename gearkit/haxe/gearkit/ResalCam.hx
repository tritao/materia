package gearkit;

/** Resal's thin ring under two opposed point loads (two rollers): Gravagno, Mucino, Pennestri 2016,
    eq 18. With theta measured from the minor axis (the rollers sit at theta = +-pi/2),
    `u(theta) = delta (1/pi - theta sin(theta)/4 - cos(theta)/4) / (1/pi - pi/8)` for |theta| <= pi/2, period pi.
    In the cam convention (phi from the major axis, `w(0) = w0`): delta = w0, theta = (phi mod pi) - pi/2.
    The mean of u is exactly zero (inextensible), `u(0) = -0.918 w0`, and u' vanishes at both axes. */
class ResalCam implements Cam {
  static final DENOMINATOR = 1 / Math.PI - Math.PI / 8;
  final amplitude:Float;

  public function new(w0:Float) {
    if (!Math.isFinite(w0) || w0 <= 0) throw "cam amplitude w0 must be finite and positive";
    amplitude = w0;
  }

  static function theta(phi:Float):Float return phi - Math.PI * Math.floor(phi / Math.PI) - Math.PI / 2;

  public function name():String return "resal";
  public function w0():Float return amplitude;
  public function w(phi:Float):Float {
    var t = theta(phi);
    return amplitude * (1 / Math.PI - t * Math.sin(t) / 4 - Math.cos(t) / 4) / DENOMINATOR;
  }
  public function dw(phi:Float):Float {
    var t = theta(phi);
    return amplitude * (-t * Math.cos(t) / 4) / DENOMINATOR;
  }
  public function breakpoints():Array<Float> return [];
}
