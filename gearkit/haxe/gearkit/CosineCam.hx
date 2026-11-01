package gearkit;

/** `w = w0 cos 2 phi`: the closed-form reference cam. */
class CosineCam implements Cam {
  final amplitude:Float;

  public function new(w0:Float) {
    if (!Math.isFinite(w0) || w0 <= 0) throw "cam amplitude w0 must be finite and positive";
    amplitude = w0;
  }

  public function name():String return "cosine";
  public function w0():Float return amplitude;
  public function w(phi:Float):Float return amplitude * Math.cos(2 * phi);
  public function dw(phi:Float):Float return -2 * amplitude * Math.sin(2 * phi);
  public function breakpoints():Array<Float> return [];
}
