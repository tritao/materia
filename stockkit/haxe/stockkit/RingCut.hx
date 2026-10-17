package stockkit;

/** Remove one counterclockwise angular span on a radial circle and consecutive Z cells.
    start wraps; sweep is in [0,2*pi]. Gash/pass identifiers must be non-negative. */
class RingCut {
  public final radialIndex:Int;
  public final axialFirst:Int;
  public final axialCount:Int;
  public final start:Float;
  public final sweep:Float;
  public final gash:Int;
  public final pass:Int;
  public function new(radialIndex:Int, axialFirst:Int, axialCount:Int, start:Float, sweep:Float,
      gash:Int, pass:Int) {
    if (radialIndex < 0 || axialFirst < 0 || axialCount < 1 || gash < 0 || pass < 0)
      throw "ring cut indices and sources must be non-negative, with positive axial count";
    if (!Math.isFinite(start) || !Math.isFinite(sweep) || sweep < 0 || sweep > 2 * Math.PI)
      throw "ring cut needs finite angles and sweep in [0,2*pi]";
    this.radialIndex = radialIndex; this.axialFirst = axialFirst; this.axialCount = axialCount;
    this.start = start; this.sweep = sweep; this.gash = gash; this.pass = pass;
  }
}
