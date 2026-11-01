package gearkit;

/** A translation of the whole flexspline tooth ring, `magnitude` mm along the direction at angle `phase` (radians,
    counter-clockwise from +Y) in `frame`. In the `Carrier` and `Flexspline` frames the direction turns with the wave
    generator and with the flexspline's mean rotation respectively; `phase` is its value at wave-generator angle 0. */
class FlexOffset {
  public final frame:OffsetFrame;
  public final magnitude:Float;
  public final phase:Float;

  public function new(frame:OffsetFrame, magnitude:Float, phase:Float = 0) {
    if (!Math.isFinite(magnitude) || !Math.isFinite(phase)) throw "offset must be finite";
    this.frame = frame;
    this.magnitude = magnitude;
    this.phase = phase;
  }
}
