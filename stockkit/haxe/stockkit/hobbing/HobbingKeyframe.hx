package stockkit.hobbing;
/** Controller sample. Rotary positions are unwrapped radians. */
class HobbingKeyframe {
  public final time:Float;
  public final axes:HobbingAxes;
  public function new(time:Float,axes:HobbingAxes) {
    if(!Math.isFinite(time))throw "controller sample time must be finite";
    this.time=time;this.axes=axes;
  }
}
