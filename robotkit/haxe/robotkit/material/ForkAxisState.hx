package robotkit.material;

/** Immutable observed position, velocity, and effort for one fork axis. */
class ForkAxisState {
  public final position:Float;
  public final velocity:Float;
  public final effort:Float;

  public function new(position:Float, velocity:Float, effort:Float) {
    for (value in [position, velocity, effort])
      if (!Math.isFinite(value)) throw "Fork axis state values must be finite";
    this.position = position;
    this.velocity = velocity;
    this.effort = effort;
  }
}
