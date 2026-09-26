package motionkit.trajectory;

/** Analytic state of a native polynomial trajectory at one instant. */
class TrajectoryState {
  public final positions:Array<Float>;
  public final velocities:Array<Float>;
  public final accelerations:Array<Float>;
  public final jerks:Array<Float>;

  public function new(positions:Array<Float>, velocities:Array<Float>,
      accelerations:Array<Float>, jerks:Array<Float>) {
    this.positions = positions;
    this.velocities = velocities;
    this.accelerations = accelerations;
    this.jerks = jerks;
  }
}
