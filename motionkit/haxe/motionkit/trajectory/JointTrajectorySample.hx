package motionkit.trajectory;

/** One immutable sample of a time-parameterized joint trajectory. */
class JointTrajectorySample {
  public final timeSeconds:Float;
  public final positions:Array<Float>;
  public final velocities:Array<Float>;
  public final accelerations:Array<Float>;

  public function new(timeSeconds:Float, positions:Array<Float>,
      ?velocities:Array<Float>, ?accelerations:Array<Float>) {
    if (!Math.isFinite(timeSeconds) || timeSeconds < 0.0)
      throw "Trajectory sample time must be finite and non-negative";
    if (positions == null || positions.length == 0)
      throw "Trajectory sample needs at least one joint position";
    this.timeSeconds = timeSeconds;
    this.positions = copyFinite(positions, "position");
    this.velocities = velocities == null
      ? [for (_ in positions) 0.0]
      : copyVector(velocities, positions.length, "velocity");
    this.accelerations = accelerations == null
      ? [for (_ in positions) 0.0]
      : copyVector(accelerations, positions.length, "acceleration");
  }

  function copyVector(values:Array<Float>, expectedLength:Int, label:String):Array<Float> {
    if (values.length != expectedLength)
      throw 'Trajectory sample $label count does not match position count';
    return copyFinite(values, label);
  }

  static function copyFinite(values:Array<Float>, label:String):Array<Float> {
    var result:Array<Float> = [];
    for (value in values) {
      if (!Math.isFinite(value)) throw 'Trajectory sample $label must be finite';
      result.push(value);
    }
    return result;
  }
}
