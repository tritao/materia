package motionkit.kinematics;

/** Accuracy and deterministic numerical-solver policy for one IK request. */
class IkTolerance {
  public final position:Float;
  public final orientation:Float;
  public final maxIterations:Int;
  public final damping:Float;
  public final candidateSeparation:Float;

  public function new(?position:Float = 1e-4, ?orientation:Float = 1e-3,
      ?maxIterations:Int = 100, ?damping:Float = 0.02,
      ?candidateSeparation:Float = 1e-3) {
    if (!Math.isFinite(position) || position <= 0.0)
      throw "IK position tolerance must be finite and positive";
    if (!Math.isFinite(orientation) || orientation <= 0.0)
      throw "IK orientation tolerance must be finite and positive";
    if (maxIterations <= 0) throw "IK iteration budget must be positive";
    if (!Math.isFinite(damping) || damping <= 0.0)
      throw "IK damping must be finite and positive";
    if (!Math.isFinite(candidateSeparation) || candidateSeparation <= 0.0)
      throw "IK candidate separation must be finite and positive";
    this.position = position;
    this.orientation = orientation;
    this.maxIterations = maxIterations;
    this.damping = damping;
    this.candidateSeparation = candidateSeparation;
  }
}
