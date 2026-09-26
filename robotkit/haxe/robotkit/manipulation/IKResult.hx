package robotkit.manipulation;

/** Outcome of one InverseKinematics.solve call; never thrown for non-convergence. */
class IKResult {
  public final converged:Bool;
  public final q:Array<Float>;
  public final positionError:Float;
  public final orientationError:Float;
  public final iterations:Int;

  public function new(converged:Bool, q:Array<Float>, positionError:Float,
      orientationError:Float, iterations:Int) {
    this.converged = converged;
    this.q = q.copy();
    this.positionError = positionError;
    this.orientationError = orientationError;
    this.iterations = iterations;
  }
}
