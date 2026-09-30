package robotkit.manipulation;

import kinematicskit.KinematicStatus;

/** Outcome of one `Manipulator` IK solve; never thrown for non-convergence. */
class IKResult {
  public final converged:Bool;
  public final q:Array<Float>;
  public final positionError:Float;
  public final orientationError:Float;
  public final iterations:Int;
  /** Why the solver stopped (see `kinematicskit.KinematicStatus`). */
  public final status:KinematicStatus;

  public function new(converged:Bool, q:Array<Float>, positionError:Float,
      orientationError:Float, iterations:Int, status:KinematicStatus) {
    this.converged = converged;
    this.q = q.copy();
    this.positionError = positionError;
    this.orientationError = orientationError;
    this.iterations = iterations;
    this.status = status;
  }
}
