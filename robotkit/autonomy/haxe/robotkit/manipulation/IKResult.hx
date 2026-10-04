package robotkit.manipulation;

import kinematicskit.KinematicStatus;
import robotkit.spatial.Transform3;

/** Outcome of one `Manipulator` IK solve; never thrown for non-convergence. */
class IKResult {
  public final converged:Bool;
  public final q:Array<Float>;
  public final positionError:Float;
  public final orientationError:Float;
  public final iterations:Int;
  /** Why the solver stopped (see `kinematicskit.KinematicStatus`). */
  public final status:KinematicStatus;
  /** World pose of the robot's root after a solve that moved the base (`IkOptions.movingBase`); null otherwise. */
  public final rootPose:Null<Transform3>;

  public function new(converged:Bool, q:Array<Float>, positionError:Float,
      orientationError:Float, iterations:Int, status:KinematicStatus, ?rootPose:Transform3) {
    this.converged = converged;
    this.q = q.copy();
    this.positionError = positionError;
    this.orientationError = orientationError;
    this.iterations = iterations;
    this.status = status;
    this.rootPose = rootPose;
  }
}
