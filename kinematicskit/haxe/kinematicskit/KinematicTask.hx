package kinematicskit;

/**
 * One term of a `KinematicProblem`. A task contributes `rowCount()` rows:
 * residuals `e = desired − current` and the Jacobian rows `∂current/∂q`
 * over all model DOFs, both already multiplied by the task's weights.
 * Hard tasks decide convergence; soft tasks (posture, preferences) only
 * shape the solution.
 */
interface KinematicTask {
  function label():String;
  function rowCount():Int;
  function isSoft():Bool;
  /**
   * Writes rows `row .. row + rowCount()` of `residual` and of the row-major
   * `jacobian` (`layout.width` columns) for the evaluated `snapshot` of
   * `state`, and records the errors reported below.
   */
  function evaluate(state:KinematicState, snapshot:KinematicSnapshot, layout:JacobianLayout,
    residual:Array<Float>, jacobian:Array<Float>, row:Int):Void;
  /** Unweighted translational error at the last `evaluate`, in model length units. */
  function positionError():Float;
  /** Unweighted angular error at the last `evaluate`, in radians. */
  function orientationError():Float;
  /** Whether the last `evaluate` met the task's tolerances (always true for a soft task). */
  function satisfied():Bool;
}
