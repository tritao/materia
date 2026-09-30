package kinematicskit;

/** Why a solver stopped. */
enum abstract KinematicStatus(String) to String {
  /** Every hard task met its tolerances. */
  var Converged = "converged";
  /** Joint limits stopped further progress before the tolerances were met. */
  var LimitBlocked = "limit-blocked";
  /** The residual stopped decreasing at a locally stationary configuration (tasks conflict or are unreachable there). */
  var Conflicting = "conflicting";
  /** The iteration budget ran out while progress was still possible. */
  var IterationLimit = "iteration-limit";
  /** A linear system could not be solved. */
  var NumericalFailure = "numerical-failure";
}
