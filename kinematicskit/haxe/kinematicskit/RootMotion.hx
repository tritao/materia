package kinematicskit;

/** How a solve may move a root body of the model (see `KinematicProblem.setRootMotion`). */
enum abstract RootMotion(Int) {
  /** The root stays where the state puts it. */
  var Fixed = 0;
  /** x, y and yaw about world Z: a wheeled mobile base. Three columns (vx, vy, ωz). */
  var Planar = 1;
  /** A free rigid pose: a floating or legged base. Six columns (v, ω), world frame. */
  var Floating = 2;
}
