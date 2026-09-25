package robotkit.navigation;

import robotkit.mobile.Pose2;

/** Plans a traversable path between poses expressed in the planner's map frame. */
interface Planner {
  /** Throws when either pose is outside the map, blocked, or unreachable. */
  public function plan(start:Pose2, goal:Pose2):Path;
}
