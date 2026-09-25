package robotkit.navigation;

/** Lifecycle state for goal planning and path following. */
enum NavigatorStatus {
  Idle;
  Navigating;
  Succeeded;
  Cancelled;
  Blocked(reason:String);
  Failed(reason:String);
}
