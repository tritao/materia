package robotkit.navigation;

/** Lifecycle state for one Navigation path-following request. */
enum NavigationStatus {
  Idle;
  Following;
  Succeeded;
  Cancelled;
  Failed(message:String);
}
