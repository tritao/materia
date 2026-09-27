package motionkit.event;

/** Output behavior while the trajectory clock is held. */
enum HoldPolicy {
  Keep;
  SafeWhileHeld;
  RestoreOnResume;
}
