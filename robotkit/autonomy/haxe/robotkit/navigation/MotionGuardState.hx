package robotkit.navigation;

/** Current software obstacle response; this is not a safety-rated state. */
enum MotionGuardState {
  Clear;
  Approaching(obstacleId:String, clearanceMeters:Float, speedScale:Float);
  Blocked(reason:String);
}
