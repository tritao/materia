package robotkit.safety;

/** Active high-level policy restriction; hard enforcement remains in RobotRuntime. */
enum SafetyRestriction {
  SpeedLimited(maxSpeedMetersPerSecond:Float);
  StopRequired(reason:String);
  ForkHeightLimited(maxHeightMeters:Float);
  PayloadLimited(maxMassKg:Float);
}
