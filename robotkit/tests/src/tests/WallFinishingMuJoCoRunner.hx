package tests;

/**
 * Entry point for the separate `robotkit/tests/mujoco` haxeon project (see
 * ARCHITECTURE.md "Simulated wall-finishing robot (M9)"): runs only the
 * MuJoCo-backend variant of the M9 wall-finishing scenario. Kept as its own
 * tiny entry rather than folding into `RobotWorldTests.main()`, since that
 * class is also the entry for the standard `robotkit/tests` project, whose
 * native build has no MuJoCo backend compiled in.
 */
class WallFinishingMuJoCoRunner {
  public static function main():Void {
    WallFinishingScenarioTests.runMuJoCo();
  }
}
