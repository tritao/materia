package tests;

/** Humanoid policy runtime tests: `haxeon run --project robotkit/tools/humanoid/tests/haxeon.json`. */
class Main {
  public static function main():Void {
    OnnxPolicyTests.run();
    PolicyRuntimeTests.run();
    Sys.println("humanoid tests passed");
  }
}
