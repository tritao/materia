package tests;

/** Focused tool and process capability tests. */
class ToolCapabilityMain {
  public static function main():Void {
    ToolTests.run();
    ProcessTests.run();
    WeldTests.run();
    ClearanceTests.run();
  }
}
