class ToolpathScenarioTests {
  public static function main():Void {
    var tests = new ProcessTests();
    tests.testCncProgramBinding();
    tests.testPhysicalAssemblyCncBinding();
    tests.testVirtualCncProgram();
    Sys.println('Toolpath scenarios passed (${MotionKitTestSupport.assertions} assertions)');
  }
}
