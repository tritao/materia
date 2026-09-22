package tests;

/** Default app regression entry point. */
class AppTests {
  static function main():Int {
    if (SceneEditingTests.main() != 0) return 1;
    return CadPlateWorkflowTests.main();
  }
}
