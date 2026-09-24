package tests;

/** Default app regression entry point. */
class AppTests {
  static function main():Int {
    if (BimEditorProjectionTests.main() != 0) return 1;
    if (SceneEditingTests.main() != 0) return 1;
    return CadPlateWorkflowTests.main();
  }
}
