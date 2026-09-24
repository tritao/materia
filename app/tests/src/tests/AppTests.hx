package tests;

/** Default app regression entry point. */
class AppTests {
	static function main():Int {
		if (SceneAtomicityTests.main() != 0) return 1;
		if (ProjectDocumentTests.main() != 0) return 1;
		if (EditorToolbarLayoutTests.main() != 0) return 1;
		if (BimEditorProjectionTests.main() != 0) return 1;
		if (WorkspaceSaveWorkerTests.main() != 0) return 1;
    if (SceneEditingTests.main() != 0) return 1;
    return CadPlateWorkflowTests.main();
  }
}
