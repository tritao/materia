package tests;

import app.MachineKitRecipeProjectTests;

/** Default app regression entry point. */
class AppTests {
	static function main():Int {
		if (AppPreferencesTests.main() != 0) return 1;
		if (EditorSettingsDialogTests.main() != 0) return 1;
		if (SceneAtomicityTests.main() != 0) return 1;
		if (ProjectDocumentTests.main() != 0) return 1;
		if (EditorToolbarLayoutTests.main() != 0) return 1;
		if (EditorWorkspaceLayoutTests.main() != 0) return 1;
		if (BimEditorProjectionTests.main() != 0) return 1;
		if (MachineKitRecipeProjectTests.main() != 0) return 1;
    if (WorkspaceSaveWorkerTests.main() != 0) return 1;
    if (SceneEditingTests.main() != 0) return 1;
    if (StockSimulationTests.main() != 0) return 1;
    if (WorkerObjectTests.main() != 0) return 1;
    ScriptedSetupTests.run();
    if (HumanSimulationTests.main() != 0) return 1;
    if (WorkerDemoTests.main() != 0) return 1;
    if (WorkerGalleryTests.main() != 0) return 1;
    SceneDocumentTests.run();
    return CadPlateWorkflowTests.main();
  }
}
