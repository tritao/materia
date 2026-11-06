package tests;

import app.MachineKitRecipeProjectTests;

/** Default app regression entry point. */
class AppTests {
  static function main():Int {
    ProjectProcessTests.run();
    SceneVisibilityTests.run();
    SceneTreeSelectionTests.run();
    MoldSourceLinkTests.run();
    ViewportSelectionTests.run();
    if (ViewportCoordinatesTests.main() != 0) return 1;
    if (StartPageTests.main() != 0) return 1;
    if (MissionJobsTests.main() != 0) return 1;
    SheetMaterialIntegrationTests.run();
    if (AppPreferencesTests.main() != 0) return 1;
    if (SceneControlsCardTests.main() != 0) return 1;
    if (EditorSettingsDialogTests.main() != 0) return 1;
    ProjectUiFrameTests.run();
    if (SceneAtomicityTests.main() != 0) return 1;
    if (ProjectDocumentTests.main() != 0) return 1;
    if (EditorToolbarLayoutTests.main() != 0) return 1;
    if (EditorWorkspaceLayoutTests.main() != 0) return 1;
    if (DocumentPathTests.main() != 0) return 1;
    if (BimEditorProjectionTests.main() != 0) return 1;
    if (MachineKitRecipeProjectTests.main() != 0) return 1;
    if (WorkspaceSaveWorkerTests.main() != 0) return 1;
    if (SceneEditingTests.main() != 0) return 1;
    if (CameraClipRangeTests.main() != 0) return 1;
    if (StockSimulationTests.main() != 0) return 1;
    if (RuntimeGeometryTests.main() != 0) return 1;
    if (CosmeticThreadTests.main() != 0) return 1;
    if (MachineControlTests.main() != 0) return 1;
    if (HobbingReportTests.main() != 0) return 1;
    HobbingHandlingTests.run();
    SpindleDriveTests.run();
    HobbingSpindleSamplingTests.run();
    if (WorkerObjectTests.main() != 0) return 1;
    if (SceneCollisionTests.main() != 0) return 1;
    ScriptedSetupTests.run();
    if (HumanSimulationTests.main() != 0) return 1;
    if (WorkerDemoTests.main() != 0) return 1;
    if (WorkerGalleryTests.main() != 0) return 1;
    SceneDocumentTests.run();
    return CadPlateWorkflowTests.main();
  }
}
