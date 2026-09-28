class MotionKitBootstrapTests {
  public static function main():Void {
    var plannerTests:PlannerTests = new PlannerTests();
    var streamTests:StreamTests = new StreamTests();
    var sessionTests:SessionTests = new SessionTests();
    var programTests:ProgramTests = new ProgramTests();
    var kinematicsTests:KinematicsTests = new KinematicsTests();
    var processTests:ProcessTests = new ProcessTests();

    if (Sys.getEnv("MOTIONKIT_CNC_ONLY") == "1") {
      plannerTests.testCircularSegments();
      processTests.testCncProgramBinding();
      processTests.testPhysicalAssemblyCncBinding();
      Sys.println('CNC focused tests passed (${MotionKitTestSupport.assertions} assertions)');
      return;
    }
    if (Sys.getEnv("MOTIONKIT_C7_ONLY") == "1") {
      processTests.testVirtualCncProgram();
      Sys.println('C7 focused tests passed (${MotionKitTestSupport.assertions} assertions)');
      return;
    }
    if (Sys.getEnv("MOTIONKIT_MACHINEKIT_ONLY") == "1") {
      processTests.testLinearAxisCompilesToRobotModel();
      processTests.testCompiledXYZGantryRunsThroughSimulation();
      processTests.testMachineKitLeadScrewThroughVirtualDevice();
      Sys.println('MachineKit compiler tests passed (${MotionKitTestSupport.assertions} assertions)');
      return;
    }
    if (Sys.getEnv("MOTIONKIT_C4_ONLY") == "1") {
      kinematicsTests.testOpwKinematics();
      Sys.println('C4 focused tests passed (${MotionKitTestSupport.assertions} assertions)');
      return;
    }
    processTests.testPoseProcessPath();
    processTests.testMotionEventContracts();
    kinematicsTests.testKinematicsContract();
    kinematicsTests.testOpwKinematics();
    programTests.testMotionProgramContracts();
    programTests.testProgramCompiler();
    programTests.testProgramStartTolerances();
    kinematicsTests.testPathConfigurationSelector();
    kinematicsTests.testAxisKinematics();
    processTests.testCncProgramBinding();
    processTests.testPhysicalAssemblyCncBinding();
    processTests.testVirtualCncProgram();
    programTests.testManipulatorMotion();
    plannerTests.testSimplePathTimingContract();
    plannerTests.testNativePathLowering();
    plannerTests.testToppraPathTiming();
    plannerTests.testGeometricPathPrimitives();
    plannerTests.testNativeTrajectoryRoundTrip();
    plannerTests.testNativeValidationAndPlan();
    plannerTests.testPlannerIsDeterministicAndBounded();
    plannerTests.testToppraExactStopsAndBindings();
    plannerTests.testToppraCircleAcceleration();
    processTests.testLinearAxisCompilesToRobotModel();
    processTests.testLeadScrewActuatorRateLimitsPlans();
    kinematicsTests.testTransmissionDerivedAxisMapping();
    processTests.testCompiledAxisRunsThroughSimulation();
    processTests.testHomingAndJogging();
    plannerTests.testMoveLinearUsesPlannerLimits();
    processTests.testCompiledXYZGantryRunsThroughSimulation();
    processTests.testMachineKitLeadScrewThroughVirtualDevice();
    kinematicsTests.testDualMotorAxisRunsThroughSimulation();
    streamTests.testBufferedExecution();
    sessionTests.testNormalAbortWaitsForRest();
    sessionTests.testMotionSessionTransitions();
    streamTests.testPlanCapableReplayRecordsMotionPlan();
    streamTests.testLongBufferedExecution();
    streamTests.testHoldRefillsNearChunkBoundary();
    sessionTests.testHoldDecelerationStaysWithinLimitsThroughoutMove();
    sessionTests.testRuntimeSynchronizedHolding();
    sessionTests.testImmediateMotionReplacesNativeQueue();
    sessionTests.testSmoothReplacementRetriesLateSubmission();
    sessionTests.testFreeRunningSmoothReplacement();
    sessionTests.testMotionChangesStayWithinLimits();
    sessionTests.testContinuousJog();
    sessionTests.testLateJogReplacementRejectsLateArrival();
    sessionTests.testPathHoldsStayOnPathWithinLimits();
    plannerTests.testToleranceBlend();
    sessionTests.testDualMotorAxisChangesStayWithinJointLimits();
    Sys.println('MotionKit bootstrap tests passed (${MotionKitTestSupport.assertions} assertions)');
  }

}
