class MotionKitBootstrapTests {
  public static function main():Void {
    var plannerTests:PlannerTests = new PlannerTests();
    var streamTests:StreamTests = new StreamTests();
    var sessionTests:SessionTests = new SessionTests();
    var programTests:ProgramTests = new ProgramTests();
    var kinematicsTests:KinematicsTests = new KinematicsTests();
    var processTests:ProcessTests = new ProcessTests();

    if (Sys.getEnv("MOTIONKIT_ARM_SESSION_ONLY") == "1") {
      programTests.testManipulatorSessionTransitions();
      Sys.println('Arm session tests passed (${MotionKitTestSupport.assertions} assertions)');
      return;
    }
    if (Sys.getEnv("MOTIONKIT_PROCESS_RECOVERY_ONLY") == "1") {
      programTests.testProcessRunVirtualArmRecovery();
      Sys.println('Process recovery tests passed (${MotionKitTestSupport.assertions} assertions)');
      return;
    }

    if (Sys.getEnv("MOTIONKIT_CIRCULAR_ONLY") == "1") {
      plannerTests.testCircularSegments();
      Sys.println('Circular focused tests passed (${MotionKitTestSupport.assertions} assertions)');
      return;
    }
    if (Sys.getEnv("MOTIONKIT_BLEND_ONLY") == "1") {
      plannerTests.testToleranceBlend();
      Sys.println('Blend focused tests passed (${MotionKitTestSupport.assertions} assertions)');
      return;
    }
    if (Sys.getEnv("MOTIONKIT_PLANCHECK_ONLY") == "1") {
      new PlanCheckTests().testPlanCheck();
      new PlanCheckTests().testCompilerRunsPlanCheck();
      Sys.println('Plan check tests passed (${MotionKitTestSupport.assertions} assertions)');
      return;
    }
    if (Sys.getEnv("MOTIONKIT_MACHINEKIT_ONLY") == "1") {
      processTests.testLinearAxisCompilesToRobotModel();
      processTests.testCompiledXYZGantryRunsThroughSimulation();
      processTests.testMachineKitLeadScrewThroughVirtualDevice();
      Sys.println('MachineKit compiler tests passed (${MotionKitTestSupport.assertions} assertions)');
      return;
    }
    if (Sys.getEnv("MOTIONKIT_REDUNDANCY_ONLY") == "1") {
      kinematicsTests.testKinematicsContract();
      programTests.testProgramCompiler();
      programTests.testRedundantArmPaths();
      programTests.testCoordinatedExternalAxes();
      kinematicsTests.testPathConfigurationSelector();
      Sys.println('Redundancy focused tests passed (${MotionKitTestSupport.assertions} assertions)');
      return;
    }
    if (Sys.getEnv("MOTIONKIT_C4_ONLY") == "1") {
      kinematicsTests.testOpwKinematics();
      Sys.println('C4 focused tests passed (${MotionKitTestSupport.assertions} assertions)');
      return;
    }
    new PlanCheckTests().testPlanCheck();
    new PlanCheckTests().testCompilerRunsPlanCheck();
    processTests.testPoseProcessPath();
    processTests.testMotionEventContracts();
    kinematicsTests.testKinematicsContract();
    kinematicsTests.testSharedGroupAcrossThreads();
    kinematicsTests.testOpwKinematics();
    programTests.testMotionProgramContracts();
    programTests.testProgramCompiler();
    programTests.testRedundantArmPaths();
    programTests.testCoordinatedExternalAxes();
    programTests.testProgramStartTolerances();
    programTests.testProgramPlanner();
    kinematicsTests.testPathConfigurationSelector();
    kinematicsTests.testAxisKinematics();
    kinematicsTests.testManipulatorServo();
    kinematicsTests.testServoSession();
    kinematicsTests.testServoPlans();
    programTests.testManipulatorMotion();
    programTests.testManipulatorSessionTransitions();
    programTests.testProcessRunVirtualArmRecovery();
    plannerTests.testSimplePathTimingContract();
    plannerTests.testNativePathLowering();
    plannerTests.testToppraPathTiming();
    plannerTests.testGeometricPathPrimitives();
    plannerTests.testNativeTrajectoryRoundTrip();
    plannerTests.testNativeValidationAndPlan();
    plannerTests.testPlannerIsDeterministicAndBounded();
    plannerTests.testToppraExactStopsAndBindings();
    plannerTests.testToppraCircleAcceleration();
    plannerTests.testCircularSegments();
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
