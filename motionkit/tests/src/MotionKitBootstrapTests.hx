import haxeon.test.Shards;

class MotionKitBootstrapTests {
  public static function main():Void {
    if (Sys.getEnv("MOTIONKIT_HOMING_ONLY") == "1") { HomingTests.run(); return; }
    var plannerTests:PlannerTests = new PlannerTests();
    var streamTests:StreamTests = new StreamTests();
    var sessionTests:SessionTests = new SessionTests();
    var programTests:ProgramTests = new ProgramTests();
    var kinematicsTests:KinematicsTests = new KinematicsTests();
    var processTests:ProcessTests = new ProcessTests();

    if (Sys.getEnv("MOTIONKIT_POSTURE_ONLY") == "1") {
      programTests.testExternalPathPosture();
      Sys.println('External path posture tests passed (${MotionKitTestSupport.assertions} assertions)');
      return;
    }
    if (Sys.getEnv("MOTIONKIT_FREEDOM_ONLY") == "1") {
      var tests = new ToolFreedomTests();
      tests.testFreeSpinPath();
      tests.testNumericalZeroMove();
      tests.testUnreachableTilt();
      tests.testConeAndRedundancy();
      tests.testFullOrientationIdentity();
      tests.testCaNearCableLimit();
      Sys.println('Tool freedom tests passed (${MotionKitTestSupport.assertions} assertions)');
      return;
    }
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
    if (Sys.getEnv("MOTIONKIT_PLANNER_LIMITS_ONLY") == "1") {
      plannerTests.testMoveLinearUsesPlannerLimits();
      Sys.println('Planner limits tests passed (${MotionKitTestSupport.assertions} assertions)');
      return;
    }
    if (Sys.getEnv("MOTIONKIT_CONTINUOUS_JOG_ONLY") == "1") {
      sessionTests.testContinuousJog();
      Sys.println('Continuous jog tests passed (${MotionKitTestSupport.assertions} assertions)');
      return;
    }
    if (Sys.getEnv("MOTIONKIT_SESSION_END_ONLY") == "1") {
      sessionTests.testImmediateMotionReplacesNativeQueue();
      sessionTests.testSmoothReplacementRetriesLateSubmission();
      sessionTests.testFreeRunningSmoothReplacement();
      sessionTests.testContinuousJog();
      sessionTests.testLateJogReplacementRejectsLateArrival();
      sessionTests.testPathHoldsStayOnPathWithinLimits();
      sessionTests.testDualMotorAxisChangesStayWithinJointLimits();
      Sys.println('Session end tests passed (${MotionKitTestSupport.assertions} assertions)');
      return;
    }
    if (Sys.getEnv("MOTIONKIT_BLEND_ONLY") == "1") {
      plannerTests.testToleranceBlend();
      Sys.println('Blend focused tests passed (${MotionKitTestSupport.assertions} assertions)');
      return;
    }
    if (Sys.getEnv("MOTIONKIT_PLANCHECK_ONLY") == "1") {
      new PlanCheckTests().testPlanningLimits();
      new PlanCheckTests().testHandlingUsesCoupledLimits();
      new PlanCheckTests().testPlanCheck();
      new PlanCheckTests().testStepperSlip();
      new PlanCheckTests().testEncoderSeesStepperSlip();
      new PlanCheckTests().testLoadSideEncoderReportsPathError();
      new PlanCheckTests().testCompilerRunsPlanCheck();
      new PlanCheckTests().testDirectMotionRunsPlanCheck();
      new PlanCheckTests().testServoStreamRunsPlanCheck();
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
    if (Sys.getEnv("MOTIONKIT_COREXY_ONLY") == "1") {
      new CoreXyTests().testTwoBeltCompliance();
      new CoreXyTests().testMotorSpaceTiming();
      new CoreXyTests().testBeltDisplay();
      new CoreXyTests().testPlotterDrawsASquare();
      new CoreXyTests().testPlanCheckAddsTheAxesOnASharedMotor();
      Sys.println('CoreXY tests passed (${MotionKitTestSupport.assertions} assertions)');
      return;
    }
    if (Sys.getEnv("MOTIONKIT_REDUNDANCY_ONLY") == "1") {
      kinematicsTests.testKinematicsContract();
      programTests.testProgramCompiler();
      programTests.testRedundantArmPaths();
      programTests.testExternalPathPosture();
      programTests.testCoordinatedExternalAxes();
      kinematicsTests.testPathConfigurationSelector();
      Sys.println('Redundancy focused tests passed (${MotionKitTestSupport.assertions} assertions)');
      return;
    }
    if (Sys.getEnv("MOTIONKIT_C4_ONLY") == "1") {
      kinematicsTests.testOpwKinematics();
      kinematicsTests.testCartesianAnalyticIk();
      kinematicsTests.testExternalOpwIk();
      kinematicsTests.testExternalUrIk();
      kinematicsTests.testNumericBranchFallback();
      kinematicsTests.testOrientationLattice();
      kinematicsTests.testCandidateProblem();
      Sys.println('C4 focused tests passed (${MotionKitTestSupport.assertions} assertions)');
      return;
    }
    var freedomTests = new ToolFreedomTests();
    // Each group is independent of the others, so that the workspace can run them as parallel shards.
    var everything = Shards.run([
      {name: "HomingTests.run", run: () -> HomingTests.run(), weight: 1.0},
      {name: "freedomTests.testFreeSpinPath", run: () -> freedomTests.testFreeSpinPath(), weight: 1.0},
      {name: "freedomTests.testNumericalZeroMove", run: () -> freedomTests.testNumericalZeroMove(), weight: 1.0},
      {name: "freedomTests.testUnreachableTilt", run: () -> freedomTests.testUnreachableTilt(), weight: 1.0},
      {name: "freedomTests.testConeAndRedundancy", run: () -> freedomTests.testConeAndRedundancy(), weight: 1.0},
      {name: "freedomTests.testFullOrientationIdentity", run: () -> freedomTests.testFullOrientationIdentity(), weight: 1.0},
      {name: "freedomTests.testCaNearCableLimit", run: () -> freedomTests.testCaNearCableLimit(), weight: 1.0},
      {name: "PlanCheckTests.testPlanningLimits", run: () -> new PlanCheckTests().testPlanningLimits(), weight: 1.0},
      {name: "PlanCheckTests.testHandlingUsesCoupledLimits", run: () -> new PlanCheckTests().testHandlingUsesCoupledLimits(), weight: 1.0},
      {name: "PlanCheckTests.testDirectMotionRunsPlanCheck", run: () -> new PlanCheckTests().testDirectMotionRunsPlanCheck(), weight: 1.0},
      {name: "PlanCheckTests.testServoStreamRunsPlanCheck", run: () -> new PlanCheckTests().testServoStreamRunsPlanCheck(), weight: 1.0},
      {name: "CoreXyTests.testMotorSpaceTiming", run: () -> new CoreXyTests().testMotorSpaceTiming(), weight: 1.0},
      {name: "CoreXyTests.testBeltDisplay", run: () -> new CoreXyTests().testBeltDisplay(), weight: 1.0},
      {name: "programTests.testExternalPathPosture", run: () -> programTests.testExternalPathPosture(), weight: 1.0},
      {name: "PlanCheckTests.testPlanCheck", run: () -> new PlanCheckTests().testPlanCheck(), weight: 0.05},
      {name: "PlanCheckTests.testStepperSlip", run: () -> new PlanCheckTests().testStepperSlip(), weight: 0.05},
      {name: "PlanCheckTests.testEncoderSeesStepperSlip", run: () -> new PlanCheckTests().testEncoderSeesStepperSlip(), weight: 0.3},
      {name: "PlanCheckTests.testLoadSideEncoderReportsPathError", run: () -> new PlanCheckTests().testLoadSideEncoderReportsPathError(), weight: 0.05},
      {name: "PlanCheckTests.testCompilerRunsPlanCheck", run: () -> new PlanCheckTests().testCompilerRunsPlanCheck(), weight: 0.1},
      {name: "CoreXyTests.testTwoBeltCompliance", run: () -> new CoreXyTests().testTwoBeltCompliance(), weight: 0.05},
      {name: "CoreXyTests.testPlotterDrawsASquare", run: () -> new CoreXyTests().testPlotterDrawsASquare(), weight: 0.5},
      {name: "CoreXyTests.testPlanCheckAddsTheAxesOnASharedMotor", run: () -> new CoreXyTests().testPlanCheckAddsTheAxesOnASharedMotor(), weight: 0.5},
      {name: "processTests.testPoseProcessPath", run: () -> processTests.testPoseProcessPath(), weight: 0.05},
      {name: "processTests.testMotionEventContracts", run: () -> processTests.testMotionEventContracts(), weight: 0.05},
      {name: "kinematicsTests.testKinematicsContract", run: () -> kinematicsTests.testKinematicsContract(), weight: 0.05},
      {name: "kinematicsTests.testSharedGroupAcrossThreads", run: () -> kinematicsTests.testSharedGroupAcrossThreads(), weight: 0.1},
      {name: "kinematicsTests.testOpwKinematics", run: () -> kinematicsTests.testOpwKinematics(), weight: 0.05},
      {name: "kinematicsTests.testCartesianAnalyticIk", run: () -> kinematicsTests.testCartesianAnalyticIk(), weight: 0.05},
      {name: "kinematicsTests.testExternalOpwIk", run: () -> kinematicsTests.testExternalOpwIk(), weight: 0.1},
      {name: "kinematicsTests.testExternalUrIk", run: () -> kinematicsTests.testExternalUrIk(), weight: 0.1},
      {name: "kinematicsTests.testNumericBranchFallback", run: () -> kinematicsTests.testNumericBranchFallback(), weight: 0.1},
      {name: "kinematicsTests.testOrientationLattice", run: () -> kinematicsTests.testOrientationLattice(), weight: 0.05},
      {name: "kinematicsTests.testCandidateProblem", run: () -> kinematicsTests.testCandidateProblem(), weight: 0.05},
      {name: "programTests.testMotionProgramContracts", run: () -> programTests.testMotionProgramContracts(), weight: 0.05},
      {name: "programTests.testProgramCompiler", run: () -> programTests.testProgramCompiler(), weight: 0.1},
      {name: "programTests.testRedundantArmPaths", run: () -> programTests.testRedundantArmPaths(), weight: 0.3},
      {name: "programTests.testCoordinatedExternalAxes", run: () -> programTests.testCoordinatedExternalAxes(), weight: 8.7},
      {name: "programTests.testProgramStartTolerances", run: () -> programTests.testProgramStartTolerances(), weight: 0.1},
      {name: "programTests.testProgramPlanner", run: () -> programTests.testProgramPlanner(), weight: 0.2},
      {name: "kinematicsTests.testPathConfigurationSelector", run: () -> kinematicsTests.testPathConfigurationSelector(), weight: 0.05},
      {name: "kinematicsTests.testAxisKinematics", run: () -> kinematicsTests.testAxisKinematics(), weight: 0.05},
      {name: "kinematicsTests.testManipulatorServo", run: () -> kinematicsTests.testManipulatorServo(), weight: 0.05},
      {name: "kinematicsTests.testServoSession", run: () -> kinematicsTests.testServoSession(), weight: 0.1},
      {name: "kinematicsTests.testServoPlans", run: () -> kinematicsTests.testServoPlans(), weight: 3.7},
      {name: "programTests.testManipulatorMotion", run: () -> programTests.testManipulatorMotion(), weight: 0.05},
      {name: "programTests.testManipulatorSessionTransitions", run: () -> programTests.testManipulatorSessionTransitions(), weight: 0.05},
      {name: "programTests.testProcessRunVirtualArmRecovery", run: () -> programTests.testProcessRunVirtualArmRecovery(), weight: 0.1},
      {name: "plannerTests.testSimplePathTimingContract", run: () -> plannerTests.testSimplePathTimingContract(), weight: 0.05},
      {name: "plannerTests.testNativePathLowering", run: () -> plannerTests.testNativePathLowering(), weight: 0.05},
      {name: "plannerTests.testToppraPathTiming", run: () -> plannerTests.testToppraPathTiming(), weight: 0.05},
      {name: "plannerTests.testGeometricPathPrimitives", run: () -> plannerTests.testGeometricPathPrimitives(), weight: 0.05},
      {name: "plannerTests.testNativeTrajectoryRoundTrip", run: () -> plannerTests.testNativeTrajectoryRoundTrip(), weight: 0.05},
      {name: "plannerTests.testNativeValidationAndPlan", run: () -> plannerTests.testNativeValidationAndPlan(), weight: 0.05},
      {name: "plannerTests.testPlannerIsDeterministicAndBounded", run: () -> plannerTests.testPlannerIsDeterministicAndBounded(), weight: 0.05},
      {name: "plannerTests.testToppraExactStopsAndBindings", run: () -> plannerTests.testToppraExactStopsAndBindings(), weight: 0.05},
      {name: "plannerTests.testToppraCircleAcceleration", run: () -> plannerTests.testToppraCircleAcceleration(), weight: 0.05},
      {name: "plannerTests.testCircularSegments", run: () -> plannerTests.testCircularSegments(), weight: 1.7},
      {name: "processTests.testLinearAxisCompilesToRobotModel", run: () -> processTests.testLinearAxisCompilesToRobotModel(), weight: 0.05},
      {name: "processTests.testLeadScrewActuatorRateLimitsPlans", run: () -> processTests.testLeadScrewActuatorRateLimitsPlans(), weight: 0.05},
      {name: "kinematicsTests.testTransmissionDerivedAxisMapping", run: () -> kinematicsTests.testTransmissionDerivedAxisMapping(), weight: 0.05},
      {name: "processTests.testCompiledAxisRunsThroughSimulation", run: () -> processTests.testCompiledAxisRunsThroughSimulation(), weight: 0.05},
      {name: "processTests.testHomingAndJogging", run: () -> processTests.testHomingAndJogging(), weight: 0.05},
      {name: "plannerTests.testMoveLinearUsesPlannerLimits", run: () -> plannerTests.testMoveLinearUsesPlannerLimits(), weight: 0.1},
      {name: "processTests.testCompiledXYZGantryRunsThroughSimulation", run: () -> processTests.testCompiledXYZGantryRunsThroughSimulation(), weight: 0.1},
      {name: "processTests.testMachineKitLeadScrewThroughVirtualDevice", run: () -> processTests.testMachineKitLeadScrewThroughVirtualDevice(), weight: 1},
      {name: "kinematicsTests.testDualMotorAxisRunsThroughSimulation", run: () -> kinematicsTests.testDualMotorAxisRunsThroughSimulation(), weight: 0.05},
      {name: "streamTests.testBufferedExecution", run: () -> streamTests.testBufferedExecution(), weight: 0.1},
      {name: "sessionTests.testNormalAbortWaitsForRest", run: () -> sessionTests.testNormalAbortWaitsForRest(), weight: 0.05},
      {name: "sessionTests.testMotionSessionTransitions", run: () -> sessionTests.testMotionSessionTransitions(), weight: 0.2},
      {name: "streamTests.testPlanCapableReplayRecordsMotionPlan", run: () -> streamTests.testPlanCapableReplayRecordsMotionPlan(), weight: 0.05},
      {name: "streamTests.testLongBufferedExecution", run: () -> streamTests.testLongBufferedExecution(), weight: 0.1},
      {name: "streamTests.testHoldRefillsNearChunkBoundary", run: () -> streamTests.testHoldRefillsNearChunkBoundary(), weight: 0.1},
      {name: "sessionTests.testHoldDecelerationStaysWithinLimitsThroughoutMove", run: () -> sessionTests.testHoldDecelerationStaysWithinLimitsThroughoutMove(), weight: 14.5},
      {name: "sessionTests.testRuntimeSynchronizedHolding", run: () -> sessionTests.testRuntimeSynchronizedHolding(), weight: 0.2},
      {name: "sessionTests.testImmediateMotionReplacesNativeQueue", run: () -> sessionTests.testImmediateMotionReplacesNativeQueue(), weight: 0.05},
      {name: "sessionTests.testSmoothReplacementRetriesLateSubmission", run: () -> sessionTests.testSmoothReplacementRetriesLateSubmission(), weight: 0.05},
      {name: "sessionTests.testFreeRunningSmoothReplacement", run: () -> sessionTests.testFreeRunningSmoothReplacement(), weight: 0.1},
      {name: "sessionTests.testMotionChangesStayWithinLimits", run: () -> sessionTests.testMotionChangesStayWithinLimits(), weight: 11},
      {name: "sessionTests.testContinuousJog", run: () -> sessionTests.testContinuousJog(), weight: 1.9},
      {name: "sessionTests.testLateJogReplacementRejectsLateArrival", run: () -> sessionTests.testLateJogReplacementRejectsLateArrival(), weight: 0.1},
      {name: "sessionTests.testPathHoldsStayOnPathWithinLimits", run: () -> sessionTests.testPathHoldsStayOnPathWithinLimits(), weight: 14},
      {name: "plannerTests.testToleranceBlend", run: () -> plannerTests.testToleranceBlend(), weight: 1},
      {name: "sessionTests.testDualMotorAxisChangesStayWithinJointLimits", run: () -> sessionTests.testDualMotorAxisChangesStayWithinJointLimits(), weight: 0.4}
    ]);
    if (everything)
      Sys.println('MotionKit bootstrap tests passed (${MotionKitTestSupport.assertions} assertions)');
  }

}
