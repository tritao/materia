package tests;

import RobotKitRuntime;
import haxe.Int64;
import sys.thread.Mutex;
import sys.thread.Thread;
import robotkit.runtime.RobotRuntimeBlueprint;
import robotkit.runtime.RobotRuntimeCompiler;
import robotkit.runtime.DifferentialDrivePlant;
import robotkit.runtime.HolonomicDrivePlant;
import robotkit.runtime.RobotRuntimeJointBlueprint;
import robotkit.runtime.RobotCompileException;
import robotkit.runtime.RobotRuntimeConfiguration;
import robotkit.runtime.RobotRuntimeMobileConfiguration;
import robotkit.runtime.RobotRuntimeForkConfiguration;
import robotkit.runtime.RobotRuntimeForkAxisConfiguration;
import robotkit.runtime.Simulation;
import robotkit.model.Joint;
import robotkit.model.JointType;
import robotkit.model.JointLimits;
import robotkit.model.Actuator;
import robotkit.model.Link;
import robotkit.model.RobotModel;
import robotkit.model.Frame;
import robotkit.model.Sensor;
import robotkit.model.RobotDriveConfiguration;
import robotkit.model.RobotMobileConfiguration;
import robotkit.model.RobotForkConfiguration;
import robotkit.model.RobotModelCodec;
import robotkit.device.DeviceChannel;
import robotkit.device.DeviceLayout;
import robotkit.world.RobotCapabilities;
import robotkit.world.RobotCommand;
import robotkit.world.RobotDescription;
import robotkit.world.RobotFault;
import robotkit.world.RobotId;
import robotkit.world.Robot;
import robotkit.world.RobotSnapshot;
import robotkit.world.RobotStatus;
import robotkit.world.RemoteRobot;
import robotkit.world.SerialRobot;
import robotkit.world.SimulatedRobot;
import robotkit.world.RecordingRobot;
import robotkit.world.StopMode;
import robotkit.world.RobotWorld;
import robotkit.world.RobotWorldEvent;
import robotkit.world.SensorFrame;
import robotkit.world.CameraImage;
import robotkit.protocol.BufferRef;
import robotkit.protocol.CameraFrame;
import robotkit.protocol.PixelFormat;
import robotkit.protocol.RobotFrame;
import robotkit.protocol.RobotMessageType;
import robotkit.protocol.RobotProtocol;
import haxeon.wire.MessagePack;
import robotkit.world.ReplayRobot;
import robotkit.world.RobotRecording;
import robotkit.world.RobotRecordingEvent;
import robotkit.world.McapRobotRecording;
import robotkit.world.McapRecordingReader;
import robotkit.world.McapRecordingStatus;
import robotkit.world.RobotRecordingCodec;
import robotkit.world.RobotRecordingEntry;
import robotkit.behavior.HoldJointBehavior;
import robotkit.behavior.WorldBehaviorRunner;
import robotkit.worldd.WorldHost;
import robotkit.mobile.Pose2;
import robotkit.mobile.Pose3;
import robotkit.mobile.Twist2;
import robotkit.mobile.MotionLimits;
import robotkit.mobile.Footprint;
import robotkit.mobile.FootprintPoint;
import robotkit.mobile.MobileBase;
import robotkit.mobile.DifferentialDrive;
import robotkit.mobile.AckermannDrive;
import robotkit.mobile.DifferentialOdometry;
import robotkit.mobile.HolonomicOdometry;
import robotkit.localization.WheelOdometryLocalization;
import robotkit.localization.WheelImuLocalization;
import robotkit.localization.SimulationTruthLocalization;
import robotkit.localization.LocalizationQuality;
import robotkit.localization.LocalizationState;
import robotkit.localization.PoseCovariance2;
import robotkit.localization.FrameTransform2;
import robotkit.localization.FrameTree2;
import robotkit.localization.PoseFusionLocalization;
import robotkit.localization.PoseFusionOptions;
import robotkit.localization.RobotFrameTree2;
import robotkit.localization.GnssPoseLocalization;
import robotkit.localization.Localization;
import robotkit.navigation.Path;
import robotkit.navigation.Trajectory;
import robotkit.navigation.TrajectorySample;
import robotkit.navigation.PathSpeedLimit;
import robotkit.navigation.NavigationGoal;
import robotkit.navigation.Navigation;
import robotkit.navigation.NavigationStatus;
import robotkit.navigation.MotionGuard;
import robotkit.navigation.MotionGuardState;
import robotkit.navigation.OccupancyGrid2;
import robotkit.navigation.OccupancyCell;
import robotkit.navigation.GridCell2;
import robotkit.navigation.Costmap2;
import robotkit.navigation.AStarPlanner;
import robotkit.navigation.Navigator;
import robotkit.navigation.NavigatorStatus;
import robotkit.material.ForkAxisConfig;
import robotkit.material.ForkAxisState;
import robotkit.material.ForkConfig;
import robotkit.material.ForkState;
import robotkit.material.Forks;
import robotkit.material.LoadLimits;
import robotkit.material.LoadState;
import robotkit.material.Payload;
import robotkit.perception.Detection;
import robotkit.perception.DepthCameraObstaclePerception;
import robotkit.perception.FiducialDetector;
import robotkit.perception.FiducialMarkerObservation;
import robotkit.perception.FiducialPerception;
import robotkit.perception.FiducialTargetConfig;
import robotkit.perception.Obstacle;
import robotkit.perception.DockingTarget;
import robotkit.perception.LidarObstaclePerception;
import robotkit.perception.PinholeCameraIntrinsics;
import robotkit.perception.PointCloudObstaclePerception;
import robotkit.perception.GroundTruthPerception;
import robotkit.perception.FrameAwarePerception;
import robotkit.perception.Pallet;
import robotkit.perception.PerceptionSnapshot;
import robotkit.safety.SafetyPhase;
import robotkit.safety.SafetyRestriction;
import robotkit.safety.SafetyState;
import robotkit.safety.StoppingEnvelope;
import robotkit.safety.LoadSafetyConfiguration;
import robotkit.safety.LoadSafetyPolicy;
import robotkit.power.BatteryState;
import robotkit.power.Power;
import robotkit.skill.Charge;
import robotkit.skill.Dock;
import robotkit.skill.FollowPath;
import robotkit.skill.GoTo;
import robotkit.skill.PickPallet;
import robotkit.skill.PlacePallet;
import robotkit.skill.Skill;
import robotkit.skill.SkillStatus;
import robotkit.skill.SkillRunner;

class RobotWorldTests {
  static var assertions = 0;

  public static function main():Void {
    testAttachDetachAndIdentity();
    testSequenceAndTopology();
    testCrossThreadEventQueue();
    testImmutableSnapshots();
    testRecordingEventLog();
    testReplayCorrectness();
    testJointTargetBatches();
    testMobileLayer();
    testModelDrivenConfiguration();
    testRobotModelCodec();
    testLocalization();
    testWheelImuLocalization();
    testGnssLocalization();
    testDepthObstaclePerception();
    testFiducialPerception();
    testNavigation();
    testMotionGuard();
    testGridPlanning();
    testNavigator();
    testGoToBlockedTimeout();
    testDifferentialDrivePlantKinematics();
    testHolonomicDrivePlantKinematics();
    testForkMechanisms();
    testPerceptionSafetyPower();
    testLoadSafetyPolicy();
    testSimulatedMaterialHandlingScenario();
    testForkliftSkillsOnSimulationAndReplay();
    testMcapRoundTrip();
    testMcapRobustness();
    testForwardingAndLifecycle();
    testMixedSimulatedAndRemoteWorld();
    testRuntimeUsesCompiledJointRate();
    testWorldHostComposition();
    testCompilerDiagnosticsAndTopology();
    testStableModelIdentity();
    testSensorAndClockContracts();
    testSensorResetPublication();
    testConfiguredSensors();
    testCameraFrameProtocol();
    testExternalSensorRuntime();
    testSerialRobotUnavailableDevice();
    assertions += SpatialTests.run();
    assertions += KinematicsTests.run();
    assertions += ToolTests.run();
    assertions += ProcessTests.run();
    assertions += WorkTests.run();
    assertions += PerceptionTests.run();
    assertions += PlacementTests.run();
    assertions += WallFinishingScenarioTests.run();
    assertions += ConstructionSkillTests.run();
    assertions += TerrainTests.run();
    assertions += ExcavatorTests.run();
    Sys.println('RobotKit world tests passed ($assertions assertions)');
  }

  static function testMcapRobustness():Void {
    var longPath='/tmp/robotkit-${Sys.getPid()}-long.mcap';
    var writer=new McapRobotRecording(longPath,16*1024*1024,false);
    equal(writer.memory,null,"file-only recording disables in-memory retention");
    for(index in 0...1000) writer.recordSnapshot(new RobotSnapshot("long",Int64.ofInt(index),
      Int64.ofInt(index),[index],[],[],1,0,Int64.ofInt(index),[],"long.clock","host"));
    writer.close();
    var terminal=McapRecordingReader.status(longPath);
    check(terminal!=null,"terminal recording status survives close");
    var terminalValue:McapRecordingStatus=cast terminal;
    equal(terminalValue.written,Int64.ofInt(1000),"persisted status retains write count");
    var cursor=new McapRecordingReader(longPath),count=0,firstTime=Int64.ofInt(0);
    while(true){var entry=cursor.next();if(entry==null)break;var entryValue:RobotRecordingEntry=cast entry;
      if(count==0)firstTime=entryValue.recordingTimestampNs;count++;}
    cursor.close();
    equal(count,1000,"incremental reader traverses a long recording");
    check(Int64.compare(firstTime,Int64.ofInt(0))>0,"recording timestamp is captured independently");
    sys.FileSystem.deleteFile(longPath);sys.FileSystem.deleteFile(longPath+".incomplete.status");

    var overflowPath='/tmp/robotkit-${Sys.getPid()}-overflow.mcap';
    var overflow=new McapRobotRecording(overflowPath,1,false);
    throws(function() overflow.recordSnapshot(new RobotSnapshot("overflow",Int64.ofInt(1),
      Int64.ofInt(1),[1.0],[],[],1,0)),"byte-bounded queue reports oversized payload overflow");
    throws(overflow.close,"close reports prior recording drop");
    var overflowStatus=McapRecordingReader.status(overflowPath);
    var overflowStatusValue:McapRecordingStatus=cast overflowStatus;
    check(overflowStatus!=null&&Int64.compare(overflowStatusValue.dropped,Int64.ofInt(1))==0,
      "drop status survives reopening");
    sys.FileSystem.deleteFile(overflowPath);sys.FileSystem.deleteFile(overflowPath+".incomplete.status");

    var failurePath='/tmp/robotkit-${Sys.getPid()}-write-failure';
    sys.FileSystem.createDirectory(failurePath);
    var failed=new McapRobotRecording(failurePath,1024,false);
    // The native writer cannot open a directory as an MCAP file.
    Sys.sleep(0.02);
    throws(failed.close,"asynchronous MCAP write-open failure is visible");
    var failureStatus=McapRecordingReader.status(failurePath);
    var failureStatusValue:McapRecordingStatus=cast failureStatus;
    check(failureStatus!=null&&failureStatusValue.error.length>0,
      "write failure survives process restart");
    sys.FileSystem.deleteFile(failurePath+".incomplete");
    sys.FileSystem.deleteFile(failurePath+".incomplete.status");
    sys.FileSystem.deleteDirectory(failurePath);

    var mismatchPath='/tmp/robotkit-${Sys.getPid()}-schema-mismatch.mcap';
    var mismatchEntry=new RobotRecordingEntry(Int64.ofInt(0),"mismatch",
      RobotRecordingEvent.Sensor("mismatch",new SensorFrame("sensor","imu","frame",
        Int64.ofInt(1),Int64.ofInt(1),[1.0])));
    var opened=RobotKitRuntime.rk_recording_writer_create(mismatchPath,Int64.ofInt(4096));
    equal(opened.status,RobotKitRuntimeConstants.RK_OK,"schema mismatch fixture opens");
    var payload=RobotRecordingCodec.encode(mismatchEntry);
    equal(RobotKitRuntime.rk_recording_writer_enqueue(opened.out_writer.borrow(),1,3,
      mismatchEntry.ordinal,mismatchEntry.recordingTimestampNs,payload),RobotKitRuntimeConstants.RK_OK,
      "schema mismatch fixture writes payload to wrong channel");
    equal(RobotKitRuntime.rk_recording_writer_finish(opened.out_writer.borrow()),RobotKitRuntimeConstants.RK_OK,
      "schema mismatch fixture closes");
    opened.out_writer.close();
    var mismatchReader=new McapRecordingReader(mismatchPath);
    throws(function(){mismatchReader.next();},"payload type must match MCAP channel schema");
    mismatchReader.close();sys.FileSystem.deleteFile(mismatchPath);
    sys.FileSystem.deleteFile(mismatchPath+".incomplete.status");
  }

  static function testReplayCorrectness():Void {
    var recording = new RobotRecording();
    recording.recordCommand(RobotCommand.JointTargets([
      robotkit.world.JointTarget.position(0, 99.0)
    ], null), "robot-a");
    recording.recordSnapshot(new RobotSnapshot("robot-a", Int64.ofInt(7), Int64.ofInt(100),
      [1.0], [], [], 1, 0, Int64.ofInt(200), [], "robot-a.boot-1", "host"));
    recording.recordSnapshot(new RobotSnapshot("robot-b", Int64.ofInt(7), Int64.ofInt(100),
      [9.0], [], [], 1, 0, Int64.ofInt(200), [], "robot-b.boot-1", "host"));
    recording.recordSensor("robot-b", new SensorFrame("wrong", "imu", "b/frame",
      Int64.ofInt(1), Int64.ofInt(101), [9.0], Int64.ofInt(201), "b/link"));
    recording.recordSensor("robot-a", new SensorFrame("imu", "imu", "a/frame",
      Int64.ofInt(1), Int64.ofInt(101), [1.0], Int64.ofInt(201), "a/link",
      null, null, "robot-a.boot-1", "host"));
    recording.recordFault(new RobotFault("robot-b", 90, "wrong robot", true));
    recording.recordFault(new RobotFault("robot-a", 12, "selected fault", false));
    // The source sequence repeats after a reset; clock identity distinguishes the observation.
    recording.recordSnapshot(new RobotSnapshot("robot-a", Int64.ofInt(7), Int64.ofInt(1),
      [2.0], [], [], 1, 0, Int64.ofInt(300), [], "robot-a.boot-2", "host"));

    var originalCommandCount = recording.commands.length;
    var replay = new ReplayRobot("robot-a", recording);
    equal(replay.snapshot().id, "robot-a", "replay selects one recorded robot");
    equal(replay.snapshot().positions.get(0), 1.0, "replay begins with selected robot snapshot");
    var runner = new WorldBehaviorRunner(new HoldJointBehavior(0, 0.25));
    equal(runner.update(replay), 1, "behavior consumes initial replay observation");
    equal(recording.commands.length, originalCommandCount, "replay leaves historical commands unchanged");
    equal(replay.generatedCommands.commands.length, 1, "replay captures generated commands separately");

    check(replay.advance(), "replay advances to selected sensor event");
    equal(replay.sensors().length, 1, "replay applies selected sensor event");
    equal(replay.sensors()[0].sensorId, "imu", "replay excludes another robot sensor");
    check(replay.advance(), "replay advances to selected fault event");
    var replayFault:RobotFault = cast replay.fault();
    equal(replayFault.code, 12, "replay applies selected fault event");
    equal(replay.snapshot().faultCode, 12, "fault event updates replay observation");
    check(replay.advance(), "replay advances across source clock reset");
    equal(replay.snapshot().positions.get(0), 2.0, "replay never substitutes interleaved robot snapshot");
    equal(replay.fault(), null, "healthy snapshot clears replayed fault");
    equal(runner.update(replay), 1, "behavior processes repeated sequence after clock reset");
    check(!replay.advance(), "replay contains only selected robot observations");
    replay.close();
  }

  static function testJointTargetBatches():Void {
    var source = [
      robotkit.world.JointTarget.position(0, 0.4),
      robotkit.world.JointTarget.velocity(1, -0.25),
      robotkit.world.JointTarget.effort(2, 3.5)
    ];
    var recording = new RobotRecording();
    recording.recordCommand(RobotCommand.JointTargets(source, null), "batch-robot");
    source[0] = robotkit.world.JointTarget.position(0, 9.0);
    source.pop();
    var recorded = recording.commands[0];
    switch recorded {
      case JointTargets(targets, _):
        equal(targets.length, 3, "recording owns the full command batch");
        equal(targets[0].target, 0.4, "recording copies immutable target values");
        equal(Std.string(targets[1].mode), Std.string(robotkit.world.JointTargetMode.Velocity),
          "recording preserves velocity interpretation");
        equal(Std.string(targets[2].mode), Std.string(robotkit.world.JointTargetMode.Effort),
          "recording preserves effort interpretation");
      case _:
        check(false, "recording retains a joint target batch");
    }

    var decoded = RobotRecordingCodec.decode(RobotRecordingCodec.encode(recording.entries[0]));
    switch decoded.event {
      case Command(JointTargets(targets, _)):
        equal(targets.length, 3, "recording codec round-trips every target in a batch");
        equal(Std.string(targets[0].mode), Std.string(robotkit.world.JointTargetMode.Position),
          "recording codec preserves position mode");
        equal(targets[1].target, -0.25, "recording codec preserves velocity values");
        equal(targets[2].target, 3.5, "recording codec preserves effort values");
      case _:
        check(false, "recording codec decodes a command batch");
    }

    var legacy = RobotRecordingCodec.decode(haxe.io.Bytes.ofString(
      '{"version":1,"ordinal":"0","recordingTimestampNs":"1","robotId":"old","sourceSequence":"0","sourceTimestampNs":"0","sourceClockId":"unspecified","type":"command","payload":{"kind":"jointPosition","joint":2,"target":0.75,"expiryNs":null}}'));
    switch legacy.event {
      case Command(JointTargets(targets, _)):
        equal(targets.length, 1, "legacy single-target command becomes a one-target batch");
        equal(Std.string(targets[0].mode), Std.string(robotkit.world.JointTargetMode.Position),
          "legacy recording defaults to position mode");
      case _:
        check(false, "legacy command recording remains replayable");
    }

    var replay = new ReplayRobot("batch-robot", recording);
    replay.submit(RobotCommand.JointTargets([
      robotkit.world.JointTarget.position(0, -0.1),
      robotkit.world.JointTarget.velocity(1, 0.5)
    ], null));
    switch replay.generatedCommands.commands[0] {
      case JointTargets(targets, _):
        equal(targets.length, 2, "replay captures generated commands as one batch");
        equal(Std.string(targets[1].mode), Std.string(robotkit.world.JointTargetMode.Velocity),
          "replay preserves generated target modes");
      case _:
        check(false, "replay generated command keeps batch form");
    }
    replay.close();
    throws(function() robotkit.world.JointTarget.copyBatch([
      robotkit.world.JointTarget.position(0, 0.0),
      robotkit.world.JointTarget.effort(0, 1.0)
    ]), "duplicate joints rejected within one atomic batch");
  }

  static function testMobileLayer():Void {
    var origin = new Pose2(1.0, 2.0, Math.PI * 0.5);
    var composed = origin.compose(new Pose2(1.0, 0.0, 0.0));
    check(Math.abs(composed.x - 1.0) < 1e-9 && Math.abs(composed.y - 3.0) < 1e-9,
      "Pose2 composes local translation in the parent frame");
    var roundTrip = composed.relativeTo(origin);
    check(Math.abs(roundTrip.x - 1.0) < 1e-9 && Math.abs(roundTrip.y) < 1e-9,
      "Pose2 computes frame-relative poses");
    var straight = new Pose2().integrate(new Twist2(1.0, 0.0), 2.0);
    check(Math.abs(straight.x - 2.0) < 1e-9 && Math.abs(straight.y) < 1e-9,
      "Pose2 integrates straight body motion");
    var curve = new Pose2().integrate(new Twist2(1.0, 1.0), Math.PI * 0.5);
    check(Math.abs(curve.x - 1.0) < 1e-9 && Math.abs(curve.y - 1.0) < 1e-9,
      "Pose2 integrates constant-curvature motion");
    var bodyArc = new Pose2(2.0, -1.0, 0.4).integrate(new Twist2(1.0, 0.5, 0.25), 2.0);
    var expectedBodyArc = new Pose2(2.0, -1.0, 0.4).integrateDisplacement(2.0, 1.0, 0.5);
    check(Math.abs(bodyArc.x - expectedBodyArc.x) < 1e-9 &&
      Math.abs(bodyArc.y - expectedBodyArc.y) < 1e-9 &&
      Math.abs(Pose2.wrapAngle(bodyArc.yaw - expectedBodyArc.yaw)) < 1e-9,
      "Pose2 integrates forward, lateral, and yaw motion as one constant body twist");

    var footprint = Footprint.rectangle(2.0, 1.0);
    equal(footprint.vertices().length, 4, "rectangular footprint exposes copied vertices");
    check(Math.abs(footprint.radius - 1.118033988749895) < 1e-9,
      "footprint radius encloses its corners");
    var vertices = footprint.vertices();
    vertices.pop();
    equal(footprint.vertices().length, 4, "footprint vertex list is immutable to callers");
    throws(function() new Footprint([
      new FootprintPoint(0.0, 0.0), new FootprintPoint(1.0, 0.0), new FootprintPoint(2.0, 0.0)
    ]), "degenerate footprint polygons are rejected");

    var differentialRobot = new FakeRobot("diff-base");
    differentialRobot.positions = [0.0, 0.0];
    var differential = new MobileBase(differentialRobot,
      new DifferentialDrive(0, 1, 0.2, 0.6),
      new MotionLimits(1.0, 2.0, 0.5, 1.0), footprint);
    var bounded = differential.command(new Twist2(2.0, 4.0), 1.0);
    check(Math.abs(bounded.linear - 0.5) < 1e-9 && Math.abs(bounded.angular - 1.0) < 1e-9,
      "mobile command applies acceleration limits after speed limits");
    check(switch differentialRobot.lastCommand {
      case JointTargets(targets, _):
        targets.length == 2 && targets[0].joint == 0 && targets[1].joint == 1 &&
          targets[0].mode == robotkit.world.JointTargetMode.Velocity &&
          targets[1].mode == robotkit.world.JointTargetMode.Velocity &&
          Math.abs(targets[0].target - 1.0) < 1e-9 &&
          Math.abs(targets[1].target - 4.0) < 1e-9;
      case _: false;
    }, "differential drive submits both wheel velocities as one atomic command");

    var mobileBlueprint = new RobotRuntimeBlueprint(1, 2, 3);
    mobileBlueprint.addJoint(new RobotRuntimeJointBlueprint(0,
      RobotKitRuntimeConstants.RK_RUNTIME_JOINT_REVOLUTE, 0, 1, -100.0, 100.0, 100.0, 50.0));
    mobileBlueprint.addJoint(new RobotRuntimeJointBlueprint(1,
      RobotKitRuntimeConstants.RK_RUNTIME_JOINT_REVOLUTE, 0, 2, -100.0, 100.0, 100.0, 50.0));
    var mobileSimulation = new Simulation(0.01);
    var simulatedRobot = new SimulatedRobot("mobile-sim",
      mobileSimulation.addRobot(mobileBlueprint), "mobile simulation",
      ["base", "left-wheel", "right-wheel"], ["left-wheel-joint", "right-wheel-joint"]);
    var simulatedBase = new MobileBase(simulatedRobot,
      new DifferentialDrive(0, 1, 0.1, 0.5), new MotionLimits(1.0, 2.0));
    simulatedBase.command(new Twist2(0.2, 0.0));
    mobileSimulation.step(Int64.ofInt(100));
    var simulatedState = simulatedRobot.snapshot();
    check(Math.abs(simulatedState.velocities.get(0) - 2.0) < 1e-9 &&
      Math.abs(simulatedState.velocities.get(1) - 2.0) < 1e-9,
      "mobile base drives both simulated wheels through the shared Simulation runtime");
    simulatedRobot.close();
    mobileSimulation.dispose();

    var speedLimited = differential.command(new Twist2(10.0, 0.0));
    check(Math.abs(speedLimited.linear - 1.0) < 1e-9,
      "mobile command applies configured body speed limits");
    differential.stop();
    check(switch differentialRobot.lastStop {
      case Normal: true;
      case _: false;
    },
      "mobile base forwards stop through the wrapped robot");

    var ackermannRobot = new FakeRobot("ackermann-base");
    ackermannRobot.positions = [0.0, 0.0];
    var ackermann = new MobileBase(ackermannRobot,
      new AckermannDrive(0, 1, 1.2, 0.25, 0.5), new MotionLimits(2.0, 2.0));
    var ackermannCommand = ackermann.command(new Twist2(1.0, 1.0));
    check(ackermannCommand.angular < 1.0 && ackermannCommand.angular > 0.0,
      "Ackermann drive limits yaw rate to the configured steering angle");
    check(switch ackermannRobot.lastCommand {
      case JointTargets(targets, _):
        targets.length == 2 && targets[0].mode == robotkit.world.JointTargetMode.Position &&
          targets[1].mode == robotkit.world.JointTargetMode.Velocity &&
          Math.abs(targets[0].target) <= 0.5 && targets[1].target > 0.0;
      case _: false;
    }, "Ackermann drive submits steering position and wheel velocity together");
    throws(function() ackermann.command(new Twist2(0.0, 1.0)),
      "Ackermann drive rejects a turn-in-place request");
    throws(function() ackermann.command(new Twist2(1.0, 0.0, 0.1)),
      "Ackermann drive rejects a lateral command");
    ackermann.command(Twist2.zero());
    check(switch ackermannRobot.lastCommand {
      case JointTargets(targets, _): targets[0].target == 0.0 && targets[1].target == 0.0;
      case _: false;
    }, "Ackermann drive safely emits zero steering and wheel targets at rest");
    var wideAckermann = new AckermannDrive(0, 1, 1.2, 0.25, 1.2);
    var wideTargets = wideAckermann.targets(new Twist2(1.0, 2.0));
    check(wideTargets[0].target > 1.1 && wideTargets[0].target < 1.2,
      "Ackermann curvature mapping handles steering ratios above one");

    var holonomicRobot = new FakeRobot("holonomic-base");
    holonomicRobot.positions = [0.0, 0.0, 0.0];
    var holonomic = new MobileBase(holonomicRobot,
      new robotkit.mobile.HolonomicDrive([0, 1, 2], 0.05, 0.3), new MotionLimits(1.0, 2.0));
    holonomic.command(new Twist2(0.4, 0.5));
    check(switch holonomicRobot.lastCommand {
      case JointTargets(targets, _):
        targets.length == 3 && targets[0].joint == 0 && targets[1].joint == 1 && targets[2].joint == 2 &&
          targets[0].mode == robotkit.world.JointTargetMode.Velocity &&
          targets[1].mode == robotkit.world.JointTargetMode.Velocity &&
          targets[2].mode == robotkit.world.JointTargetMode.Velocity;
      case _: false;
    }, "holonomic drive submits all three wheel velocities as one atomic command");
    var straightTargets = new robotkit.mobile.HolonomicDrive([0, 1, 2], 0.05, 0.3).targets(new Twist2(1.0, 0.0));
    var sumOfSpeeds = straightTargets[0].target + straightTargets[1].target + straightTargets[2].target;
    check(Math.abs(sumOfSpeeds) < 1e-9,
      "holonomic wheel speeds sum to zero for pure translation (three wheels at 120 degrees)");
    var lateralTargets = new robotkit.mobile.HolonomicDrive([0, 1, 2], 0.05, 0.3).targets(new Twist2(0.0, 0.0, 1.0));
    for (i in 0...3) {
      var angle = Math.PI * 0.5 + i * Math.PI * 2.0 / 3.0;
      check(Math.abs(lateralTargets[i].target - Math.cos(angle) / 0.05) < 1e-9,
        "holonomic drive maps body lateral velocity to each wheel's tangent");
    }
    var spinTargets = new robotkit.mobile.HolonomicDrive([0, 1, 2], 0.05, 0.3).targets(new Twist2(0.0, 2.0));
    for (target in spinTargets)
      check(Math.abs(target.target - 0.3 * 2.0 / 0.05) < 1e-9,
        "holonomic in-place rotation drives every wheel at the same tangential rate");
    check(new robotkit.mobile.HolonomicDrive([0, 1, 2], 0.05, 0.3).supportsInPlaceRotation(),
      "holonomic drive supports in-place rotation");
    throws(function() new robotkit.mobile.HolonomicDrive([0, 1, 1], 0.05, 0.3),
      "holonomic drive rejects duplicate wheel joint indices");
    throws(function() differential.command(new Twist2(0.0, 0.0, 0.1)),
      "differential drive rejects a lateral command");

    var odometry = new DifferentialOdometry(0, 1, 0.1, 0.5);
    function sample(time:Int, left:Float, right:Float, clock:String):RobotSnapshot
      return new RobotSnapshot("odom", Int64.ofInt(time), Int64.ofInt(time),
        [left, right], [], [], 1, 0, null, [], clock, "host");
    var initial = odometry.update(sample(10, 0.0, 0.0, "source-A"));
    var forward = odometry.update(sample(20, 1.0, 1.0, "source-A"));
    check(Math.abs(initial.x) < 1e-9 && Math.abs(forward.x - 0.1) < 1e-9,
      "wheel odometry integrates straight wheel travel");
    var turning = odometry.update(sample(30, 1.0, 2.0, "source-A"));
    check(Math.abs(turning.yaw - 0.2) < 1e-9 && turning.y > forward.y,
      "wheel odometry integrates differential wheel displacement");
    odometry.update(sample(1, 4.0, 4.0, "source-B"));
    var afterClockReset = odometry.update(sample(2, 4.2, 4.2, "source-B"));
    check(Math.abs(afterClockReset.yaw - turning.yaw) < 1e-9 &&
      Math.abs(afterClockReset.x - turning.x) < 0.03,
      "wheel odometry resets its encoder baseline when the source clock changes");
    odometry.update(sample(1, 20.0, 20.0, "source-B"));
    var afterStale = odometry.update(sample(3, 4.3, 4.3, "source-B"));
    check(Math.abs(afterStale.x - afterClockReset.x) < 0.03,
      "wheel odometry ignores stale timestamps without moving its baseline");

    var holonomicOdometry = new HolonomicOdometry([0, 1, 2], 0.05, 0.3);
    function omniSample(time:Int, positions:Array<Float>):RobotSnapshot
      return new RobotSnapshot("omni-odom", Int64.ofInt(time), Int64.ofInt(time),
        positions, [], [], 1, 0, null, [], "omni-clock", "host");
    holonomicOdometry.update(omniSample(10, [0.0, 0.0, 0.0]));
    var lateralDistance = 0.1;
    var lateralPositions = [for (i in 0...3) {
      var angle = Math.PI * 0.5 + i * Math.PI * 2.0 / 3.0;
      Math.cos(angle) * lateralDistance / 0.05;
    }];
    var lateralPose = holonomicOdometry.update(omniSample(20, lateralPositions));
    check(Math.abs(lateralPose.x) < 1e-9 && Math.abs(lateralPose.y - lateralDistance) < 1e-9 &&
      Math.abs(holonomicOdometry.lastLateralDistance - lateralDistance) < 1e-9,
      "holonomic odometry recovers pure body lateral travel");
    var forwardDistance = 0.2, nextLateral = 0.1, headingChange = 0.3;
    var diagonalPositions = [for (i in 0...3) {
      var angle = Math.PI * 0.5 + i * Math.PI * 2.0 / 3.0;
      var wheelDistance = -Math.sin(angle) * forwardDistance +
        Math.cos(angle) * nextLateral + 0.3 * headingChange;
      lateralPositions[i] + wheelDistance / 0.05;
    }];
    var diagonalPose = holonomicOdometry.update(omniSample(30, diagonalPositions));
    var expectedDiagonal = lateralPose.integrateDisplacement(forwardDistance, headingChange, nextLateral);
    check(Math.abs(diagonalPose.x - expectedDiagonal.x) < 1e-9 &&
      Math.abs(diagonalPose.y - expectedDiagonal.y) < 1e-9 &&
      Math.abs(Pose2.wrapAngle(diagonalPose.yaw - expectedDiagonal.yaw)) < 1e-9,
      "holonomic odometry recovers diagonal translation and yaw");
  }

  static function testModelDrivenConfiguration():Void {
    var model = new RobotModel("authored-forklift");
    var base = model.addLink(new Link("base", "link/base"));
    var left = model.addLink(new Link("left wheel", "link/left-wheel"));
    var right = model.addLink(new Link("right wheel", "link/right-wheel"));
    var mast = model.addLink(new Link("mast", "link/mast"));
    var tilt = model.addLink(new Link("fork carriage", "link/carriage"));
    var spread = model.addLink(new Link("forks", "link/forks"));
    function addJoint(id:String, name:String, type:JointType, child:Link,
        lower:Float, upper:Float):Joint {
      var joint = new Joint(name, type, base, child, id);
      joint.limits = new JointLimits(lower, upper, 20.0, 100.0);
      return model.addJoint(joint);
    }
    addJoint("joint/left-wheel", "left wheel joint", JointType.Continuous, left, -100.0, 100.0);
    addJoint("joint/right-wheel", "right wheel joint", JointType.Continuous, right, -100.0, 100.0);
    addJoint("joint/lift", "mast lift", JointType.Prismatic, mast, 0.0, 2.0);
    addJoint("joint/tilt", "fork tilt", JointType.Revolute, tilt, -0.5, 0.7);
    addJoint("joint/spread", "fork spread", JointType.Prismatic, spread, 0.0, 0.8);
    model.mobileBase = new RobotMobileConfiguration(
      RobotDriveConfiguration.Differential("joint/left-wheel", "joint/right-wheel", 0.1, 0.5),
      1.2, 1.5, 0.8, 1.0, 2.2, 1.1);
    model.forkMechanism = new RobotForkConfiguration("joint/lift",
      1000.0, 600.0, 1.8, "joint/tilt", "joint/spread");

    var blueprint = RobotRuntimeCompiler.compile(model);
    var runtimeConfiguration:Null<RobotRuntimeConfiguration> = blueprint.configuration;
    check(runtimeConfiguration != null,
      "runtime compilation preserves authored mobile and fork roles");
    var resolvedConfiguration:RobotRuntimeConfiguration = cast runtimeConfiguration;
    var mobileRuntime:Null<RobotRuntimeMobileConfiguration> = resolvedConfiguration.mobileBase;
    var forkRuntime:Null<RobotRuntimeForkConfiguration> = resolvedConfiguration.forks;
    check(mobileRuntime != null && forkRuntime != null,
      "runtime compilation includes both mobile and fork views");
    var mobileConfig:RobotRuntimeMobileConfiguration = cast mobileRuntime;
    var forkConfig:RobotRuntimeForkConfiguration = cast forkRuntime;
    check(switch mobileConfig.drive {
      case robotkit.runtime.RobotRuntimeDriveConfiguration.Differential(leftIndex, leftName,
          rightIndex, rightName, radius, track):
        leftIndex == 0 && leftName == "left wheel joint" && rightIndex == 1 &&
          rightName == "right wheel joint" && radius == 0.1 && track == 0.5;
      case _: false;
    }, "runtime compilation resolves stable drive joint IDs to command indices");
    equal(forkConfig.lift.jointIndex, 2,
      "runtime compilation resolves the authored fork lift role");
    var tiltRuntime:RobotRuntimeForkAxisConfiguration = cast forkConfig.tilt;
    var spreadRuntime:RobotRuntimeForkAxisConfiguration = cast forkConfig.spread;
    check(tiltRuntime.jointIndex == 3 && spreadRuntime.jointIndex == 4,
      "runtime compilation resolves optional fork axes");

    var robot = new FakeRobot("authored-forklift");
    robot.jointNames = [for (joint in model.joints) joint.name];
    robot.positions = [0.0, 0.0, 0.25, 0.1, 0.3];
    robot.velocities = [0.0, 0.0, 0.0, -0.02, 0.0];
    robot.efforts = [0.0, 0.0, 2.0, 0.5, 0.25];
    var mobile = MobileBase.fromRobot(robot, model);
    var footprint:Footprint = cast mobile.footprint;
    check(mobile.footprint != null && Math.abs(footprint.radius - 1.23) < 0.01,
      "MobileBase factory applies the model-authored footprint");
    mobile.command(new Twist2(0.4, 0.0));
    check(switch robot.lastCommand {
      case JointTargets(targets, _): targets.length == 2 &&
        targets[0].joint == 0 && targets[1].joint == 1 &&
        Math.abs(targets[0].target - 4.0) < 1e-9 &&
        Math.abs(targets[1].target - 4.0) < 1e-9;
      case _: false;
    }, "MobileBase factory commands the joints selected by authored roles");

    var forks = Forks.fromRobot(robot, model);
    var blueprintForks = Forks.fromBlueprint(robot, blueprint);
    check(forks.config.lift.jointName == blueprintForks.config.lift.jointName,
      "Forks can be constructed from either the model or its compiled blueprint");
    var forkState = forks.state();
    var tiltState:ForkAxisState = cast forkState.tilt;
    var spreadState:ForkAxisState = cast forkState.spread;
    check(forkState.lift.position == 0.25 &&
      tiltState.position == 0.1 && spreadState.position == 0.3,
      "Forks factory maps model roles and limits into live fork state");
    forks.raise(1.0);
    check(switch robot.lastCommand {
      case JointTargets(targets, _): targets.length == 1 &&
        targets[0].joint == 2 && targets[0].target == 1.0;
      case _: false;
    }, "Forks factory commands the authored lift joint without manual names or indices");

    var savedForkConfig = model.forkMechanism;
    model.forkMechanism = null;
    model.mobileBase = new RobotMobileConfiguration(
      RobotDriveConfiguration.Ackermann("joint/tilt", "joint/left-wheel",
        1.2, 0.1, 0.5), 1.2, 1.5);
    var ackermannBlueprint = RobotRuntimeCompiler.compile(model);
    var ackermann = MobileBase.fromBlueprint(robot, ackermannBlueprint);
    ackermann.command(new Twist2(0.5, 0.2));
    check(switch robot.lastCommand {
      case JointTargets(targets, _): targets.length == 2 &&
        targets[0].joint == 3 && targets[0].mode == robotkit.world.JointTargetMode.Position &&
        targets[1].joint == 0 && targets[1].mode == robotkit.world.JointTargetMode.Velocity;
      case _: false;
    }, "model-driven Ackermann roles preserve the steering and wheel target modes");
    model.forkMechanism = savedForkConfig;

    var mismatched = new FakeRobot("wrong-joint-order");
    mismatched.jointNames = ["right wheel joint", "left wheel joint", "mast lift",
      "fork tilt", "fork spread"];
    throws(function() MobileBase.fromBlueprint(mismatched, blueprint),
      "model-driven factory detects a runtime robot with a different joint order");
    var ambiguous = new FakeRobot("ambiguous-joint-description");
    ambiguous.jointNames = ["mast lift", "right wheel joint", "mast lift",
      "fork tilt", "fork spread"];
    throws(function() Forks.fromBlueprint(ambiguous, blueprint),
      "model-driven fork factory rejects ambiguous duplicate joint names");

    model.mobileBase = new RobotMobileConfiguration(
      RobotDriveConfiguration.Differential("missing-left", "joint/right-wheel", 0.1, 0.5),
      1.2, 1.5);
    var missingDiagnostics = RobotRuntimeCompiler.validate(model);
    var hasMissingRole = false;
    for (value in missingDiagnostics) if (value.code == "RK_ROLE_JOINT") hasMissingRole = true;
    check(missingDiagnostics.length > 0 &&
      hasMissingRole,
      "robot model validation rejects mechanism roles that reference missing joints");
    model.mobileBase = new RobotMobileConfiguration(
      RobotDriveConfiguration.Differential("joint/lift", "joint/right-wheel", 0.1, 0.5),
      1.2, 1.5);
    var typeDiagnostics = RobotRuntimeCompiler.validate(model);
    var hasTypeError = false;
    for (value in typeDiagnostics) if (value.code == "RK_ROLE_TYPE") hasTypeError = true;
    check(hasTypeError,
      "robot model validation rejects joint types that cannot fill a mechanism role");
    model.mobileBase = new RobotMobileConfiguration(
      RobotDriveConfiguration.Differential("joint/left-wheel", "joint/left-wheel", 0.1, 0.5),
      1.2, 1.5);
    var duplicateDiagnostics = RobotRuntimeCompiler.validate(model);
    var hasDuplicateRole = false;
    for (value in duplicateDiagnostics) if (value.code == "RK_ROLE_DUPLICATE") hasDuplicateRole = true;
    check(hasDuplicateRole,
      "robot model validation rejects assigning one joint to multiple mechanism roles");
  }

  static function testRobotModelCodec():Void {
    var source = configuredForkliftModel();
    source.collisionApproximation = robotkit.model.CollisionApproximation.None;
    source.links[0].mass = 42.5;
    source.links[0].centerOfMass = [0.1, -0.2, 0.3];
    source.links[0].inertiaTensor = [2.0, 0.1, 0.0, 0.1, 3.0, 0.2, 0.0, 0.2, 4.0];
    source.links[0].visualGeometry = "meshes/base.glb";
    source.links[0].collisionGeometry = "colliders/base.obj";
    source.joints[0].drive = new Actuator("left-wheel-drive", 90.0, 12.0);
    source.joints[0].parentFramePosition = [0.0, 0.25, 0.1];
    source.joints[0].parentFrameRotation = [0.0, 0.0, 0.1, 0.99498743710662];
    source.joints[0].axis = [0.0, 1.0, 0.0];
    source.frames[0].rotation = [0.0, 0.0, 0.38268343236509, 0.923879532511287];
    source.sensors[0].startAngleRadians = -0.4;
    source.sensors[0].fieldOfViewRadians = 2.4;

    var encoded = RobotModelCodec.encode(source);
    var restored = RobotModelCodec.decode(encoded);
    equal(restored.schemaVersion, RobotModel.CURRENT_VERSION,
      "decoded RobotModel uses the current semantic schema");
    equal(RobotModelCodec.encode(restored).toString(), encoded.toString(),
      "canonical RobotModel artifact round-trips byte for byte");
    equal(restored.links[0].mass, 42.5, "RobotModel codec preserves link mass");
    equal(restored.links[0].inertiaTensor[7], 0.2, "RobotModel codec preserves inertia tensor");
    equal(restored.links[0].visualGeometry, "meshes/base.glb",
      "RobotModel codec preserves geometry references");
    equal(restored.joints[0].parentFramePosition[1], 0.25,
      "RobotModel codec preserves joint frame transforms");
    var restoredDrive:Actuator = cast restored.joints[0].drive;
    equal(restoredDrive.maxRate, 12.0, "RobotModel codec preserves actuator settings");
    equal(restored.frames[0].link.id, "link/base", "RobotModel codec resolves frame link references");
    check(restored.sensors[0].frame == restored.frames[0],
      "RobotModel codec resolves sensor frame references to shared frame objects");
    equal(restored.sensors[0].startAngleRadians, -0.4,
      "RobotModel codec preserves LiDAR angular origin");
    check(restored.mobileBase != null && restored.forkMechanism != null,
      "RobotModel codec preserves mobile and fork configurations");
    equal(RobotRuntimeCompiler.compile(restored).jointCount, source.joints.length,
      "decoded canonical model compiles through the normal runtime path");
    var ackermannSource = configuredForkliftModel();
    ackermannSource.mobileBase = new RobotMobileConfiguration(
      RobotDriveConfiguration.Ackermann("joint/tilt", "joint/left-wheel", 1.2, 0.1, 0.5),
      1.2, 1.5);
    var ackermannRestored = RobotModelCodec.decode(RobotModelCodec.encode(ackermannSource));
    var ackermannMobile:RobotMobileConfiguration = cast ackermannRestored.mobileBase;
    check(switch ackermannMobile.drive {
      case Ackermann(steering, drive, wheelBase, radius, maxAngle):
        steering == "joint/tilt" && drive == "joint/left-wheel" &&
          wheelBase == 1.2 && radius == 0.1 && maxAngle == 0.5;
      case _: false;
    }, "RobotModel codec round-trips Ackermann drive configuration");

    var channelRecords = [for (index in 0...source.joints.length)
      {index: index, joint: source.joints[index].id}];
    var deviceLayout = DeviceLayout.decode(haxe.io.Bytes.ofString(
      haxe.Json.stringify({channels: channelRecords})));
    deviceLayout.validateAgainst(source);
    var wrongLayout = new DeviceLayout([for (index in 0...source.joints.length)
      new DeviceChannel(index, source.joints[source.joints.length - index - 1].id)]);
    throws(function() wrongLayout.validateAgainst(source),
      "device channel mapping rejects order that differs from semantic model joints");

    var legacyV1 = haxe.io.Bytes.ofString('{"schemaVersion":1,"name":"legacy-arm",'
      + '"links":[{"id":"base","name":"base"}],"joints":[],"frames":[],"sensors":[]}');
    var migratedV1 = RobotModelCodec.decode(legacyV1);
    equal(migratedV1.schemaVersion, RobotModel.CURRENT_VERSION,
      "v1 RobotModel artifact migrates to the current schema");
    equal(migratedV1.links[0].mass, 1.0, "v1 migration supplies default link mass");
    equal(migratedV1.links[0].inertiaTensor[8], 1.0,
      "v1 migration supplies default link inertia");
    equal(RobotModelCodec.decode(RobotModelCodec.encode(migratedV1)).name, "legacy-arm",
      "migrated RobotModel can be saved in the current canonical format");

    var legacyV2:Dynamic = haxe.Json.parse(encoded.toString());
    Reflect.setField(legacyV2, "schemaVersion", 2);
    Reflect.setField(legacyV2, "mobileBase", null);
    Reflect.setField(legacyV2, "forkMechanism", null);
    var legacySensors:Array<Dynamic> = cast Reflect.field(legacyV2, "sensors");
    Reflect.setField(legacySensors[0], "startAngleRadians", null);
    Reflect.setField(legacySensors[0], "fieldOfViewRadians", null);
    var migratedV2 = RobotModelCodec.decode(haxe.io.Bytes.ofString(haxe.Json.stringify(legacyV2)));
    equal(migratedV2.sensors[0].startAngleRadians, 0.0,
      "v2 migration supplies default sensor angle");
    equal(migratedV2.sensors[0].fieldOfViewRadians, Math.PI * 2.0,
      "v2 migration supplies full-circle sensor coverage");
    equal(migratedV2.mobileBase, null, "v2 migration defaults newer semantic roles");

    throws(function() RobotModelCodec.decode(haxe.io.Bytes.ofString('{"schemaVersion":99}')),
      "future RobotModel schema versions are rejected");
    var brokenReference:Dynamic = haxe.Json.parse(encoded.toString());
    var brokenJoints:Array<Dynamic> = cast Reflect.field(brokenReference, "joints");
    Reflect.setField(brokenJoints[0], "parentLink", "link/missing");
    throws(function() RobotModelCodec.decode(haxe.io.Bytes.ofString(haxe.Json.stringify(brokenReference))),
      "RobotModel codec rejects unresolved link references");
  }

  static function testGnssLocalization():Void {
    // 8.9832e-6 degrees of longitude is one metre east at the equator.
    var oneMetreEast = 0.000008983152841195214;
    var gnss = new GnssPoseLocalization("gnss-front", "map", "base", 0.0, 0.0,
      new Pose2(1.0, 0.0, 0.0), 0.04, 0.01, 500000000.0);
    function fixSnapshot(sequence:Int, timestamp:Int, latitude:Float, longitude:Float,
        yaw:Float, sampleTimestamp:Int, fixClock:String, fixReceivedClock:String,
        frameId:String):RobotSnapshot {
      var fix = new SensorFrame("gnss-front", "gnss_pose", frameId, Int64.ofInt(sequence),
        Int64.ofInt(sampleTimestamp), [latitude, longitude, yaw],
        Int64.ofInt(sampleTimestamp + 5), "antenna-link", null, null, fixClock, fixReceivedClock);
      return new RobotSnapshot("gnss-base", Int64.ofInt(sequence), Int64.ofInt(timestamp),
        [], [], [], 1, 0, Int64.ofInt(timestamp + 10), [fix], "gnss-clock", "host-clock");
    }
    var state = gnss.update(fixSnapshot(1, 100, 0.0, oneMetreEast, Math.PI * 0.5, 100, "gnss-clock", "host-clock", "gnss-antenna"));
    check(Math.abs(state.pose.x - 1.0) < 1e-4 && Math.abs(state.pose.y + 1.0) < 1e-4 &&
      Math.abs(state.pose.yaw - Math.PI * 0.5) < 1e-8 && state.referenceFrame == "map",
      "GNSS converts a geodetic fix to local ENU and removes the antenna lever arm");
    check(Math.abs(state.covariance.xx - 0.05) < 1e-8 &&
      Math.abs(state.covariance.xYaw - 0.01) < 1e-8 && state.sourceTimestampNs == Int64.ofInt(100),
      "GNSS propagates heading variance through the lever arm and keeps the fix time");
    check(gnss.update(fixSnapshot(2, 700000000, 0.0, oneMetreEast, Math.PI * 0.5, 100, "gnss-clock", "host-clock", "gnss-antenna")).quality ==
      LocalizationQuality.Invalid, "GNSS rejects a fix outside its age window");
    check(gnss.update(fixSnapshot(3, 100, 0.0, oneMetreEast, Math.PI * 0.5, 150, "gnss-clock", "host-clock", "gnss-antenna")).quality !=
      LocalizationQuality.Invalid, "GNSS accepts a fix newer than the robot state");
    check(gnss.update(fixSnapshot(4, 100, 0.0, oneMetreEast, Math.PI * 0.5, 900000000, "receiver-clock", "host-clock", "gnss-antenna")).quality == LocalizationQuality.Invalid,
      "GNSS ages a fix on its own receiver clock by the shared receive clock");
    check(gnss.update(fixSnapshot(5, 100, 0.0, oneMetreEast, Math.PI * 0.5, 100, "receiver-clock", "host-clock", "gnss-antenna")).quality != LocalizationQuality.Invalid,
      "GNSS accepts a receiver-clock fix received close to the robot state");
    check(gnss.update(fixSnapshot(6, 100, 0.0, oneMetreEast, Math.PI * 0.5, 100, "receiver-clock", "other-host", "gnss-antenna")).quality == LocalizationQuality.Invalid,
      "GNSS ignores a fix that shares no clock with the robot state");
    gnss.reset(new Pose2(4.0, 5.0, 0.25));
    var aligned = gnss.update(fixSnapshot(7, 200, 0.0, oneMetreEast, Math.PI * 0.5, 200, "gnss-clock", "host-clock", "gnss-antenna"));
    check(Math.abs(aligned.pose.x - 4.0) < 1e-8 && Math.abs(aligned.pose.y - 5.0) < 1e-8 &&
      Math.abs(aligned.pose.yaw - 0.25) < 1e-8,
      "GNSS reset establishes a new local alignment");

    // Model-driven: the antenna lever arm and frame come from the authored mount.
    var model = new RobotModel("gnss robot");
    var baseLink = model.addLink(new Link("base", "gnss/base"));
    var mast = model.addLink(new Link("mast", "gnss/mast"));
    var mastJoint = new Joint("mast joint", JointType.Fixed, baseLink, mast, "gnss/mast-joint");
    model.addJoint(mastJoint);
    var antenna = model.addFrame(new Frame("antenna", baseLink, "gnss-antenna"));
    antenna.position = [1.0, 0.0, 0.8];
    var sensor = model.addSensor(new Sensor("gnss", "gnss_pose", 10.0, "gnss-front"));
    sensor.frame = antenna;
    var authored = GnssPoseLocalization.fromRobotModel(model, "gnss-front", "gnss/base",
      "map", "base", 0.0, 0.0, 0.04, 0.01, 500000000.0);
    var authoredState = authored.update(fixSnapshot(1, 100, 0.0, oneMetreEast, Math.PI * 0.5, 100, "gnss-clock", "host-clock", "gnss-antenna"));
    check(Math.abs(authoredState.pose.x - 1.0) < 1e-4 && Math.abs(authoredState.pose.y + 1.0) < 1e-4,
      "model-driven GNSS takes its lever arm from the authored antenna frame");
    check(authored.update(fixSnapshot(2, 100, 0.0, oneMetreEast, Math.PI * 0.5, 100, "gnss-clock", "host-clock", "other-antenna")).quality == LocalizationQuality.Invalid,
      "model-driven GNSS rejects fixes from another sensor frame");
    var mastAntenna = model.addFrame(new Frame("mast antenna", mast, "mast-antenna"));
    var mastSensor = model.addSensor(new Sensor("mast gnss", "gnss_pose", 10.0, "mast-gnss"));
    mastSensor.frame = mastAntenna;
    throws(function() GnssPoseLocalization.fromRobotModel(model, "mast-gnss", "gnss/base",
      "map", "base", 0.0, 0.0), "must be mounted on body link");

    // End to end: a receiver publishes through the runtime, and the fix corrects fusion.
    var simulated = new RobotModel("gnss sim");
    var simBase = simulated.addLink(new Link("base", "sim/base"));
    var simAntenna = simulated.addFrame(new Frame("antenna", simBase, "sim/antenna"));
    simAntenna.position = [1.0, 0.0, 0.8];
    var simSensor = simulated.addSensor(new Sensor("gnss", "gnss_pose", 10.0, "sim/gnss"));
    simSensor.frame = simAntenna;
    var simulation = new Simulation();
    var runtime = simulation.addRobot(RobotRuntimeCompiler.compile(simulated));
    var robot = new SimulatedRobot("gnss-sim", runtime, simulated.name, [simBase.name], []);
    simulation.step(Int64.ofInt(1));
    simulation.step(Int64.ofInt(2));
    runtime.publishSensorFrame("sim/gnss", [0.0, oneMetreEast, Math.PI * 0.5], Int64.ofInt(1),
      Int64.ofInt(987654321), "gnss.receiver");
    var observed = robot.snapshot();
    var receiver = GnssPoseLocalization.fromRobotModel(simulated, "sim/gnss", "sim/base",
      "map", "base", 0.0, 0.0);
    var fix = receiver.update(observed);
    check(fix.quality != LocalizationQuality.Invalid && Math.abs(fix.pose.x - 1.0) < 1e-4 &&
      Math.abs(fix.pose.y + 1.0) < 1e-4,
      "a fix published through the runtime localizes the robot despite its own receiver clock");
    var odometry = new FixedLocalization(new LocalizationState(observed.sourceSequence,
      new Pose2(), "odom", "base", new PoseCovariance2(0.01, 0.0, 0.0, 0.01, 0.0, 0.01),
      LocalizationQuality.Good, observed.sourceTimestampNs, observed.receivedTimestampNs,
      observed.sourceClockId, observed.receivedClockId));
    var fusion = new PoseFusionLocalization(odometry, new FrameTree2(), "map", "base");
    fusion.update(observed);
    var fused = fusion.fuse(fix);
    check(fused.quality != LocalizationQuality.Invalid && Math.abs(fused.pose.x - 1.0) < 1e-3 &&
      Math.abs(fused.pose.y + 1.0) < 1e-3, "GNSS fixes anchor pose fusion in the map frame");
    robot.close();
    simulation.dispose();
  }

  static function testFiducialPerception():Void {
    var model = new RobotModel("fiducial robot");
    var body = model.addLink(new Link("base", "fiducial/base"));
    var mount = model.addFrame(new Frame("camera mount", body, "fiducial/camera"));
    mount.position = [0.5, 0.0, 1.0];
    var sensor = model.addSensor(new Sensor("front camera", "camera", 10.0,
      "fiducial/front"));
    sensor.frame = mount;
    var perception = FiducialPerception.fromRobotModel(model, "fiducial/front",
      "fiducial/base", "base", "fiducial/base", [
        new FiducialTargetConfig(11, "pallet", 1.2, 0.8, 0.15),
        new FiducialTargetConfig(22, "dock", 0.0, 0.0, 0.0,
          new Pose2(-0.5, 0.0))], 0.8);
    var detections = new SensorFrame("fiducial/front", "camera_detections",
      "fiducial/camera", Int64.ofInt(1), Int64.ofInt(100), [
        11.0, 1.0, 0.0, 0.0, 0.95, 22.0, 2.0, 0.0, 0.0, 0.98,
        99.0, 3.0, 0.0, 0.0, 0.99
      ], Int64.ofInt(110), "fiducial/base");
    var mapped = perception.observe([detections]);
    check(mapped.pallets().length == 1 && mapped.dockingTargets().length == 1 &&
      Math.abs(mapped.pallets()[0].detection.pose.x - 1.5) < 1e-8 &&
      Math.abs(mapped.dockingTargets()[0].approachPose.x - 2.0) < 1e-8 &&
      mapped.detections()[0].frameId == "fiducial/base",
      "fiducial IDs map to framed pallet and dock targets");
    throws(function() perception.observe([new SensorFrame("fiducial/front",
      "camera_detections", "fiducial/camera", Int64.ofInt(2), Int64.ofInt(120),
      [11.0, 1.0], Int64.ofInt(130))]),
      "fiducial records require complete values");
    var pixels = haxe.io.Bytes.alloc(3);
    pixels.set(0, 10); pixels.set(1, 20); pixels.set(2, 30);
    var camera = new SensorFrame("fiducial/front", "camera", "fiducial/camera",
      Int64.ofInt(3), Int64.ofInt(140), [], Int64.ofInt(150), "fiducial/base",
      null, null, "camera.clock", "host.clock",
      new CameraImage(1, 1, "rgb8", pixels));
    var typed = perception.observeCameraFrames([camera], new PortFiducialDetector([
      new FiducialMarkerObservation(11, new Pose2(1.0, 0.0), 0.95),
      new FiducialMarkerObservation(22, new Pose2(2.0, 0.0), 0.98)]));
    check(typed.pallets().length == 1 && typed.dockingTargets().length == 1,
      "injected image detectors publish typed fiducial targets");
    mount.rotation = [Math.sqrt(0.5), 0.0, 0.0, Math.sqrt(0.5)];
    var tilted = FiducialPerception.fromRobotModel(model, "fiducial/front",
      "fiducial/base", "base", "fiducial/base", [
        new FiducialTargetConfig(11, "pallet", 1.2, 0.8, 0.15)]);
    var tiltedResult = tilted.observeCameraFrames([camera], new PortFiducialDetector([
      FiducialMarkerObservation.fromPose3(11, new Pose3(1.0, 0.0, 0.0), 0.95)]));
    check(tiltedResult.pallets().length == 1 &&
      Math.abs(tiltedResult.pallets()[0].detection.pose.x - 1.5) < 1e-8 &&
      tiltedResult.pallets()[0].detection.frameId == "fiducial/base",
      "3D fiducial poses compose through a tilted camera mount");
  }

  static function testDepthObstaclePerception():Void {
    var cloudFrame = new SensorFrame("depth-front", "point_cloud", "depth-camera",
      Int64.ofInt(10), Int64.ofInt(150), [
        1.0, 0.0, 0.5, 1.04, 0.0, 0.5, 1.0, 0.04, 0.4,
        3.0, 0.0, 0.4, 3.04, 0.0, 0.4,
        0.5, 0.0, 3.0, 8.0, 0.0, 0.2
      ], Int64.ofInt(160), "depth-link", null, null, "robot-boot", "host-clock");
    var cloudObstacles = new PointCloudObstaclePerception(5.0, 0.15, 0.05,
      0.9, 0.12, 0.1, 1.0, 2, 100).observe([cloudFrame]).obstacles();
    check(cloudObstacles.length == 2 &&
      Math.abs(cloudObstacles[0].detection.pose.x - 1.0133333333) < 1e-6 &&
      Math.abs(cloudObstacles[0].detection.pose.y - 0.0133333333) < 1e-6 &&
      cloudObstacles[0].detection.frameId == "depth-camera",
      "point-cloud perception clusters XYZ returns in the source frame");
    throws(function() new PointCloudObstaclePerception(5.0, 0.15).observe([
      new SensorFrame("malformed-depth", "point_cloud", "camera",
        Int64.ofInt(1), Int64.ofInt(1), [1.0, 2.0], Int64.ofInt(1))]),
      "point-cloud perception rejects incomplete XYZ triples");

    var depthModel = new RobotModel("mounted depth camera");
    var depthBaseLink = depthModel.addLink(new Link("base", "base-link"));
    var opticalMount = new Frame("depth optical mount", depthBaseLink,
      "depth-optical");
    opticalMount.position = [0.5, 0.0, 1.2];
    opticalMount.rotation = [-0.5, 0.5, -0.5, 0.5];
    depthModel.addFrame(opticalMount);
    var depthSensor = new Sensor("front-depth", "camera");
    depthSensor.frame = opticalMount;
    depthModel.addSensor(depthSensor);
    var depthPerception = DepthCameraObstaclePerception.fromRobotModel(
      depthModel, "front-depth", "base-link", "base", "base-link",
      new PinholeCameraIntrinsics(100.0, 100.0, 1.0, 1.0),
      5.0, 0.1, 0.1, 0.8, 0.1, -0.25, 2.5, 3, 8);
    var depthPixels = haxe.io.Bytes.alloc(4 * 4 * 4);
    for (index in 0...16) depthPixels.setFloat(index * 4, 2.0);
    var depthFrame = new SensorFrame("front-depth", "camera", "depth-optical",
      Int64.ofInt(7), Int64.ofInt(250), [], Int64.ofInt(260), "base-link",
      null, null, "camera-clock", "host-clock",
      new CameraImage(4, 4, "depth32f", depthPixels));
    var depthObstacles = depthPerception.observe([depthFrame]).obstacles();
    check(depthObstacles.length == 1 &&
      Math.abs(depthObstacles[0].detection.pose.x - 2.5) < 1e-8 &&
      Math.abs(depthObstacles[0].detection.pose.y) < 1e-8 &&
      depthObstacles[0].detection.frameId == "base-link" &&
      depthObstacles[0].detection.sourceSequence == Int64.ofInt(7),
      "depth images unproject through the authored mount into body-frame obstacles");
  }

  static function testWheelImuLocalization():Void {
    var robot = new FakeRobot("wheel-imu");
    robot.positions = [0.0, 0.0];
    var base = new MobileBase(robot, new DifferentialDrive(0, 1, 0.1, 0.5),
      new MotionLimits(1.0, 2.0));
    var localization = new WheelImuLocalization(base, "imu", "odom", "base",
      1.0, 0.01, 200000000.0, "imu-frame");
    function observation(sequence:Int, timestamp:Int, left:Float, right:Float,
        gyro:Float, ?frameId:String = "imu-frame", ?sampleTimestamp:Int = -1,
        ?sourceClock:String = "clock", ?receiveClock:String = "host"):RobotSnapshot {
      if (sampleTimestamp < 0) sampleTimestamp = timestamp;
      var sensor = new SensorFrame("imu", "imu", frameId, Int64.ofInt(sequence),
        Int64.ofInt(sampleTimestamp), [0.0, 0.0, gyro, 0.0, 0.0, 9.81],
        Int64.ofInt(timestamp + 1), "base", null, null, sourceClock, receiveClock);
      return new RobotSnapshot("wheel-imu", Int64.ofInt(sequence), Int64.ofInt(timestamp),
        [left, right], [], [], 1, 0, Int64.ofInt(timestamp + 2), [sensor], "clock", "host");
    }
    localization.update(observation(1, 100, 0.0, 0.0, 0.0, "imu-frame", -1, "clock", "host"));
    var corrected = localization.update(observation(2, 100000100, 0.0, 1.0, 2.0, "imu-frame", -1, "clock", "host"));
    check(Math.abs(corrected.pose.yaw - 0.2) < 1e-8 &&
      Math.abs(corrected.pose.x - 0.05) < 0.01,
      "IMU gyro rate corrects wheel heading while wheel travel supplies distance");
    var fallback = localization.update(observation(3, 200000100, 0.0, 2.0, 9.0,
      "other-frame", -1, "clock", "host"));
    check(Math.abs(fallback.pose.yaw - 0.4) < 1e-8,
      "IMU localization falls back to wheel heading for another sensor frame");
    var receiver = new WheelImuLocalization(base, "imu", "odom", "base", 1.0);
    receiver.update(observation(1, 100, 0.0, 0.0, 0.0, "imu-frame", -1, "clock", "host"));
    var received = receiver.update(observation(2, 100000100, 0.0, 1.0, 2.0,
      "imu-frame", 100000100, "receiver", "host"));
    check(Math.abs(received.pose.yaw - 0.2) < 1e-8,
      "IMU samples on a receiver clock use the shared local receive clock");
    var model = new RobotModel("imu robot");
    var body = model.addLink(new Link("base", "imu/base"));
    var frame = model.addFrame(new Frame("imu mount", body, "imu-frame"));
    var sensor = model.addSensor(new Sensor("imu", "imu", 20.0, "imu"));
    sensor.frame = frame;
    var authored = WheelImuLocalization.fromRobotModel(base, model, "imu", "imu/base");
    authored.update(observation(1, 100, 0.0, 0.0, 0.0, "imu-frame", -1, "clock", "host"));
    check(Math.abs(authored.update(observation(2, 100000100, 0.0, 1.0, 4.0, "imu-frame", -1, "clock", "host")).pose.yaw - 0.3) < 1e-8,
      "model-driven IMU localization resolves its sensor frame and blends rate with wheels");
    frame.rotation = [Math.sqrt(0.5), 0.0, 0.0, Math.sqrt(0.5)];
    var tilted = WheelImuLocalization.fromRobotModel(base, model, "imu", "imu/base",
      "odom", "base", 1.0);
    tilted.update(observation(1, 100, 0.0, 0.0, 0.0, "imu-frame", -1, "clock", "host"));
    var tiltedSample = new SensorFrame("imu", "imu", "imu-frame", Int64.ofInt(2),
      Int64.ofInt(100000100), [0.0, 2.0, 0.0, 0.0, 0.0, 9.81],
      Int64.ofInt(100000101), "imu/base", null, null, "clock", "host");
    var tiltedSnapshot = new RobotSnapshot("wheel-imu", Int64.ofInt(2),
      Int64.ofInt(100000100), [0.0, 1.0], [], [], 1, 0,
      Int64.ofInt(100000102), [tiltedSample], "clock", "host");
    check(Math.abs(tilted.update(tiltedSnapshot).pose.yaw - 0.2) < 1e-8,
      "a tilted IMU mount rotates angular velocity into the body yaw axis");
  }

  static function testLocalization():Void {
    var robot = new FakeRobot("localized-base");
    robot.positions = [0.0, 0.0];
    var base = new MobileBase(robot, new DifferentialDrive(0, 1, 0.1, 0.5),
      new MotionLimits(1.0, 2.0));
    var localization = new WheelOdometryLocalization(base);
    function observation(sequence:Int, timestamp:Int, left:Float, right:Float,
        sourceClock:String):RobotSnapshot
      return new RobotSnapshot("localized-base", Int64.ofInt(sequence), Int64.ofInt(timestamp),
        [left, right], [], [], 1, 0, Int64.ofInt(timestamp + 10), [], sourceClock, "host-clock");

    var initial = localization.update(observation(1, 100, 0.0, 0.0, "boot-A"));
    equal(initial.referenceFrame, "odom", "wheel localization labels its reference frame");
    equal(initial.bodyFrame, "base", "wheel localization labels its body frame");
    equal(initial.sourceClockId, "boot-A", "localization preserves source clock identity");
    equal(initial.receivedClockId, "host-clock", "localization preserves receive clock identity");
    var moved = localization.update(observation(2, 200, 1.0, 1.0, "boot-A"));
    check(Math.abs(moved.pose.x - 0.1) < 1e-9 && moved.covariance.xx > 0.0 &&
      moved.covariance.yawYaw == 0.0,
      "wheel localization integrates pose and accumulates covariance");
    check(switch moved.quality {
      case Degraded: true;
      case _: false;
    }, "wheel odometry marks its estimate degraded");
    var afterReset = localization.update(observation(1, 1, 4.0, 4.0, "boot-B"));
    check(Math.abs(afterReset.pose.x - moved.pose.x) < 1e-9,
      "wheel localization avoids a pose jump across source clock changes");
    localization.update(observation(2, 2, 4.5, 4.5, "boot-B"));
    check(localization.state() != null, "localization retains its latest immutable state");
    localization.reset();
    equal(localization.state(), null, "localization reset clears its published state");
    throws(function() new PoseCovariance2(-1.0),
      "pose covariance rejects negative variances");

    var frames = new FrameTree2();
    frames.add(new FrameTransform2("map", "facility", new Pose2(5.0, 0.0, 0.0)));
    frames.add(new FrameTransform2("base", "imu", new Pose2(0.2, 0.0, 0.0)));
    check(Math.abs(frames.lookup("map", "facility").x - 5.0) < 1e-9 &&
      Math.abs(frames.lookup("imu", "base").x + 0.2) < 1e-9,
      "Frame tree lookups preserve target/source transform direction");
    var cyclicFrames = new FrameTree2();
    cyclicFrames.add(new FrameTransform2("map", "odom", new Pose2()));
    cyclicFrames.add(new FrameTransform2("odom", "base", new Pose2()));
    throws(function() cyclicFrames.add(new FrameTransform2("base", "map", new Pose2())),
      "Frame tree rejects cyclic parent relationships");

    var fusionRobot = new FakeRobot("fused-base");
    fusionRobot.positions = [0.0, 0.0];
    var fusionBase = new MobileBase(fusionRobot,
      new DifferentialDrive(0, 1, 0.1, 0.5), new MotionLimits(1.0, 2.0));
    var wheelSource = new WheelOdometryLocalization(fusionBase);
    var fusion = new PoseFusionLocalization(wheelSource, frames, "map", "base",
      new PoseFusionOptions(0.00000002, 0.00000005, 2.0, 0.5,
        0.25, 0.1, 0.05));
    var fusedInput = new RobotSnapshot("fused-base", Int64.ofInt(1), Int64.ofInt(100),
      [0.0, 0.0], [], [], 1, 0, Int64.ofInt(110), [], "fused-clock", "host-clock");
    var unanchored = fusion.update(fusedInput);
    check(switch unanchored.quality { case Invalid: true; case _: false; },
      "Pose fusion marks output invalid until frames are connected by an absolute pose");
    var externalPose = new LocalizationState(Int64.ofInt(1),
      new Pose2(0.3, 0.0, 0.0), "facility", "imu",
      new PoseCovariance2(0.04, 0.0, 0.0, 0.04, 0.0, 0.01), Good,
      Int64.ofInt(100), Int64.ofInt(110), "gps-clock", "host-clock");
    var fused = fusion.fuse(externalPose);
    check(fused.referenceFrame == "map" && fused.bodyFrame == "base" &&
      Math.abs(fused.pose.x - 5.1) < 1e-9 &&
      switch fused.quality { case Good: true; case _: false; },
      "Pose fusion transforms external sensor poses into the requested robot frame");
    var secondExternalPose = new LocalizationState(Int64.ofInt(2),
      new Pose2(0.5, 0.0, 0.0), "facility", "imu",
      new PoseCovariance2(0.04, 0.0, 0.0, 0.04, 0.0, 0.01), Good,
      Int64.ofInt(120), Int64.ofInt(130), "gps-clock", "host-clock");
    var smoothed = fusion.fuse(secondExternalPose);
    check(Math.abs(smoothed.pose.x - 5.2) < 1e-9 && smoothed.covariance.xx < 0.04,
      "Pose fusion weights repeated absolute observations by covariance");
    var fusedMoved = fusion.update(new RobotSnapshot("fused-base", Int64.ofInt(2),
      Int64.ofInt(200), [1.0, 1.0], [], [], 1, 0, Int64.ofInt(210), [],
      "fused-clock", "host-clock"));
    check(Math.abs(fusedMoved.pose.x - 5.3) < 1e-9,
      "Fused map pose continues to follow wheel odometry between absolute updates");
    check(switch fusedMoved.quality { case Degraded: true; case _: false; } &&
      fusedMoved.covariance.xx > smoothed.covariance.xx,
      "absolute pose timeout degrades quality and grows uncertainty");

    var outOfOrder = fusion.fuse(new LocalizationState(Int64.ofInt(1),
      new Pose2(50.0, 0.0, 0.0), "map", "base",
      new PoseCovariance2(0.01, 0.0, 0.0, 0.01, 0.0, 0.01), Good,
      Int64.ofInt(110), Int64.ofInt(210), "gps-clock", "host-clock"));
    check(Math.abs(outOfOrder.pose.x - fusedMoved.pose.x) < 1e-9,
      "pose fusion rejects duplicate or out-of-order absolute measurements");

    var impossibleJump = fusion.fuse(new LocalizationState(Int64.ofInt(3),
      new Pose2(25.0, 0.0, 0.0), "map", "base",
      new PoseCovariance2(0.01, 0.0, 0.0, 0.01, 0.0, 0.01), Good,
      Int64.ofInt(220), Int64.ofInt(210), "gps-clock", "host-clock"));
    check(Math.abs(impossibleJump.pose.x - fusedMoved.pose.x) < 1e-9 &&
      switch impossibleJump.quality { case Degraded: true; case _: false; },
      "innovation gating rejects a large localization jump and degrades quality");

    var recovered = fusion.fuse(new LocalizationState(Int64.ofInt(4),
      new Pose2(5.9, 0.0, 0.0), "map", "base",
      new PoseCovariance2(0.01, 0.0, 0.0, 0.01, 0.0, 0.01), Good,
      Int64.ofInt(230), Int64.ofInt(210), "gps-clock", "host-clock"));
    check(Math.abs(recovered.pose.x - impossibleJump.pose.x) <= 0.25 &&
      switch recovered.quality { case Good: true; case _: false; },
      "valid absolute localization recovers through bounded pose corrections");

    var stale = fusion.fuse(new LocalizationState(Int64.ofInt(5),
      new Pose2(6.0, 0.0, 0.0), "map", "base",
      new PoseCovariance2(0.01, 0.0, 0.0, 0.01, 0.0, 0.01), Good,
      Int64.ofInt(240), Int64.ofInt(0), "gps-clock", "host-clock"));
    check(Math.abs(stale.pose.x - recovered.pose.x) < 1e-9 &&
      switch stale.quality { case Good: true; case _: false; },
      "stale external localization is ignored without disturbing a fresh estimate");

    var blueprint = new RobotRuntimeBlueprint(1, 0, 1);
    var simulation = new Simulation();
    simulation.addRobot(blueprint);
    var yaw = Math.PI * 0.5;
    simulation.teleportRobot(0, [2.0, 3.0, 0.0],
      [0.0, 0.0, Math.sin(yaw * 0.5), Math.cos(yaw * 0.5)]);
    var truth = new SimulationTruthLocalization(simulation, 0);
    var source = new RobotSnapshot("truth", Int64.ofInt(5), Int64.ofInt(500),
      [], [], [], 1, 0, Int64.ofInt(700), [], "simulation-clock", "host-clock");
    var trueState = truth.update(source);
    check(Math.abs(trueState.pose.x - 2.0) < 1e-9 &&
      Math.abs(trueState.pose.y - 3.0) < 1e-9 &&
      Math.abs(trueState.pose.yaw - yaw) < 1e-8,
      "simulation truth localization projects a 3D simulation pose into Pose2");
    equal(trueState.referenceFrame, "map", "simulation truth uses an explicit map frame");
    check(switch trueState.quality {
      case Good: true;
      case _: false;
    } && trueState.covariance.xx == 0.0,
      "simulation truth publishes exact quality and zero truth covariance");
    throws(function() truth.reset(new Pose2(9.0, 9.0, 0.0)),
      "simulation truth cannot be reset to a fabricated pose");
    simulation.dispose();
  }

  static function testNavigation():Void {
    var points = [new Pose2(0.0, 0.0, 0.0), new Pose2(1.0, 0.0, 0.0)];
    var path = new Path(points, "odom");
    points[1] = new Pose2(9.0, 9.0, 1.0);
    check(Math.abs(path.length - 1.0) < 1e-9 &&
      Math.abs(path.poseAt(0.5).x - 0.5) < 1e-9,
      "Path owns waypoints and interpolates by arc length");
    equal(path.count(), 2, "Path exposes a stable waypoint count");
    throws(function() new Path([new Pose2(), new Pose2()], ""),
      "Path requires an explicit frame ID");

    var trajectory = new Trajectory([
      new TrajectorySample(0.0, new Pose2(), new Twist2()),
      new TrajectorySample(2.0, new Pose2(2.0, 0.0, 1.0), new Twist2(1.0, 0.5))
    ], "odom");
    var midway = trajectory.sampleAt(1.0);
    check(Math.abs(trajectory.durationSeconds - 2.0) < 1e-9 &&
      Math.abs(midway.pose.x - 1.0) < 1e-9 &&
      Math.abs(midway.pose.yaw - 0.5) < 1e-9 &&
      Math.abs(midway.twist.angular - 0.25) < 1e-9,
      "Trajectory interpolates pose and velocity samples");
    throws(function() new Trajectory([
      new TrajectorySample(0.0, new Pose2(), Twist2.zero()),
      new TrajectorySample(0.0, new Pose2(), Twist2.zero())
    ]), "Trajectory requires strictly increasing sample times");

    var profileRobot = new FakeRobot("trajectory-profile");
    profileRobot.positions = [0.0, 0.0];
    var profileBase = new MobileBase(profileRobot,
      new DifferentialDrive(0, 1, 0.1, 0.5), new MotionLimits(1.0, 1.0, 0.8, 0.5));
    var profilePath = new Path([new Pose2(), new Pose2(2.0, 0.0, 0.0)], "odom");
    var profiled = Trajectory.fromPath(profilePath, profileBase, 1.0, 0.5, 0.1);
    var profiledSamples = profiled.samples();
    var respectsLimits = profiledSamples.length > 10 &&
      profiledSamples[0].twist.linear == 0.0 &&
      profiledSamples[profiledSamples.length - 1].twist.linear == 0.0 &&
      Math.abs(profiled.goal().x - 2.0) < 1e-9;
    for (index in 0...profiledSamples.length - 1) {
      var from = profiledSamples[index];
      var to = profiledSamples[index + 1];
      var interval = to.timeFromStartSeconds - from.timeFromStartSeconds;
      var acceleration = Math.abs(to.twist.linear - from.twist.linear) / interval;
      respectsLimits = respectsLimits && Math.abs(to.twist.linear) <= 1.0 + 1e-9 &&
        acceleration <= 0.801;
    }
    check(respectsLimits,
      "Trajectory parameterization preserves the path and profiles speed under acceleration limits");
    var arcPoses:Array<Pose2> = [];
    for (index in 0...9) {
      var angle = index * 0.125;
      arcPoses.push(new Pose2(2.0 * Math.sin(angle),
        2.0 * (1.0 - Math.cos(angle)), angle));
    }
    var arcTrajectory = Trajectory.fromPath(new Path(arcPoses, "odom"),
      profileBase, 1.0, 0.125, 0.1);
    var maxArcSpeed = 0.0;
    for (sample in arcTrajectory.samples())
      maxArcSpeed = Math.max(maxArcSpeed, Math.abs(sample.twist.linear));
    check(maxArcSpeed <= 0.51 && maxArcSpeed > 0.0,
      "Trajectory speed obeys lateral acceleration through a curved path");
    var trajectoryLocalization = new WheelOdometryLocalization(profileBase);
    trajectoryLocalization.reset(new Pose2());
    var trajectoryNavigation = new Navigation(profileBase, trajectoryLocalization,
      0.2, 1.0, 1.0);
    trajectoryNavigation.followTrajectory(profiled);
    var trajectoryStatus = trajectoryNavigation.updateObservation(new RobotSnapshot(
      "trajectory-profile", Int64.ofInt(1), Int64.ofInt(1), [0.0, 0.0], [], [],
      1, 0, Int64.ofInt(2), [], "trajectory-clock", "host"), 0.1);
    check(switch trajectoryStatus { case Following: true; case _: false; } &&
      switch profileRobot.lastCommand {
        case JointTargets(targets, _): targets.length == 2 && targets[0].target > 0.0 &&
          targets[1].target > 0.0;
        case _: false;
      }, "Navigation tracks a time-parameterized trajectory through MobileBase");

    var reverseTrajectory = Trajectory.fromPath(new Path([
      new Pose2(), new Pose2(-1.0, 0.0, 0.0)
    ], "odom"), profileBase, 0.5, 0.5, 0.1);
    check(reverseTrajectory.sampleAt(reverseTrajectory.durationSeconds * 0.5).twist.linear < 0.0,
      "Trajectory parameterization preserves reverse body velocity");
    var reversingTrajectory = Trajectory.fromPath(new Path([
      new Pose2(), new Pose2(-0.4, 0.0, 0.0), new Pose2(0.2, 0.0, 0.0)
    ], "odom"), profileBase, 0.5, 0.5, 0.1);
    var reversedBeforeCusp = false;
    var stoppedAtCusp = false;
    var forwardAfterCusp = false;
    var reversingSamples = reversingTrajectory.samples();
    for (sample in reversingSamples) {
      if (sample.pose.x < -0.05 && sample.pose.x > -0.4 && sample.twist.linear < 0.0)
        reversedBeforeCusp = true;
      if (Math.abs(sample.pose.x + 0.4) < 1e-9 && Math.abs(sample.twist.linear) < 1e-9)
        stoppedAtCusp = true;
      if (sample.pose.x > -0.35 && sample.twist.linear > 0.0)
        forwardAfterCusp = true;
    }
    check(reversedBeforeCusp && stoppedAtCusp && forwardAfterCusp,
      "Trajectory stops at a gear-change cusp and resumes with the new travel direction");
    var shortTrajectory = Trajectory.fromPath(new Path([
      new Pose2(), new Pose2(0.04, 0.0, 0.0)
    ], "odom"), profileBase, 0.5, 0.5, 0.1);
    check(shortTrajectory.count() >= 3 && shortTrajectory.durationSeconds > 0.0,
      "Trajectory profiles short paths with an interior acceleration sample");
    var ackermannProfileBase = new MobileBase(new FakeRobot("ackermann-profile"),
      new AckermannDrive(0, 1, 1.0, 0.1, 0.2), new MotionLimits(1.0, 1.0));
    throws(function() Trajectory.fromPath(new Path([
      new Pose2(), new Pose2(1.0, 0.0, 1.0)
    ], "odom"), ackermannProfileBase, 0.5, 0.5, 0.1),
      "Trajectory rejects curvature beyond the authored Ackermann steering range");
    throws(function() Trajectory.fromPath(new Path([
      new Pose2(), new Pose2(0.0, 0.0, 1.0), new Pose2(1.0, 0.0, 1.0)
    ], "odom"), profileBase),
      "Trajectory rejects zero-distance pose changes that require an in-place rotation");

    var cornerPath = new Path([
      new Pose2(0.0, 0.0, 0.0),
      new Pose2(1.0, 0.0, 0.0),
      new Pose2(1.0, 1.0, Math.PI * 0.5)
    ], "map");
    var firstProjection = cornerPath.project(new Pose2(0.6, 0.2, 0.0), 0.0);
    check(Math.abs(firstProjection.distanceAlongPath - 0.6) < 1e-9 &&
      Math.abs(firstProjection.crossTrackError - 0.2) < 1e-9 &&
      Math.abs(firstProjection.distanceToPath - 0.2) < 1e-9,
      "Path projection returns arc length and signed cross-track error");
    var forwardProjection = cornerPath.project(new Pose2(0.2, 0.3, 0.0), 1.2);
    check(forwardProjection.distanceAlongPath >= 1.2 &&
      Math.abs(forwardProjection.crossTrackError - 0.8) < 1e-9,
      "Path projection stays monotonic and signs cross-track relative to the active segment");
    var loopPath = new Path([
      new Pose2(0.0, 0.0), new Pose2(1.0, 0.0), new Pose2(1.0, 1.0),
      new Pose2(0.0, 1.0), new Pose2(0.0, 0.05)
    ], "map");
    var nearLoopStart = new Pose2(0.0, 0.04, 0.0);
    var globalLoopProjection = loopPath.project(nearLoopStart, 0.0);
    check(globalLoopProjection.distanceAlongPath > 3.9,
      "unbounded path projection exposes a close future branch at a loop");
    var boundedLoopProjection = loopPath.project(nearLoopStart, 0.0, 0.2);
    check(boundedLoopProjection.distanceAlongPath <= 0.2 &&
      boundedLoopProjection.segmentIndex == 0,
      "bounded path projection stays on the nearby current branch");
    throws(function() loopPath.project(nearLoopStart, 0.5, 0.4),
      "bounded path projection rejects an upper bound before its minimum");

    var loopRobot = new FakeRobot("nav-loop");
    loopRobot.positions = [0.0, 0.0];
    var loopBase = new MobileBase(loopRobot,
      new DifferentialDrive(0, 1, 0.1, 0.5), new MotionLimits(1.0, 2.0));
    var loopLocalization = new WheelOdometryLocalization(loopBase);
    loopLocalization.reset(nearLoopStart);
    var loopNavigation = new Navigation(loopBase, loopLocalization);
    var loopGoal = new NavigationGoal(loopPath.goal(), "map", 0.001, 0.1);
    loopNavigation.follow(loopPath, loopGoal);
    loopNavigation.updateObservation(new RobotSnapshot("nav-loop", Int64.ofInt(1),
      Int64.ofInt(1), [0.0, 0.0], [], [], 1, 0, Int64.ofInt(2), [],
      "loop-clock", "host"), 0.1);
    check(loopNavigation.progressDistance <= 0.05,
      "Navigation does not jump to a later loop branch from a small localization offset");
    var resumedNavigation = new Navigation(loopBase, loopLocalization);
    resumedNavigation.follow(loopPath, loopGoal, null, 2.0);
    equal(resumedNavigation.progressDistance, 2.0,
      "Navigation accepts an explicit path distance when resuming mid-route");
    throws(function() resumedNavigation.follow(loopPath, loopGoal, null,
      loopPath.length + 0.1), "Navigation rejects a resume distance outside its path");

    var robot = new FakeRobot("nav-base");
    robot.positions = [0.0, 0.0];
    var base = new MobileBase(robot, new DifferentialDrive(0, 1, 0.1, 0.5),
      new MotionLimits(1.0, 2.0));
    var localization = new WheelOdometryLocalization(base);
    var navigation = new Navigation(base, localization, 0.25, 0.4, 0.8);
    var goal = new NavigationGoal(path.goal(), "odom", 0.15, 0.1);
    navigation.follow(path, goal);
    function navSnapshot(sequence:Int, time:Int, left:Float, right:Float):RobotSnapshot
      return new RobotSnapshot("nav-base", Int64.ofInt(sequence), Int64.ofInt(time),
        [left, right], [], [], 1, 0, Int64.ofInt(time + 5), [], "nav-boot", "host");
    var firstStatus = navigation.updateObservation(navSnapshot(1, 10, 0.0, 0.0), 0.1);
    check(switch firstStatus {
      case Following: true;
      case _: false;
    }, "Navigation starts and updates a path-following request");
    check(switch robot.lastCommand {
      case JointTargets(targets, _):
        targets.length == 2 && targets[0].mode == robotkit.world.JointTargetMode.Velocity &&
          targets[1].mode == robotkit.world.JointTargetMode.Velocity &&
          Math.abs(targets[0].target - targets[1].target) < 1e-9;
      case _: false;
    }, "path follower commands both differential wheels through MobileBase");
    var finalStatus = navigation.updateObservation(navSnapshot(2, 20, 9.0, 9.0), 0.1);
    check(switch finalStatus {
      case Succeeded: true;
      case _: false;
    }, "Navigation succeeds when localized pose enters the goal tolerances");
    check(switch robot.lastStop {
      case Normal: true;
      case _: false;
    }, "Navigation stops the robot on successful arrival");

    localization.reset(new Pose2(0.0, 0.1, 0.0));
    navigation.follow(path);
    var offPathStatus = navigation.updateObservation(navSnapshot(3, 30, 0.0, 0.0), 0.1);
    check(switch offPathStatus { case Following: true; case _: false; } &&
      Math.abs(navigation.crossTrackError - 0.1) < 1e-9,
      "Navigation exposes left-positive cross-track error from localization");

    localization.reset(new Pose2(0.0, 0.0, 0.0));
    var reversePath = new Path([
      new Pose2(0.0, 0.0, 0.0), new Pose2(-1.0, 0.0, 0.0)
    ], "odom");
    navigation.follow(reversePath);
    var reverseStatus = navigation.updateObservation(navSnapshot(4, 40, 0.0, 0.0), 0.1);
    check(switch reverseStatus { case Following: true; case _: false; } &&
      switch robot.lastCommand {
        case JointTargets(targets, _): targets.length == 2 &&
          targets[0].target < 0.0 && targets[1].target < 0.0;
        case _: false;
      }, "Navigation reverses along a path whose authored body heading faces opposite its tangent");

    var overshootRobot = new FakeRobot("nav-overshoot");
    overshootRobot.positions = [0.0, 0.0];
    var overshootBase = new MobileBase(overshootRobot,
      new DifferentialDrive(0, 1, 0.1, 0.5), new MotionLimits(1.0, 2.0));
    var overshootLocalization = new WheelOdometryLocalization(overshootBase);
    overshootLocalization.update(new RobotSnapshot("nav-overshoot", Int64.ofInt(1),
      Int64.ofInt(10), [0.0, 0.0], [], [], 1, 0, Int64.ofInt(11), [],
      "overshoot-clock", "host"));
    var overshootNavigation = new Navigation(overshootBase, overshootLocalization,
      0.2, 0.5, 1.0);
    var straightGoal = new NavigationGoal(new Pose2(1.0, 0.0, 0.0), "odom", 0.01, 0.1);
    overshootNavigation.follow(new Path([new Pose2(), straightGoal.pose], "odom"), straightGoal);
    overshootNavigation.updateObservation(new RobotSnapshot("nav-overshoot",
      Int64.ofInt(2), Int64.ofInt(20), [0.0, 0.0], [], [], 1, 0,
      Int64.ofInt(21), [], "overshoot-clock", "host"), 0.1);
    var overshootStatus = overshootNavigation.updateObservation(new RobotSnapshot(
      "nav-overshoot", Int64.ofInt(3), Int64.ofInt(30), [12.0, 12.0], [], [],
      1, 0, Int64.ofInt(31), [], "overshoot-clock", "host"), 0.1);
    check(switch overshootStatus { case Following: true; case _: false; } &&
      switch overshootRobot.lastCommand {
        case JointTargets(targets, _): targets.length == 2 &&
          targets[0].target < 0.0 && targets[1].target < 0.0;
        case _: false;
      }, "Navigation reverses toward the goal if motion carries the robot past the path end");

    var noReverseNavigation = new Navigation(overshootBase, overshootLocalization,
      0.2, 0.5, 1.0, false);
    noReverseNavigation.follow(new Path([new Pose2(), straightGoal.pose], "odom"), straightGoal);
    check(switch noReverseNavigation.update(0.1) {
      case Failed(message): message.indexOf("reverse motion is disabled") >= 0;
      case _: false;
    }, "Navigation reports an unrecoverable path-end overshoot when reverse is disabled");

    function commandedSpeedAt(x:Float, id:String):Float {
      var speedRobot = new FakeRobot(id);
      speedRobot.positions = [0.0, 0.0];
      var speedBase = new MobileBase(speedRobot,
        new DifferentialDrive(0, 1, 0.1, 0.5), new MotionLimits(1.0, 2.0, 2.0, 10.0));
      var speedLocalization = new WheelOdometryLocalization(speedBase);
      speedLocalization.reset(new Pose2(x, 0.0, 0.0));
      var speedNavigation = new Navigation(speedBase, speedLocalization,
        0.2, 0.5, 1.0);
      speedNavigation.follow(path, new NavigationGoal(path.goal(), "odom", 0.005, 0.1));
      speedNavigation.updateObservation(new RobotSnapshot(id, Int64.ofInt(1), Int64.ofInt(1),
        [0.0, 0.0], [], [], 1, 0, Int64.ofInt(2), [], "speed-clock", "host"), 1.0);
      return switch speedRobot.lastCommand {
        case JointTargets(targets, _): (targets[0].target + targets[1].target) * 0.05;
        case _: 0.0;
      };
    }
    var farSpeed = commandedSpeedAt(0.0, "nav-speed-far");
    var nearSpeed = commandedSpeedAt(0.98, "nav-speed-near");
    check(farSpeed > nearSpeed && nearSpeed > 0.0,
      "Navigation profiles speed down using braking distance near the goal");

    var zoneRobot = new FakeRobot("nav-speed-zone");
    zoneRobot.positions = [0.0, 0.0];
    var zoneBase = new MobileBase(zoneRobot, new DifferentialDrive(0, 1, 0.1, 0.5),
      new MotionLimits(1.0, 2.0, 1.0, 10.0));
    zoneBase.command(new Twist2(0.6, 0.0));
    var zoneLocalization = new WheelOdometryLocalization(zoneBase);
    zoneLocalization.reset(new Pose2());
    var zoneNavigation = new Navigation(zoneBase, zoneLocalization,
      0.2, 0.8, 1.0);
    var zonePath = new Path([new Pose2(), new Pose2(2.0, 0.0, 0.0)], "odom");
    zoneNavigation.follow(zonePath, null, [new PathSpeedLimit(0.01, 2.0, 0.05)]);
    zoneNavigation.updateObservation(new RobotSnapshot("nav-speed-zone",
      Int64.ofInt(1), Int64.ofInt(1), [0.0, 0.0], [], [], 1, 0,
      Int64.ofInt(2), [], "speed-zone-clock", "host"), 1.0);
    var zoneSpeed = switch zoneRobot.lastCommand {
      case JointTargets(targets, _): (targets[0].target + targets[1].target) * 0.05;
      case _: 0.0;
    };
    check(zoneSpeed > 0.14 && zoneSpeed <= 0.151,
      "Navigation brakes before an upcoming lane speed limit");
    throws(function() zoneNavigation.follow(zonePath, null,
      [new PathSpeedLimit(1.0, 2.1, 0.2)]),
      "Navigation rejects speed-limit intervals beyond the path");

    navigation.follow(path);
    navigation.cancel();
    check(switch navigation.status {
      case Cancelled: true;
      case _: false;
    }, "Navigation supports cancellation");
    navigation.follow(new Path([new Pose2(), new Pose2(1.0, 0.0, 0.0)], "map"));
    var mismatch = navigation.update(0.1);
    check(switch mismatch {
      case Failed(_): true;
      case _: false;
    }, "Navigation fails explicitly when path and localization frames differ");
  }

  static function testMotionGuard():Void {
    var model = new RobotModel("motion-guard-sim");
    var baseLink = model.addLink(new Link("base", "base"));
    var leftLink = model.addLink(new Link("left wheel", "left-wheel"));
    var rightLink = model.addLink(new Link("right wheel", "right-wheel"));
    var left = new Joint("left wheel", JointType.Continuous, baseLink, leftLink,
      "joint/left-wheel");
    left.limits = new JointLimits(-100.0, 100.0, 20.0, 100.0);
    model.addJoint(left);
    var right = new Joint("right wheel", JointType.Continuous, baseLink, rightLink,
      "joint/right-wheel");
    right.limits = new JointLimits(-100.0, 100.0, 20.0, 100.0);
    model.addJoint(right);
    model.mobileBase = new RobotMobileConfiguration(
      RobotDriveConfiguration.Differential("joint/left-wheel", "joint/right-wheel",
        0.1, 0.5), 0.6, 1.0, 2.0, 4.0, 0.6, 0.4);
    var lidar = model.addSensor(new Sensor("front lidar", "lidar", 0.0,
      "sensor/front-lidar"));
    lidar.rayCount = 64;
    lidar.maxRange = 5.0;

    var blueprint = RobotRuntimeCompiler.compile(model);
    var simulation = new Simulation(0.01);
    var runtime = simulation.addRobot(blueprint);
    var robot = new SimulatedRobot("motion-guard-sim", runtime, model.name,
      [for (link in model.links) link.name], [for (joint in model.joints) joint.name]);
    var base = MobileBase.fromBlueprint(robot, blueprint);
    var localization = new FixedLocalization(new LocalizationState(Int64.ofInt(0),
      new Pose2(), "map", "base", PoseCovariance2.zero(), Good,
      Int64.ofInt(0), Int64.ofInt(0), "sim-clock", "host-clock"));
    var lidarPerception = LidarObstaclePerception.fromBlueprint(blueprint,
      "sensor/front-lidar", 0.08, 0.05, 0.4);
    var framedPerception = new FrameAwarePerception(lidarPerception, localization);
    var navigation = new Navigation(base, localization, 0.2, 0.6, 1.0);
    var guard = new MotionGuard(navigation, null, 0.2, 2.0, 0.15, 0.5);
    var path = new Path([new Pose2(), new Pose2(5.0, 0.0, 0.0)], "map");
    navigation.follow(path);

    function observe(tick:Int):PerceptionSnapshot {
      simulation.step(Int64.ofInt(tick));
      var snapshot = robot.snapshot();
      return framedPerception.observe(snapshot.sensors.toArray());
    }
    var clear = observe(1);
    equal(clear.obstacles().length, 0, "simulated LiDAR reports a clear path initially");
    guard.update(clear, 1.0);
    check(switch guard.state { case Clear: true; case _: false; } &&
      base.currentCommand().linear > 0.5,
      "MotionGuard leaves navigation speed unchanged when the path is clear");

    var objectId = simulation.spawnBox([1.0, 0.0, 0.0], [0.1, 0.1, 0.1]);
    var detected = observe(2);
    check(detected.obstacles().length > 0,
      "simulated LiDAR perception detects an obstacle in the navigation corridor");
    guard.update(detected, 0.1);
    var approachSpeed = base.currentCommand().linear;
    check(switch guard.state { case Approaching(_, _, _): true; case _: false; } &&
      approachSpeed > 0.0 && approachSpeed < 0.6,
      "MotionGuard reduces navigation speed inside the stopping envelope");

    simulation.teleportObject(objectId, [0.65, 0.0, 0.0]);
    var blocked = observe(3);
    check(blocked.obstacles().length > 0,
      "simulated perception continues observing an obstacle near the footprint");
    guard.update(blocked, 0.1);
    guard.update(blocked, 0.1);
    check(switch guard.state { case Blocked(_): true; case _: false; } &&
      Math.abs(base.currentCommand().linear) < 1e-9,
      "MotionGuard commands a stop when obstacle clearance reaches its margin");

    simulation.removeObject(objectId);
    var resumed = observe(4);
    equal(resumed.obstacles().length, 0, "simulated LiDAR clears the removed obstacle");
    guard.update(resumed, 0.1);
    check(switch guard.state { case Clear: true; case _: false; } &&
      base.currentCommand().linear > 0.0,
      "MotionGuard lets navigation resume after the obstacle is removed");
    var unframedObstacle = new Obstacle(new Detection("unframed-obstacle", "obstacle",
      1.0, new Pose2(1.0, 0.0), "camera-frame", Int64.ofInt(1), Int64.ofInt(10),
      Int64.ofInt(10), "sim-clock", "host-clock"), 0.1);
    guard.update(new PerceptionSnapshot([], [unframedObstacle]), 0.1);
    check(switch guard.state { case Blocked(_): true; case _: false; },
      "MotionGuard blocks when obstacle frame transforms are unavailable");
    guard.detach();
    robot.close();
    simulation.dispose();
  }

  static function testGridPlanning():Void {
    var rotated = new OccupancyGrid2(0.5, new Pose2(10.0, 20.0, Math.PI * 0.5),
      4, 3, "rotated-map", OccupancyCell.Free);
    var worldCenter = rotated.cellCenter(2, 1);
    var roundTrip = rotated.worldToCell(worldCenter);
    check(roundTrip != null && roundTrip.x == 2 && roundTrip.y == 1,
      "occupancy grid maps cell centers through rotated origins");
    check(rotated.worldToCell(rotated.origin.compose(new Pose2(-0.01, 0.0))) == null,
      "occupancy grid rejects poses outside its lower-left boundary");

    var grid = new OccupancyGrid2(1.0, new Pose2(), 7, 7, "map", OccupancyCell.Free);
    for (y in 0...5) grid.setCell(3, y, OccupancyCell.Occupied);
    var costmap = new Costmap2(grid, 0.2, true, 0.5, 2.0);
    check(!costmap.isTraversable(2, 2),
      "costmap inflates static obstacles by the robot footprint");
    check(costmap.isTraversable(3, 6),
      "costmap preserves clearance beyond the inflated obstacle boundary");
    var planner = new AStarPlanner(costmap);
    var start = grid.cellCenter(0, 3);
    var goalCenter = grid.cellCenter(6, 3);
    var goal = new Pose2(goalCenter.x, goalCenter.y, 0.4);
    var path = planner.plan(start, goal);
    equal(path.frameId, "map", "grid planner frames its path in the map frame");
    equal(path.goal().yaw, goal.yaw,
      "grid planner preserves the requested final heading");
    var visitsGap = false;
    var pathSignature:Array<String> = [];
    for (point in path.poses()) {
      var mapped = grid.worldToCell(point);
      check(mapped != null,
        "A* path waypoints remain inside the occupancy grid");
      var cell:GridCell2 = cast mapped;
      check(costmap.isTraversable(cell.x, cell.y),
        "A* path waypoints remain in traversable costmap cells");
      if (cell.y >= 5) visitsGap = true;
      pathSignature.push('${cell.x},${cell.y}');
    }
    check(visitsGap, "A* routes around a wall through its only open gap");
    var repeatedPath = planner.plan(start, goal);
    var repeatedSignature:Array<String> = [];
    for (point in repeatedPath.poses()) {
      var cell:GridCell2 = cast grid.worldToCell(point);
      repeatedSignature.push('${cell.x},${cell.y}');
    }
    equal(repeatedSignature.join(";"), pathSignature.join(";"),
      "A* uses deterministic tie-breaking for repeated plans");

    var unknownGrid = new OccupancyGrid2(1.0, new Pose2(), 5, 1, "map", OccupancyCell.Free);
    unknownGrid.setCell(2, 0, OccupancyCell.Unknown);
    var unknownCostmap = new Costmap2(unknownGrid, 0.0, true, 0.0, 0.0);
    var unknownPlanner = new AStarPlanner(unknownCostmap);
    throws(function() unknownPlanner.plan(unknownGrid.cellCenter(0, 0),
      unknownGrid.cellCenter(4, 0)),
      "A* treats unknown cells as blocked by default");

    var dynamicGrid = new OccupancyGrid2(1.0, new Pose2(), 5, 5, "map",
      OccupancyCell.Free);
    var dynamicCostmap = new Costmap2(dynamicGrid, 0.0, false, 1.0, 2.0);
    var dynamicObstacle = new Obstacle(new Detection("dynamic-obstacle", "obstacle",
      1.0, new Pose2(2.5, 2.5), "map", Int64.ofInt(1), Int64.ofInt(1),
      Int64.ofInt(1), "sim-clock", "host-clock"), 0.1);
    dynamicCostmap.setDynamicObstacles([dynamicObstacle]);
    check(!dynamicCostmap.isTraversable(2, 2),
      "costmap rasterizes dynamic perception obstacles");
    check(dynamicCostmap.isTraversable(3, 2) && dynamicCostmap.cellCost(3, 2) > 0.0,
      "costmap assigns soft costs around dynamic obstacles");
    dynamicCostmap.clearDynamicObstacles();
    check(dynamicCostmap.isTraversable(2, 2) && dynamicCostmap.cellCost(3, 2) == 0.0,
      "costmap clears removed dynamic obstacles");

    // A 0.1 m obstacle and a 0.3 m robot: a cell is lethal within 0.4 m of the
    // obstacle and blocked within 0.4 m plus half a 0.2 m cell diagonal.
    var marginGrid = new OccupancyGrid2(0.2, new Pose2(), 25, 25, "map", OccupancyCell.Free);
    var marginCostmap = new Costmap2(marginGrid, 0.3, false, 0.3, 2.0);
    marginCostmap.setDynamicObstacles([new Obstacle(new Detection("margin-obstacle",
      "obstacle", 1.0, new Pose2(2.0, 2.0), "map", Int64.ofInt(1), Int64.ofInt(1),
      Int64.ofInt(1), "sim-clock", "host-clock"), 0.1)]);
    check(!marginCostmap.isTraversable(12, 10) && !marginCostmap.isLethal(12, 10) &&
      marginCostmap.isLethal(10, 10) && marginCostmap.isLethal(-1, 0),
      "costmap separates lethal cells from its blocked discretization margin");
    var marginPlanner = new AStarPlanner(marginCostmap);
    var marginGoal = new Pose2(4.3, 3.9);
    var escapePath = marginPlanner.plan(new Pose2(2.47, 2.0), marginGoal);
    var reachedFree = false;
    var escapeIsClean = true;
    var escapePoses = escapePath.poses();
    for (index in 1...escapePoses.length) {
      var cell:GridCell2 = cast marginGrid.worldToCell(escapePoses[index]);
      if (marginCostmap.isLethal(cell.x, cell.y)) escapeIsClean = false;
      if (marginCostmap.isTraversable(cell.x, cell.y)) reachedFree = true;
      else if (reachedFree) escapeIsClean = false;
    }
    check(reachedFree && escapeIsClean,
      "A* escapes a start in the blocked margin without lethal cells or re-entry");
    throws(function() marginPlanner.plan(new Pose2(2.1, 2.0), marginGoal),
      "A* finds no route out of a start surrounded by lethal cells");
    throws(function() marginPlanner.plan(marginGoal, new Pose2(2.47, 2.0)),
      "A* still rejects a goal in a blocked cell");
  }

  static function requireLocalizationState(localization:Localization):LocalizationState {
    var estimate = localization.state();
    if (estimate == null) throw "Simulation scenario has no localization state";
    return estimate;
  }

  static function testNavigator():Void {
    var recoveryRobot = new FakeRobot("navigator-recovery");
    recoveryRobot.positions = [0.0, 0.0];
    var recoveryBase = new MobileBase(recoveryRobot,
      new DifferentialDrive(0, 1, 0.1, 0.5), new MotionLimits(1.0, 1.0));
    var recoveryLocalization = new FixedLocalization(new LocalizationState(
      Int64.ofInt(0), new Pose2(0.5, 0.5), "map", "base",
      PoseCovariance2.zero(), Good, Int64.ofInt(0), Int64.ofInt(0),
      "recovery-clock", "host-clock"));
    var recoveryNavigation = new Navigation(recoveryBase, recoveryLocalization);
    var recoveryGrid = new OccupancyGrid2(1.0, new Pose2(), 7, 1,
      "map", OccupancyCell.Free);
    var recoveryCostmap = new Costmap2(recoveryGrid, 0.0, false, 0.0, 0.0);
    var recoveryNavigator = new Navigator(recoveryNavigation,
      new AStarPlanner(recoveryCostmap), recoveryCostmap, 0.1);
    var recoveryGoal = new NavigationGoal(recoveryGrid.cellCenter(6, 0), "map");
    recoveryNavigator.navigateTo(recoveryGoal);
    var blockingObstacle = new Obstacle(new Detection("recovery-blocker", "obstacle",
      1.0, recoveryGrid.cellCenter(3, 0), "map", Int64.ofInt(1), Int64.ofInt(1),
      Int64.ofInt(1), "recovery-clock", "host-clock"), 0.1);
    var blockedStatus = recoveryNavigator.update(
      new PerceptionSnapshot([], [blockingObstacle]), 0.1);
    check(switch blockedStatus {
      case NavigatorStatus.Blocked(_): true;
      case _: false;
    } && switch recoveryRobot.lastStop {
      case Normal: true;
      case _: false;
    }, "Navigator stops and reports blocked when no route exists");
    var recoveredStatus = recoveryNavigator.update(new PerceptionSnapshot(), 0.1);
    check(recoveredStatus == NavigatorStatus.Navigating &&
      recoveryNavigator.replanCount >= 2,
      "Navigator retries and resumes when a blocked route becomes clear");
    recoveryNavigator.cancel();

    // A robot that has cut into an obstacle's blocked margin, touching
    // nothing, keeps navigating out of it instead of deadlocking on its own
    // cell; the route ahead of the escape stays checked as usual.
    var marginRobot = new FakeRobot("navigator-margin");
    marginRobot.positions = [0.0, 0.0];
    var marginBase = new MobileBase(marginRobot,
      new DifferentialDrive(0, 1, 0.1, 0.5), new MotionLimits(1.0, 1.0));
    var marginLocalization = new FixedLocalization(new LocalizationState(
      Int64.ofInt(0), new Pose2(2.47, 1.0), "map", "base",
      PoseCovariance2.zero(), Good, Int64.ofInt(0), Int64.ofInt(0),
      "margin-clock", "host-clock"));
    var marginNavigation = new Navigation(marginBase, marginLocalization);
    var marginGrid = new OccupancyGrid2(0.2, new Pose2(), 30, 10, "map", OccupancyCell.Free);
    var marginCostmap = new Costmap2(marginGrid, 0.3, false, 0.0, 0.0);
    var marginObstacle = new Obstacle(new Detection("margin-obstacle", "obstacle", 1.0,
      new Pose2(2.0, 1.0), "map", Int64.ofInt(1), Int64.ofInt(1), Int64.ofInt(1),
      "margin-clock", "host-clock"), 0.1);
    marginCostmap.setDynamicObstacles([marginObstacle]);
    var marginNavigator = new Navigator(marginNavigation,
      new AStarPlanner(marginCostmap), marginCostmap, 0.1);
    var marginStatus = marginNavigator.navigateTo(
      new NavigationGoal(new Pose2(5.0, 1.0), "map"));
    for (_ in 0...5)
      marginStatus = marginNavigator.update(new PerceptionSnapshot([], [marginObstacle]), 0.1);
    check(marginStatus == NavigatorStatus.Navigating && marginNavigator.replanCount == 0,
      'Navigator escapes a blocked margin without deadlocking or replanning ($marginStatus)');
    var wall = new Obstacle(new Detection("margin-wall", "obstacle", 1.0,
      new Pose2(4.0, 1.0), "map", Int64.ofInt(1), Int64.ofInt(1), Int64.ofInt(1),
      "margin-clock", "host-clock"), 0.9);
    marginStatus = marginNavigator.update(
      new PerceptionSnapshot([], [marginObstacle, wall]), 0.1);
    check(switch marginStatus { case NavigatorStatus.Blocked(_): true; case _: false; } &&
      marginNavigator.replanCount == 1,
      "Navigator still replans when the route ahead of an escape becomes blocked");
    marginNavigator.cancel();

    var model = new RobotModel("navigator-sim");
    var baseLink = model.addLink(new Link("base", "link/base"));
    var leftLink = model.addLink(new Link("left wheel", "link/left-wheel"));
    var rightLink = model.addLink(new Link("right wheel", "link/right-wheel"));
    var left = new Joint("left wheel", JointType.Continuous, baseLink, leftLink,
      "joint/left-wheel");
    left.limits = new JointLimits(-100.0, 100.0, 20.0, 100.0);
    model.addJoint(left);
    var right = new Joint("right wheel", JointType.Continuous, baseLink, rightLink,
      "joint/right-wheel");
    right.limits = new JointLimits(-100.0, 100.0, 20.0, 100.0);
    model.addJoint(right);
    model.mobileBase = new RobotMobileConfiguration(
      RobotDriveConfiguration.Differential("joint/left-wheel", "joint/right-wheel",
        0.1, 0.5), 0.8, 1.5, 1.5, 2.0, 0.6, 0.4);
    var lidarFrame = model.addFrame(new Frame("lidar mount", baseLink, "frame/lidar"));
    lidarFrame.position = [0.3, 0.0, 0.2];
    var lidar = model.addSensor(new Sensor("front lidar", "lidar", 0.0,
      "sensor/front-lidar"));
    lidar.frame = lidarFrame;
    lidar.rayCount = 64;
    lidar.maxRange = 4.0;

    var blueprint = RobotRuntimeCompiler.compile(model);
    var simulation = new Simulation(0.02);
    var runtime = simulation.addRobot(blueprint);
    var robot = new SimulatedRobot("navigator-sim", runtime, model.name,
      [for (link in model.links) link.name], [for (joint in model.joints) joint.name]);
    var base = MobileBase.fromBlueprint(robot, blueprint);
    var localization = new SimulationTruthLocalization(simulation, 0,
      "map", baseLink.id);
    var scenarioLocalization = new QualityOverrideLocalization(localization);
    var lidarPerception = LidarObstaclePerception.fromBlueprint(blueprint,
      "sensor/front-lidar", 0.12, 0.05, 0.25, 0.4);
    var framedPerception = new FrameAwarePerception(lidarPerception, localization);
    var navigation = new Navigation(base, scenarioLocalization, 0.25, 0.55, 1.2);
    var grid = new OccupancyGrid2(0.2, new Pose2(-2.0, -2.5),
      50, 25, "map", OccupancyCell.Free);
    var footprint = base.footprint;
    if (footprint == null) throw "Simulation scenario requires a robot footprint";
    var costmap = new Costmap2(grid, footprint.radius, true, 0.3, 1.5);
    var planner = new AStarPlanner(costmap);
    var guard = new MotionGuard(navigation, footprint, 0.2, 1.5, 0.1, 0.4);
    var navigator = new Navigator(navigation, planner, costmap, 0.2, guard);

    // The articulated physics backend does not model wheel traction, so an ideal
    // differential-drive plant couples commanded wheel rates to the chassis.
    var plant = new DifferentialDrivePlant(simulation, 0, base);
    var lastPerception = new PerceptionSnapshot();
    function robotObservation(tick:Int):RobotSnapshot
      return plant.step(Int64.ofInt(tick));
    function perceive(snapshot:RobotSnapshot):PerceptionSnapshot {
      lastPerception = framedPerception.observeRobotSnapshot(snapshot, model,
        blueprint, baseLink.id);
      return lastPerception;
    }
    function observe(tick:Int):PerceptionSnapshot return perceive(robotObservation(tick));
    var initialPerception = observe(1);
    equal(initialPerception.obstacles().length, 0,
      "simulated navigation starts with a clear LiDAR observation");
    var initialState = requireLocalizationState(localization);
    var offPathGoalPose = new Pose2(initialState.pose.x + 1.0,
      initialState.pose.y, initialState.pose.yaw);
    var offPathGoal = new NavigationGoal(offPathGoalPose, "map", 0.12, 0.15);
    var goalRunner = new SkillRunner();
    var goalSkill = new GoTo(navigator, offPathGoal, perceive);
    check(goalRunner.start(goalSkill) == SkillStatus.Running &&
      navigator.status == NavigatorStatus.Navigating,
      "GoTo plans and starts a goal through SkillRunner");

    plant.teleport(new Pose2(plant.pose.x, plant.pose.y + 0.6, plant.pose.yaw));
    var tick = 2;
    var goalStatus = goalRunner.update(robotObservation(tick++), 0.02);
    var status = navigator.status;
    var initialOffPathError = Math.abs(navigation.crossTrackError);
    while (tick < 1000 && goalStatus == SkillStatus.Running) {
      goalStatus = goalRunner.update(robotObservation(tick++), 0.02);
      status = navigator.status;
    }
    var offPathFinal = requireLocalizationState(localization);
    check(initialOffPathError >= 0.45 && goalStatus == SkillStatus.Succeeded &&
      status == NavigatorStatus.Succeeded && goalRunner.result() != null &&
      Math.abs(offPathFinal.pose.x - offPathGoalPose.x) <= 0.16 &&
      Math.abs(offPathFinal.pose.y - offPathGoalPose.y) <= 0.16,
      "SkillRunner GoTo reaches its goal after starting off the planned path");

    var goalPose = new Pose2(initialState.pose.x + 4.0,
      initialState.pose.y, initialState.pose.yaw);
    var goal = new NavigationGoal(goalPose, "map", 0.12, 0.15);
    var obstacleGoalSkill = new GoTo(navigator, goal, perceive);
    var obstacleGoalStatus = goalRunner.start(obstacleGoalSkill);
    check(obstacleGoalStatus == SkillStatus.Running &&
      navigator.status == NavigatorStatus.Navigating,
      "GoTo starts another simulated goal before an obstacle appears");
    for (_ in 0...40) {
      obstacleGoalStatus = goalRunner.update(robotObservation(tick++), 0.02);
      status = navigator.status;
    }
    var movingState = requireLocalizationState(localization);
    check(movingState.pose.x > offPathFinal.pose.x + 0.05,
      "Navigator advances the simulated robot before an obstacle appears");
    var obstaclePose = movingState.pose.compose(new Pose2(1.3, 0.0));
    simulation.spawnBox([obstaclePose.x, obstaclePose.y, 0.2], [0.12, 0.12, 0.3]);

    var detectedObstacle = false;
    var pathDetoured = false;
    for (_ in 0...1200) {
      obstacleGoalStatus = goalRunner.update(robotObservation(tick++), 0.02);
      if (lastPerception.obstacles().length > 0) detectedObstacle = true;
      status = navigator.status;
      var currentPath:Null<Path> = navigator.activePath;
      if (navigator.replanCount > 0 && currentPath != null) {
        for (point in currentPath.poses())
          if (Math.abs(point.y - goalPose.y) > 0.45) pathDetoured = true;
      }
      if (obstacleGoalStatus == SkillStatus.Succeeded) break;
    }
    check(detectedObstacle,
      "simulated LiDAR perception detects an obstacle during navigation");
    check(navigator.replanCount > 0 && pathDetoured,
      "Navigator replans onto a route around the detected obstacle");
    var finalState = requireLocalizationState(localization);
    var routeSummary:Array<String> = [];
    var finalPath:Null<Path> = navigator.activePath;
    if (finalPath != null) {
      var routePoints = finalPath.poses();
      for (index in 0...Std.int(Math.min(12, routePoints.length))) {
        var point = routePoints[index];
        routeSummary.push('${point.x},${point.y},${point.yaw}');
      }
    }
    check(obstacleGoalStatus == SkillStatus.Succeeded &&
      status == NavigatorStatus.Succeeded,
      'GoTo reaches its goal through SkillRunner after replanning (state: ${Std.string(status)}, pose: ${finalState.pose.x},${finalState.pose.y},${finalState.pose.yaw}, goal: ${goalPose.x},${goalPose.y}, replans: ${navigator.replanCount}, guard: ${Std.string(guard.state)}, command: ${Std.string(base.currentCommand().linear)},${Std.string(base.currentCommand().angular)}, path: ${routeSummary.join(";")})');
    var finalDx = finalState.pose.x - goalPose.x;
    var finalDy = finalState.pose.y - goalPose.y;
    check(Math.sqrt(finalDx * finalDx + finalDy * finalDy) <= 0.16,
      "replanned simulated route ends inside the goal position tolerance");

    var resumeGoal = new NavigationGoal(new Pose2(finalState.pose.x + 0.8,
      finalState.pose.y, finalState.pose.yaw), "map", 0.12, 0.15);
    check(switch navigator.navigateTo(resumeGoal) {
      case NavigatorStatus.Navigating: true;
      case _: false;
    }, "Navigator accepts a follow-up goal before localization dropout");
    scenarioLocalization.setQualityOverride(Invalid);
    status = navigator.update(observe(tick++), 0.02);
    check(switch status { case NavigatorStatus.Blocked(_): true; case _: false; } &&
      Math.abs(base.currentCommand().linear) < 1e-9 &&
      Math.abs(base.currentCommand().angular) < 1e-9,
      "simulated Navigator stops and blocks when localization becomes invalid");
    scenarioLocalization.setQualityOverride(Good);
    for (_ in 0...1000) {
      if (status == NavigatorStatus.Succeeded) break;
      status = navigator.update(observe(tick++), 0.02);
    }
    check(status == NavigatorStatus.Succeeded && navigator.replanCount > 0,
      "simulated Navigator replans and reaches its goal after localization recovers");

    navigator.cancel();
    var reverseStart = requireLocalizationState(localization);
    var reverseGoalPose = new Pose2(reverseStart.pose.x - 0.65,
      reverseStart.pose.y, reverseStart.pose.yaw);
    var reverseGoal = new NavigationGoal(reverseGoalPose, "map", 0.12, 0.2);
    navigation.follow(new Path([reverseStart.pose, reverseGoalPose], "map"),
      reverseGoal);
    var reverseStatus = navigation.update(0.02);
    var commandedReverse = base.currentCommand().linear < 0.0;
    for (_ in 0...500) {
      if (reverseStatus == NavigationStatus.Succeeded) break;
      observe(tick++);
      reverseStatus = navigation.update(0.02);
    }
    var reverseFinal = requireLocalizationState(localization);
    check(commandedReverse && reverseStatus == NavigationStatus.Succeeded &&
      Math.abs(reverseFinal.pose.x - reverseGoalPose.x) <= 0.16,
      "simulated path follower executes and completes a reverse section");

    var wallColumn = 36;
    for (y in 0...grid.height)
      if (y < 7 || y > 17) grid.setCell(wallColumn, y, OccupancyCell.Occupied);
    costmap.refresh();
    var narrowGoalPose = new Pose2(7.0, 0.0, 0.0);
    status = navigator.navigateTo(new NavigationGoal(narrowGoalPose,
      "map", 0.14, 0.2));
    check(switch status {
      case NavigatorStatus.Navigating: true;
      case _: false;
    }, "Navigator plans through a footprint-inflated narrow passage");
    var usedNarrowGap = false;
    var narrowPath = navigator.activePath;
    if (narrowPath != null) {
      for (point in narrowPath.poses()) {
        var cell = grid.worldToCell(point);
        if (cell != null && cell.x == wallColumn && cell.y >= 7 && cell.y <= 17)
          usedNarrowGap = true;
      }
    }
    for (_ in 0...1200) {
      if (status == NavigatorStatus.Succeeded) break;
      status = navigator.update(observe(tick++), 0.02);
    }
    var narrowFinal = requireLocalizationState(localization);
    check(usedNarrowGap && status == NavigatorStatus.Succeeded &&
      Math.abs(narrowFinal.pose.x - narrowGoalPose.x) <= 0.18,
      "simulated Navigator traverses the narrow passage and reaches its goal");
    navigator.cancel();
    robot.close();
    simulation.dispose();
  }

  static function testGoToBlockedTimeout():Void {
    var robot = new FakeRobot("goto-blocked");
    robot.positions = [0.0, 0.0];
    var base = new MobileBase(robot, new DifferentialDrive(0, 1, 0.1, 0.5),
      new MotionLimits(1.0, 1.0));
    var localization = new FixedLocalization(new LocalizationState(
      Int64.ofInt(0), new Pose2(0.5, 0.5), "map", "base",
      PoseCovariance2.zero(), Good, Int64.ofInt(0), Int64.ofInt(0),
      "blocked-clock", "host-clock"));
    var navigation = new Navigation(base, localization);
    var grid = new OccupancyGrid2(1.0, new Pose2(), 7, 1, "map", OccupancyCell.Free);
    var costmap = new Costmap2(grid, 0.0, false, 0.0, 0.0);
    var navigator = new Navigator(navigation, new AStarPlanner(costmap), costmap, 0.1);
    var goal = new NavigationGoal(grid.cellCenter(6, 0), "map");
    var blocker = new Obstacle(new Detection("goto-blocker", "obstacle",
      1.0, grid.cellCenter(3, 0), "map", Int64.ofInt(1), Int64.ofInt(1),
      Int64.ofInt(1), "blocked-clock", "host-clock"), 0.1);
    var blocked = new PerceptionSnapshot([], [blocker]);
    var clear = new PerceptionSnapshot();
    var perception = blocked;
    var observation = robot.snapshot();
    function isBlocked():Bool
      return switch navigator.status { case NavigatorStatus.Blocked(_): true; case _: false; };

    var sustained = new GoTo(navigator, goal, function(_) return perception, 0.35);
    throws(function() new GoTo(navigator, goal, function(_) return perception, -1.0),
      "GoTo rejects a negative blocked timeout");
    sustained.start();
    for (_ in 0...3) sustained.update(observation, 0.1);
    check(sustained.status() == SkillStatus.Running && isBlocked() &&
      navigator.replanCount >= 2,
      "GoTo keeps running while the Navigator retries within its blocked timeout");
    sustained.update(observation, 0.1);
    var sustainedMessage = switch sustained.status() {
      case Failed(message): message;
      case _: "";
    };
    check(sustainedMessage.indexOf("GoTo remained blocked beyond its timeout") == 0 &&
      sustainedMessage.length > "GoTo remained blocked beyond its timeout: ".length &&
      navigator.status == NavigatorStatus.Cancelled &&
      switch robot.lastStop { case Normal: true; case _: false; },
      'GoTo fails with the blocked reason and stops the Navigator after sustained blocking ($sustainedMessage)');

    perception = blocked;
    var recovering = new GoTo(navigator, goal, function(_) return perception, 0.35);
    recovering.start();
    recovering.update(observation, 0.1);
    recovering.update(observation, 0.1);
    check(recovering.status() == SkillStatus.Running && isBlocked(),
      "GoTo starts a second blocked interval");
    perception = clear;
    recovering.update(observation, 0.1);
    check(recovering.status() == SkillStatus.Running &&
      navigator.status == NavigatorStatus.Navigating,
      "GoTo resumes when the Navigator finds a clear route");
    perception = blocked;
    for (_ in 0...3) recovering.update(observation, 0.1);
    check(recovering.status() == SkillStatus.Running && isBlocked(),
      "GoTo restarts its blocked timeout after navigation resumes");
    recovering.cancel();
    check(recovering.status() == SkillStatus.Cancelled &&
      navigator.status == NavigatorStatus.Cancelled,
      "GoTo cancels a blocked navigation cleanly");
  }

  static function testDifferentialDrivePlantKinematics():Void {
    var model = new RobotModel("plant-kinematics");
    var baseLink = model.addLink(new Link("base", "link/base"));
    var leftLink = model.addLink(new Link("left wheel", "link/left-wheel"));
    var rightLink = model.addLink(new Link("right wheel", "link/right-wheel"));
    var left = new Joint("left wheel", JointType.Continuous, baseLink, leftLink,
      "joint/left-wheel");
    left.limits = new JointLimits(-1000.0, 1000.0, 10.0, 100.0);
    model.addJoint(left);
    var right = new Joint("right wheel", JointType.Continuous, baseLink, rightLink,
      "joint/right-wheel");
    right.limits = new JointLimits(-1000.0, 1000.0, 10.0, 100.0);
    model.addJoint(right);
    model.mobileBase = new RobotMobileConfiguration(
      RobotDriveConfiguration.Differential("joint/left-wheel", "joint/right-wheel",
        0.1, 0.5), 0.8, 1.5, 100.0, 100.0);
    var imu = model.addSensor(new Sensor("base imu", "imu", 0.0, "sensor/imu"));
    imu.frame = model.addFrame(new Frame("imu mount", baseLink, "frame/imu"));
    var blueprint = RobotRuntimeCompiler.compile(model);
    var simulation = new Simulation(0.02);
    var robot = new SimulatedRobot("plant-kinematics", simulation.addRobot(blueprint),
      model.name, [for (link in model.links) link.name], [for (joint in model.joints) joint.name]);
    var base = MobileBase.fromBlueprint(robot, blueprint);
    var startYaw = 0.5;
    simulation.teleportRobot(0, [1.0, 2.0, 0.3],
      [0.0, 0.0, Math.sin(startYaw * 0.5), Math.cos(startYaw * 0.5)]);
    var plant = new DifferentialDrivePlant(simulation, 0, base);
    check(Math.abs(plant.pose.x - 1.0) < 1e-9 && Math.abs(plant.pose.y - 2.0) < 1e-9 &&
      Math.abs(plant.pose.yaw - startYaw) < 1e-9 && Math.abs(plant.baseHeight - 0.3) < 1e-9,
      "DifferentialDrivePlant starts from the simulated base pose and authored height");
    var tick = 1;
    function imuFrame(snapshot:RobotSnapshot):Null<SensorFrame> {
      for (index in 0...snapshot.sensors.length)
        if (snapshot.sensors.get(index).sensorId == "sensor/imu")
          return snapshot.sensors.get(index);
      return null; // Unpublished while the derivative is priming.
    }
    function imuSequence(snapshot:RobotSnapshot):Int64 {
      var frame = imuFrame(snapshot);
      return frame == null ? Int64.ofInt(0) : frame.sequence;
    }
    function imuValue(snapshot:RobotSnapshot, index:Int):Float {
      var frame = imuFrame(snapshot);
      if (frame == null) throw "IMU sample was not published";
      return frame.values.get(index);
    }
    // Straight line: 0.5 m/s is 5 rad/s on each 0.1 m wheel, so ten 0.02 s
    // ticks roll 0.1 m along the heading.
    base.command(new Twist2(0.5, 0.0));
    var firstImu = imuSequence(plant.step(Int64.ofInt(tick++)));
    var snapshot = plant.step(Int64.ofInt(tick++));
    for (_ in 0...8) snapshot = plant.step(Int64.ofInt(tick++));
    check(Math.abs(plant.pose.x - (1.0 + 0.1 * Math.cos(startYaw))) < 1e-9 &&
      Math.abs(plant.pose.y - (2.0 + 0.1 * Math.sin(startYaw))) < 1e-9 &&
      Math.abs(plant.pose.yaw - startYaw) < 1e-9,
      "DifferentialDrivePlant rolls equal wheel rates straight along the heading for v*dt per tick");
    var physics = simulation.robotPose(0);
    check(Math.abs(physics.position[0] - plant.pose.x) < 1e-6 &&
      Math.abs(physics.position[1] - plant.pose.y) < 1e-6 &&
      Math.abs(physics.position[2] - 0.3) < 1e-6,
      "DifferentialDrivePlant drives the physics base and preserves its authored height");
    check(Int64.compare(imuSequence(snapshot), Int64.add(firstImu, Int64.ofInt(9))) == 0,
      "IMU keeps publishing every tick while the plant drives the base");
    check(Math.abs(imuValue(snapshot, 2)) < 1e-4 && Math.abs(imuValue(snapshot, 3)) < 1e-2 &&
      Math.abs(imuValue(snapshot, 4)) < 1e-2 && Math.abs(imuValue(snapshot, 5) - 9.81) < 1e-6,
      "IMU measures a straight constant-speed run as no rotation and no acceleration");

    // Turn sign: a positive yaw rate spins the right wheel forward and turns
    // counter-clockwise at (vr - vl) / track.
    base.command(new Twist2(0.0, 1.0));
    var turnStart = plant.pose;
    for (_ in 0...10) snapshot = plant.step(Int64.ofInt(tick++));
    check(Math.abs(Pose2.wrapAngle(plant.pose.yaw - turnStart.yaw) - 0.2) < 1e-9 &&
      Math.abs(plant.pose.x - turnStart.x) < 1e-9 && Math.abs(plant.pose.y - turnStart.y) < 1e-9,
      "DifferentialDrivePlant turns in place counter-clockwise at (vr-vl)/track");
    check(Math.abs(imuValue(snapshot, 2) - 1.0) < 1e-3 &&
      Math.abs(imuValue(snapshot, 0)) < 1e-4 && Math.abs(imuValue(snapshot, 1)) < 1e-4 &&
      Math.abs(imuValue(snapshot, 3)) < 1e-2 && Math.abs(imuValue(snapshot, 4)) < 1e-2,
      "IMU gyro reads the in-place turn rate with no linear acceleration");

    // An arc at 0.5 m/s and 1 rad/s pulls the IMU 0.5 m/s^2 to the left.
    base.command(new Twist2(0.5, 1.0));
    for (_ in 0...5) snapshot = plant.step(Int64.ofInt(tick++));
    check(Math.abs(imuValue(snapshot, 2) - 1.0) < 1e-3 &&
      Math.abs(imuValue(snapshot, 3)) < 2e-2 && Math.abs(imuValue(snapshot, 4) - 0.5) < 2e-2,
      "IMU measures the arc's yaw rate and centripetal acceleration");

    // 0.8 m/s with 1.5 rad/s asks 11.75 rad/s of the right wheel; the runtime
    // saturates that wheel alone at its 10 rad/s limit and the plant rolls by
    // the applied rates.
    base.command(new Twist2(0.8, 1.5));
    var saturatedStart = plant.pose;
    plant.step(Int64.ofInt(tick++));
    var expectedSaturated = saturatedStart.integrateDisplacement(
      (4.25 + 10.0) * 0.1 * 0.02 * 0.5, (10.0 - 4.25) * 0.1 * 0.02 / 0.5);
    check(Math.abs(plant.pose.x - expectedSaturated.x) < 1e-9 &&
      Math.abs(plant.pose.y - expectedSaturated.y) < 1e-9 &&
      Math.abs(plant.pose.yaw - expectedSaturated.yaw) < 1e-9 &&
      plant.appliedWheelRates().right == 10.0,
      "DifferentialDrivePlant rolls by the runtime's rate-limited wheel targets");

    // Wheel targets submitted straight to the robot, bypassing MobileBase,
    // drive the chassis on the tick they are applied.
    base.stop();
    plant.step(Int64.ofInt(tick++));
    var directStart = plant.pose;
    robot.submit(RobotCommand.JointTargets([
      robotkit.world.JointTarget.velocity(0, 3.0),
      robotkit.world.JointTarget.velocity(1, 3.0)
    ], null));
    plant.step(Int64.ofInt(tick++));
    var directExpected = directStart.integrateDisplacement(3.0 * 0.1 * 0.02, 0.0);
    check(base.currentCommand().linear == 0.0 &&
      Math.abs(plant.pose.x - directExpected.x) < 1e-9 &&
      Math.abs(plant.pose.y - directExpected.y) < 1e-9,
      "DifferentialDrivePlant moves with wheel targets submitted directly to the robot");

    // A stop issued directly on the robot halts the chassis on the tick the
    // runtime applies it, even though MobileBase still caches its last command.
    base.command(new Twist2(0.5, 0.0));
    plant.step(Int64.ofInt(tick++));
    robot.stop(StopMode.Emergency);
    var stopped = plant.pose;
    for (_ in 0...4) plant.step(Int64.ofInt(tick++));
    check(base.currentCommand().linear > 0.0 &&
      Math.abs(plant.pose.x - stopped.x) < 1e-12 && Math.abs(plant.pose.y - stopped.y) < 1e-12,
      "DifferentialDrivePlant does not move a robot under a direct emergency stop");
    robot.resetSafety();
    for (_ in 0...3) plant.step(Int64.ofInt(tick++));
    check(Math.abs(plant.pose.x - stopped.x) < 1e-12 && Math.abs(plant.pose.y - stopped.y) < 1e-12,
      "DifferentialDrivePlant ignores the stale cached command after a safety reset");
    base.command(new Twist2(0.5, 0.0));
    for (_ in 0...3) plant.step(Int64.ofInt(tick++));
    check(Math.abs(plant.pose.x - stopped.x) > 1e-3,
      "DifferentialDrivePlant moves again after a new command");
    robot.stop(StopMode.Normal);
    var normalStopped = plant.pose;
    for (_ in 0...4) snapshot = plant.step(Int64.ofInt(tick++));
    check(base.currentCommand().linear > 0.0 &&
      Math.abs(plant.pose.x - normalStopped.x) < 1e-12 &&
      Math.abs(plant.pose.y - normalStopped.y) < 1e-12 &&
      snapshot.velocities.get(0) == 0.0 && snapshot.velocities.get(1) == 0.0,
      "DifferentialDrivePlant and the wheels stop during a direct normal stop");

    // Driving never replaces the reset pose chosen by the editable-scene teleport.
    simulation.resetRobot(0);
    simulation.step(Int64.ofInt(tick++));
    var reset = simulation.robotPose(0);
    check(Math.abs(reset.position[0] - 1.0) < 1e-6 && Math.abs(reset.position[1] - 2.0) < 1e-6 &&
      Math.abs(reset.position[2] - 0.3) < 1e-6,
      "resetRobot restores the authored base pose after the plant drove it");
    robot.close();
    simulation.dispose();
  }

  static function testHolonomicDrivePlantKinematics():Void {
    var model = new RobotModel("omni-plant-kinematics");
    var baseLink = model.addLink(new Link("base", "link/base"));
    var wheelRadius = 0.05, baseRadius = 0.3;
    var wheelIds:Array<String> = [];
    for (i in 0...3) {
      var wheel = model.addLink(new Link('wheel$i', 'link/wheel$i'));
      var joint = new Joint('wheel$i', JointType.Continuous, baseLink, wheel, 'joint/wheel$i');
      joint.limits = new JointLimits(-1000.0, 1000.0, 10.0, 100.0);
      model.addJoint(joint);
      wheelIds.push(joint.id);
    }
    model.mobileBase = new RobotMobileConfiguration(
      RobotDriveConfiguration.Holonomic(wheelIds, wheelRadius, baseRadius), 0.8, 1.5, 100.0, 100.0);
    var imu = model.addSensor(new Sensor("base imu", "imu", 0.0, "sensor/imu"));
    imu.frame = model.addFrame(new Frame("imu mount", baseLink, "frame/imu"));
    var blueprint = RobotRuntimeCompiler.compile(model);
    var simulation = new Simulation(0.02);
    var robot = new SimulatedRobot("omni-plant-kinematics", simulation.addRobot(blueprint),
      model.name, [for (link in model.links) link.name], [for (joint in model.joints) joint.name]);
    var base = MobileBase.fromBlueprint(robot, blueprint);
    var startYaw = 0.5;
    simulation.teleportRobot(0, [1.0, 2.0, 0.3],
      [0.0, 0.0, Math.sin(startYaw * 0.5), Math.cos(startYaw * 0.5)]);
    var plant = new HolonomicDrivePlant(simulation, 0, base);
    check(Math.abs(plant.pose.x - 1.0) < 1e-9 && Math.abs(plant.pose.y - 2.0) < 1e-9 &&
      Math.abs(plant.pose.yaw - startYaw) < 1e-9 && Math.abs(plant.baseHeight - 0.3) < 1e-9,
      "HolonomicDrivePlant starts from the simulated base pose and authored height");
    var tick = 1;
    function imuFrame(snapshot:RobotSnapshot):Null<SensorFrame> {
      for (index in 0...snapshot.sensors.length)
        if (snapshot.sensors.get(index).sensorId == "sensor/imu")
          return snapshot.sensors.get(index);
      return null;
    }
    function imuSequence(snapshot:RobotSnapshot):Int64 {
      var frame = imuFrame(snapshot);
      return frame == null ? Int64.ofInt(0) : frame.sequence;
    }
    function imuValue(snapshot:RobotSnapshot, index:Int):Float {
      var frame = imuFrame(snapshot);
      if (frame == null) throw "IMU sample was not published";
      return frame.values.get(index);
    }

    // Forward at 0.3 m/s: ten 0.02 s ticks move 0.06 m along the heading.
    base.command(new Twist2(0.3, 0.0));
    var firstImu = imuSequence(plant.step(Int64.ofInt(tick++)));
    var snapshot = plant.step(Int64.ofInt(tick++));
    for (_ in 0...8) snapshot = plant.step(Int64.ofInt(tick++));
    check(Math.abs(plant.pose.x - (1.0 + 0.06 * Math.cos(startYaw))) < 1e-9 &&
      Math.abs(plant.pose.y - (2.0 + 0.06 * Math.sin(startYaw))) < 1e-9 &&
      Math.abs(plant.pose.yaw - startYaw) < 1e-9,
      "HolonomicDrivePlant drives a forward twist along the heading for v*dt per tick");
    var physics = simulation.robotPose(0);
    check(Math.abs(physics.position[0] - plant.pose.x) < 1e-6 &&
      Math.abs(physics.position[1] - plant.pose.y) < 1e-6 &&
      Math.abs(physics.position[2] - 0.3) < 1e-6,
      "HolonomicDrivePlant drives the physics base and preserves its authored height");
    check(Int64.compare(imuSequence(snapshot), Int64.add(firstImu, Int64.ofInt(9))) == 0,
      "IMU keeps publishing every tick while the holonomic plant drives the base");

    // An arc at 0.3 m/s and 1 rad/s turns 0.02 rad per tick.
    base.command(new Twist2(0.3, 1.0));
    var arcStart = plant.pose;
    for (_ in 0...5) snapshot = plant.step(Int64.ofInt(tick++));
    check(Math.abs(plant.pose.yaw - arcStart.yaw - 0.1) < 1e-9 &&
      Math.abs(imuValue(snapshot, 2) - 1.0) < 1e-3,
      "HolonomicDrivePlant turns at the commanded yaw rate and the gyro reads it");

    // Wheels driven straight on the robot can strafe, which Twist2 cannot
    // express: 0.3 m/s along body +Y for ten ticks is 0.06 m to the left.
    base.stop();
    var strafeBaseline = plant.step(Int64.ofInt(tick++));
    var strafeStart = plant.pose;
    var strafeOdometry = new HolonomicOdometry([0, 1, 2], wheelRadius, baseRadius, strafeStart);
    strafeOdometry.update(strafeBaseline);
    base.command(new Twist2(0.0, 0.0, 0.3));
    for (_ in 0...10) snapshot = plant.step(Int64.ofInt(tick++));
    var expectedStrafe = strafeStart.integrate(new Twist2(0.0, 0.0, 0.3), 0.2);
    check(Math.abs(plant.pose.x - expectedStrafe.x) < 1e-9 &&
      Math.abs(plant.pose.y - expectedStrafe.y) < 1e-9 &&
      Math.abs(plant.pose.yaw - expectedStrafe.yaw) < 1e-9,
      "HolonomicDrivePlant drives a pure lateral twist along body +Y");
    strafeOdometry.update(snapshot);
    check(Math.abs(strafeOdometry.current().x - plant.pose.x) < 1e-9 &&
      Math.abs(strafeOdometry.current().y - plant.pose.y) < 1e-9 &&
      Math.abs(strafeOdometry.current().yaw - plant.pose.yaw) < 1e-9,
      "Holonomic odometry agrees with the simulated pure lateral drive");
    base.command(new Twist2(0.2, 0.5, 0.15));
    var diagonalStart = plant.pose;
    for (_ in 0...10) snapshot = plant.step(Int64.ofInt(tick++));
    var expectedDiagonal = diagonalStart.integrate(new Twist2(0.2, 0.5, 0.15), 0.2);
    check(Math.abs(plant.pose.x - expectedDiagonal.x) < 1e-9 &&
      Math.abs(plant.pose.y - expectedDiagonal.y) < 1e-9 &&
      Math.abs(plant.pose.yaw - expectedDiagonal.yaw) < 1e-9,
      "HolonomicDrivePlant integrates diagonal translation with yaw");
    base.stop();
    plant.step(Int64.ofInt(tick++));
    var wheelStart = plant.pose;
    var strafe = [for (i in 0...3) {
      var angle = Math.PI * 0.5 + i * Math.PI * 2.0 / 3.0;
      robotkit.world.JointTarget.velocity(i, Math.cos(angle) * 0.3 / wheelRadius);
    }];
    robot.submit(RobotCommand.JointTargets(strafe, null));
    for (_ in 0...10) plant.step(Int64.ofInt(tick++));
    check(base.currentCommand().linear == 0.0 &&
      Math.abs(plant.pose.x - (wheelStart.x - 0.06 * Math.sin(wheelStart.yaw))) < 1e-9 &&
      Math.abs(plant.pose.y - (wheelStart.y + 0.06 * Math.cos(wheelStart.yaw))) < 1e-9 &&
      Math.abs(plant.pose.yaw - wheelStart.yaw) < 1e-9,
      "HolonomicDrivePlant strafes with wheel targets submitted directly to the robot");

    // A stop issued directly on the robot halts the chassis at once, even
    // though MobileBase still caches its last command.
    base.command(new Twist2(0.3, 0.0));
    plant.step(Int64.ofInt(tick++));
    robot.stop(StopMode.Emergency);
    var stopped = plant.pose;
    for (_ in 0...4) plant.step(Int64.ofInt(tick++));
    var rates = plant.appliedWheelRates();
    check(base.currentCommand().linear > 0.0 &&
      Math.abs(plant.pose.x - stopped.x) < 1e-12 && Math.abs(plant.pose.y - stopped.y) < 1e-12 &&
      rates[0] == 0.0 && rates[1] == 0.0 && rates[2] == 0.0,
      "HolonomicDrivePlant does not move a robot under a direct emergency stop");

    // Teleport keeps the authored height.
    plant.teleport(new Pose2(3.0, 4.0, 0.0));
    plant.step(Int64.ofInt(tick++));
    check(Math.abs(plant.pose.x - 3.0) < 1e-9 && Math.abs(plant.pose.y - 4.0) < 1e-9 &&
      Math.abs(simulation.robotPose(0).position[2] - 0.3) < 1e-6,
      "HolonomicDrivePlant teleports to a planar pose at the authored height");
    robot.close();
    simulation.dispose();
  }

  static function testForkMechanisms():Void {
    var robot = new FakeRobot("fork-test");
    robot.jointNames = ["mast-lift", "fork-tilt", "fork-spread"];
    robot.positions = [0.5, 0.1, 0.3];
    robot.velocities = [0.0, -0.02, 0.0];
    robot.efforts = [2.0, 0.5, 0.25];
    var config = new ForkConfig(new ForkAxisConfig("mast-lift", 0.0, 2.0),
      new LoadLimits(1000.0, 600.0, 1.8),
      new ForkAxisConfig("fork-tilt", -0.5, 0.7),
      new ForkAxisConfig("fork-spread", 0.0, 0.8));
    var forks = new Forks(robot, config);
    var state = forks.state();
    check(Math.abs(state.lift.position - 0.5) < 1e-9 &&
      Math.abs(cast(state.tilt, robotkit.material.ForkAxisState).velocity + 0.02) < 1e-9 &&
      Math.abs(cast(state.spread, robotkit.material.ForkAxisState).effort - 0.25) < 1e-9,
      "ForkState maps named axes to robot snapshot position, velocity, and effort");
    equal(state.sourceClockId, "unspecified", "ForkState retains source clock identity");

    var payload = new Payload(500.0, 1.0, 0.8, 0.7, 0.6, 0.0, 0.35);
    forks.setLoadState(LoadState.carried(payload));
    forks.command(1.5, 0.2, 0.4);
    check(switch robot.lastCommand {
      case JointTargets(targets, _):
        targets.length == 3 && targets[0].joint == 0 && targets[1].joint == 1 &&
          targets[2].joint == 2 && targets[0].target == 1.5 &&
          targets[1].target == 0.2 && targets[2].target == 0.4 &&
          targets[0].mode == robotkit.world.JointTargetMode.Position &&
          targets[1].mode == robotkit.world.JointTargetMode.Position &&
          targets[2].mode == robotkit.world.JointTargetMode.Position;
      case _: false;
    }, "Forks sends lift, tilt, and spread targets as one atomic batch");

    forks.setLoadState(LoadState.carried(new Payload(1200.0, 1.0, 0.8, 0.7, 0.4)));
    throws(function() forks.raise(1.0), "Forks rejects payloads above the configured mass envelope");
    forks.setLoadState(LoadState.carried(payload));
    throws(function() forks.raise(2.0), "Forks enforces the configured maximum lift height");
    throws(function() forks.spreadTo(1.0), "Forks enforces configured axis position limits");
    throws(function() new Forks(robot, new ForkConfig(
      new ForkAxisConfig("missing-lift", 0.0, 1.0), new LoadLimits(100.0, 100.0, 1.0))),
      "Forks requires configured joint names to exist in the robot description");
    throws(function() LoadState.detected(null), "detected load state requires a payload value");
    check(!LoadState.unknown().observed && LoadState.empty().observed,
      "load state distinguishes unknown from confirmed empty");
  }

  static function testPerceptionSafetyPower():Void {
    var lidar = new SensorFrame("front-lidar", "lidar", "base", Int64.ofInt(8),
      Int64.ofInt(100), [1.0, 10.0, 0.5, 0.0], Int64.ofInt(120), "base-link",
      null, null, "robot-boot", "host-clock");
    var imu = new SensorFrame("imu", "imu", "base", Int64.ofInt(2),
      Int64.ofInt(100), [0.0, 0.0, 0.0], Int64.ofInt(120), "base-link",
      null, null, "robot-boot", "host-clock");
    var perception = new LidarObstaclePerception(10.0, 0.2, 0.05, 0.85);
    var observed = perception.observe([lidar, imu]);
    var obstacles = observed.obstacles();
    equal(obstacles.length, 2, "LiDAR perception emits hits and ignores max-range and zero rays");
    check(Math.abs(obstacles[0].detection.pose.x - 1.0) < 1e-9 &&
      Math.abs(obstacles[1].detection.pose.x + 0.5) < 1e-9 &&
      obstacles[1].detection.frameId == "base",
      "LiDAR obstacle values retain planar coordinates and source frame");
    equal(obstacles[0].detection.sourceClockId, "robot-boot",
      "semantic obstacle preserves source clock identity");
    var mapEstimate = new LocalizationState(Int64.ofInt(4),
      new Pose2(5.0, 2.0, Math.PI * 0.5), "map", "base",
      PoseCovariance2.zero(), Good, Int64.ofInt(140), Int64.ofInt(150),
      "robot-boot", "host-clock");
    var perceptionModel = new RobotModel("framed-perception");
    var perceptionBase = perceptionModel.addLink(new Link("base", "base"));
    var mast = perceptionModel.addLink(new Link("mast", "mast"));
    var mastLift = perceptionModel.addJoint(new Joint("mast-lift",
      JointType.Prismatic, perceptionBase, mast));
    mastLift.limits = new JointLimits(0.0, 1.5, 0.5, 100.0);
    var laserMount = perceptionModel.addFrame(new Frame("laser mount",
      perceptionBase, "laser"));
    laserMount.position = [0.2, 0.0, 0.3];
    laserMount.rotation = [0.0, 0.0, Math.sin(Math.PI * 0.25),
      Math.cos(Math.PI * 0.25)];
    var mountedLidar = perceptionModel.addSensor(new Sensor("front-lidar", "lidar"));
    mountedLidar.frame = laserMount;
    var mastCameraMount = perceptionModel.addFrame(new Frame("mast camera mount",
      mast, "mast-camera"));
    mastCameraMount.position = [0.4, 0.0, 0.2];
    var perceptionFrames = FrameTree2.fromRobotModel(perceptionModel, "base");
    var authoredMount = perceptionFrames.lookup("base", "laser");
    check(Math.abs(authoredMount.x - 0.2) < 1e-9 &&
      Math.abs(authoredMount.yaw - Math.PI * 0.5) < 1e-9,
      "planar frame tree compiles body sensor mounts from the robot model");
    throws(function() perceptionFrames.lookup("base", "mast-camera"),
      "model frame helper omits articulated-link mounts that need joint-state transforms");
    var articulatedBlueprint = RobotRuntimeCompiler.compile(perceptionModel);
    var articulatedTree = RobotFrameTree2.fromSnapshot(perceptionModel,
      articulatedBlueprint, new RobotSnapshot("mast", Int64.ofInt(1),
        Int64.ofInt(100), [0.75], [0.0], [0.0], 1, 0), "base");
    var mastCamera = articulatedTree.lookup("base", "mast-camera");
    check(Math.abs(mastCamera.x - 0.4) < 1e-9 &&
      Math.abs(mastCamera.y) < 1e-9,
      "snapshot frame tree resolves an elevated sensor through its current lift joint");

    var rotatingModel = new RobotModel("rotating-frame");
    var rotatingBase = rotatingModel.addLink(new Link("base", "base"));
    var turret = rotatingModel.addLink(new Link("turret", "turret"));
    var turretJoint = rotatingModel.addJoint(new Joint("turret-yaw",
      JointType.Revolute, rotatingBase, turret));
    turretJoint.limits = new JointLimits(-Math.PI, Math.PI, 2.0, 100.0);
    var turretFrame = rotatingModel.addFrame(new Frame("turret sensor", turret,
      "turret-sensor"));
    turretFrame.position = [1.0, 0.0, 0.0];
    var rotatingBlueprint = RobotRuntimeCompiler.compile(rotatingModel);
    var rotatingTree = RobotFrameTree2.fromSnapshot(rotatingModel,
      rotatingBlueprint, new RobotSnapshot("turret", Int64.ofInt(1),
        Int64.ofInt(100), [Math.PI * 0.5], [0.0], [0.0], 1, 0), "base");
    var turretSensor = rotatingTree.lookup("base", "turret-sensor");
    check(Math.abs(turretSensor.x) < 1e-9 &&
      Math.abs(turretSensor.y - 1.0) < 1e-9 &&
      Math.abs(turretSensor.yaw - Math.PI * 0.5) < 1e-9,
      "snapshot frame tree composes revolute joint motion with an authored sensor mount");
    var fixedLocalization = new FixedLocalization(mapEstimate);
    var framed = new FrameAwarePerception(perception, fixedLocalization);
    var framedSensor = new SensorFrame("front-lidar", "lidar", "laser",
      Int64.ofInt(9), Int64.ofInt(140), [1.0, 10.0, 10.0, 10.0],
      Int64.ofInt(150), "base", [0.2, 0.0, 0.3], laserMount.rotation,
      "robot-boot", "host-clock");
    var framedLidar = framed.observeRobotSnapshot(new RobotSnapshot("framed-lidar",
      Int64.ofInt(9), Int64.ofInt(140), [0.75], [0.0], [0.0], 1, 0,
      Int64.ofInt(150), [framedSensor], "robot-boot", "host-clock"),
      perceptionModel, articulatedBlueprint, "base");
    var framedObstacle = framedLidar.obstacles()[0];
    check(framedObstacle.detection.frameId == "map" &&
      Math.abs(framedObstacle.detection.pose.x - 4.0) < 1e-9 &&
      Math.abs(framedObstacle.detection.pose.y - 2.2) < 1e-9 &&
      Math.abs(Pose2.wrapAngle(framedObstacle.detection.pose.yaw - Math.PI)) < 1e-9,
      "frame-aware perception composes localization and sensor mount transforms");
    equal(fixedLocalization.updateCount, 1,
      "robot-snapshot perception updates localization from the same observation");
    equal(framedObstacle.radiusMeters, obstacles[0].radiusMeters,
      "frame-aware perception retains obstacle geometry");
    equal(lidar.frameId, "base", "frame-aware perception does not mutate input sensor frames");

    var detections = observed.detections();
    detections.pop();
    var scanRanges = [for (_ in 0...64) 10.0];
    scanRanges[0] = 2.0;
    scanRanges[63] = 2.0;
    scanRanges[10] = 1.0;
    scanRanges[11] = 1.0;
    var clustered = perception.observe([new SensorFrame("dense-lidar", "lidar", "laser",
      Int64.ofInt(9), Int64.ofInt(130), scanRanges, Int64.ofInt(135), "base-link",
      null, null, "robot-boot", "host-clock")]);
    var clusteredObstacles = clustered.obstacles();
    check(clusteredObstacles.length == 2 &&
      clusteredObstacles[0].detection.confidence > perception.minConfidence &&
      clusteredObstacles[0].detection.pose.x > 1.9 &&
      Math.abs(clusteredObstacles[0].detection.pose.y) < 0.11 &&
      clusteredObstacles[0].radiusMeters > perception.obstacleRadiusMeters,
      "LiDAR clustering merges nearby returns across the circular scan seam");
    var partialScan = new LidarObstaclePerception(10.0, 0.1, 0.05, 0.5,
      0.1, -Math.PI * 0.5, Math.PI).observe([new SensorFrame("front-scan", "lidar",
      "laser", Int64.ofInt(10), Int64.ofInt(140), [1.0, 10.0, 10.0],
      Int64.ofInt(145), "base-link", null, null, "robot-boot", "host-clock")]);
    check(Math.abs(partialScan.obstacles()[0].detection.pose.x) < 1e-9 &&
      Math.abs(partialScan.obstacles()[0].detection.pose.y + 1.0) < 1e-9,
      "LiDAR perception respects configured start angle and partial field of view");
    equal(observed.detections().length, 2, "perception snapshot returns owned collections");

    var palletDetection = new Detection("pallet-1", "pallet", 0.95,
      new Pose2(2.0, 1.0, 0.2), "laser", Int64.ofInt(1), Int64.ofInt(200),
      Int64.ofInt(220), "camera-boot", "host-clock");
    var pallet = new Pallet(palletDetection, 1.2, 0.8, 0.15);
    var dock = new DockingTarget(new Detection("dock-1", "dock", 0.9,
      new Pose2(4.0, 0.0, 0.0), "laser", Int64.ofInt(2), Int64.ofInt(210),
      Int64.ofInt(230), "camera-boot", "host-clock"), new Pose2(3.0, 0.0, 0.0));
    var semantic = new PerceptionSnapshot([palletDetection], [], [pallet], [dock]);
    check(semantic.pallets()[0].lengthMeters == 1.2 &&
      semantic.dockingTargets()[0].approachPose.x == 3.0,
      "perception values represent pallet and docking targets");
    var sceneTruth:PerceptionSnapshot = semantic;
    var truthPerception = new GroundTruthPerception(function() return sceneTruth);
    var truthObservation = truthPerception.observe([]);
    check(truthObservation.pallets()[0].detection.id == "pallet-1" &&
      truthObservation.dockingTargets()[0].detection.id == "dock-1",
      "ground-truth perception supplies semantic scene values through the perception API");
    var framedTruth = new FrameAwarePerception(truthPerception,
      new FixedLocalization(mapEstimate), perceptionFrames).observe([]);
    var framedPallet = framedTruth.pallets()[0];
    var framedDock = framedTruth.dockingTargets()[0];
    check(framedPallet.detection.frameId == "map" &&
      Math.abs(framedPallet.detection.pose.x - 3.0) < 1e-9 &&
      framedDock.detection.frameId == "map" &&
      Math.abs(framedDock.approachPose.x - 2.0) < 1e-9 &&
      Math.abs(framedDock.approachPose.y - 2.2) < 1e-9,
      "frame-aware perception transforms pallet poses and docking approach poses");
    var invalidEstimate = new FixedLocalization(new LocalizationState(
      Int64.ofInt(5), new Pose2(), "map", "base", PoseCovariance2.zero(),
      Invalid, Int64.ofInt(150), Int64.ofInt(160), "robot-boot", "host-clock"));
    throws(function() new FrameAwarePerception(perception, invalidEstimate,
      perceptionFrames).observe([lidar]),
      "frame-aware perception refuses to project detections from invalid localization");
    throws(function() new FrameAwarePerception(perception,
      new FixedLocalization(mapEstimate), new FrameTree2()).observe([
        new SensorFrame("unframed-lidar", "lidar", "unconnected-laser",
          Int64.ofInt(10), Int64.ofInt(160), [1.0], Int64.ofInt(170),
          "base-link", null, null, "robot-boot", "host-clock")]),
      "frame-aware perception refuses disconnected sensor frames");
    var tiltedModel = new RobotModel("tilted-sensor");
    var tiltedBase = tiltedModel.addLink(new Link("base", "base"));
    var tiltedMount = tiltedModel.addFrame(new Frame("tilted laser", tiltedBase,
      "tilted-laser"));
    tiltedMount.rotation = [Math.sin(0.1), 0.0, 0.0, Math.cos(0.1)];
    throws(function() FrameTree2.fromRobotModel(tiltedModel, "base"),
      "planar frame tree rejects authored sensor mounts with roll or pitch");
    sceneTruth = new PerceptionSnapshot([], [obstacles[0]], [], []);
    check(truthPerception.observe([lidar]).obstacles()[0].detection.id ==
      obstacles[0].detection.id && truthPerception.observe([]).pallets().length == 0,
      "ground-truth perception reads current scene truth independently of sensor frames");

    var envelope = new StoppingEnvelope(2.0, 0.5, 2.0);
    check(Math.abs(envelope.distanceMeters - 2.0) < 1e-9,
      "stopping envelope includes reaction distance and braking distance");
    var restrictions:Array<SafetyRestriction> = [SpeedLimited(0.4), StopRequired("aisle blocked")];
    var safety = new SafetyState(SafetyPhase.Restricted, 0.4, envelope, restrictions,
      Int64.ofInt(300), Int64.ofInt(320), "robot-boot", "host-clock");
    restrictions.pop();
    equal(safety.restrictions().length, 2, "safety state owns active restrictions");
    check(safety.speedLimitMetersPerSecond == 0.4 &&
      safety.stoppingEnvelope.distanceMeters == 2.0,
      "safety state exposes current speed limit and stopping envelope");
    throws(function() new StoppingEnvelope(1.0, 0.0, 0.0),
      "stopping envelope requires positive deceleration");

    var battery = new BatteryState("traction-pack", 0.75, 48.0, 12.0, 31.0,
      Int64.ofInt(400), Int64.ofInt(420), "battery-boot", "host-clock", 720.0);
    check(battery.chargeFraction == 0.75 && battery.currentAmps > 0.0 &&
      battery.remainingEnergyWattHours == 720.0,
      "battery state exposes charge, electrical state, and remaining energy");
    throws(function() new BatteryState("bad-pack", 1.1, 48.0, 0.0, 20.0,
      Int64.ofInt(0), Int64.ofInt(0), "battery", "host"),
      "battery state rejects invalid charge fractions");
  }

  static function testLoadSafetyPolicy():Void {
    var robot = new FakeRobot("load-safety");
    robot.jointNames = ["left-wheel", "right-wheel", "lift"];
    robot.positions = [0.0, 0.0, 0.0];
    robot.velocities = [0.0, 0.0, 0.0];
    robot.efforts = [0.0, 0.0, 0.0];
    var base = new MobileBase(robot, new DifferentialDrive(0, 1, 0.1, 0.5),
      new MotionLimits(1.0, 2.0, 2.0, 3.0), Footprint.rectangle(0.8, 0.6));
    var forks = new Forks(robot, new ForkConfig(new ForkAxisConfig("lift", 0.0, 2.0),
      new LoadLimits(1000.0, 600.0, 2.0)));
    var policy = new LoadSafetyPolicy(base, forks, new LoadSafetyConfiguration());
    var unknown = policy.state();
    check(unknown.phase == SafetyPhase.Restricted &&
      base.motionLimits.maxLinearSpeed < base.nominalMotionLimits.maxLinearSpeed &&
      switch unknown.restrictions()[0] { case LoadStateUnknown: true; case _: false; },
      "Load safety applies conservative motion limits when load state is unknown");
    var unknownFootprint:Footprint = cast unknown.footprint;
    var baseFootprint:Footprint = cast base.footprint;
    check(unknownFootprint.radius > baseFootprint.radius,
      "Unknown load state expands the reported footprint by its configured margin");

    forks.setLoadState(LoadState.empty());
    var empty = policy.refresh();
    var emptyLimits:MotionLimits = cast empty.effectiveMotionLimits;
    check(empty.phase == SafetyPhase.Normal &&
      base.motionLimits.maxLinearSpeed == base.nominalMotionLimits.maxLinearSpeed,
      "Confirmed empty forks restore nominal mobile limits");
    base.command(new Twist2(0.8, 0.0), 1.0);
    empty = policy.refresh();

    var payload = new Payload(500.0, 1.2, 0.8, 0.8, 0.6, 0.0, 0.4);
    forks.setLoadState(LoadState.carried(payload));
    var loaded = policy.refresh();
    var loadedLimits:MotionLimits = cast loaded.effectiveMotionLimits;
    var loadedFootprint:Footprint = cast loaded.footprint;
    var vertices = loadedFootprint.vertices();
    var maxX = vertices[0].x;
    for (point in vertices) maxX = Math.max(maxX, point.x);
    check(loaded.phase == SafetyPhase.Restricted &&
      loadedLimits.maxLinearSpeed < emptyLimits.maxLinearSpeed &&
      loadedLimits.maxLinearAcceleration < emptyLimits.maxLinearAcceleration &&
      loaded.stoppingEnvelope.distanceMeters > empty.stoppingEnvelope.distanceMeters &&
      maxX >= 1.19,
      "Payload mass reduces driving limits and stopping margin while expanding the footprint");

    robot.positions[2] = 1.5;
    var raised = policy.refresh();
    var raisedLimits:MotionLimits = cast raised.effectiveMotionLimits;
    var hasHeightRestriction = false;
    for (restriction in raised.restrictions()) {
      if (switch restriction {
        case ForkHeightLimited(_): true;
        case _: false;
      }) hasHeightRestriction = true;
    }
    check(raisedLimits.maxAngularSpeed < loadedLimits.maxAngularSpeed &&
      hasHeightRestriction,
      "Raised forks further reduce turning speed and report the height restriction");

    robot.positions[2] = 0.0;
    forks.setLoadState(LoadState.carried(new Payload(1200.0,
      1.0, 0.8, 0.8, 0.4)));
    var invalid = policy.refresh();
    check(invalid.phase == SafetyPhase.ProtectiveStop && base.safetyStopRequired &&
      invalid.speedLimitMetersPerSecond == 0.0,
      "Load envelope violations latch a protective stop in the mobile view");
    throws(function() base.command(new Twist2(0.1, 0.0), 0.1),
      "MobileBase blocks new motion commands while its load policy requires a stop");
    forks.setLoadState(LoadState.empty());
    var recovered = policy.refresh();
    check(recovered.phase == SafetyPhase.Normal && !base.safetyStopRequired,
      "A valid refreshed load state clears the policy stop and restores nominal limits");
    base.command(new Twist2(0.1, 0.0), 0.1);
  }

  static function testSimulatedMaterialHandlingScenario():Void {
    var model = configuredForkliftModel();
    var blueprint = RobotRuntimeCompiler.compile(model);
    var simulation = new Simulation(0.02);
    var runtime = simulation.addRobot(blueprint);
    var robot = new SimulatedRobot("material-handling", runtime, model.name,
      [for (link in model.links) link.name], [for (joint in model.joints) joint.name]);
    var base = MobileBase.fromBlueprint(robot, blueprint);
    var forks = Forks.fromBlueprint(robot, blueprint);
    forks.setLoadState(LoadState.empty());
    var loadSafety = new LoadSafetyPolicy(base, forks);
    var localization = new SimulationTruthLocalization(simulation, 0,
      "map", "link/base");
    var navigation = new Navigation(base, localization, 0.3, 0.45, 1.0);
    var plant = new DifferentialDrivePlant(simulation, 0, base);
    var runner = new SkillRunner();
    var tick = 0;
    var timestep = 0.02;
    var latestSnapshot:RobotSnapshot = plant.step(Int64.ofInt(tick++));
    localization.update(latestSnapshot);
    loadSafety.refresh();
    var unloadedLimits = base.motionLimits;
    var grid = new OccupancyGrid2(0.2, new Pose2(-1.0, -2.0), 40, 20,
      "map", OccupancyCell.Free);
    var baseFootprint = base.footprint;
    if (baseFootprint == null)
      throw "Forklift model did not provide a base footprint";
    var unloadedCostmap = new Costmap2(grid, baseFootprint.radius, true, 0.3, 1.5);
    var unloadedNavigator = new Navigator(navigation,
      new AStarPlanner(unloadedCostmap), unloadedCostmap);

    var dockApproach = new Pose2(0.4, 0.0, 0.0);
    var palletPose = new Pose2(1.4, 0.0, 0.0);
    var pickApproach = new Pose2(0.9, 0.0, 0.0);
    var placeApproach = new Pose2(2.7, 0.0, 0.0);
    var chargerPose = new Pose2(3.4, 0.0, 0.0);
    var chargerApproach = new Pose2(3.0, 0.0, 0.0);
    function detection(id:String, kind:String, pose:Pose2):Detection {
      return new Detection(id, kind, 0.98, pose, "map",
        latestSnapshot.sourceSequence, latestSnapshot.sourceTimestampNs,
        latestSnapshot.receivedTimestampNs, latestSnapshot.sourceClockId,
        latestSnapshot.receivedClockId);
    }
    // Scene truth and a deterministic load sensor make pallet interaction
    // repeatable; chassis and fork actuation still advance through SimKit.
    var groundTruth = new GroundTruthPerception(function() {
      var palletDetection = detection("pallet-17", "pallet", palletPose);
      var dockDetection = detection("pallet-staging", "dock", palletPose);
      var chargerDetection = detection("charger-1", "charger", chargerPose);
      return new PerceptionSnapshot([], [],
        [new Pallet(palletDetection, 1.2, 0.8, 0.15)],
        [new DockingTarget(dockDetection, dockApproach),
          new DockingTarget(chargerDetection, chargerApproach)]);
    });
    var scene = groundTruth.observe(latestSnapshot.sensors.toArray());
    check(scene.dockingTargets().length == 2 && scene.pallets().length == 1,
      "simulated scene supplies pallet and charger detections");
    var observeNavigation:RobotSnapshot -> PerceptionSnapshot = function(snapshot) {
      latestSnapshot = snapshot;
      localization.update(snapshot);
      return groundTruth.observe(snapshot.sensors.toArray());
    };

    var observationHook:RobotSnapshot -> Void = function(_) {};
    var controlHook:Void -> Void = function() {};
    function runSkill(skill:Skill):SkillStatus {
      var status = runner.start(skill);
      var steps = 0;
      while (status == SkillStatus.Running && steps < 1500) {
        var snapshot = plant.step(Int64.ofInt(tick++));
        latestSnapshot = snapshot;
        observationHook(snapshot);
        loadSafety.refresh();
        status = runner.update(snapshot, timestep);
        controlHook();
        steps++;
      }
      return status;
    }

    var dock = new Dock(unloadedNavigator, scene.dockingTargets()[0],
      observeNavigation);
    check(runSkill(dock) == SkillStatus.Succeeded &&
      runner.result() != null,
      "SkillRunner docks at the pallet staging pose in simulation");

    var payload = new Payload(500.0, 1.2, 0.8, 0.15, 0.6, 0.0, 0.35);
    var pickScene = groundTruth.observe(latestSnapshot.sensors.toArray());
    var pallet = pickScene.pallets()[0];
    var pick = new PickPallet(unloadedNavigator, observeNavigation, forks, pallet,
      payload, pickApproach, 0.5, 0.1, 0.45, 0.06, 0.1);
    observationHook = function(_) {
      if (runner.activeSkill() == pick && forks.loadState.payload == payload &&
          !forks.loadState.secured && forks.state().lift.position >= 0.49)
        forks.setLoadState(LoadState.carried(payload));
    };
    var pickStatus = runSkill(pick);
    observationHook = function(_) {};
    check(pickStatus == SkillStatus.Succeeded && runner.result() != null &&
      forks.loadState.secured && forks.loadState.payload == payload &&
      Math.abs(forks.state().lift.position - 0.5) < 1e-6,
      "SkillRunner picks and secures the pallet after simulated fork actuation");

    var loadedState = loadSafety.refresh();
    var loadedLimits = base.motionLimits;
    var loadedFootprint = loadedState.footprint;
    if (loadedFootprint == null)
      throw "Forklift load policy did not provide both robot footprints";
    check(loadedState.phase == SafetyPhase.Restricted &&
      loadedLimits.maxLinearSpeed < unloadedLimits.maxLinearSpeed &&
      loadedLimits.maxLinearAcceleration < unloadedLimits.maxLinearAcceleration &&
      loadedFootprint.radius > baseFootprint.radius,
      "confirmed pallet load lowers driving limits and expands the planning footprint");

    var costmap = new Costmap2(grid, loadedFootprint.radius, true, 0.3, 1.5);
    var planner = new AStarPlanner(costmap);
    var loadedNavigator = new Navigator(navigation, planner, costmap);
    var loadedGoalPose = new Pose2(2.4, 0.0, 0.0);
    var loadedGoal = new GoTo(loadedNavigator,
      new NavigationGoal(loadedGoalPose, "map", 0.12, 0.12), observeNavigation);
    var maxLoadedCommand = 0.0;
    controlHook = function() {
      maxLoadedCommand = Math.max(maxLoadedCommand,
        Math.abs(base.currentCommand().linear));
    };
    var loadedStatus = runSkill(loadedGoal);
    controlHook = function() {};
    var loadedEstimate = localization.state();
    check(loadedStatus == SkillStatus.Succeeded && runner.result() != null &&
      maxLoadedCommand > 0.05 &&
      maxLoadedCommand <= loadedLimits.maxLinearSpeed + 1e-9 &&
      loadedEstimate != null && Math.abs(loadedEstimate.pose.x - loadedGoalPose.x) <= 0.14,
      "goal-level GoTo reaches the delivery area within load-reduced speed limits");

    var place = new PlacePallet(loadedNavigator, observeNavigation, forks,
      payload, placeApproach, 0.0, 0.0, 0.45, 0.06, 0.1);
    observationHook = function(_) {
      if (runner.activeSkill() == place && forks.loadState.secured &&
          forks.state().lift.position <= 0.01)
        forks.setLoadState(LoadState.empty());
    };
    var placeStatus = runSkill(place);
    check(placeStatus == SkillStatus.Succeeded && runner.result() != null &&
      forks.loadState.observed && !forks.loadState.secured &&
      forks.loadState.payload == null,
      "SkillRunner places the pallet after simulated release confirmation");
    loadSafety.refresh();
    check(base.motionLimits.maxLinearSpeed == unloadedLimits.maxLinearSpeed &&
      !base.safetyStopRequired,
      "confirmed pallet release restores the unloaded motion limits");

    var chargingScene = groundTruth.observe(latestSnapshot.sensors.toArray());
    var chargerTarget = chargingScene.dockingTargets()[1];
    var power = new FakePower();
    power.battery = new BatteryState("traction-pack", 0.3, 48.0, 10.0, 25.0,
      latestSnapshot.sourceTimestampNs, latestSnapshot.receivedTimestampNs,
      latestSnapshot.sourceClockId, latestSnapshot.receivedClockId);
    var charge = new Charge(loadedNavigator, chargerTarget, power, 0.8,
      observeNavigation);
    var chargeStatus = runner.start(charge);
    var chargeTicks = 0;
    while (chargeStatus == SkillStatus.Running &&
        charge.dock.status() == SkillStatus.Running && chargeTicks < 1500) {
      latestSnapshot = plant.step(Int64.ofInt(tick++));
      loadSafety.refresh();
      chargeStatus = runner.update(latestSnapshot, timestep);
      chargeTicks++;
    }
    check(chargeStatus == SkillStatus.Running &&
      charge.dock.status() == SkillStatus.Succeeded && runner.activeSkill() == charge,
      "Charge docks at the detected charger and waits while the battery is low");
    power.battery = new BatteryState("traction-pack", 0.85, 48.0, -4.0, 25.0,
      latestSnapshot.sourceTimestampNs, latestSnapshot.receivedTimestampNs,
      latestSnapshot.sourceClockId, latestSnapshot.receivedClockId);
    chargeStatus = runner.update(latestSnapshot, timestep);
    check(chargeStatus == SkillStatus.Succeeded && runner.result() != null &&
      runner.activeSkill() == null,
      "Charge completes through SkillRunner when the battery reaches its target");

    robot.close();
    simulation.dispose();
  }

  static function testForkliftSkillsOnSimulationAndReplay():Void {
    var model = configuredForkliftModel();
    var blueprint = RobotRuntimeCompiler.compile(model);
    var linkNames = [for (link in model.links) link.name];
    var jointNames = [for (joint in model.joints) joint.name];
    var simulation = new Simulation(0.01);
    var recordingPath = '/tmp/robotkit-${Sys.getPid()}-forklift-skills.mcap';
    var writer = new McapRobotRecording(recordingPath, 4 * 1024 * 1024);
    var sourceRobot = new SimulatedRobot("forklift", simulation.addRobot(blueprint),
      "simulated forklift", linkNames, jointNames);
    var simulatedRobot = new RecordingRobot(sourceRobot, writer);
    var base = MobileBase.fromRobot(simulatedRobot, model);
    var localization = new WheelOdometryLocalization(base);
    var navigation = new Navigation(base, localization, 0.2, 0.2, 0.8);
    var liveFootprint = base.footprint;
    if (liveFootprint == null) throw "Forklift model did not provide a footprint";
    var liveGrid = new OccupancyGrid2(0.1, new Pose2(-5.0, -5.0),
      100, 100, "odom", OccupancyCell.Free);
    var liveCostmap = new Costmap2(liveGrid, liveFootprint.radius);
    var liveNavigator = new Navigator(navigation,
      new AStarPlanner(liveCostmap), liveCostmap);
    var observeLive:RobotSnapshot -> PerceptionSnapshot = function(snapshot) {
      localization.update(snapshot);
      return new PerceptionSnapshot();
    };
    var forks = Forks.fromRobot(simulatedRobot, model);
    simulation.step(Int64.ofInt(1));
    var configuredScan = sourceRobot.snapshot().sensors;
    check(configuredScan.length == 1 &&
      configuredScan.get(0).sensorId == "sensor/front-lidar" &&
      configuredScan.get(0).frameId == "frame/front-lidar" &&
      configuredScan.get(0).linkId == "link/base" &&
      configuredScan.get(0).mountPosition.get(0) == 0.35,
      "authored sensor frame and mount reach the simulated forklift observation");
    var path = new Path([new Pose2(0.0, 0.0, 0.0), new Pose2(0.18, 0.0, 0.0)], "odom");
    var pathSkill = new FollowPath(navigation, path,
      new NavigationGoal(path.goal(), "odom", 0.02, 0.1));
    var skillRunner = new SkillRunner();
    var liveStatus = skillRunner.start(pathSkill);
    check(liveStatus == SkillStatus.Running && skillRunner.activeSkill() == pathSkill,
      "SkillRunner starts one robot-local skill and exposes it as active");
    var conflictingSkill = new FollowPath(navigation, path);
    throws(function() skillRunner.start(conflictingSkill),
      "SkillRunner rejects a second skill while one is running");
    var ticks = 0;
    while (switch liveStatus { case Running: true; case _: false; } && ticks < 300) {
      var observation = simulatedRobot.snapshot();
      liveStatus = skillRunner.update(observation, 0.01);
      if (switch liveStatus { case Running: true; case _: false; })
        simulation.step(Int64.ofInt((ticks + 1) * 10000000));
      ticks++;
    }
    check(switch liveStatus { case Succeeded: true; case _: false; } && ticks < 300 &&
      skillRunner.activeSkill() == null && skillRunner.result() != null,
      "SkillRunner completes FollowPath and retains its terminal result");
    var liveSnapshot = simulatedRobot.snapshot();
    var liveEstimate = localization.state();
    var liveEstimateValue:robotkit.localization.LocalizationState = cast liveEstimate;
    var cancelPose = new Pose2(liveEstimateValue.pose.x + 1.0,
      liveEstimateValue.pose.y, liveEstimateValue.pose.yaw);
    var cancelPath = new Path([liveEstimateValue.pose, cancelPose], "odom");
    var cancelledSkill = new FollowPath(navigation, cancelPath);
    skillRunner.start(cancelledSkill);
    skillRunner.cancel();
    check(skillRunner.status() == SkillStatus.Cancelled &&
      skillRunner.activeSkill() == null && skillRunner.result() != null,
      "SkillRunner cancels its active skill and records a terminal result");

    // Every approach below is far outside its goal tolerance, so each skill must
    // plan and drive the forklift before it can complete.
    var liveTick = ticks + 1;
    function nextLiveTimestamp():Int64
      return Int64.fromFloat((liveTick++) * 10000000.0);
    function liveApproachReached():Bool return liveNavigator.status == NavigatorStatus.Succeeded;
    function runLive(initial:SkillStatus, update:RobotSnapshot -> SkillStatus,
        until:Void -> Bool):SkillStatus {
      var status = initial;
      var count = 0;
      while (status == SkillStatus.Running && !until() && count < 3000) {
        status = update(simulatedRobot.snapshot());
        if (status == SkillStatus.Running && !until()) simulation.step(nextLiveTimestamp());
        count++;
      }
      return status;
    }
    function livePose():Pose2 {
      var estimate:robotkit.localization.LocalizationState = cast localization.state();
      return estimate.pose;
    }
    function distance(a:Pose2, b:Pose2):Float
      return Math.sqrt((a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y));

    var dockStart = livePose();
    var dockPose = new Pose2(dockStart.x + 0.3, dockStart.y, dockStart.yaw);
    var dockDetection = new Detection("charger-dock", "dock", 0.95, dockPose,
      "odom", liveSnapshot.sourceSequence, liveSnapshot.sourceTimestampNs,
      liveSnapshot.receivedTimestampNs, liveSnapshot.sourceClockId, liveSnapshot.receivedClockId);
    var dock = new Dock(liveNavigator, new DockingTarget(dockDetection, dockPose),
      observeLive);
    var dockStatus = runLive(skillRunner.start(dock),
      function(snapshot) return skillRunner.update(snapshot, 0.01), function() return false);
    var dockEnd = livePose();
    check(dockStatus == SkillStatus.Succeeded && skillRunner.activeSkill() == null &&
      distance(dockEnd, dockStart) > 0.2 && distance(dockEnd, dockPose) <= 0.06,
      'SkillRunner runs Dock and drives the forklift to its approach (status ${Std.string(dockStatus)}, pose ${dockEnd.x},${dockEnd.y})');

    var pickStart = livePose();
    var pickPose = new Pose2(pickStart.x + 0.3, pickStart.y, pickStart.yaw);
    liveSnapshot = simulatedRobot.snapshot();
    var palletDetection = new Detection("pallet-17", "pallet", 0.98,
      new Pose2(pickPose.x + 0.5, pickPose.y, pickPose.yaw), "odom",
      liveSnapshot.sourceSequence, liveSnapshot.sourceTimestampNs,
      liveSnapshot.receivedTimestampNs, liveSnapshot.sourceClockId, liveSnapshot.receivedClockId);
    var pallet = new Pallet(palletDetection, 1.2, 0.8, 0.15);
    var payload = new Payload(500.0, 1.2, 0.8, 0.15, 0.6, 0.0, 0.35);
    var pick = new PickPallet(liveNavigator, observeLive, forks, pallet, payload,
      pickPose, 0.5, 0.1, 0.45, 0.05, 0.1);
    pick.start();
    var pickStatus = runLive(pick.status(),
      function(snapshot) return pick.update(snapshot, 0.01), liveApproachReached);
    var pickEnd = livePose();
    check(pickStatus == SkillStatus.Running && forks.loadState.payload == payload &&
      !forks.loadState.secured && distance(pickEnd, pickStart) > 0.2 &&
      distance(pickEnd, pickPose) <= 0.06,
      "PickPallet drives to its approach and then commands its simulated fork mechanism");
    simulation.step(nextLiveTimestamp());
    var forkObservation = forks.state();
    check(Math.abs(forkObservation.lift.position - 0.5) < 1e-9 &&
      Math.abs(cast(forkObservation.tilt, robotkit.material.ForkAxisState).position - 0.1) < 1e-9,
      "simulated runtime applies the fork position batch");
    forks.setLoadState(LoadState.carried(payload));
    pickStatus = pick.update(simulatedRobot.snapshot(), 0.01);
    check(switch pickStatus { case Succeeded: true; case _: false; } && pick.result() != null,
      "PickPallet completes only after secured-load confirmation");

    var travelStart = localization.state();
    var travelStartValue:robotkit.localization.LocalizationState = cast travelStart;
    var travelPose = new Pose2(travelStartValue.pose.x + 0.12,
      travelStartValue.pose.y, travelStartValue.pose.yaw);
    var travelPath = new Path([travelStartValue.pose, travelPose], "odom");
    var travel = new FollowPath(navigation, travelPath,
      new NavigationGoal(travelPose, "odom", 0.02, 0.1));
    travel.start();
    var travelStatus = travel.status();
    var travelTicks = 0;
    while (switch travelStatus { case Running: true; case _: false; } && travelTicks < 300) {
      var observation = simulatedRobot.snapshot();
      travelStatus = travel.update(observation, 0.01);
      if (switch travelStatus { case Running: true; case _: false; })
        simulation.step(nextLiveTimestamp());
      travelTicks++;
    }
    check(switch travelStatus { case Succeeded: true; case _: false; } && travelTicks < 300,
      "FollowPath drives the loaded forklift to a second location");

    var placeStart = livePose();
    var placePose = new Pose2(placeStart.x + 0.3, placeStart.y, placeStart.yaw);
    var place = new PlacePallet(liveNavigator, observeLive, forks, payload,
      placePose, 0.0, 0.0, 0.35);
    place.start();
    var placeStatus = runLive(place.status(),
      function(snapshot) return place.update(snapshot, 0.01), liveApproachReached);
    var placeEnd = livePose();
    check(switch placeStatus { case Running: true; case _: false; } &&
      forks.loadState.secured && forks.loadState.payload == payload &&
      distance(placeEnd, placeStart) > 0.2 && distance(placeEnd, placePose) <= 0.09,
      "PlacePallet drives to its drop pose, commands the forks, and waits for release");
    simulation.step(nextLiveTimestamp());
    forks.setLoadState(LoadState.empty());
    placeStatus = place.update(simulatedRobot.snapshot(), 0.01);
    check(switch placeStatus { case Succeeded: true; case _: false; },
      "PlacePallet completes only after empty-load confirmation");

    var chargeStart = livePose();
    var chargePose = new Pose2(chargeStart.x + 0.3, chargeStart.y, chargeStart.yaw);
    liveSnapshot = simulatedRobot.snapshot();
    var chargeDetection = new Detection("charger-1", "charger", 0.95, chargePose,
      "odom", liveSnapshot.sourceSequence, liveSnapshot.sourceTimestampNs,
      liveSnapshot.receivedTimestampNs, liveSnapshot.sourceClockId, liveSnapshot.receivedClockId);
    var livePower = new FakePower();
    livePower.battery = new BatteryState("traction-pack", 0.6, 48.0, -4.0, 25.0,
      liveSnapshot.sourceTimestampNs, liveSnapshot.receivedTimestampNs,
      liveSnapshot.sourceClockId, liveSnapshot.receivedClockId);
    var charge = new Charge(liveNavigator,
      new DockingTarget(chargeDetection, chargePose), livePower, 0.8, observeLive);
    charge.start();
    var chargeStatus = runLive(charge.status(),
      function(snapshot) return charge.update(snapshot, 0.01), liveApproachReached);
    var chargeEnd = livePose();
    check(chargeStatus == SkillStatus.Running &&
      charge.dock.status() == SkillStatus.Succeeded &&
      distance(chargeEnd, chargeStart) > 0.2 && distance(chargeEnd, chargePose) <= 0.06,
      "Charge docks at the charger and then waits while the battery is below its target");
    liveSnapshot = simulatedRobot.snapshot();
    livePower.battery = new BatteryState("traction-pack", 0.85, 48.0, -4.0, 25.0,
      liveSnapshot.sourceTimestampNs, liveSnapshot.receivedTimestampNs,
      liveSnapshot.sourceClockId, liveSnapshot.receivedClockId);
    check(switch charge.update(liveSnapshot, 0.01) { case Succeeded: true; case _: false; },
      "Charge completes when the battery reaches its target fraction");

    check(simulatedRobot.recordingError == null,
      "recording adapter preserves an error-free forklift run");
    check(Int64.compare(writer.status().dropped, Int64.ofInt(0)) == 0,
      "MCAP recording accepts the complete forklift run");
    writer.close();
    var recording = McapRecordingReader.load(recordingPath);
    check(recording.commands.length > 3 && recording.snapshots.length > 3,
      "MCAP stores forklift target batches and the observation stream");

    var replayDescription = new RobotDescription("forklift", "recorded forklift",
      linkNames, jointNames);
    var replayCapabilities = new RobotCapabilities("forklift", 5, true, true, true, false);

    var goalReplay = new ReplayRobot("forklift", recording,
      replayDescription, replayCapabilities);
    var goalReplayBase = MobileBase.fromBlueprint(goalReplay, blueprint);
    var goalReplayLocalization = new WheelOdometryLocalization(goalReplayBase);
    var goalReplayNavigation = new Navigation(goalReplayBase,
      goalReplayLocalization, 0.2, 0.2, 0.8);
    var goalReplayGrid = new OccupancyGrid2(0.1, new Pose2(-5.0, -5.0),
      100, 100, "odom", OccupancyCell.Free);
    var goalReplayCostmap = new Costmap2(goalReplayGrid, 0.25);
    var goalReplayNavigator = new Navigator(goalReplayNavigation,
      new AStarPlanner(goalReplayCostmap), goalReplayCostmap);
    goalReplayLocalization.update(goalReplay.snapshot());
    var goalReplaySkill = new GoTo(goalReplayNavigator,
      new NavigationGoal(path.goal(), "odom", 0.02, 0.1), function(snapshot) {
        goalReplayLocalization.update(snapshot);
        return new PerceptionSnapshot();
      });
    var goalReplayRunner = new SkillRunner();
    var goalReplayStatus = goalReplayRunner.start(goalReplaySkill);
    while (goalReplayStatus == SkillStatus.Running && goalReplay.advance())
      goalReplayStatus = goalReplayRunner.update(goalReplay.snapshot(), 0.01);
    check(goalReplayStatus == SkillStatus.Succeeded && goalReplayRunner.result() != null,
      "goal-level GoTo completes against recorded ReplayRobot observations");
    goalReplay.close();

    var replay = new ReplayRobot("forklift", recording, replayDescription, replayCapabilities);
    var replayBase = MobileBase.fromBlueprint(replay, blueprint);
    var replayLocalization = new WheelOdometryLocalization(replayBase);
    var replayNavigation = new Navigation(replayBase, replayLocalization, 0.2, 0.2, 0.8);
    var replayFootprint = replayBase.footprint;
    if (replayFootprint == null) throw "Recorded forklift model did not provide a footprint";
    var replayGrid = new OccupancyGrid2(0.1, new Pose2(-5.0, -5.0),
      100, 100, "odom", OccupancyCell.Free);
    var replayCostmap = new Costmap2(replayGrid, replayFootprint.radius);
    var replayNavigator = new Navigator(replayNavigation,
      new AStarPlanner(replayCostmap), replayCostmap);
    var observeReplay:RobotSnapshot -> PerceptionSnapshot = function(snapshot) {
      replayLocalization.update(snapshot);
      return new PerceptionSnapshot();
    };
    var replayFollowPath = new FollowPath(replayNavigation, path,
      new NavigationGoal(path.goal(), "odom", 0.02, 0.1));
    var replaySkillRunner = new SkillRunner();
    var replayStatus = replaySkillRunner.start(replayFollowPath);
    replayStatus = replaySkillRunner.update(replay.snapshot(), 0.01);
    while (switch replayStatus { case Running: true; case _: false; } && replay.advance())
      replayStatus = replaySkillRunner.update(replay.snapshot(), 0.01);
    check(switch replayStatus { case Succeeded: true; case _: false; } &&
      replaySkillRunner.activeSkill() == null && replaySkillRunner.result() != null,
      "SkillRunner completes FollowPath against the recorded ReplayRobot observations");
    function replayApproachReached():Bool
      return replayNavigator.status == NavigatorStatus.Succeeded;
    // Mirrors runLive: one update per distinct recorded observation.
    function runReplay(initial:SkillStatus, update:RobotSnapshot -> SkillStatus,
        until:Void -> Bool):SkillStatus {
      var status = initial;
      if (status == SkillStatus.Running && !until()) status = update(replay.snapshot());
      while (status == SkillStatus.Running && !until() && advanceReplaySample(replay))
        status = update(replay.snapshot());
      return status;
    }
    function replayPose():Pose2 {
      var estimate:robotkit.localization.LocalizationState = cast replayLocalization.state();
      return estimate.pose;
    }
    var replaySnapshot = replay.snapshot();
    var replayDockStart = replayPose();
    var replayDockPose = new Pose2(replayDockStart.x + 0.3,
      replayDockStart.y, replayDockStart.yaw);
    var replayDockDetection = new Detection("charger-dock", "dock", 0.95, replayDockPose,
      "odom", replaySnapshot.sourceSequence, replaySnapshot.sourceTimestampNs,
      replaySnapshot.receivedTimestampNs, replaySnapshot.sourceClockId, replaySnapshot.receivedClockId);
    var replayDock = new Dock(replayNavigator,
      new DockingTarget(replayDockDetection, replayDockPose), observeReplay);
    var replayDockStatus = runReplay(replaySkillRunner.start(replayDock),
      function(snapshot) return replaySkillRunner.update(snapshot, 0.01),
      function() return false);
    check(replayDockStatus == SkillStatus.Succeeded &&
      replaySkillRunner.activeSkill() == null &&
      distance(replayPose(), dockEnd) < 1e-9,
      "SkillRunner reproduces the recorded Dock approach through ReplayRobot");
    var replayForks = Forks.fromBlueprint(replay, blueprint);
    var replayPickStart = replayPose();
    var replayPickPose = new Pose2(replayPickStart.x + 0.3,
      replayPickStart.y, replayPickStart.yaw);
    replaySnapshot = replay.snapshot();
    var replayPallet = new Pallet(new Detection("pallet-17", "pallet", 0.98,
      new Pose2(replayPickPose.x + 0.5, replayPickPose.y, replayPickPose.yaw), "odom",
      replaySnapshot.sourceSequence, replaySnapshot.sourceTimestampNs,
      replaySnapshot.receivedTimestampNs, replaySnapshot.sourceClockId,
      replaySnapshot.receivedClockId), 1.2, 0.8, 0.15);
    var replayPick = new PickPallet(replayNavigator, observeReplay, replayForks,
      replayPallet, payload, replayPickPose, 0.5, 0.1, 0.45, 0.05, 0.1);
    replayPick.start();
    var replayPickStatus = runReplay(replayPick.status(),
      function(snapshot) return replayPick.update(snapshot, 0.01), replayApproachReached);
    check(replayPickStatus == SkillStatus.Running &&
      replayForks.loadState.payload == payload && distance(replayPose(), pickEnd) < 1e-9,
      "PickPallet reproduces the recorded approach before commanding the forks");
    check(advanceReplaySample(replay),
      "replay consumes the recorded fork-actuation tick");
    replayForks.setLoadState(LoadState.carried(payload));
    check(switch replayPick.update(replay.snapshot(), 0.01) {
      case Succeeded: true;
      case _: false;
    }, "PickPallet confirms the same load during replay");

    var replayTravelStart = replayLocalization.state();
    var replayTravelStartValue:robotkit.localization.LocalizationState = cast replayTravelStart;
    var replayTravelPose = new Pose2(travelPose.x, travelPose.y, travelPose.yaw);
    var replayTravelPath = new Path([replayTravelStartValue.pose, replayTravelPose], "odom");
    var replayTravel = new FollowPath(replayNavigation, replayTravelPath,
      new NavigationGoal(replayTravelPose, "odom", 0.02, 0.1));
    replayTravel.start();
    var replayTravelStatus = replayTravel.update(replay.snapshot(), 0.01);
    var replayTravelTicks = 0;
    while (switch replayTravelStatus { case Running: true; case _: false; } && advanceReplaySample(replay) &&
        replayTravelTicks < 300) {
      replaySnapshot = replay.snapshot();
      replayTravelStatus = replayTravel.update(replaySnapshot, 0.01);
      replayTravelTicks++;
    }
    check(switch replayTravelStatus { case Succeeded: true; case _: false; },
      "FollowPath reaches the second recorded location through ReplayRobot");
    var replayPlaceStart = replayPose();
    var replayPlacePose = new Pose2(replayPlaceStart.x + 0.3,
      replayPlaceStart.y, replayPlaceStart.yaw);
    var replayPlace = new PlacePallet(replayNavigator, observeReplay, replayForks,
      payload, replayPlacePose, 0.0, 0.0, 0.35);
    replayPlace.start();
    var replayPlaceStatus = runReplay(replayPlace.status(),
      function(snapshot) return replayPlace.update(snapshot, 0.01), replayApproachReached);
    check(switch replayPlaceStatus { case Running: true; case _: false; } &&
      replayForks.loadState.secured && distance(replayPose(), placeEnd) < 1e-9,
      "PlacePallet replays the same approach and fork command and awaits release");
    check(advanceReplaySample(replay),
      "replay consumes the recorded fork-release tick");
    replayForks.setLoadState(LoadState.empty());
    check(switch replayPlace.update(replay.snapshot(), 0.01) {
      case Succeeded: true;
      case _: false;
    }, "PlacePallet confirms release during replay");

    var replayChargeStart = replayPose();
    var replayChargePose = new Pose2(replayChargeStart.x + 0.3,
      replayChargeStart.y, replayChargeStart.yaw);
    replaySnapshot = replay.snapshot();
    var replayChargeDetection = new Detection("charger-1", "charger", 0.95,
      replayChargePose, "odom", replaySnapshot.sourceSequence,
      replaySnapshot.sourceTimestampNs, replaySnapshot.receivedTimestampNs,
      replaySnapshot.sourceClockId, replaySnapshot.receivedClockId);
    var replayPower = new FakePower();
    replayPower.battery = new BatteryState("traction-pack", 0.6, 48.0, -4.0, 25.0,
      replaySnapshot.sourceTimestampNs, replaySnapshot.receivedTimestampNs,
      replaySnapshot.sourceClockId, replaySnapshot.receivedClockId);
    var replayCharge = new Charge(replayNavigator,
      new DockingTarget(replayChargeDetection, replayChargePose), replayPower, 0.8,
      observeReplay);
    replayCharge.start();
    var replayChargeStatus = runReplay(replayCharge.status(),
      function(snapshot) return replayCharge.update(snapshot, 0.01), replayApproachReached);
    check(replayChargeStatus == SkillStatus.Running &&
      replayCharge.dock.status() == SkillStatus.Succeeded &&
      distance(replayPose(), chargeEnd) < 1e-9,
      "Charge reproduces the recorded docking and waits for the battery target");
    replaySnapshot = replay.snapshot();
    replayPower.battery = new BatteryState("traction-pack", 0.85, 48.0, -4.0, 25.0,
      replaySnapshot.sourceTimestampNs, replaySnapshot.receivedTimestampNs,
      replaySnapshot.sourceClockId, replaySnapshot.receivedClockId);
    check(switch replayCharge.update(replaySnapshot, 0.01) {
      case Succeeded: true;
      case _: false;
    }, "Charge completes deterministically against ReplayRobot");
    var generated = replay.generatedCommands.commands;
    var generatedForkBatches = 0;
    for (command in generated) switch command {
      case JointTargets(targets, _):
        if (targets.length == 3 && targets[0].joint == 2) generatedForkBatches++;
      case _:
    }
    check(generated.length >= 3 && generatedForkBatches == 2,
      "ReplayRobot captures pick and place fork commands without altering source history");
    var forkliftCommandsMatch = generated.length == recording.commands.length;
    var firstMismatch = -1;
    for (commandIndex in 0...recording.commands.length) {
      if (commandIndex >= generated.length) break;
      switch recording.commands[commandIndex] {
        case JointTargets(sourceTargets, _):
          switch generated[commandIndex] {
            case JointTargets(replayTargets, _):
              if (sourceTargets.length != replayTargets.length) {
                forkliftCommandsMatch = false;
                if (firstMismatch < 0) firstMismatch = commandIndex;
              }
              else for (targetIndex in 0...sourceTargets.length) {
                var sourceTarget = sourceTargets[targetIndex];
                var replayTarget = replayTargets[targetIndex];
                if (sourceTarget.joint != replayTarget.joint ||
                    Std.string(sourceTarget.mode) != Std.string(replayTarget.mode) ||
                    Math.abs(sourceTarget.target - replayTarget.target) > 1e-9) {
                  forkliftCommandsMatch = false;
                  if (firstMismatch < 0) firstMismatch = commandIndex;
                }
              }
            case _:
              forkliftCommandsMatch = false;
              if (firstMismatch < 0) firstMismatch = commandIndex;
          }
        case _:
          forkliftCommandsMatch = false;
          if (firstMismatch < 0) firstMismatch = commandIndex;
      }
    }
    if (firstMismatch < 0 && generated.length != recording.commands.length)
      firstMismatch = generated.length < recording.commands.length ? generated.length : recording.commands.length;
    if (!forkliftCommandsMatch) {
      Sys.println('forklift command batches: live=${recording.commands.length}, replay=${generated.length}, first mismatch=$firstMismatch');
      var start = firstMismatch > 2 ? firstMismatch - 2 : 0;
      var end = Std.int(Math.min(firstMismatch + 3,
        Math.max(recording.commands.length, generated.length)));
      for (commandIndex in start...end) {
        var liveCommand = commandIndex < recording.commands.length
          ? commandSummary(recording.commands[commandIndex]) : "<missing>";
        var replayCommand = commandIndex < generated.length
          ? commandSummary(generated[commandIndex]) : "<missing>";
        Sys.println('command $commandIndex live=[$liveCommand] replay=[$replayCommand]');
      }
    }
    check(forkliftCommandsMatch,
      "forklift MCAP replay reproduces every recorded joint target batch");
    replay.close();
    simulatedRobot.close();
    simulation.dispose();
    if (sys.FileSystem.exists(recordingPath)) sys.FileSystem.deleteFile(recordingPath);
    if (sys.FileSystem.exists(recordingPath + ".incomplete.status"))
      sys.FileSystem.deleteFile(recordingPath + ".incomplete.status");
  }

  static function configuredForkliftModel():RobotModel {
    var model = new RobotModel("authored-forklift");
    var base = model.addLink(new Link("base", "link/base"));
    var leftWheel = model.addLink(new Link("left wheel", "link/left-wheel"));
    var rightWheel = model.addLink(new Link("right wheel", "link/right-wheel"));
    var mast = model.addLink(new Link("mast", "link/mast"));
    var carriage = model.addLink(new Link("fork carriage", "link/carriage"));
    var forks = model.addLink(new Link("forks", "link/forks"));
    function addJoint(id:String, name:String, type:JointType, child:robotkit.model.Link,
        lower:Float, upper:Float, velocity:Float):Void {
      var joint = new Joint(name, type, base, child, id);
      joint.limits = new JointLimits(lower, upper, velocity, 1000.0);
      model.addJoint(joint);
    }
    addJoint("joint/left-wheel", "left-wheel", JointType.Continuous,
      leftWheel, -1000.0, 1000.0, 20.0);
    addJoint("joint/right-wheel", "right-wheel", JointType.Continuous,
      rightWheel, -1000.0, 1000.0, 20.0);
    addJoint("joint/lift", "lift", JointType.Prismatic, mast, 0.0, 1.5, 1000.0);
    addJoint("joint/tilt", "tilt", JointType.Revolute, carriage, -0.5, 0.5, 1000.0);
    addJoint("joint/spread", "spread", JointType.Prismatic, forks, 0.0, 0.8, 1000.0);
    model.mobileBase = new RobotMobileConfiguration(
      RobotDriveConfiguration.Differential("joint/left-wheel", "joint/right-wheel",
        0.1, 0.5), 0.5, 1.0, 2.0, 10.0, 2.0, 1.0);
    model.forkMechanism = new RobotForkConfiguration("joint/lift",
      1000.0, 700.0, 1.5, "joint/tilt", "joint/spread");
    var lidarFrame = model.addFrame(new Frame("front lidar mount", base,
      "frame/front-lidar"));
    lidarFrame.position = [0.35, 0.0, 0.3];
    var lidar = model.addSensor(new Sensor("front lidar", "lidar", 10.0,
      "sensor/front-lidar"));
    lidar.frame = lidarFrame;
    lidar.rayCount = 32;
    lidar.maxRange = 8.0;
    return model;
  }

  static function testMcapRoundTrip():Void {
    var path = '/tmp/robotkit-${Sys.getPid()}-roundtrip.mcap';
    var writer = new McapRobotRecording(path, 1024 * 1024);
    var sensor = new SensorFrame("lidar/front", "lidar", "frame/front",
      Int64.parseString("9007199254740993"), Int64.parseString("9223372036854775000"),
      [1.25, 2.5], Int64.parseString("9223372036854775001"), "link/base",
      [0.1, 0.2, 0.3], [0.0, 0.0, 0.0, 1.0], "robot-a.reset-2", "host.monotonic");
    var cameraBytes = haxe.io.Bytes.alloc(6);
    for (index in 0...6) cameraBytes.set(index, index + 1);
    var camera = new SensorFrame("camera/front", "camera", "frame/camera-front",
      Int64.ofInt(9), Int64.ofInt(150), [], Int64.ofInt(160), "link/base",
      [0.2, 0.0, 0.8], [0.0, 0.0, 0.0, 1.0], "robot-a.reset-2",
      "host.monotonic", new CameraImage(2, 1, "rgb8", cameraBytes));
    var first = new RobotSnapshot("robot-a", Int64.parseString("9007199254740995"),
      Int64.parseString("9223372036854775000"), [0.5], [0.25], [0.125], 1, 0,
      Int64.parseString("9223372036854775002"), [sensor, camera], "robot-a.reset-2", "host.monotonic",
      RobotKitRuntimeConstants.RK_SAFETY_EMERGENCY_STOP);
    var second = new RobotSnapshot("robot-b", Int64.ofInt(3), Int64.ofInt(10),
      [0.75], [], [], 1, 0, Int64.ofInt(20), [], "robot-b.boot-1", "host.monotonic");
    writer.recordCommand(RobotCommand.JointTargets([
      robotkit.world.JointTarget.position(0, 0.75),
      robotkit.world.JointTarget.velocity(1, -0.25),
      robotkit.world.JointTarget.effort(2, 3.5)
    ], Int64.parseString("9223372036854775003")), "robot-a");
    writer.recordSnapshot(first);
    writer.recordSensor("robot-a", sensor);
    writer.recordFault(new RobotFault("robot-b", 42, "recorded fault", false));
    var robots = new Map<RobotId,RobotSnapshot>(); robots.set(first.id,first); robots.set(second.id,second);
    writer.recordWorld(new robotkit.world.WorldSnapshot(7,2,Int64.ofInt(0),robots,Int64.ofInt(50)));
    writer.recordEvent(RobotWorldEvent.RobotChanged("robot-a"));
    writer.close();

    var loaded = McapRecordingReader.load(path);
    equal(loaded.entries.length, 6, "MCAP reload preserves every event type");
    switch loaded.commands[0] {
      case JointTargets(targets, expiry):
        equal(targets.length, 3, "MCAP preserves batched target count");
        equal(Std.string(targets[1].mode), Std.string(robotkit.world.JointTargetMode.Velocity),
          "MCAP preserves velocity mode");
        equal(targets[2].target, 3.5, "MCAP preserves effort target value");
        equal(expiry, Int64.parseString("9223372036854775003"),
          "MCAP preserves command deadline integer");
      case _:
        check(false, "MCAP preserves command batch variant");
    }
    equal(loaded.snapshots[0].sourceTimestampNs, first.sourceTimestampNs, "MCAP preserves exact 64-bit timestamps");
    equal(loaded.snapshots[0].sourceSequence, first.sourceSequence, "MCAP preserves exact 64-bit sequences");
    equal(loaded.snapshots[0].sourceClockId, "robot-a.reset-2", "MCAP preserves reset clock identity");
    equal(loaded.snapshots[0].safety, first.safety, "MCAP preserves robot safety state");
    equal(loaded.snapshots[0].sensors.get(0).frameId, "frame/front", "MCAP preserves sensor frame identity");
    equal(loaded.snapshots[0].sensors.get(0).mountPosition.get(2), 0.3, "MCAP preserves sensor mount");
    var recordedCamera:CameraImage = cast loaded.snapshots[0].sensors.get(1).image;
    check(recordedCamera != null && recordedCamera.width == 2 &&
      recordedCamera.encoding == "rgb8" && recordedCamera.bytes().get(0) == 1 &&
      recordedCamera.bytes().get(5) == 6,
      "MCAP preserves camera dimensions, encoding, and pixels");
    equal(loaded.worlds[0].robotIds().length, 2, "MCAP preserves multiple robots");
    for (index in 0...loaded.entries.length) equal(loaded.entries[index].ordinal, Int64.ofInt(index), "MCAP uses arrival ordinals");
    var replay = new ReplayRobot("robot-a", loaded);
    var behavior = new WorldBehaviorRunner(new HoldJointBehavior(0, 0.25));
    equal(behavior.update(replay), 1, "reloaded observations use unchanged behavior APIs");
    replay.close();
    if (Sys.getEnv("ROBOTKIT_KEEP_MCAP") == null) sys.FileSystem.deleteFile(path);
    else Sys.println('RobotKit MCAP fixture: $path');
    var unsupported = haxe.io.Bytes.ofString('{"version":4,"ordinal":"0","robotId":"","sourceSequence":"0","sourceTimestampNs":"0","sourceClockId":"x","type":"worldEvent","payload":{"kind":"changed","robotId":"x"}}');
    var legacyV2 = RobotRecordingCodec.decode(haxe.io.Bytes.ofString(
      '{"version":2,"ordinal":"0","recordingTimestampNs":"1","robotId":"x","sourceSequence":"0","sourceTimestampNs":"0","sourceClockId":"clock","type":"worldEvent","payload":{"kind":"changed","robotId":"x"}}'));
    equal(legacyV2.schemaVersion, 2, "recording reader accepts schema v2");
    throws(function() RobotRecordingCodec.decode(unsupported), "unsupported recording schema rejected");
    throws(function() RobotRecordingCodec.decode(haxe.io.Bytes.ofString("{")), "malformed recording payload rejected");
    var invalidMount:Dynamic = haxe.Json.parse(RobotRecordingCodec.encode(loaded.entries[2]).toString());
    Reflect.setField(Reflect.field(invalidMount, "payload"), "mountPosition", [0.0, 0.0]);
    throws(function() RobotRecordingCodec.decode(haxe.io.Bytes.ofString(haxe.Json.stringify(invalidMount))),
      "recording rejects malformed sensor mount shapes");
    var invalidNumber:Dynamic = haxe.Json.parse(RobotRecordingCodec.encode(loaded.entries[0]).toString());
    var commandTargets:Array<Dynamic> = cast Reflect.field(
      Reflect.field(invalidNumber, "payload"), "targets");
    Reflect.setField(commandTargets[0], "target", 1e400);
    throws(function() RobotRecordingCodec.decode(haxe.io.Bytes.ofString(haxe.Json.stringify(invalidNumber))),
      "recording rejects non-finite numeric payloads");
  }

  static function testExternalSensorRuntime():Void {
    var model = new RobotModel("external-sensor robot");
    var base = model.addLink(new Link("base", "external/base"));
    var cameraMount = model.addFrame(new Frame("front camera", base, "external/front-optical"));
    cameraMount.position = [0.15, 0.02, 0.7];
    var camera = model.addSensor(new Sensor("front camera", "camera", 30.0, "sensor/front-camera"));
    camera.frame = cameraMount;
    var antennaMount = model.addFrame(new Frame("gnss antenna", base, "external/antenna"));
    antennaMount.position = [-0.2, 0.0, 1.1];
    var gnss = model.addSensor(new Sensor("gnss", "gnss_pose", 10.0, "sensor/gnss"));
    gnss.frame = antennaMount;
    var blueprint = RobotRuntimeCompiler.compile(model);
    equal(blueprint.externalSensorLayout().length, 2,
      "model compilation keeps camera and GNSS sensors outside the native runtime");
    equal(blueprint.nativeSensorLayout().length, 3,
      "a robot with only external sensors keeps the native runtime's default slots");
    check(blueprint.sensorById("sensor/gnss") != null && blueprint.sensorById("imu") != null,
      "compiled identity resolves both default native and authored external sensors");

    var simulation = new Simulation();
    var runtime = simulation.addRobot(blueprint);
    var robot = new SimulatedRobot("external-sensors", runtime, model.name, [base.name], []);
    simulation.step(Int64.ofInt(1));
    simulation.step(Int64.ofInt(2));
    var nativeCount = robot.snapshot().sensors.length;
    var pixels = haxe.io.Bytes.alloc(6);
    for (index in 0...pixels.length) pixels.set(index, index + 31);
    runtime.publishCameraFrame("sensor/front-camera", new CameraImage(2, 1, "rgb8", pixels),
      Int64.ofInt(1), Int64.ofInt(100), "camera.boot-3");
    pixels.set(0, 255);
    runtime.publishSensorFrame("sensor/gnss", [48.1, 11.5, 0.3], Int64.ofInt(1),
      Int64.ofInt(100), "gnss.receiver");
    var snapshot = robot.snapshot();
    equal(snapshot.sensors.length, nativeCount + 2,
      "external frames merge with native sensors without a physics step");
    var cameraFrame:Null<SensorFrame> = null;
    var gnssFrame:Null<SensorFrame> = null;
    for (sensor in snapshot.sensors.toArray()) {
      if (sensor.sensorId == "sensor/front-camera") cameraFrame = sensor;
      if (sensor.sensorId == "sensor/gnss") gnssFrame = sensor;
    }
    if (cameraFrame == null || gnssFrame == null) throw "external frames are missing";
    var cameraSample:SensorFrame = cast cameraFrame;
    var gnssSample:SensorFrame = cast gnssFrame;
    check(cameraSample.frameId == "external/front-optical" &&
      cameraSample.linkId == "external/base" && cameraSample.mountPosition.get(2) == 0.7 &&
      cameraSample.sourceClockId == "camera.boot-3",
      "the runtime stamps the authored camera mount and keeps the source clock");
    var image = cameraSample.image;
    check(image != null && image.width == 2 && image.bytes().get(0) == 31,
      "a published camera image is owned by the frame");
    check(gnssSample.frameId == "external/antenna" && gnssSample.values.get(0) == 48.1 &&
      gnssSample.image == null, "a GNSS fix carries its values at the authored antenna mount");
    throws(function() runtime.publishCameraFrame("sensor/missing",
      new CameraImage(1, 1, "jpeg", haxe.io.Bytes.ofString("x")), Int64.ofInt(2),
      Int64.ofInt(101)), "externally sourced");
    throws(function() runtime.publishSensorFrame("imu", [0.0, 0.0, 0.0], Int64.ofInt(2),
      Int64.ofInt(101), "imu"), "externally sourced");
    throws(function() runtime.publishCameraFrame("sensor/front-camera",
      new CameraImage(1, 1, "jpeg", haxe.io.Bytes.ofString("x")), Int64.ofInt(1),
      Int64.ofInt(101)), "stale sequence");
    throws(function() runtime.publishSensorFrame("sensor/gnss", [48.1, 11.5], Int64.ofInt(2),
      Int64.ofInt(101), "gnss.receiver"), "latitude, longitude, and yaw");
    throws(function() runtime.publishSensorFrame("sensor/front-camera", [], Int64.ofInt(2),
      Int64.ofInt(101), "camera.boot-3"), "requires an image");
    throws(function() new CameraImage(2, 2, "rgb8", haxe.io.Bytes.alloc(5)),
      "does not match its dimensions");
    runtime.publishCameraFrame("sensor/front-camera",
      new CameraImage(1, 1, "jpeg", haxe.io.Bytes.ofString("y")), Int64.ofInt(2),
      Int64.ofInt(102), "camera.boot-3");
    var updated = 0;
    for (sensor in robot.snapshot().sensors.toArray())
      if (sensor.sensorId == "sensor/front-camera") updated = Int64.toInt(sensor.sequence);
    equal(updated, 2, "the simulated robot observes an image update without a physics step");
    robot.close();
    simulation.dispose();
  }

  static function testCameraFrameProtocol():Void {
    var pixels = haxe.io.Bytes.alloc(12);
    for (index in 0...pixels.length) pixels.set(index, index + 1);
    var metadata = new CameraFrame(Int64.ofInt(42), "camera/front", "camera",
      "robot/base/camera", Int64.ofInt(8), Int64.ofInt(100), Int64.ofInt(105),
      2, 2, PixelFormat.RGB8, null, "base", [0.2, 0.0, 0.8],
      [0.0, 0.0, 0.0, 1.0], "robot.boot-1", "sensor.monotonic");
    var outgoing = RobotProtocol.cameraFrame(metadata, pixels, Int64.ofInt(7),
      Int64.ofInt(8), Int64.ofInt(110));
    equal(outgoing.attachments.length, 1,
      "camera frame stores pixels in a separate RobotFrame attachment");
    equal(metadata.pixels.length, 0,
      "camera frame encoding does not mutate caller-owned metadata");
    var decoded = RobotProtocol.decodeCameraFrame(RobotFrame.decode(outgoing.encode()));
    equal(decoded.frame.robotId, Int64.ofInt(42), "camera protocol preserves robot identity");
    equal(decoded.frame.sensorId, "camera/front", "camera protocol preserves sensor identity");
    equal(decoded.frame.frameId, "robot/base/camera", "camera protocol preserves frame identity");
    equal(decoded.frame.sequence, Int64.ofInt(8), "camera protocol preserves sensor sequence");
    equal(decoded.frame.sourceClockId, "robot.boot-1", "camera protocol preserves source clock");
    equal(decoded.frame.receivedClockId, "sensor.monotonic", "camera protocol preserves receipt clock");
    equal(decoded.frame.mountPosition[0], 0.2, "camera protocol preserves sensor mount");
    var received = decoded.pixels();
    equal(received.length, pixels.length, "camera protocol resolves the complete attachment");
    for (index in 0...pixels.length)
      equal(received.get(index), index + 1, "camera protocol preserves pixel byte $index");
    outgoing.attachments[0].set(0, 255);
    equal(decoded.pixels().get(0), 1,
      "decoded camera pixels own a copy independent of the frame attachment");
    var remote = new RemoteRobot("camera-remote");
    remote.onCamera(decoded);
    var remoteSensors = remote.sensors();
    equal(remoteSensors.length, 1, "remote robot publishes received camera as a sensor frame");
    equal(remoteSensors[0].sensorId, "camera/front", "remote camera keeps sensor identity");
    equal(remoteSensors[0].values.length, 0, "remote camera does not invent scalar readings");
    var remoteImage = remoteSensors[0].image;
    check(remoteImage != null, "remote sensor frame includes the camera image");
    if (remoteImage != null) {
      equal(remoteImage.width, 2, "remote camera retains image dimensions");
      equal(remoteImage.bytes().get(5), 6, "remote camera retains image bytes");
    }
    equal(remoteSensors[0].sourceClockId, "robot.boot-1",
      "remote camera retains the source clock");
    equal(remoteSensors[0].receivedClockId, "robotkit.monotonic",
      "remote camera stamps receipt time with the local monotonic clock");

    var padded = haxe.io.Bytes.alloc(16);
    for (index in 0...12) padded.set(index + 2, index + 21);
    var offsetMetadata = new CameraFrame(Int64.ofInt(42), "camera/offset", "camera",
      "camera-frame", Int64.ofInt(1), Int64.ofInt(2), Int64.ofInt(3), 2, 2,
      PixelFormat.RGB8, new BufferRef(0, 2, 12), "base");
    var offsetFrame = new RobotFrame(RobotMessageType.CameraFrame,
      MessagePack.encode(offsetMetadata), 0, [padded]);
    equal(RobotProtocol.decodeCameraFrame(offsetFrame).pixels().get(0), 21,
      "camera protocol resolves a valid byte range inside an attachment");

    throws(function() RobotProtocol.cameraFrame(new CameraFrame(Int64.ofInt(42),
      "camera/bad", "camera", "camera-frame", Int64.ofInt(1), Int64.ofInt(2),
      Int64.ofInt(3), 2, 2, PixelFormat.RGB8), haxe.io.Bytes.alloc(11)),
      "camera protocol rejects pixel lengths inconsistent with dimensions");
    var outside = new CameraFrame(Int64.ofInt(42), "camera/outside", "camera",
      "camera-frame", Int64.ofInt(1), Int64.ofInt(2), Int64.ofInt(3), 2, 2,
      PixelFormat.RGB8, new BufferRef(0, 8, 12));
    throws(function() RobotProtocol.decodeCameraFrame(new RobotFrame(
      RobotMessageType.CameraFrame, MessagePack.encode(outside), 0, [haxe.io.Bytes.alloc(16)])),
      "camera protocol rejects buffer ranges outside their attachments");
    var missing = new CameraFrame(Int64.ofInt(42), "camera/missing", "camera",
      "camera-frame", Int64.ofInt(1), Int64.ofInt(2), Int64.ofInt(3), 2, 2,
      PixelFormat.RGB8, new BufferRef(1, 0, 12));
    throws(function() RobotProtocol.decodeCameraFrame(new RobotFrame(
      RobotMessageType.CameraFrame, MessagePack.encode(missing), 0, [haxe.io.Bytes.alloc(12)])),
      "camera protocol rejects references to missing attachments");
    var wrongLength = new CameraFrame(Int64.ofInt(42), "camera/wrong-size", "camera",
      "camera-frame", Int64.ofInt(1), Int64.ofInt(2), Int64.ofInt(3), 2, 2,
      PixelFormat.RGB8, new BufferRef(0, 0, 11));
    throws(function() RobotProtocol.decodeCameraFrame(new RobotFrame(
      RobotMessageType.CameraFrame, MessagePack.encode(wrongLength), 0,
      [haxe.io.Bytes.alloc(11)])),
      "camera protocol rejects malformed raw image dimensions on receipt");
  }

  static function testConfiguredSensors():Void {
    var model = new RobotModel("configured");
    var base = model.addLink(new Link("base", "link/stable"));
    var mount = model.addFrame(new robotkit.model.Frame("mount", base, "frame/stable"));
    mount.position = [0.5, 0.0, 0.0];
    mount.rotation = [0.0, 0.0, 0.7071067811865476, 0.7071067811865476];
    var scan = model.addSensor(new robotkit.model.Sensor("scan", "lidar", 10, "sensor/scan"));
    scan.frame = mount;
    scan.rayCount = 16;
    scan.maxRange = 3.0;
    var noisy = model.addSensor(new robotkit.model.Sensor("noisy", "lidar", 10, "sensor/noisy"));
    noisy.frame = mount; noisy.rayCount = 16; noisy.maxRange = 3.0;
    noisy.noiseStddev = 0.01; noisy.noiseSeed = 42;
    var imu = model.addSensor(new robotkit.model.Sensor("imu", "imu", 0, "sensor/imu"));
    imu.frame = mount;
    var partial = model.addSensor(new robotkit.model.Sensor("partial", "lidar", 0, "sensor/partial"));
    partial.frame = mount;
    partial.rayCount = 3;
    partial.maxRange = 3.0;
    partial.startAngleRadians = -Math.PI * 0.5;
    partial.fieldOfViewRadians = Math.PI;
    var blueprint = RobotRuntimeCompiler.compile(model);
    mount.position[0] = 100.0;
    mount.name = "renamed";
    scan.name = "renamed scan";
    model.sensors.reverse();
    var simulation = new Simulation();
    var runtime = simulation.addRobot(blueprint);
    var robot = new SimulatedRobot("configured", runtime, "configured", ["base"], []);
    simulation.spawnBox([0.5, 2.0, 0.0], [0.25, 0.25, 0.25]);
    simulation.step(Int64.ofInt(1));
    var first = robot.snapshot();
    var firstScan = first.sensors.get(0);
    equal(firstScan.sensorId, "sensor/scan", "compiled sensor identity survives model rename and reorder");
    equal(firstScan.frameId, "frame/stable", "configured frame identity reaches measurement");
    equal(firstScan.linkId, "link/stable", "mount link identity reaches measurement");
    equal(firstScan.mountPosition.get(0), 0.5, "mount detached from editable model");
    equal(firstScan.values.length, 16, "configured resolution reaches native scanner");
    check(Math.abs(firstScan.values.get(0) - 1.75) < 0.000001, "mounted scanner rotates and translates rays");
    equal(firstScan.values.get(8), 3.0, "configured maximum range reaches native scanner");
    // The IMU has no sample on the first tick because it needs a velocity
    // derivative, so the partial scan is the third published frame here.
    var partialScan = first.sensors.get(2);
    equal(partialScan.values.length, 3, "partial field of view keeps configured ray count");
    equal(partialScan.values.get(0), 3.0, "partial scan starts at configured bearing");
    check(Math.abs(partialScan.values.get(1) - 1.75) < 0.000001,
      "partial scan distributes rays across configured angular coverage");
    equal(partialScan.values.get(2), 3.0, "partial scan includes its final bearing");
    var perception = LidarObstaclePerception.fromBlueprint(blueprint,
      "sensor/partial", 0.1);
    throws(function() LidarObstaclePerception.fromBlueprint(blueprint,
      "sensor/missing", 0.1),
      "LiDAR perception rejects unknown authored sensor IDs");
    var observations = perception.observe([partialScan]);
    equal(observations.obstacles().length, 1, "compiled LiDAR settings construct matching perception");
    check(Math.abs(observations.obstacles()[0].detection.pose.x - 1.75) < 0.000001,
      "perception ray angles match simulated scan angles");
    var noisyValue = first.sensors.get(1).values.get(0);
    check(noisyValue != firstScan.values.get(0), "configured noise changes measurement");
    simulation.step(Int64.ofInt(2));
    var second = robot.snapshot();
    equal(second.sensors.get(0).sequence, firstScan.sequence, "slow sensor sequence held between acquisitions");
    equal(second.sensors.get(0).receivedTimestampNs, firstScan.receivedTimestampNs, "cached sensor receipt does not become fresh");
    equal(second.sensors.get(0).sourceTimestampNs, firstScan.sourceTimestampNs, "cached sensor source time preserved");
    equal(second.sensors.get(2).values.get(5), 9.81, "configured mounted IMU is measured");
    for (_ in 0...8) simulation.step(Int64.ofInt(3));
    equal(robot.snapshot().sensors.get(0).sequence, Int64.ofInt(2), "10 Hz scan updates on source-clock schedule");
    var recording = new RobotRecording();
    recording.recordSnapshot(second);
    var replay = new ReplayRobot("configured", recording);
    equal(replay.snapshot().sensors.get(0).frameId, "frame/stable", "recording retains frame identity");
    equal(replay.snapshot().sensors.get(0).mountPosition.get(0), 0.5, "recording retains mount metadata");
    simulation.reset();
    simulation.step(Int64.ofInt(1));
    equal(robot.snapshot().sensors.get(1).values.get(0), noisyValue, "reset repeats seeded noise deterministically");
    robot.close(); simulation.dispose(); replay.close();
    scan.rayCount = 65;
    mount.rotation = [0.0, 0.0, 0.0, 0.0];
    noisy.updateRate = -1.0;
    var diagnostics = RobotRuntimeCompiler.validate(model);
    check(hasDiagnostic(diagnostics, "RK_SENSOR_SCAN"), "oversized scans rejected before native lowering");
    scan.rayCount = 16;
    partial.fieldOfViewRadians = Math.PI * 2.1;
    check(hasDiagnostic(RobotRuntimeCompiler.validate(model), "RK_SENSOR_SCAN"),
      "LiDAR angular coverage beyond one revolution is rejected");
    partial.fieldOfViewRadians = Math.PI;
    check(hasDiagnostic(diagnostics, "RK_FRAME_POSE"), "non-unit mount rotations rejected");
    check(hasDiagnostic(diagnostics, "RK_SENSOR_RATE"), "negative sample rates rejected");
  }

  static function testSerialRobotUnavailableDevice():Void {
    var model = new RobotModel("serial probe");
    model.addLink(new Link("base", "base"));
    var robot:Null<SerialRobot> = null;
    var failed = false;
    try robot = new SerialRobot("serial-probe", model,
      '/dev/robotkit-missing-${Sys.getPid()}',
      "000102030405060708090a0b0c0d0e0f", 1e-6) catch (_:Dynamic) failed = true;
    if (robot != null) robot.close();
    check(failed, "serial adapter reports an unavailable device path through Haxe FFI");
  }

  static function testSensorResetPublication():Void {
    var model = new RobotModel("sensor-reset");
    model.addLink(new Link("base"));
    var simulation = new Simulation();
    var runtime = simulation.addRobot(RobotRuntimeCompiler.compile(model));
    var robot = new SimulatedRobot("sensor-reset", runtime, "sensor-reset", ["base"], []);
    simulation.spawnBox([2.0, 0.0, 0.0], [0.25, 0.25, 0.25]);
    simulation.step(Int64.ofInt(100));
    var before = robot.snapshot();
    equal(before.sensors.get(1).values.get(0), 1.75, "adapter exposes scene ray distance");
    simulation.reset();
    simulation.teleportRobot(0, [1.0, 0.0, 0.0]);
    simulation.step(Int64.ofInt(100));
    var after = robot.snapshot();
    equal(after.sourceSequence, before.sourceSequence, "reset repeats source sequence");
    equal(after.sensors.get(1).values.get(0), 0.75,
      "adapter refreshes sensors even when reset repeats sequence");
    equal(before.sensors.get(1).values.get(0), 1.75, "old sensor snapshot survives reset");
    simulation.step(Int64.ofInt(200));
    equal(robot.snapshot().sensors.length, 3, "second sample publishes measured IMU");
    var rejected = false;
    try robot.submit(RobotCommand.JointTargets([
      robotkit.world.JointTarget.position(0, 0.0)
    ], Int64.ofInt(100)))
    catch (_:Dynamic) rejected = true;
    check(rejected, "simulation never silently ignores an unsupported deadline");
    robot.close();
    simulation.dispose();
  }

  static function testSensorAndClockContracts():Void {
    var input = [0.0, 0.0, 0.0, 0.0, 0.0, 9.81];
    var sourceFrames = [
      new SensorFrame("enc", "joint_encoder", "base", Int64.ofInt(2), Int64.ofInt(3), [], Int64.ofInt(4)),
      new SensorFrame("imu", "imu", "base", Int64.ofInt(2), Int64.ofInt(3), input, Int64.ofInt(4)),
      new SensorFrame("lidar", "lidar", "base", Int64.ofInt(2), Int64.ofInt(3), [for (_ in 0...8) 2.0], Int64.ofInt(4))];
    var snapshot = new robotkit.runtime.RobotSnapshot(Int64.ofInt(1), Int64.ofInt(2),
      Int64.ofInt(3), 0, 0, 1, 0, [], [], [], Int64.ofInt(4), sourceFrames);
    input[5] = -1.0;
    equal(snapshot.sensors.get(1).values.get(5), 9.81, "runtime sensor payload owns a copy");
    var copy = snapshot.withRobotId(Int64.ofInt(5));
    var frames = robotkit.world.RobotSensorFrames.fromRuntimeSnapshot(copy);
    equal(frames.length, 3, "runtime projection preserves valid measurements");
    var mutable = frames[1].values.toArray();
    mutable[5] = 0.0;
    equal(frames[1].values.get(5), 9.81, "sensor frames expose defensive copies");
    equal(frames[1].receivedTimestampNs, Int64.ofInt(4), "projection preserves receive time");
    var absent = new robotkit.runtime.RobotSnapshot(Int64.ofInt(1), Int64.ofInt(0),
      Int64.ofInt(999), 0, 0, 1, 0, [], [], []);
    equal(absent.receivedTimestampNs, Int64.ofInt(0), "unknown receipt is not source time");
    equal(robotkit.world.RobotSensorFrames.fromRuntimeSnapshot(absent).length, 0,
      "endpoints without sensors do not fabricate IMU or LiDAR");
    var intents = new robotkit.behavior.IntentBuffer();
    intents.publish(new robotkit.behavior.JointTargetIntent(0, 1, 0.5, Int64.ofInt(100)));
    check(intents.current(Int64.ofInt(99)) != null, "intent valid before local deadline");
    equal(intents.current(Int64.ofInt(100)), null, "intent expires exactly at local deadline");
  }

  static function testStableModelIdentity():Void {
    var model = new RobotModel("identity-arm");
    var base = model.addLink(new Link("base", "link/base"));
    var elbow = model.addLink(new Link("elbow", "link/elbow"));
    var tool = model.addLink(new Link("tool", "link/tool"));
    var shoulder = model.addJoint(new Joint("shoulder", JointType.Revolute,
      base, elbow, "joint/shoulder"));
    var wrist = model.addJoint(new Joint("wrist", JointType.Revolute,
      elbow, tool, "joint/wrist"));
    var imu = model.addSensor(new robotkit.model.Sensor("imu", "imu", 100, "sensor/imu"));
    model.addSensor(new robotkit.model.Sensor("lidar", "lidar", 10, "sensor/lidar"));
    var frame = model.addFrame(new robotkit.model.Frame("base frame", base, "frame/base"));
    model.addFrame(new robotkit.model.Frame("tool frame", tool, "frame/tool"));
    var original = RobotRuntimeCompiler.compile(model, 1);
    var originalIds = original.identity;
    check(originalIds != null, "compiler supplies semantic identity mappings");
    if (originalIds == null) throw "missing compiled identity";

    base.name = "renamed base";
    shoulder.name = "renamed shoulder";
    imu.name = "renamed imu";
    model.links.reverse();
    model.joints.reverse();
    model.sensors.reverse();
    frame.name = "renamed frame";
    model.frames.reverse();
    var edited = RobotRuntimeCompiler.compile(model, 2);
    var editedIds = edited.identity;
    if (editedIds == null) throw "missing edited identity";
    equal(originalIds.linkIndex(base.id), 0, "old blueprint retains link mapping");
    equal(editedIds.linkIndex(base.id), 2, "new blueprint maps stable ID after reorder");
    equal(editedIds.jointIndex(shoulder.id), 1, "joint ID survives rename and reorder");
    equal(editedIds.sensorIndex(imu.id), 1, "sensor ID survives rename and reorder");
    equal(originalIds.sensorId(0), imu.id, "old sensor mapping is detached from model");
    equal(edited.frameCount, 2, "frame count reaches blueprint");
    equal(originalIds.frameId(0), frame.id, "old frame mapping is detached");
    equal(editedIds.frameIndex(frame.id), 1, "frame ID survives rename and reorder");
    equal(editedIds.frameLinkId(1), base.id, "frame attachment survives link reorder");
    equal(editedIds.frameId(-1), null, "invalid frame index rejected");
    equal(editedIds.linkId(edited.joints[0].parentLink), elbow.id,
      "native parent index resolves to semantic parent after reorder");
    equal(editedIds.jointId(edited.joints[0].joint), wrist.id,
      "native joint index resolves to semantic joint");
    equal(editedIds.linkIndex("missing"), -1, "unknown ID has no runtime index");
    equal(editedIds.jointId(-1), null, "invalid index has no semantic ID");

    var legacy = new Link("legacy");
    legacy.name = "renamed legacy";
    equal(legacy.id, "legacy", "legacy identity is assigned only at construction");
    model.addLink(new Link("different name", base.id));
    model.addJoint(new Joint("different joint", JointType.Fixed, base, tool, shoulder.id));
    model.addSensor(new robotkit.model.Sensor("different sensor", "imu", 0, imu.id));
    model.addSensor(new robotkit.model.Sensor("empty ID", "imu", 0, ""));
    model.addFrame(new robotkit.model.Frame("duplicate", base, frame.id));
    model.addFrame(new robotkit.model.Frame("foreign", new Link("foreign"), ""));
    var diagnostics = RobotRuntimeCompiler.validate(model);
    check(hasDiagnostic(diagnostics, "RK_LINK_ID_DUPLICATE"), "duplicate link IDs rejected");
    check(hasDiagnostic(diagnostics, "RK_JOINT_ID_DUPLICATE"), "duplicate joint IDs rejected");
    check(hasDiagnostic(diagnostics, "RK_SENSOR_ID_DUPLICATE"), "duplicate sensor IDs rejected");
    check(hasDiagnostic(diagnostics, "RK_SENSOR_ID"), "empty semantic IDs rejected");
    check(hasDiagnostic(diagnostics, "RK_FRAME_ID_DUPLICATE"), "duplicate frame IDs rejected");
    check(hasDiagnostic(diagnostics, "RK_FRAME_ID"), "empty frame IDs rejected");
    check(hasDiagnostic(diagnostics, "RK_FRAME_LINK"), "foreign frame links rejected");
  }

  static function testCompilerDiagnosticsAndTopology():Void {
    var model = new RobotModel("diagnostic-arm");
    var base = model.addLink(new Link("base"));
    var tool = model.addLink(new Link("tool"));
    var shoulder = model.addJoint(new Joint("shoulder", JointType.Revolute, base, tool));
    shoulder.limits.lower = -1.0;
    shoulder.limits.upper = 1.0;
    var blueprint = RobotRuntimeCompiler.compile(model);
    equal(blueprint.jointCount, 1, "compiler lowers a valid topology");
    equal(blueprint.linkCount, 2, "compiler preserves link count");

    var invalid = new RobotModel("invalid-arm");
    var root = invalid.addLink(new Link("root"));
    var branch = invalid.addLink(new Link("branch"));
    invalid.addLink(new Link("branch"));
    var cycle = invalid.addJoint(new Joint("cycle", JointType.Revolute, root, branch));
    cycle.limits.lower = 2.0;
    cycle.limits.upper = -2.0;
    invalid.addJoint(new Joint("cycle", JointType.Floating, branch, root));
    var diagnostics = RobotRuntimeCompiler.validate(invalid);
    check(hasDiagnostic(diagnostics, "RK_LINK_DUPLICATE"),
      "compiler reports duplicate link names");
    check(hasDiagnostic(diagnostics, "RK_JOINT_DUPLICATE"),
      "compiler reports duplicate joint names");
    check(hasDiagnostic(diagnostics, "RK_LIMITS"),
      "compiler reports invalid joint limits");
    check(hasDiagnostic(diagnostics, "RK_JOINT_UNSUPPORTED"),
      "compiler reports unsupported joint types");
    check(hasDiagnostic(diagnostics, "RK_TOPOLOGY_CYCLE"),
      "compiler reports topology cycles");
    check(hasDiagnostic(diagnostics, "RK_TOPOLOGY_DISCONNECTED"),
      "compiler reports links disconnected from the root");
    var forest = new RobotModel("forest");
    forest.addLink(new Link("left"));
    forest.addLink(new Link("right"));
    check(hasDiagnostic(RobotRuntimeCompiler.validate(forest), "RK_TOPOLOGY_ROOT"),
      "compiler requires exactly one topology root");
    var threw = false;
    try {
      RobotRuntimeCompiler.compile(invalid);
    } catch (error:RobotCompileException) {
      threw = true;
      check(error.diagnostics.length >= 5,
        "compile exception retains all structured diagnostics");
      check(error.diagnostics[0].path != null && error.diagnostics[0].code != null,
        "compile diagnostics expose paths and stable codes");
    }
    check(threw, "invalid topology refuses native lowering");
  }

  static function testWorldHostComposition():Void {
    var model = new RobotModel("headless-arm");
    model.addLink(new Link("base"));
    var host = new WorldHost();
    var robot = host.addSimulatedRobot("headless-arm", model);
    var snapshot = host.step(Int64.ofInt(3000));
    check(snapshot.robot(robot.id()) != null,
      "worldd composes one world and one shared simulation");
    host.close();
  }

  static function hasDiagnostic(diagnostics:Array<robotkit.runtime.RobotCompileDiagnostic>,
      code:String):Bool {
    for (diagnostic in diagnostics)
      if (diagnostic.code == code) return true;
    return false;
  }

  static function testAttachDetachAndIdentity():Void {
    var world = new RobotWorld();
    var robot = new FakeRobot("warehouse/forklift-17");
    world.attach(robot);
    equal(world.robot("warehouse/forklift-17"), robot, "attach registers logical ID");
    equal(world.snapshot().robotIds()[0], "warehouse/forklift-17", "snapshot keeps logical ID");
    equal(world.detach("missing"), null, "missing detach is harmless");
    equal(world.detach(robot.id()), robot, "detach returns ownership");
    equal(world.robot(robot.id()), null, "detach unregisters robot");
    check(!robot.closed, "detach does not close returned robot");
    robot.emitChange();
    equal(world.snapshot().sequence, 2, "detached robot no longer changes world");
    world.close();
    check(!robot.closed, "world does not close detached robot");
  }

  static function testSequenceAndTopology():Void {
    var world = new RobotWorld();
    var events = 0;
    var subscription = world.subscribe(function(_) events++);
    var left = new FakeRobot("left");
    var right = new FakeRobot("right");
    equal(world.snapshot().sequence, 0, "new world sequence");
    equal(world.snapshot().topologyRevision, 0, "new topology revision");
    world.attach(left);
    equal(world.snapshot().sequence, 1, "attach advances sequence");
    equal(world.snapshot().topologyRevision, 1, "attach advances topology");
    left.emitChange();
    equal(world.pendingEventCount(), 1, "adapter changes wait in the owner event queue");
    equal(world.pump(), 1, "owner pump applies one queued adapter change");
    equal(world.pendingEventCount(), 0, "owner pump drains adapter changes");
    equal(world.snapshot().sequence, 2, "state change advances sequence");
    equal(world.snapshot().topologyRevision, 1, "state change preserves topology");
    equal(events, 2, "world subscriptions run on the owner pump");
    equal(world.health().ready, 1, "world health summarizes attached robots");
    world.attach(right);
    equal(world.snapshot().sequence, 3, "second attach advances sequence");
    equal(world.snapshot().topologyRevision, 2, "second attach advances topology");
    world.detach(left.id());
    equal(world.snapshot().sequence, 4, "detach advances sequence");
    equal(world.snapshot().topologyRevision, 3, "detach advances topology");
    subscription.dispose();
    world.close();
  }

  static function testImmutableSnapshots():Void {
    var world = new RobotWorld();
    var robot = new FakeRobot("arm");
    robot.positions = [1.0, 2.0];
    world.attach(robot);
    var first = world.snapshot();
    robot.positions[0] = 9.0;
    robot.emitChange();
    var second = world.snapshot();
    var firstRobot = first.robot("arm");
    var secondRobot = second.robot("arm");
    check(firstRobot != null && firstRobot.positions.get(0) == 1.0, "old snapshot owns copied arrays");
    check(secondRobot != null && secondRobot.positions.get(0) == 9.0, "new snapshot observes new state");
    first.robots().resize(0);
    check(second.robot("arm") != null, "snapshot maps are independent");
    world.close();
  }

  static function testCrossThreadEventQueue():Void {
    var world = new RobotWorld();
    var doneMutex = new Mutex();
    var done = false;
    var rejected = false;
    Thread.create(function() {
      try {
        world.robotIds();
      } catch (_:Dynamic) {
        rejected = true;
      }
      world.enqueue(RobotWorldEvent.RobotChanged("owner-test"));
      doneMutex.acquire();
      done = true;
      doneMutex.release();
    });
    var completed = false;
    for (_ in 0...500) {
      doneMutex.acquire();
      completed = done;
      doneMutex.release();
      if (completed) break;
      Sys.sleep(0.01);
    }
    check(completed, "worker thread completes owner-boundary probe");
    check(rejected, "RobotWorld rejects direct non-owner map access");
    equal(world.pendingEventCount(), 1, "worker thread feeds changes through the event queue");
    world.pump();
    equal(world.pendingEventCount(), 0, "owner applies queued cross-thread events");
    world.close();
  }

  static function testForwardingAndLifecycle():Void {
    var world = new RobotWorld();
    var robot = new FakeRobot("arm");
    world.attach(robot);
    var command = RobotCommand.JointTargets([
      robotkit.world.JointTarget.position(3, 1.25)
    ], Int64.ofInt(99));
    world.submit("arm", command);
    equal(Std.string(robot.lastCommand), Std.string(command), "command forwarded to selected robot");
    world.stop("arm", StopMode.Emergency);
    equal(Std.string(robot.lastStop), Std.string(StopMode.Emergency), "stop forwarded to selected robot");
    world.resetSafety("arm");
    equal(robot.resetCount, 1, "safety reset acknowledgement forwarded to selected robot");
    throws(function() world.attach(new FakeRobot("arm")), "duplicate logical ID rejected");
    world.close();
    check(robot.closed, "world closes attached robot it owns");
    equal(robot.listener, null, "world clears listener before close");
    world.close();
    equal(robot.closeCount, 1, "world close is idempotent");
    throws(function() world.submit("arm", command), "closed world rejects commands");
  }

  static function testRecordingEventLog():Void {
    var world = new RobotWorld();
    var recording = new RobotRecording();
    var subscription = recording.attach(world);
    var robot = new FakeRobot("recorded-arm");
    world.attach(robot);
    equal(recording.events.length, 0,
      "recording observes world events only on the owner pump");
    world.pump();
    equal(recording.events.length, 1, "recording captures attachment events");
    switch recording.events[0] {
      case WorldEvent(RobotAttached(id)):
        equal(id, "recorded-arm", "recording preserves attached robot identity");
      case _:
        check(false, "recording first event is attachment");
    }
    robot.emitChange();
    world.pump();
    equal(recording.events.length, 2, "recording captures queued adapter changes");
    var snapshot = world.snapshot().robot("recorded-arm");
    check(snapshot != null, "recording test has a robot snapshot");
    recording.recordCommand(RobotCommand.JointTargets([
      robotkit.world.JointTarget.position(0, 0.5)
    ], null));
    recording.recordSnapshot(cast snapshot);
    recording.recordFault(new RobotFault("fault-1", 7, "test fault", false));
    recording.recordWorld(world.snapshot());
    equal(recording.events.length, 6, "recording keeps one ordered typed event stream");
    check(recording.commands.length == 1 && recording.snapshots.length == 1 &&
      recording.faults.length == 1 && recording.worlds.length == 1,
      "recording retains typed indexes alongside the event stream");
    subscription.dispose();
    world.detach(robot.id());
    world.pump();
    equal(recording.events.length, 6, "disposed recording stops receiving world events");
    world.close();
  }

  static function testMixedSimulatedAndRemoteWorld():Void {
    var blueprint = new RobotRuntimeBlueprint(1, 2, 3);
    blueprint.addJoint(new RobotRuntimeJointBlueprint(
      0,
      RobotKitRuntimeConstants.RK_RUNTIME_JOINT_REVOLUTE,
      0,
      1,
      -3.14,
      3.14,
      100.0
    ));
    blueprint.addJoint(new RobotRuntimeJointBlueprint(
      1,
      RobotKitRuntimeConstants.RK_RUNTIME_JOINT_REVOLUTE,
      1,
      2,
      -3.14,
      3.14,
      100.0
    ));
    var simulation = new Simulation();
    var first = new SimulatedRobot(
      "sim-a",
      simulation.addRobot(blueprint),
      "simulated A",
      ["base", "tool", "wrist"],
      ["shoulder", "wrist"]
    );
    var second = new SimulatedRobot(
      "sim-b",
      simulation.addRobot(blueprint),
      "simulated B",
      ["base", "tool", "wrist"],
      ["shoulder", "wrist"]
    );
    var remote = new RemoteRobot("remote-c");
    var world = new RobotWorld();
    world.attach(first);
    world.attach(second);
    world.attach(remote);

    var duplicateRejected = false;
    try world.submit("sim-a", RobotCommand.JointTargets([
      robotkit.world.JointTarget.position(0, 0.1),
      robotkit.world.JointTarget.effort(0, 2.0)
    ], null)) catch (_:Dynamic) duplicateRejected = true;
    check(duplicateRejected, "simulated adapter rejects an ambiguous atomic batch");
    world.submit("sim-a", RobotCommand.JointTargets([
      robotkit.world.JointTarget.position(0, 0.4),
      robotkit.world.JointTarget.position(1, -0.2)
    ], null));
    world.submit("sim-b", RobotCommand.JointTargets([
      robotkit.world.JointTarget.position(0, -0.3)
    ], null));
    simulation.step(Int64.ofInt(1000));

    var value = world.snapshot();
    var firstState = value.robot("sim-a");
    var secondState = value.robot("sim-b");
    var remoteState = value.robot("remote-c");
    check(firstState != null, "mixed world contains first simulated robot");
    check(secondState != null, "mixed world contains second simulated robot");
    check(remoteState != null, "mixed world contains remote robot");
    var firstValue:RobotSnapshot = cast firstState;
    var secondValue:RobotSnapshot = cast secondState;
    var remoteValue:RobotSnapshot = cast remoteState;
    check(Math.abs(firstValue.positions.get(0) - 0.4) < 0.000000001,
      "first simulated command routed independently");
    check(Math.abs(firstValue.positions.get(1) + 0.2) < 0.000000001,
      "simulated adapter applies both targets in one batch");
    check(Math.abs(secondValue.positions.get(0) + 0.3) < 0.000000001,
      "second simulated command routed independently");
    equal(firstValue.sourceTimestampNs, Int64.ofInt(10000000), "simulation clock reaches first robot");
    equal(secondValue.sourceTimestampNs, Int64.ofInt(10000000), "simulation clock reaches second robot");
    check(Int64.compare(firstValue.receivedTimestampNs, Int64.ofInt(1000)) > 0,
      "runtime receive clock is not simulation tick argument");
    check(Int64.compare(value.receivedTimestampNs, secondValue.receivedTimestampNs) >= 0,
      "world stamps its own publication after runtime acceptance");
    equal(value.sourceTimestampNs, Int64.ofInt(0), "mixed source clocks are not aggregated");
    equal(firstValue.sensors.length, 2,
      "first sample primes IMU derivative and publishes encoders plus LiDAR");
    equal(firstValue.sensors.get(0).sourceTimestampNs, Int64.ofInt(10000000),
      "sensor source clock matches robot source clock");
    equal(firstValue.sensors.get(1).frameId, "base_link",
      "sensor frame identity is explicit");
    var recording = new RobotRecording();
    recording.recordSnapshot(firstValue);
    recording.recordSnapshot(secondValue);
    recording.recordWorld(value);
    var replay = new ReplayRobot("sim-a", recording);
    var replayWorld = new RobotWorld();
    replayWorld.attach(replay);
    var replayInitial = replayWorld.snapshot().robot("sim-a");
    check(replayInitial != null, "replay world publishes a robot snapshot");
    var replayInitialValue:RobotSnapshot = cast replayInitial;
    equal(replayInitialValue.sourceTimestampNs, Int64.ofInt(10000000),
      "replay preserves source timestamps");
    check(replay.advance(), "replay advances through the same snapshot boundary");
    var replayNext = replayWorld.snapshot().robot("sim-a");
    var replayNextValue:RobotSnapshot = cast replayNext;
    check(replayNextValue != null && replayNextValue.sensors.length == 2,
      "replay preserves sensor frames");
    var behavior = new WorldBehaviorRunner(new HoldJointBehavior(0, 0.25));
    equal(behavior.update(first), 1, "world behavior emits a transport-neutral command");
    equal(behavior.update(first), 0, "world behavior does not repeat an unchanged snapshot");
    var replayBehavior = new WorldBehaviorRunner(new HoldJointBehavior(0, 0.25));
    equal(replayBehavior.update(replay), 1, "same behavior can consume replay state");
    replayWorld.close();
    equal(remoteValue.sourceSequence, Int64.ofInt(0), "remote state remains independently sourced");
    var remoteStatus = remote.status();
    check(switch remoteStatus {
      case RobotStatus.Disconnected: true;
      case _: false;
    }, "unconnected remote remains disconnected");

    world.close();
    simulation.step(Int64.ofInt(2000));
    equal(simulation.stepIndex(), Int64.ofInt(2), "RobotWorld does not own shared simulation");
    simulation.dispose();
  }

  static function testRuntimeUsesCompiledJointRate():Void {
    var model = new RobotModel("rate-limited-arm");
    var base = model.addLink(new Link("base"));
    var tool = model.addLink(new Link("tool"));
    var joint = model.addJoint(new Joint("shoulder", JointType.Revolute, base, tool));
    joint.limits.lower = -1.0;
    joint.limits.upper = 1.0;
    joint.limits.velocity = 2.0;
    joint.drive = new Actuator("shoulder-motor", 100.0, 1.0);
    var blueprint = RobotRuntimeCompiler.compile(model);
    equal(blueprint.joints[0].maxRate, 1.0,
      "runtime compiler combines joint and actuator rate limits");

    var simulation = new Simulation(0.1);
    var runtime = simulation.addRobot(blueprint);
    runtime.submitPosition(0, 0.8, 1);
    simulation.step(Int64.ofInt(100));
    check(Math.abs(runtime.snapshot().q.get(0) - 0.1) < 0.000000001,
      "runtime applies the compiled rate limit on the first shared tick");
    simulation.step(Int64.ofInt(200));
    check(Math.abs(runtime.snapshot().q.get(0) - 0.2) < 0.000000001,
      "runtime keeps advancing the same target on later shared ticks");
    simulation.dispose();
  }

  static function check(value:Bool, message:String):Void {
    assertions++;
    if (!value) throw 'assertion failed: $message';
  }

  static function commandSummary(command:RobotCommand):String return switch command {
    case JointTargets(targets, _): [for (target in targets)
      '${target.joint}:${Std.string(target.mode)}:${target.target}'].join(",");
    case TrajectoryChunk(chunk): 'trajectory:${chunk.points.length}';
  };

  static function advanceReplaySample(replay:ReplayRobot):Bool {
    var current = replay.snapshot();
    while (replay.advance()) {
      var next = replay.snapshot();
      if (Int64.compare(next.sourceSequence, current.sourceSequence) != 0 ||
          Int64.compare(next.sourceTimestampNs, current.sourceTimestampNs) != 0 ||
          next.sourceClockId != current.sourceClockId)
        return true;
    }
    return false;
  }

  static function equal(actual:Dynamic, expected:Dynamic, message:String):Void check(
    actual == expected,
    '$message (expected ${Std.string(expected)}, got ${Std.string(actual)})'
  );

  static function throws(action:Void -> Void, message:String):Void {
    var didThrow = false;
    try action() catch (_:Dynamic) didThrow = true;
    check(didThrow, message);
  }
}

private class FakeRobot implements Robot {
  final logicalId:RobotId;
  public var positions:Array<Float> = [0.0];
  public var velocities:Array<Float> = [0.0];
  public var efforts:Array<Float> = [0.0];
  public var jointNames:Array<String> = [];
  public var listener:Null < RobotId -> Void > = null;
  public var lastCommand:Null<RobotCommand> = null;
  public var lastStop:Null<StopMode> = null;
  public var resetCount = 0;
  public var closed = false;
  public var closeCount = 0;

  public function new(id:RobotId) logicalId = id;
  public function id():RobotId return logicalId;
  public function status():RobotStatus return Ready;
  public function description():RobotDescription return new RobotDescription(
    logicalId, logicalId, [], jointNames);
  public function capabilities():RobotCapabilities return new RobotCapabilities(
    logicalId,
    positions.length,
    true,
    true,
    true,
    false
  );
  public function snapshot():RobotSnapshot return new RobotSnapshot(
    logicalId,
    Int64.ofInt(1),
    Int64.ofInt(10),
    positions,
    velocities,
    efforts,
    1,
    0
  );
  public function sensors():Array<SensorFrame> return [];
  public function fault():Null < RobotFault > return null;
  public function submit(command:RobotCommand):Void lastCommand = command;
  public function stop(mode:StopMode):Void lastStop = mode;
  public function resetSafety():Void resetCount++;
  public function setChangeListener(value:Null < RobotId -> Void >):Void listener = value;
  public function emitChange():Void if (listener != null) listener(logicalId);
  public function close():Void {
    closed = true;
    closeCount++;
  }
}

private class FakePower implements Power {
  public var battery:Null<BatteryState> = null;

  public function new() {}

  public function batteryState():Null<BatteryState> return battery;
}

private class FixedLocalization implements Localization {
  var current:Null<LocalizationState>;
  public var updateCount:Int = 0;

  public function new(state:LocalizationState) current = state;

  public function update(snapshot:RobotSnapshot):LocalizationState {
    if (current == null) throw "Fixed test localization has no state";
    updateCount++;
    return cast current;
  }

  public function state():Null<LocalizationState> return current;

  public function reset(?pose:Pose2):Void current = null;
}

/** Test adapter for simulating temporary loss and recovery of localization quality. */
private class QualityOverrideLocalization implements Localization {
  public final source:Localization;
  var hasQualityOverride = false;
  var qualityOverride:LocalizationQuality = Good;

  public function new(source:Localization) {
    if (source == null) throw "Quality override requires a localization source";
    this.source = source;
  }

  public function setQualityOverride(value:LocalizationQuality):Void {
    hasQualityOverride = true;
    qualityOverride = value;
  }

  public function update(snapshot:RobotSnapshot):LocalizationState {
    return applyOverride(source.update(snapshot));
  }

  public function state():Null<LocalizationState> {
    var estimate:Null<LocalizationState> = source.state();
    if (estimate == null) return null;
    return applyOverride(estimate);
  }

  function applyOverride(value:LocalizationState):LocalizationState {
    var quality = hasQualityOverride ? qualityOverride : value.quality;
    return new LocalizationState(value.sequence, value.pose, value.referenceFrame,
      value.bodyFrame, value.covariance, quality, value.sourceTimestampNs,
      value.receivedTimestampNs, value.sourceClockId, value.receivedClockId);
  }

  public function reset(?pose:Pose2):Void {
    hasQualityOverride = false;
    source.reset(pose);
  }
}

private class PortFiducialDetector implements FiducialDetector {
  final observations:Array<FiducialMarkerObservation>;

  public function new(observations:Array<FiducialMarkerObservation>) {
    this.observations = observations;
  }

  public function detect(frame:SensorFrame):Array<FiducialMarkerObservation>
    return observations.copy();
}
