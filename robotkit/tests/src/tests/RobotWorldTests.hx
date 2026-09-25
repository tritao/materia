package tests;

import RobotKitRuntime;
import haxe.Int64;
import sys.thread.Mutex;
import sys.thread.Thread;
import robotkit.runtime.RobotRuntimeBlueprint;
import robotkit.runtime.RobotRuntimeCompiler;
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
import robotkit.model.RobotDriveConfiguration;
import robotkit.model.RobotMobileConfiguration;
import robotkit.model.RobotForkConfiguration;
import robotkit.world.RobotCapabilities;
import robotkit.world.RobotCommand;
import robotkit.world.RobotDescription;
import robotkit.world.RobotFault;
import robotkit.world.RobotId;
import robotkit.world.Robot;
import robotkit.world.RobotSnapshot;
import robotkit.world.RobotStatus;
import robotkit.world.RemoteRobot;
import robotkit.world.SimulatedRobot;
import robotkit.world.RecordingRobot;
import robotkit.world.StopMode;
import robotkit.world.RobotWorld;
import robotkit.world.RobotWorldEvent;
import robotkit.world.SensorFrame;
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
import robotkit.mobile.Twist2;
import robotkit.mobile.MotionLimits;
import robotkit.mobile.Footprint;
import robotkit.mobile.FootprintPoint;
import robotkit.mobile.MobileBase;
import robotkit.mobile.DifferentialDrive;
import robotkit.mobile.AckermannDrive;
import robotkit.mobile.DifferentialOdometry;
import robotkit.localization.WheelOdometryLocalization;
import robotkit.localization.SimulationTruthLocalization;
import robotkit.localization.LocalizationQuality;
import robotkit.localization.LocalizationState;
import robotkit.localization.PoseCovariance2;
import robotkit.localization.FrameTransform2;
import robotkit.localization.FrameTree2;
import robotkit.localization.PoseFusionLocalization;
import robotkit.navigation.Path;
import robotkit.navigation.Trajectory;
import robotkit.navigation.TrajectorySample;
import robotkit.navigation.NavigationGoal;
import robotkit.navigation.Navigation;
import robotkit.navigation.NavigationStatus;
import robotkit.material.ForkAxisConfig;
import robotkit.material.ForkAxisState;
import robotkit.material.ForkConfig;
import robotkit.material.ForkState;
import robotkit.material.Forks;
import robotkit.material.LoadLimits;
import robotkit.material.LoadState;
import robotkit.material.Payload;
import robotkit.perception.Detection;
import robotkit.perception.DockingTarget;
import robotkit.perception.LidarObstaclePerception;
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
import robotkit.skill.GoTo;
import robotkit.skill.PickPallet;
import robotkit.skill.PlacePallet;
import robotkit.skill.SkillStatus;

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
    testLocalization();
    testNavigation();
    testForkMechanisms();
    testPerceptionSafetyPower();
    testLoadSafetyPolicy();
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
    equal(RobotKitRuntime.rk_recording_writer_enqueue(opened.out_writer.borrow(),1,2,
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
    ackermann.command(Twist2.zero());
    check(switch ackermannRobot.lastCommand {
      case JointTargets(targets, _): targets[0].target == 0.0 && targets[1].target == 0.0;
      case _: false;
    }, "Ackermann drive safely emits zero steering and wheel targets at rest");
    var wideAckermann = new AckermannDrive(0, 1, 1.2, 0.25, 1.2);
    var wideTargets = wideAckermann.targets(new Twist2(1.0, 2.0));
    check(wideTargets[0].target > 1.1 && wideTargets[0].target < 1.2,
      "Ackermann curvature mapping handles steering ratios above one");

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
    var fusion = new PoseFusionLocalization(wheelSource, frames, "map", "base");
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
      new Pose2(2.0, 1.0, 0.2), "map", Int64.ofInt(1), Int64.ofInt(200),
      Int64.ofInt(220), "camera-boot", "host-clock");
    var pallet = new Pallet(palletDetection, 1.2, 0.8, 0.15);
    var dock = new DockingTarget(new Detection("dock-1", "dock", 0.9,
      new Pose2(4.0, 0.0, 0.0), "map", Int64.ofInt(2), Int64.ofInt(210),
      Int64.ofInt(230), "camera-boot", "host-clock"), new Pose2(3.0, 0.0, 0.0));
    var semantic = new PerceptionSnapshot([palletDetection], [], [pallet], [dock]);
    check(semantic.pallets()[0].lengthMeters == 1.2 &&
      semantic.dockingTargets()[0].approachPose.x == 3.0,
      "perception values represent pallet and docking targets");

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

  static function testForkliftSkillsOnSimulationAndReplay():Void {
    var jointNames = ["left-wheel", "right-wheel", "lift", "tilt", "spread"];
    var linkNames = ["base", "left-wheel-link", "right-wheel-link", "mast", "fork-carriage", "forks"];
    var blueprint = new RobotRuntimeBlueprint(1, 5, 6);
    for (index in 0...5)
      blueprint.addJoint(new RobotRuntimeJointBlueprint(index,
        RobotKitRuntimeConstants.RK_RUNTIME_JOINT_REVOLUTE, 0, index + 1,
        -1000.0, 1000.0, 1000.0, 1000.0));
    var simulation = new Simulation(0.01);
    var recordingPath = '/tmp/robotkit-${Sys.getPid()}-forklift-skills.mcap';
    var writer = new McapRobotRecording(recordingPath, 4 * 1024 * 1024);
    var simulatedRobot = new RecordingRobot(new SimulatedRobot("forklift",
      simulation.addRobot(blueprint), "simulated forklift", linkNames, jointNames), writer);
    var motionLimits = new MotionLimits(0.5, 1.0);
    var base = new MobileBase(simulatedRobot, new DifferentialDrive(0, 1, 0.1, 0.5),
      motionLimits);
    var localization = new WheelOdometryLocalization(base);
    var navigation = new Navigation(base, localization, 0.2, 0.2, 0.8);
    var forkConfig = new ForkConfig(new ForkAxisConfig("lift", 0.0, 1.5),
      new LoadLimits(1000.0, 700.0, 1.5),
      new ForkAxisConfig("tilt", -0.5, 0.5),
      new ForkAxisConfig("spread", 0.0, 0.8));
    var forks = new Forks(simulatedRobot, forkConfig);
    var path = new Path([new Pose2(0.0, 0.0, 0.0), new Pose2(0.18, 0.0, 0.0)], "odom");
    var goTo = new GoTo(navigation, path, new NavigationGoal(path.goal(), "odom", 0.02, 0.1));
    goTo.start();
    var liveStatus = goTo.status();
    var ticks = 0;
    while (switch liveStatus { case Running: true; case _: false; } && ticks < 300) {
      var observation = simulatedRobot.snapshot();
      liveStatus = goTo.update(observation, 0.01);
      if (switch liveStatus { case Running: true; case _: false; })
        simulation.step(Int64.ofInt((ticks + 1) * 10000000));
      ticks++;
    }
    check(switch liveStatus { case Succeeded: true; case _: false; } && ticks < 300,
      "GoTo drives a simulated forklift along its path");
    var liveSnapshot = simulatedRobot.snapshot();
    var liveEstimate = localization.state();
    var liveEstimateValue:robotkit.localization.LocalizationState = cast liveEstimate;
    var dockPose = new Pose2(liveEstimateValue.pose.x + 0.02,
      liveEstimateValue.pose.y, liveEstimateValue.pose.yaw);
    var dockDetection = new Detection("charger-dock", "dock", 0.95, dockPose,
      "odom", liveSnapshot.sourceSequence, liveSnapshot.sourceTimestampNs,
      liveSnapshot.receivedTimestampNs, liveSnapshot.sourceClockId, liveSnapshot.receivedClockId);
    var dock = new Dock(navigation, new DockingTarget(dockDetection, dockPose));
    dock.start();
    var dockStatus = dock.update(liveSnapshot, 0.01);
    check(switch dockStatus { case Succeeded: true; case _: false; },
      "Dock aligns the simulated base with a detected approach pose");
    var pickPose = new Pose2(liveEstimateValue.pose.x + 0.02,
      liveEstimateValue.pose.y, liveEstimateValue.pose.yaw);
    var palletDetection = new Detection("pallet-17", "pallet", 0.98,
      new Pose2(pickPose.x + 0.5, pickPose.y, pickPose.yaw), "odom",
      liveSnapshot.sourceSequence, liveSnapshot.sourceTimestampNs,
      liveSnapshot.receivedTimestampNs, liveSnapshot.sourceClockId, liveSnapshot.receivedClockId);
    var pallet = new Pallet(palletDetection, 1.2, 0.8, 0.15);
    var payload = new Payload(500.0, 1.2, 0.8, 0.15, 0.6, 0.0, 0.35);
    var pick = new PickPallet(navigation, forks, pallet, payload, pickPose,
      0.5, 0.1, 0.45, 0.05, 0.1);
    pick.start();
    var pickStatus = pick.update(liveSnapshot, 0.01);
    check(switch pickStatus { case Running: true; case _: false; } &&
      forks.loadState.payload == payload && !forks.loadState.secured,
      "PickPallet approaches and commands its simulated fork mechanism");
    simulation.step(Int64.ofInt((ticks + 1) * 10000000));
    liveSnapshot = simulatedRobot.snapshot();
    var forkObservation = forks.state();
    check(Math.abs(forkObservation.lift.position - 0.5) < 1e-9 &&
      Math.abs(cast(forkObservation.tilt, robotkit.material.ForkAxisState).position - 0.1) < 1e-9,
      "simulated runtime applies the fork position batch");
    forks.setLoadState(LoadState.carried(payload));
    pickStatus = pick.update(liveSnapshot, 0.01);
    check(switch pickStatus { case Succeeded: true; case _: false; } && pick.result() != null,
      "PickPallet completes only after secured-load confirmation");

    var travelStart = localization.state();
    var travelStartValue:robotkit.localization.LocalizationState = cast travelStart;
    var travelPose = new Pose2(travelStartValue.pose.x + 0.12,
      travelStartValue.pose.y, travelStartValue.pose.yaw);
    var travelPath = new Path([travelStartValue.pose, travelPose], "odom");
    var travel = new GoTo(navigation, travelPath,
      new NavigationGoal(travelPose, "odom", 0.02, 0.1));
    travel.start();
    var travelStatus = travel.status();
    var travelTicks = 0;
    while (switch travelStatus { case Running: true; case _: false; } && travelTicks < 300) {
      var observation = simulatedRobot.snapshot();
      travelStatus = travel.update(observation, 0.01);
      if (switch travelStatus { case Running: true; case _: false; })
        simulation.step(Int64.ofInt((ticks + travelTicks + 2) * 10000000));
      travelTicks++;
    }
    check(switch travelStatus { case Succeeded: true; case _: false; } && travelTicks < 300,
      "GoTo drives the loaded forklift to a second location");
    liveSnapshot = simulatedRobot.snapshot();

    var placePose = new Pose2(travelPose.x + 0.02, travelPose.y, travelPose.yaw);
    var place = new PlacePallet(navigation, forks, payload, placePose, 0.0, 0.0, 0.35);
    place.start();
    var placeStatus = place.update(liveSnapshot, 0.01);
    check(switch placeStatus { case Running: true; case _: false; } &&
      forks.loadState.secured && forks.loadState.payload == payload,
      "PlacePallet commands the forks and waits for release confirmation");
    simulation.step(Int64.ofInt((ticks + travelTicks + 2) * 10000000));
    liveSnapshot = simulatedRobot.snapshot();
    forks.setLoadState(LoadState.empty());
    placeStatus = place.update(liveSnapshot, 0.01);
    check(switch placeStatus { case Succeeded: true; case _: false; },
      "PlacePallet completes only after empty-load confirmation");

    var chargeEstimate = localization.state();
    var chargeEstimateValue:robotkit.localization.LocalizationState = cast chargeEstimate;
    var chargePose = new Pose2(chargeEstimateValue.pose.x + 0.01,
      chargeEstimateValue.pose.y, chargeEstimateValue.pose.yaw);
    var chargeDetection = new Detection("charger-1", "charger", 0.95, chargePose,
      "odom", liveSnapshot.sourceSequence, liveSnapshot.sourceTimestampNs,
      liveSnapshot.receivedTimestampNs, liveSnapshot.sourceClockId, liveSnapshot.receivedClockId);
    var livePower = new FakePower();
    livePower.battery = new BatteryState("traction-pack", 0.6, 48.0, -4.0, 25.0,
      liveSnapshot.sourceTimestampNs, liveSnapshot.receivedTimestampNs,
      liveSnapshot.sourceClockId, liveSnapshot.receivedClockId);
    var charge = new Charge(navigation, new DockingTarget(chargeDetection, chargePose),
      livePower, 0.8);
    charge.start();
    var chargeStatus = charge.update(liveSnapshot, 0.01);
    check(switch chargeStatus { case Running: true; case _: false; },
      "Charge docks and waits while the battery is below its target");
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
    var replay = new ReplayRobot("forklift", recording, replayDescription, replayCapabilities);
    var replayBase = new MobileBase(replay, new DifferentialDrive(0, 1, 0.1, 0.5),
      motionLimits);
    var replayLocalization = new WheelOdometryLocalization(replayBase);
    var replayNavigation = new Navigation(replayBase, replayLocalization, 0.2, 0.2, 0.8);
    var replayGoTo = new GoTo(replayNavigation, path,
      new NavigationGoal(path.goal(), "odom", 0.02, 0.1));
    replayGoTo.start();
    var replayStatus = replayGoTo.update(replay.snapshot(), 0.01);
    while (switch replayStatus { case Running: true; case _: false; } && replay.advance())
      replayStatus = replayGoTo.update(replay.snapshot(), 0.01);
    check(switch replayStatus { case Succeeded: true; case _: false; },
      "GoTo reaches the same recorded path through ReplayRobot");
    var replaySnapshot = replay.snapshot();
    var replayEstimate = replayLocalization.state();
    var replayEstimateValue:robotkit.localization.LocalizationState = cast replayEstimate;
    var replayDockPose = new Pose2(replayEstimateValue.pose.x + 0.02,
      replayEstimateValue.pose.y, replayEstimateValue.pose.yaw);
    var replayDockDetection = new Detection("charger-dock", "dock", 0.95, replayDockPose,
      "odom", replaySnapshot.sourceSequence, replaySnapshot.sourceTimestampNs,
      replaySnapshot.receivedTimestampNs, replaySnapshot.sourceClockId, replaySnapshot.receivedClockId);
    var replayDock = new Dock(replayNavigation,
      new DockingTarget(replayDockDetection, replayDockPose));
    replayDock.start();
    check(switch replayDock.update(replaySnapshot, 0.01) {
      case Succeeded: true;
      case _: false;
    }, "Dock replays the same recorded approach");
    var replayForks = new Forks(replay, forkConfig);
    var replayPickPose = new Pose2(replayEstimateValue.pose.x + 0.02,
      replayEstimateValue.pose.y, replayEstimateValue.pose.yaw);
    var replayPallet = new Pallet(new Detection("pallet-17", "pallet", 0.98,
      new Pose2(replayPickPose.x + 0.5, replayPickPose.y, replayPickPose.yaw), "odom",
      replaySnapshot.sourceSequence, replaySnapshot.sourceTimestampNs,
      replaySnapshot.receivedTimestampNs, replaySnapshot.sourceClockId,
      replaySnapshot.receivedClockId), 1.2, 0.8, 0.15);
    var replayPick = new PickPallet(replayNavigation, replayForks, replayPallet,
      payload, replayPickPose, 0.5, 0.1, 0.45, 0.05, 0.1);
    replayPick.start();
    replayPick.update(replaySnapshot, 0.01);
    replayForks.setLoadState(LoadState.carried(payload));
    check(switch replayPick.update(replaySnapshot, 0.01) {
      case Succeeded: true;
      case _: false;
    }, "PickPallet confirms the same load during replay");

    check(replay.advance() && replay.advance() && replay.advance(),
      "replay consumes the duplicated goal samples and the recorded fork-actuation tick");
    replaySnapshot = replay.snapshot();
    replayLocalization.update(replaySnapshot);

    var replayTravelStart = replayLocalization.state();
    var replayTravelStartValue:robotkit.localization.LocalizationState = cast replayTravelStart;
    var replayTravelPose = new Pose2(travelPose.x, travelPose.y, travelPose.yaw);
    var replayTravelPath = new Path([replayTravelStartValue.pose, replayTravelPose], "odom");
    var replayTravel = new GoTo(replayNavigation, replayTravelPath,
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
      "GoTo reaches the second recorded location through ReplayRobot");
    replaySnapshot = replay.snapshot();
    var replayPlacePose = new Pose2(placePose.x, placePose.y, placePose.yaw);
    var replayPlace = new PlacePallet(replayNavigation, replayForks, payload,
      replayPlacePose, 0.0, 0.0, 0.35);
    replayPlace.start();
    var replayPlaceStatus = replayPlace.update(replaySnapshot, 0.01);
    check(switch replayPlaceStatus { case Running: true; case _: false; } &&
      replayForks.loadState.secured,
      "PlacePallet replays the same fork command and awaits release");
    replayForks.setLoadState(LoadState.empty());
    check(switch replayPlace.update(replaySnapshot, 0.01) {
      case Succeeded: true;
      case _: false;
    }, "PlacePallet confirms release during replay");

    var replayChargeEstimate = replayLocalization.state();
    var replayChargeEstimateValue:robotkit.localization.LocalizationState = cast replayChargeEstimate;
    var replayChargePose = new Pose2(replayChargeEstimateValue.pose.x + 0.01,
      replayChargeEstimateValue.pose.y, replayChargeEstimateValue.pose.yaw);
    var replayChargeDetection = new Detection("charger-1", "charger", 0.95,
      replayChargePose, "odom", replaySnapshot.sourceSequence,
      replaySnapshot.sourceTimestampNs, replaySnapshot.receivedTimestampNs,
      replaySnapshot.sourceClockId, replaySnapshot.receivedClockId);
    var replayPower = new FakePower();
    replayPower.battery = new BatteryState("traction-pack", 0.6, 48.0, -4.0, 25.0,
      replaySnapshot.sourceTimestampNs, replaySnapshot.receivedTimestampNs,
      replaySnapshot.sourceClockId, replaySnapshot.receivedClockId);
    var replayCharge = new Charge(replayNavigation,
      new DockingTarget(replayChargeDetection, replayChargePose), replayPower, 0.8);
    replayCharge.start();
    check(switch replayCharge.update(replaySnapshot, 0.01) {
      case Running: true;
      case _: false;
    }, "Charge waits for recorded robot to reach the battery target");
    replayPower.battery = new BatteryState("traction-pack", 0.85, 48.0, -4.0, 25.0,
      replaySnapshot.sourceTimestampNs, replaySnapshot.receivedTimestampNs,
      replaySnapshot.sourceClockId, replaySnapshot.receivedClockId);
    check(switch replayCharge.update(replaySnapshot, 0.01) {
      case Succeeded: true;
      case _: false;
    }, "Charge completes deterministically against ReplayRobot");
    var generated = replay.generatedCommands.commands;
    check(generated.length >= 3 && switch generated[generated.length - 1] {
      case JointTargets(targets, _): targets.length == 3 && targets[0].joint == 2;
      case _: false;
    }, "ReplayRobot captures pick and place fork commands without altering source history");
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

  static function testMcapRoundTrip():Void {
    var path = '/tmp/robotkit-${Sys.getPid()}-roundtrip.mcap';
    var writer = new McapRobotRecording(path, 1024 * 1024);
    var sensor = new SensorFrame("lidar/front", "lidar", "frame/front",
      Int64.parseString("9007199254740993"), Int64.parseString("9223372036854775000"),
      [1.25, 2.5], Int64.parseString("9223372036854775001"), "link/base",
      [0.1, 0.2, 0.3], [0.0, 0.0, 0.0, 1.0], "robot-a.reset-2", "host.monotonic");
    var first = new RobotSnapshot("robot-a", Int64.parseString("9007199254740995"),
      Int64.parseString("9223372036854775000"), [0.5], [0.25], [0.125], 1, 0,
      Int64.parseString("9223372036854775002"), [sensor], "robot-a.reset-2", "host.monotonic");
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
    equal(loaded.snapshots[0].sensors.get(0).frameId, "frame/front", "MCAP preserves sensor frame identity");
    equal(loaded.snapshots[0].sensors.get(0).mountPosition.get(2), 0.3, "MCAP preserves sensor mount");
    equal(loaded.worlds[0].robotIds().length, 2, "MCAP preserves multiple robots");
    for (index in 0...loaded.entries.length) equal(loaded.entries[index].ordinal, Int64.ofInt(index), "MCAP uses arrival ordinals");
    var replay = new ReplayRobot("robot-a", loaded);
    var behavior = new WorldBehaviorRunner(new HoldJointBehavior(0, 0.25));
    equal(behavior.update(replay), 1, "reloaded observations use unchanged behavior APIs");
    replay.close();
    if (Sys.getEnv("ROBOTKIT_KEEP_MCAP") == null) sys.FileSystem.deleteFile(path);
    else Sys.println('RobotKit MCAP fixture: $path');
    var unsupported = haxe.io.Bytes.ofString('{"version":3,"ordinal":"0","robotId":"","sourceSequence":"0","sourceTimestampNs":"0","sourceClockId":"x","type":"worldEvent","payload":{"kind":"changed","robotId":"x"}}');
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
    check(hasDiagnostic(diagnostics, "RK_FRAME_POSE"), "non-unit mount rotations rejected");
    check(hasDiagnostic(diagnostics, "RK_SENSOR_RATE"), "negative sample rates rejected");
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
