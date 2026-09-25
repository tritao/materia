package tests;

import RobotKitRuntime;
import NativeKitRuntime;
import haxe.Int64;
import materia.automation.facility.Facility;
import materia.automation.facility.FacilityRouter;
import materia.automation.facility.Lane;
import materia.automation.facility.Station;
import materia.automation.facility.Zone;
import materia.automation.fleet.Dispatcher;
import materia.automation.fleet.Fleet;
import materia.automation.fleet.FleetAssignment;
import materia.automation.mission.Mission;
import materia.automation.mission.MissionExecutionStatus;
import materia.automation.mission.MissionExecutor;
import materia.automation.task.Task;
import materia.automation.task.TaskKind;
import materia.automation.task.Transport;
import robotkit.material.Payload;
import robotkit.world.RemoteRobot;
import robotkit.world.RobotStatus;
import robotkit.world.RobotWorld;
import robotkit.behavior.HoldJointBehavior;
import robotkit.behavior.WorldBehaviorRunner;
import robotkit.world.McapRobotRecording;
import robotkit.world.McapRecordingReader;
import robotkit.world.ReplayRobot;
import robotkit.mobile.MobileBase;
import robotkit.mobile.Pose2;
import robotkit.localization.WheelOdometryLocalization;
import robotkit.navigation.Navigation;
import robotkit.navigation.NavigationGoal;
import robotkit.navigation.Path;
import robotkit.skill.GoTo;
import robotkit.skill.SkillRunner;
import robotkit.skill.SkillStatus;

/** End-to-end assertion of the world adapter against a real robotd TCP peer. */
class WorldTcpIntegration {
  static inline final LOGICAL_ID = "warehouse/forklift-17";

  /** Checks a fresh robotd process against a serial device that stayed powered. */
  public static function runRestartCheck(host:String, port:Int):Void {
    var runtime = NativeKitRuntime.start();
    var world = new RobotWorld();
    var remote = new RemoteRobot(LOGICAL_ID);
    world.attach(remote);
    var failure:Dynamic = null;
    try {
      remote.connect(host, port, runtime.events);
      waitUntil(runtime, function() return remote.status() == RobotStatus.Ready,
        "restarted robotd did not become ready");
      waitUntil(runtime, function() {
        var state = remote.snapshot();
        return state.positions.length == 3 && state.safety ==
          RobotKitRuntimeConstants.RK_SAFETY_EMERGENCY_STOP;
      }, "new host process did not latch the persistent device in emergency-stop");
      world.resetSafety(LOGICAL_ID);
      waitUntil(runtime, function() return remote.snapshot().safety ==
          RobotKitRuntimeConstants.RK_SAFETY_READY,
        "restarted serial session did not accept an explicit safety reset");
      remote.submit(robotkit.world.RobotCommand.JointTargets([
        robotkit.world.JointTarget.position(0, 0.5)
      ], null));
      waitUntil(runtime, function() {
        var state = remote.snapshot();
        return state.positions.length == 3 && state.positions.get(0) == 0.5;
      }, "restarted host sequence did not reach the still-powered device");
      Sys.println("robotd restart negotiated a new safe serial session and reset command sequence");
    } catch (error:Dynamic) failure = error;
    world.close();
    runtime.dispose();
    if (failure != null) throw failure;
  }

  public static function run(host:String, port:Int):Void {
    var runtime = NativeKitRuntime.start();
    var world = new RobotWorld();
    var remote = new RemoteRobot(LOGICAL_ID);
    world.attach(remote);
    var simulation = new robotkit.runtime.Simulation();
    var failure:Dynamic = null;
    try {
      var model = new robotkit.model.RobotModel("demo-forklift");
      var base = model.addLink(new robotkit.model.Link("base"));
      var leftWheel = model.addLink(new robotkit.model.Link("left wheel", "link/left-wheel"));
      var rightWheel = model.addLink(new robotkit.model.Link("right wheel", "link/right-wheel"));
      var carriage = model.addLink(new robotkit.model.Link("fork carriage", "link/carriage"));
      var leftWheelJoint = model.addJoint(new robotkit.model.Joint("left wheel joint",
        robotkit.model.JointType.Continuous, base, leftWheel, "joint/left-wheel"));
      leftWheelJoint.limits.lower = -100.0;
      leftWheelJoint.limits.upper = 100.0;
      leftWheelJoint.limits.effort = 100.0;
      var rightWheelJoint = model.addJoint(new robotkit.model.Joint("right wheel joint",
        robotkit.model.JointType.Continuous, base, rightWheel, "joint/right-wheel"));
      rightWheelJoint.limits.lower = -100.0;
      rightWheelJoint.limits.upper = 100.0;
      rightWheelJoint.limits.effort = 100.0;
      var liftJoint = model.addJoint(new robotkit.model.Joint("mast lift",
        robotkit.model.JointType.Prismatic, base, carriage, "joint/lift"));
      liftJoint.limits.lower = 0.0;
      liftJoint.limits.upper = 1.0;
      liftJoint.limits.velocity = 2.0;
      liftJoint.limits.effort = 100.0;
      model.mobileBase = new robotkit.model.RobotMobileConfiguration(
        robotkit.model.RobotDriveConfiguration.Differential("joint/left-wheel",
          "joint/right-wheel", 0.1, 0.5), 0.5, 1.0, 1.0, 1.0);
      model.forkMechanism = new robotkit.model.RobotForkConfiguration("joint/lift",
        1000.0, 600.0, 1.0);
      var mount = model.addFrame(new robotkit.model.Frame("sensor mount", base, "demo/sensor-mount"));
      mount.position = [0.2, 0.0, 0.0];
      for (kind in ["joint_encoder", "imu", "lidar"]) {
        var sensor = model.addSensor(new robotkit.model.Sensor(kind, kind, 0, 'demo/$kind'));
        sensor.frame = mount;
      }
      var local = new robotkit.world.SimulatedRobot("local", simulation.addRobot(
        robotkit.runtime.RobotRuntimeCompiler.compile(model)), model.name,
        [for (link in model.links) link.name], [for (joint in model.joints) joint.name]);
      world.attach(local);
      simulation.step(Int64.ofInt(1));
      simulation.step(Int64.ofInt(2));
      remote.connect(host, port, runtime.events);
      waitUntil(runtime, function() return remote.status() == RobotStatus.Ready, "RemoteRobot did not become ready");
      if (remote.id() != LOGICAL_ID) throw "world changed the logical robot ID";
      var protocolId = remote.protocolRobotId();
      if (protocolId == null || Int64.compare(protocolId,
        Int64.ofInt(42)) != 0) throw 'expected protocol robot ID 42, got ${Std.string(protocolId)}';
      waitUntil(runtime, function() {
        var capabilities = remote.capabilities();
        return capabilities.jointCount == 3 && capabilities.supportsPosition
          && capabilities.supportsVelocity && capabilities.supportsEffort;
      }, "robotd did not advertise all joint target modes");

      waitUntil(runtime, function() {
        var state = world.snapshot().robot(LOGICAL_ID);
        return state != null && state.positions.length > 0;
      }, "remote robot did not publish its initial state");
      remote.stop(robotkit.world.StopMode.Emergency);
      waitUntil(runtime, function() return remote.snapshot().safety ==
          RobotKitRuntimeConstants.RK_SAFETY_EMERGENCY_STOP,
        "simulated robot did not latch an explicit remote emergency stop");
      world.resetSafety(LOGICAL_ID);
      waitUntil(runtime, function() return remote.snapshot().safety ==
          RobotKitRuntimeConstants.RK_SAFETY_READY,
        "explicit safety reset did not release the new serial session");
      var deadlineRejected = false;
      try remote.submit(robotkit.world.RobotCommand.JointTargets([
        robotkit.world.JointTarget.position(0, 0.9)
      ], Int64.ofInt(123)))
      catch (_:Dynamic) deadlineRejected = true;
      if (!deadlineRejected) throw "remote adapter forwarded an unmapped absolute deadline";
      var behavior = new WorldBehaviorRunner(new HoldJointBehavior(0, 0.5));
      if (behavior.update(remote) != 1)
        throw "transport-neutral behavior did not submit a remote command";
      waitUntil(runtime, function() {
        var state = world.snapshot().robot(LOGICAL_ID);
        return state != null && state.id == LOGICAL_ID && state.positions.length > 0 && state.positions.get(0) == 0.5;
      }, "translated world command did not update robotd state");

      var state = world.snapshot().robot(LOGICAL_ID);
      var position = state == null ? 0.0 : state.positions.get(0);
      if (state == null || state.sensors.length != 3)
        throw "robotd did not transport simulated encoder, IMU, and LiDAR frames";
      for (sensor in state.sensors.toArray()) {
        if (sensor.sensorId != 'demo/${sensor.kind}' || sensor.frameId != "demo/sensor-mount"
            || sensor.linkId != "base" || sensor.mountPosition.get(0) != 0.2)
          throw "transport lost configured sensor identity or mount";
        if (sensor.kind == "imu" && (sensor.values.length != 6 || Math.abs(sensor.values.get(5) - 9.81) > 1e-9))
          throw "transport changed the stationary IMU specific force";
        if (sensor.kind == "lidar")
          for (range in sensor.values.toArray())
            if (range != 10.0) throw "single-robot LiDAR must exclude self geometry";
        if (Int64.compare(sensor.receivedTimestampNs, sensor.sourceTimestampNs) == 0)
          throw "sensor receipt reused simulation source time";
      }
      // Same behavior instance, fresh runner per adapter. Compare settled values,
      // not sequence numbers or timestamps from independently ticking clocks.
      var tick = 3;
      var recordingPath = '/tmp/robotkit-world-tcp-${Sys.getPid()}.mcap';
      var recording = new McapRobotRecording(recordingPath, 1024 * 1024);
      var recordedTargets:Array<Float> = [];
      for (target in [-0.4, 0.75, 0.0]) {
        var shared = new HoldJointBehavior(0, target);
        var localRunner = new WorldBehaviorRunner(shared);
        var remoteRunner = new WorldBehaviorRunner(shared);
        if (localRunner.update(local) != 1 || remoteRunner.update(remote) != 1)
          throw "shared behavior did not emit one command on each adapter";
        recording.recordCommand(robotkit.world.RobotCommand.JointTargets([
          robotkit.world.JointTarget.position(0, target)
        ], null), LOGICAL_ID);
        if (localRunner.update(local) != 0 || remoteRunner.update(remote) != 0)
          throw "runner emitted duplicate commands for an unchanged snapshot";
        simulation.step(Int64.ofInt(tick++));
        simulation.step(Int64.ofInt(tick++));
        waitUntil(runtime, function() {
          var value = remote.snapshot();
          if (value.positions.length != 3 || value.positions.get(0) != target) return false;
          for (sensor in value.sensors.toArray())
            if (sensor.kind == "joint_encoder" && sensor.values.get(0) == target) return true;
          return false;
        }, "shared behavior failed to converge remotely");
        var expected = local.snapshot();
        var actual = remote.snapshot();
        recording.recordSnapshot(actual);
        recordedTargets.push(target);
        if (expected.positions.get(0) != target || actual.positions.get(0) != expected.positions.get(0)
            || expected.faultCode != 0 || actual.faultCode != 0)
          throw "local and remote behavior outcomes differ";
        if (actual.sensors.length != expected.sensors.length) throw "sensor counts differ";
        for (sensor in expected.sensors.toArray()) {
          var found = false;
          for (other in actual.sensors.toArray()) {
            if (other.sensorId != sensor.sensorId) continue;
            found = true;
            if (other.kind != sensor.kind || other.frameId != sensor.frameId || other.linkId != sensor.linkId
                || other.values.length != sensor.values.length) throw "sensor contracts differ";
            for (i in 0...3)
              if (other.mountPosition.get(i) != sensor.mountPosition.get(i)) throw "sensor mounts differ";
            for (i in 0...4)
              if (other.mountRotation.get(i) != sensor.mountRotation.get(i)) throw "sensor rotations differ";
            for (i in 0...sensor.values.length) {
              var expectedValue = sensor.values.get(i);
              var actualValue = other.values.get(i);
              if (Math.abs(actualValue - expectedValue) > 1e-9)
                throw 'sensor measurements differ for ${sensor.sensorId}[$i]: expected=$expectedValue actual=$actualValue';
            }
          }
          if (!found) throw "remote sensor identity missing";
        }
      }
      recording.close();
      var loaded = McapRecordingReader.load(recordingPath);
      var replay = new ReplayRobot(LOGICAL_ID, loaded);
      for (index in 0...recordedTargets.length) {
        if (index > 0 && !replay.advance()) throw "replay ended before recorded observations";
        if (replay.snapshot().positions.get(0) != recordedTargets[index])
          throw "replay changed the recorded observation sequence";
        if (new WorldBehaviorRunner(new HoldJointBehavior(0,recordedTargets[index])).update(replay) != 1)
          throw "replayed observation changed behavior output";
      }
      replay.close(); sys.FileSystem.deleteFile(recordingPath);
      remote.submit(robotkit.world.RobotCommand.JointTargets([
        robotkit.world.JointTarget.position(0, 0.25),
        robotkit.world.JointTarget.velocity(1, 0.5),
        robotkit.world.JointTarget.effort(2, 2.0)
      ], null));
      waitUntil(runtime, function() {
        var state = remote.snapshot();
        return state.positions.length == 3 && state.positions.get(0) == 0.25
          && state.velocities.get(1) > 0.0 && state.efforts.get(2) != 0.0;
      }, "atomic mixed-mode target batch did not reach robotd runtime");
      var mobileBase = MobileBase.fromRobot(remote, model);
      var localization = new WheelOdometryLocalization(mobileBase);
      var current = remote.snapshot();
      var estimate = localization.update(current);
      var goalPose = new Pose2(estimate.pose.x + 0.1, estimate.pose.y, estimate.pose.yaw);
      var path = new Path([estimate.pose, goalPose], "odom");
      var goTo = new GoTo(new Navigation(mobileBase, localization, 0.2, 0.2, 0.8), path,
        new NavigationGoal(goalPose, "odom", 0.01, 0.05));
      var skillRunner = new SkillRunner();
      if (skillRunner.start(goTo) != SkillStatus.Running ||
          skillRunner.activeSkill() != goTo)
        throw "SkillRunner did not start GoTo for RemoteRobot";
      if (skillRunner.update(current, 0.02) != SkillStatus.Running)
        throw "RemoteRobot GoTo did not remain active after its first update";
      waitUntil(runtime, function() {
        var state = remote.snapshot();
        return state.velocities.length == 3 && state.velocities.get(0) > 0.0
          && state.velocities.get(1) > 0.0;
      }, "GoTo did not send an atomic two-wheel velocity target through robotd");
      skillRunner.cancel();
      if (skillRunner.status() != SkillStatus.Cancelled || skillRunner.activeSkill() != null ||
          skillRunner.result() == null)
        throw "SkillRunner did not cancel RemoteRobot GoTo cleanly";

      var facility = new Facility("serial-facility", "Serial integration facility");
      facility.addZone(new Zone("floor", "Floor", "map",
        robotkit.mobile.Footprint.rectangle(5.0, 5.0)));
      var receiving = new Station("receiving", "Receiving", "floor", "map", new Pose2());
      var staging = new Station("staging", "Staging", "floor", "map", new Pose2(0.1, 0.0, 0.0));
      facility.addStation(receiving);
      facility.addStation(staging);
      facility.addLane(new Lane("receiving-to-staging", receiving.id, staging.id,
        new Path([receiving.pose, staging.pose], "map"), 1.5, 0.5));
      var fleet = new Fleet("serial-fleet", world);
      fleet.addRobot(LOGICAL_ID);
      var mission = new Mission("serial-transport", "Serial robot transport", [
        new Transport("transport-pallet", receiving.id, staging.id,
          new Payload(100.0, 0.8, 0.6, 0.4, 0.4))
      ]);
      var assignmentValue = new Dispatcher(fleet).dispatch(mission);
      if (assignmentValue == null) throw "serial robot was not assigned the facility mission";
      var assignment:FleetAssignment = cast assignmentValue;
      var missionFactory = new SerialTransportSkillFactory(model);
      var executor = new MissionExecutor(fleet, assignment, facility, missionFactory);
      executor.start();
      var missionStatus = executor.status;
      var missionTicks = 0;
      while (switch missionStatus { case MissionExecutionStatus.Running: true; case _: false; } &&
          missionTicks < 800) {
        while (runtime.events.poll()) {}
        world.pump();
        missionStatus = executor.update(0.02);
        missionTicks++;
        if (switch missionStatus { case MissionExecutionStatus.Running: true; case _: false; })
          runtime.events.wait(0.01);
      }
      if (!switch missionStatus { case MissionExecutionStatus.Succeeded: true; case _: false; })
        throw 'RemoteRobot mission did not succeed: ${Std.string(missionStatus)}';
      if (mission.status != materia.automation.mission.MissionStatus.Succeeded ||
          fleet.availableRobotIds().indexOf(LOGICAL_ID) < 0)
        throw "serial mission did not complete and release its fleet assignment";
      Sys.println("SkillRunner GoTo, shared behavior, and MissionExecutor passed: local simulation and robotd serial");
      Sys.println('RobotKit TCP world test passed: logical=$LOGICAL_ID protocol=42 q0=$position');
    } catch (error:Dynamic) failure = error;
    world.close();
    simulation.dispose();
    runtime.dispose();
    if (failure != null) throw failure;
  }

  static function waitUntil(runtime:NativeKitRuntime, condition:Void -> Bool, failure:String):Void {
    for (_ in 0...500) {
      while (runtime.events.poll()) {
      }
      if (condition()) return;
      runtime.events.wait(0.01);
    }
    throw failure;
  }
}

private class SerialTransportSkillFactory implements materia.automation.mission.TaskSkillFactory {
  final model:robotkit.model.RobotModel;

  public function new(model:robotkit.model.RobotModel) this.model = model;

  public function create(task:Task, robot:robotkit.world.Robot,
      facility:Facility):robotkit.skill.Skill {
    return switch task.kind {
      case TaskKind.Transport:
        var transport:Transport = cast task;
        var route = new FacilityRouter(facility).route(
          transport.pickupStationId, transport.destinationStationId);
        var base = MobileBase.fromRobot(robot, model);
        var localization = new WheelOdometryLocalization(base, route.path.frameId);
        var navigation = new Navigation(base, localization, 0.2,
          route.maximumSpeedMetersPerSecond, 1.0);
        var goalStation = facility.station(transport.destinationStationId);
        if (goalStation == null) throw "transport mission destination is missing";
        new GoTo(navigation, route.path,
          new NavigationGoal(goalStation.pose, goalStation.frameId, 0.02, 0.1));
      case _: throw 'serial integration does not support task kind ${Std.string(task.kind)}';
    }
  }
}
