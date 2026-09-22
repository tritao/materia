package tests;

import NativeKitRuntime;
import haxe.Int64;
import robotkit.world.RemoteRobot;
import robotkit.world.RobotStatus;
import robotkit.world.RobotWorld;
import robotkit.behavior.HoldJointBehavior;
import robotkit.behavior.WorldBehaviorRunner;
import robotkit.world.McapRobotRecording;
import robotkit.world.McapRecordingReader;
import robotkit.world.ReplayRobot;

/** End-to-end assertion of the world adapter against a real robotd TCP peer. */
class WorldTcpIntegration {
  static inline final LOGICAL_ID = "warehouse/forklift-17";

  public static function run(host:String, port:Int):Void {
    var runtime = NativeKitRuntime.start();
    var world = new RobotWorld();
    var remote = new RemoteRobot(LOGICAL_ID);
    world.attach(remote);
    var simulation = new robotkit.runtime.Simulation();
    var failure:Dynamic = null;
    try {
      var model = new robotkit.model.RobotModel("demo-arm");
      var base = model.addLink(new robotkit.model.Link("base"));
      var tool = model.addLink(new robotkit.model.Link("tool"));
      var joint = model.addJoint(new robotkit.model.Joint("shoulder",
        robotkit.model.JointType.Revolute, base, tool));
      joint.limits.lower = -3.14;
      joint.limits.upper = 3.14;
      joint.limits.effort = 100;
      var mount = model.addFrame(new robotkit.model.Frame("sensor mount", base, "demo/sensor-mount"));
      mount.position = [0.2, 0.0, 0.0];
      for (kind in ["joint_encoder", "imu", "lidar"]) {
        var sensor = model.addSensor(new robotkit.model.Sensor(kind, kind, 0, 'demo/$kind'));
        sensor.frame = mount;
      }
      var local = new robotkit.world.SimulatedRobot("local", simulation.addRobot(
        robotkit.runtime.RobotRuntimeCompiler.compile(model)), model.name, ["base", "tool"], ["shoulder"]);
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
        var state = world.snapshot().robot(LOGICAL_ID);
        return state != null && state.positions.length > 0;
      }, "remote robot did not publish its initial state");
      var deadlineRejected = false;
      try remote.submit(robotkit.world.RobotCommand.JointPosition(0, 0.9, Int64.ofInt(123)))
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
        recording.recordCommand(robotkit.world.RobotCommand.JointPosition(0,target,null), LOGICAL_ID);
        if (localRunner.update(local) != 0 || remoteRunner.update(remote) != 0)
          throw "runner emitted duplicate commands for an unchanged snapshot";
        simulation.step(Int64.ofInt(tick++));
        simulation.step(Int64.ofInt(tick++));
        waitUntil(runtime, function() {
          var value = remote.snapshot();
          if (value.positions.length != 1 || value.positions.get(0) != target) return false;
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
            for (i in 0...sensor.values.length)
              if (Math.abs(other.values.get(i) - sensor.values.get(i)) > 1e-9) throw "sensor measurements differ";
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
      Sys.println("Shared behavior parity passed: three targets, local simulation and robotd TCP");
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
