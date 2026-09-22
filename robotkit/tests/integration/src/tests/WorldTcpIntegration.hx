package tests;

import NativeKitRuntime;
import haxe.Int64;
import robotkit.world.RemoteRobot;
import robotkit.world.RobotStatus;
import robotkit.world.RobotWorld;
import robotkit.behavior.HoldJointBehavior;
import robotkit.behavior.WorldBehaviorRunner;

/** End-to-end assertion of the world adapter against a real robotd TCP peer. */
class WorldTcpIntegration {
  static inline final LOGICAL_ID = "warehouse/forklift-17";

  public static function run(host:String, port:Int):Void {
    var runtime = NativeKitRuntime.start();
    var world = new RobotWorld();
    var remote = new RemoteRobot(LOGICAL_ID);
    world.attach(remote);
    var failure:Dynamic = null;
    try {
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
      Sys.println('RobotKit TCP world test passed: logical=$LOGICAL_ID protocol=42 q0=$position');
    } catch (error:Dynamic) failure = error;
    world.close();
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
