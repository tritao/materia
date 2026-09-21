package tests;

import NativeKitRuntime;
import haxe.Int64;
import robotkit.world.RemoteRobot;
import robotkit.world.RobotCommand;
import robotkit.world.RobotStatus;
import robotkit.world.WorldHost;

/** End-to-end assertion of the world adapter against a real robotd TCP peer. */
class WorldTcpIntegration {
  static inline final LOGICAL_ID = "warehouse/forklift-17";

  public static function run(host:String, port:Int):Void {
    var runtime = NativeKitRuntime.start();
    var world = new WorldHost();
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

      world.submit(LOGICAL_ID, RobotCommand.JointPosition(0, 0.5, null));
      waitUntil(runtime, function() {
        var state = world.snapshot().robot(LOGICAL_ID);
        return state != null && state.id == LOGICAL_ID && state.positions.length > 0 && state.positions[0] == 0.5;
      }, "translated world command did not update robotd state");

      var state = world.snapshot().robot(LOGICAL_ID);
      var position = state == null ? 0.0 : state.positions[0];
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
