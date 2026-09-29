package worldd;

import haxe.Int64;
import NativeKitRuntime;
import robotkit.deployment.SerialDeployment;
import robotkit.model.Joint;
import robotkit.model.JointType;
import robotkit.model.Link;
import robotkit.model.RobotModel;
import robotkit.world.WorldSnapshot;
import robotkit.worldd.WorldHost;

/** Minimal headless composition runner for the existing RobotKit boundaries. */
class Main {
  public static function main():Void {
    var args = Sys.args();
    if (args.indexOf("--help") >= 0) {
      Sys.println("Usage: worldd [--robots=N] [--ticks=N] [--remote=HOST:PORT --deployment=FILE] [--help]");
      return;
    }
    var robotCount = option(args, "--robots=", 2);
    var tickCount = option(args, "--ticks=", 3);
    var remoteAddress = stringOption(args, "--remote=");
    var deploymentPath = stringOption(args, "--deployment=");
    if (robotCount < 0 || tickCount < 0 || (robotCount == 0 && remoteAddress == null))
      throw "worldd: --robots and --ticks must be non-negative, with at least one robot";
    if ((remoteAddress == null) != (deploymentPath == null))
      throw "worldd: --remote and --deployment must be provided together";

    var host = new WorldHost();
    var runtime:Null<NativeKitRuntime> = null;
    try {
      for (index in 0...robotCount)
        host.addSimulatedRobot('sim-$index', demoModel('sim-$index'));
      if (remoteAddress != null && deploymentPath != null) {
        var separator = remoteAddress.lastIndexOf(":");
        if (separator <= 0) throw "worldd: --remote must be HOST:PORT";
        var port = Std.parseInt(remoteAddress.substr(separator + 1));
        if (port == null || port <= 0 || port > 65535)
          throw "worldd: --remote port is invalid";
        runtime = NativeKitRuntime.start();
        var remote = host.addRemoteRobot("remote", false, new SerialDeployment(deploymentPath));
        remote.connect(remoteAddress.substr(0, separator), port, runtime.events);
      }
      var snapshot:Null<WorldSnapshot> = null;
      for (tick in 0...tickCount) {
        if (runtime != null) while (runtime.events.poll()) {}
        snapshot = host.step(Int64.ofInt((tick + 1) * 10_000_000));
      }
      if (snapshot == null)
        snapshot = host.world.snapshot();
      Sys.println('worldd: robots=${snapshot.robotIds().length} '
        + 'sequence=${snapshot.sequence} ticks=$tickCount');
    } catch (error:Dynamic) {
      host.close();
      if (runtime != null) runtime.dispose();
      throw error;
    }
    host.close();
    if (runtime != null) runtime.dispose();
  }

  static function demoModel(name:String):RobotModel {
    var model = new RobotModel(name);
    var base = model.addLink(new Link("base"));
    var tool = model.addLink(new Link("tool"));
    var joint = model.addJoint(new Joint("shoulder", JointType.Revolute, base, tool));
    joint.limits.lower = -3.14;
    joint.limits.upper = 3.14;
    joint.limits.effort = 100.0;
    return model;
  }

  static function option(args:Array<String>, prefix:String, fallback:Int):Int {
    for (arg in args) {
      if (arg.indexOf(prefix) != 0) continue;
      var value = Std.parseInt(arg.substr(prefix.length));
      if (value == null) throw 'worldd: $prefix requires an integer';
      return value;
    }
    return fallback;
  }

  static function stringOption(args:Array<String>, prefix:String):Null<String> {
    for (arg in args) if (arg.indexOf(prefix) == 0) {
      var value = arg.substr(prefix.length);
      if (value.length == 0) throw 'worldd: $prefix requires a value';
      return value;
    }
    return null;
  }
}
