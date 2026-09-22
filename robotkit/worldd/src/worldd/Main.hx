package worldd;

import haxe.Int64;
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
      Sys.println("Usage: worldd [--robots=N] [--ticks=N] [--help]");
      return;
    }
    var robotCount = option(args, "--robots=", 2);
    var tickCount = option(args, "--ticks=", 3);
    if (robotCount <= 0 || tickCount < 0)
      throw "worldd: --robots must be positive and --ticks must be non-negative";

    var host = new WorldHost();
    try {
      for (index in 0...robotCount)
        host.addSimulatedRobot('sim-$index', demoModel('sim-$index'));
      var snapshot:Null<WorldSnapshot> = null;
      for (tick in 0...tickCount)
        snapshot = host.step(Int64.ofInt((tick + 1) * 10_000_000));
      if (snapshot == null)
        snapshot = host.world.snapshot();
      Sys.println('worldd: robots=${snapshot.robotIds().length} '
        + 'sequence=${snapshot.sequence} ticks=$tickCount');
    } catch (error:Dynamic) {
      host.close();
      throw error;
    }
    host.close();
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
}
