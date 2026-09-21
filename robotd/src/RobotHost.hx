package robotd;

import robot.model.Joint;
import robot.model.JointType;
import robot.model.Link;
import robot.model.Robot;
import robot.runtime.RuntimeCompiler;
import robotkit.Runtime;
import robotkit.RuntimeLayout;
import robotkit.behavior.RobotBehavior;
import robotd.behaviors.OscillateBehavior;

class RobotHost {
  final args:Array<String>;

  public function new(args:Array<String>) {
    this.args = args;
  }

  public function run():Void {
    if (args.indexOf("--help") >= 0) {
      Sys.println("Usage: robotd [--server [--once]] [--client] [--port=N] "
        + "[--behavior=oscillate] [--in-memory] [--help]");
      return;
    }
    var port = parsePort();
    var behavior = parseBehavior();
    if (args.indexOf("--client") >= 0) {
      RobotClientSmoke.run("127.0.0.1", port);
      return;
    }
    var robot = new Robot("demo-arm");
    var base = robot.addLink(new Link("base"));
    var tool = robot.addLink(new Link("tool"));
    var shoulder = robot.addJoint(new Joint("shoulder", JointType.Revolute, base, tool));
    shoulder.limits.lower = -3.14;
    shoulder.limits.upper = 3.14;
    shoulder.limits.effort = 100.0;
    var compiled = RuntimeCompiler.compile(robot);
    if (args.indexOf("--server") >= 0) {
      var serverRuntime = Runtime.createSim(RuntimeCompiler.blueprint(compiled));
      var server = new RobotServer(robot, compiled, serverRuntime, port, behavior);
      server.run(args.indexOf("--once") >= 0);
      return;
    }
    var runtime = if (args.indexOf("--in-memory") >= 0) {
      Runtime.create(new RuntimeLayout(compiled.revision,
        compiled.jointCount, compiled.source.links.length));
    } else {
      Runtime.createSim(RuntimeCompiler.blueprint(compiled));
    };
    runtime.submitPositions([0.5], 1);
    runtime.step(haxe.Int64.ofInt(1));
    var snapshot = runtime.snapshot();
    var endpoint = args.indexOf("--in-memory") >= 0 ? "in-memory" : "simkit";
    Sys.println('robotd: compiled ${compiled.source.name} with ${compiled.jointCount} joints via $endpoint; '
      + 'native position=${snapshot.positions[0]}');
    runtime.dispose();
  }

  function parsePort():Int {
    for (arg in args) {
      if (arg.indexOf("--port=") == 0) {
        var value = Std.parseInt(arg.substr(7));
        if (value == null || value <= 0 || value > 65535)
          throw "robotd: --port requires an integer from 1 to 65535";
        return value;
      }
    }
    return 17890;
  }

  function parseBehavior():Null<RobotBehavior> {
    for (arg in args) {
      if (arg.indexOf("--behavior=") != 0)
        continue;
      return switch (arg.substr(11)) {
        case "oscillate": new OscillateBehavior();
        case value: throw 'robotd: unsupported behavior "$value"';
      };
    }
    return null;
  }
}
