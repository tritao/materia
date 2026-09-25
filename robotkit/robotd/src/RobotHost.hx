package robotd;

import robotkit.model.Joint;
import robotkit.model.JointType;
import robotkit.model.Link;
import robotkit.model.RobotModel;
import robotkit.runtime.RobotRuntime;
import robotkit.runtime.RobotRuntimeCompiler;
import robotkit.runtime.Simulation;
import robotkit.behavior.RobotBehavior;
import robotd.behaviors.OscillateBehavior;

class RobotHost {
  final args:Array<String>;

  public function new(args:Array<String>) {
    this.args = args;
  }

  public function run():Void {
    if (args.indexOf("--help") >= 0) {
      Sys.println("Usage: robotd [--server [--once]] [--port=N] "
        + "[--robot-id=N] [--multi-joint] [--behavior=oscillate] [--in-memory] "
        + "[--serial=DEVICE [--baud=BAUD]] [--help]");
      return;
    }
    var port = parsePort();
    var robotId = parseRobotId();
    var serialPath = parseSerialPath();
    var baud = parseBaud();
    var hasBaud = false;
    for (arg in args) if (arg.indexOf("--baud=") == 0) hasBaud = true;
    if (serialPath == null && hasBaud)
      throw "robotd: --baud requires --serial";
    if (serialPath != null && args.indexOf("--in-memory") >= 0)
      throw "robotd: --serial cannot be combined with --in-memory";
    var behavior = parseBehavior();
    var robot = new RobotModel("demo-arm");
    var base = robot.addLink(new Link("base"));
    var tool = robot.addLink(new Link("tool"));
    var shoulder = robot.addJoint(new Joint("shoulder", JointType.Revolute, base, tool));
    shoulder.limits.lower = -3.14;
    shoulder.limits.upper = 3.14;
    shoulder.limits.effort = 100.0;
    if (args.indexOf("--multi-joint") >= 0) {
      var wheel = robot.addLink(new Link("wheel"));
      var carriage = robot.addLink(new Link("carriage"));
      var wheelJoint = robot.addJoint(new Joint("wheel-joint", JointType.Revolute,
        tool, wheel));
      wheelJoint.limits.lower = -100.0;
      wheelJoint.limits.upper = 100.0;
      wheelJoint.limits.velocity = 10.0;
      wheelJoint.limits.effort = 100.0;
      var liftJoint = robot.addJoint(new Joint("lift", JointType.Prismatic,
        tool, carriage));
      liftJoint.limits.lower = -1.0;
      liftJoint.limits.upper = 1.0;
      liftJoint.limits.velocity = 2.0;
      liftJoint.limits.effort = 100.0;
    }
    var mount = robot.addFrame(new robotkit.model.Frame("sensor mount", base, "demo/sensor-mount"));
    mount.position = [0.2, 0.0, 0.0];
    for (kind in ["joint_encoder", "imu", "lidar"]) {
      var sensor = robot.addSensor(new robotkit.model.Sensor(kind, kind, 0, 'demo/$kind'));
      sensor.frame = mount;
    }
    var blueprint = RobotRuntimeCompiler.compile(robot);
    if (args.indexOf("--server") >= 0) {
      var serverSimulation:Null<Simulation> = null;
      var serverRuntime:Null<RobotRuntime> = null;
      try {
        if (serialPath != null)
          serverRuntime = RobotRuntime.createSerial(blueprint, serialPath, baud);
        else {
          serverSimulation = new Simulation();
          serverRuntime = serverSimulation.addRobot(blueprint);
        }
        if (serverRuntime == null) throw "robotd: failed to create runtime";
        var hostedRuntime:RobotRuntime = serverRuntime;
        var server = new RobotServer(robot, blueprint, hostedRuntime, serverSimulation,
          port, robotId, behavior);
        server.run(args.indexOf("--once") >= 0);
      } catch (error:Dynamic) {
        if (serverSimulation != null) serverSimulation.dispose();
        else if (serverRuntime != null) serverRuntime.dispose();
        throw error;
      }
      if (serverRuntime != null) serverRuntime.dispose();
      if (serverSimulation != null) serverSimulation.dispose();
      return;
    }
    var inMemory = args.indexOf("--in-memory") >= 0;
    var simulation:Null<Simulation> = inMemory || serialPath != null ? null : new Simulation();
    var runtime = serialPath != null
      ? RobotRuntime.createSerial(blueprint, serialPath, baud)
      : inMemory
        ? RobotRuntime.create(blueprint)
        : simulation.addRobot(blueprint);
    runtime.submitPositions([0.5], 1);
    if (simulation == null) {
      runtime.start();
      Sys.sleep(0.02);
      runtime.stop();
    } else {
      simulation.step(haxe.Int64.ofInt(1));
    }
    var snapshot = runtime.snapshot();
    var endpoint = serialPath != null ? "serial"
      : inMemory ? "in-memory" : "simkit";
    Sys.println('robotd: compiled ${robot.name} with ${blueprint.jointCount} joints via $endpoint; '
      + 'native position=${snapshot.q.get(0)}');
    runtime.dispose();
    if (simulation != null)
      simulation.dispose();
  }

  function parsePort():Int {
    for (arg in args) {
      if (arg.indexOf("--port=") == 0) {
        var value = Std.parseInt(arg.substr(7));
        if (value == null || value <= 0 || value > 65535) throw "robotd: --port requires an integer from 1 to 65535";
        return value;
      }
    }
    return 17890;
  }

  function parseRobotId():Int {
    for (arg in args) {
      if (arg.indexOf("--robot-id=") == 0) {
        var value = Std.parseInt(arg.substr(11));
        if (value == null || value <= 0) throw "robotd: --robot-id requires a positive integer";
        return value;
      }
    }
    return 1;
  }

  function parseSerialPath():Null<String> {
    for (arg in args) {
      if (arg.indexOf("--serial=") != 0) continue;
      var value = StringTools.trim(arg.substr(9));
      if (value.length == 0) throw "robotd: --serial requires a device path";
      return value;
    }
    return null;
  }

  function parseBaud():Int {
    for (arg in args) {
      if (arg.indexOf("--baud=") != 0) continue;
      var value = Std.parseInt(arg.substr(7));
      if (value == null || [9600, 19200, 38400, 57600, 115200].indexOf(value) < 0)
        throw "robotd: --baud supports 9600, 19200, 38400, 57600, or 115200";
      return value;
    }
    return 115200;
  }

  function parseBehavior():Null < RobotBehavior > {
    for (arg in args) {
      if (arg.indexOf("--behavior=") != 0) continue;
      return switch (arg.substr(11)) {
        case "oscillate":
          new OscillateBehavior();
        case value:
          throw 'robotd: unsupported behavior "$value"';
      };
    }
    return null;
  }
}
