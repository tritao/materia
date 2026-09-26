package robotd;

import robotkit.model.Joint;
import robotkit.model.JointType;
import robotkit.model.Link;
import robotkit.model.RobotModel;
import robotkit.model.RobotDriveConfiguration;
import robotkit.model.RobotForkConfiguration;
import robotkit.model.RobotMobileConfiguration;
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
      Sys.println("Usage: robotd [--server [--once]] [--port=N] [--listen=IPv4] "
        + "[--robot-id=N] [--multi-joint] [--behavior=oscillate] [--in-memory] "
        + "[--deployment=FILE] [--camera-fixture] [--help]");
      return;
    }
    var port = parsePort();
    var listenAddress = parseListenAddress();
    var robotId = parseRobotId();
    var deploymentPath = optionValue("--deployment=");
    var deployment = deploymentPath == null ? null : new RobotDeployment(deploymentPath);
    var serialPath = deployment == null ? null : deployment.serialPath;
    var baud = deployment == null ? 115200 : deployment.baud;
    var fingerprint = deployment == null ? null : deployment.fingerprint;
    var targetError = deployment == null ? 0.0 : deployment.targetError;
    if (optionValue("--serial=") != null || optionValue("--fingerprint=") != null ||
        optionValue("--target-error=") != null || optionValue("--baud=") != null)
      throw "robotd: serial settings belong in --deployment";
    if (deployment != null && args.indexOf("--multi-joint") >= 0)
      throw "robotd: --deployment cannot be combined with --multi-joint";
    if (deployment != null && args.indexOf("--server") < 0)
      throw "robotd: --deployment requires --server";
    if (deployment != null && args.indexOf("--in-memory") >= 0)
      throw "robotd: --deployment cannot be combined with --in-memory";
    var behavior = parseBehavior();
    var multiJoint = args.indexOf("--multi-joint") >= 0;
    var cameraFixture = args.indexOf("--camera-fixture") >= 0;
    if (cameraFixture && deployment != null)
      throw "robotd: --camera-fixture requires the demo robot";
    var robot = deployment == null ? new RobotModel(multiJoint ? "demo-forklift" : "demo-arm") : deployment.robot;
    if (deployment == null) {
    var base = robot.addLink(new Link("base"));
    if (multiJoint) {
      var leftWheel = robot.addLink(new Link("left wheel", "link/left-wheel"));
      var rightWheel = robot.addLink(new Link("right wheel", "link/right-wheel"));
      var carriage = robot.addLink(new Link("fork carriage", "link/carriage"));
      var leftWheelJoint = robot.addJoint(new Joint("left wheel joint", JointType.Continuous,
        base, leftWheel, "joint/left-wheel"));
      leftWheelJoint.limits.lower = -100.0;
      leftWheelJoint.limits.upper = 100.0;
      leftWheelJoint.limits.effort = 100.0;
      var rightWheelJoint = robot.addJoint(new Joint("right wheel joint", JointType.Continuous,
        base, rightWheel, "joint/right-wheel"));
      rightWheelJoint.limits.lower = -100.0;
      rightWheelJoint.limits.upper = 100.0;
      rightWheelJoint.limits.effort = 100.0;
      var liftJoint = robot.addJoint(new Joint("mast lift", JointType.Prismatic,
        base, carriage, "joint/lift"));
      liftJoint.limits.lower = 0.0;
      liftJoint.limits.upper = 1.0;
      liftJoint.limits.velocity = 2.0;
      liftJoint.limits.effort = 100.0;
      robot.mobileBase = new RobotMobileConfiguration(
        RobotDriveConfiguration.Differential("joint/left-wheel", "joint/right-wheel",
          0.1, 0.5), 0.5, 1.0, 1.0, 1.0);
      robot.forkMechanism = new RobotForkConfiguration("joint/lift", 1000.0,
        600.0, 1.0);
    } else {
      var tool = robot.addLink(new Link("tool"));
      var shoulder = robot.addJoint(new Joint("shoulder", JointType.Revolute, base, tool));
      shoulder.limits.lower = -3.14;
      shoulder.limits.upper = 3.14;
      shoulder.limits.effort = 100.0;
    }
    var mount = robot.addFrame(new robotkit.model.Frame("sensor mount", base, "demo/sensor-mount"));
    mount.position = [0.2, 0.0, 0.0];
    for (kind in ["joint_encoder", "imu", "lidar"]) {
      var sensor = robot.addSensor(new robotkit.model.Sensor(kind, kind, 0, 'demo/$kind'));
      sensor.frame = mount;
    }
    if (cameraFixture) {
      var camera = robot.addSensor(new robotkit.model.Sensor("camera", "camera", 0,
        "demo/camera"));
      camera.frame = mount;
    }
    }
    var blueprint = RobotRuntimeCompiler.compile(robot);
    if (args.indexOf("--server") >= 0) {
      var serverSimulation:Null<Simulation> = null;
      var serverRuntime:Null<RobotRuntime> = null;
      try {
        if (serialPath != null)
          serverRuntime = RobotRuntime.createSerial(blueprint, serialPath,
            fingerprint, targetError, baud);
        else {
          serverSimulation = new Simulation();
          serverRuntime = serverSimulation.addRobot(blueprint);
        }
        if (serverRuntime == null) throw "robotd: failed to create runtime";
        var hostedRuntime:RobotRuntime = serverRuntime;
        if (cameraFixture) {
          var pixels = haxe.io.Bytes.alloc(6);
          for (index in 0...6) pixels.set(index, index + 1);
          hostedRuntime.publishCameraFrame("demo/camera",
            new robotkit.world.CameraImage(2, 1, "rgb8", pixels),
            haxe.Int64.ofInt(1), haxe.Int64.ofInt(1), "camera.fixture");
        }
        var server = new RobotServer(robot, blueprint, hostedRuntime, serverSimulation,
          port, robotId, behavior, listenAddress);
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
      ? RobotRuntime.createSerial(blueprint, serialPath, fingerprint, targetError, baud)
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

  function parseListenAddress():String {
    var value = optionValue("--listen=");
    if (value == null) return "127.0.0.1";
    var parts = value.split(".");
    if (parts.length != 4) throw "robotd: --listen requires an IPv4 address";
    for (part in parts) {
      if (part.length == 0 || part.length > 3 || !~/^[0-9]+$/.match(part))
        throw "robotd: --listen requires an IPv4 address";
      var octet = Std.parseInt(part);
      if (octet == null || octet > 255)
        throw "robotd: --listen requires an IPv4 address";
    }
    return value;
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

  function optionValue(prefix:String):Null<String> {
    for (arg in args) if (arg.indexOf(prefix) == 0) return StringTools.trim(arg.substr(prefix.length));
    return null;
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
