package robotd;

import robotkit.model.Joint;
import robotkit.model.JointType;
import robotkit.model.Link;
import robotkit.model.RobotModel;
import robotkit.profile.RobotDriveConfiguration;
import robotkit.profile.RobotForkConfiguration;
import robotkit.profile.RobotMobileConfiguration;
import robotkit.runtime.RobotRuntime;
import robotkit.runtime.RobotRuntimeCompiler;
import robotkit.runtime.Simulation;
import robotkit.runtime.SimulationHarness;
import robotkit.behavior.RobotBehavior;
import robotd.behaviors.OscillateBehavior;

class RobotHost {
  final args:Array<String>;

  public function new(args:Array<String>) {
    this.args = args;
  }

  public function run():Void {
    if (args.length > 0 && args[0] == "identify") {
      identify();
      return;
    }
    if (args.indexOf("--help") >= 0) {
      Sys.println("Usage: robotd [--server [--once]] [--port=N] [--listen=IPv4] "
        + "[--robot-id=N] [--multi-joint] [--behavior=oscillate] [--in-memory] "
        + "[--deployment=FILE] [--camera-fixture] [--camera-fixture-stream] "
        + "[--bulk-budget-bytes=N] [--help]");
      Sys.println("       robotd identify DEVICE_PATH BAUD   print the connected board's controller id");
      return;
    }
    var port = parsePort();
    var listenAddress = parseListenAddress();
    var robotId = parseRobotId();
    var bulkBudgetBytes = parseBulkBudget();
    var deploymentPath = optionValue("--deployment=");
    var perceptionFixtureModel = optionValue("--perception-fixture-model=");
    var deployment = deploymentPath == null ? null : new RobotDeployment(deploymentPath);
    var serialPath = deployment == null ? null : deployment.serialPath;
    var baud = deployment == null ? 115200 : deployment.baud;
    var controller = deployment == null ? null : deployment.controller;
    var binding = deployment == null ? null : deployment.binding;
    var targetError = deployment == null ? 0.0 : deployment.targetError;
    var linkLossTimeoutNs = deployment == null ? haxe.Int64.ofInt(500000000) : deployment.linkLossTimeoutNs;
    var clockSyncBoundNs = deployment == null ? haxe.Int64.ofInt(30000000) : deployment.clockSyncBoundNs;
    if (optionValue("--serial=") != null || optionValue("--controller=") != null ||
        optionValue("--target-error=") != null || optionValue("--baud=") != null)
      throw "robotd: serial settings belong in --deployment";
    if (deployment != null && args.indexOf("--multi-joint") >= 0)
      throw "robotd: --deployment cannot be combined with --multi-joint";
    if (deployment != null && args.indexOf("--server") < 0)
      throw "robotd: --deployment requires --server";
    if (deployment != null && args.indexOf("--in-memory") >= 0)
      throw "robotd: --deployment cannot be combined with --in-memory";
    if (perceptionFixtureModel != null && deployment != null)
      throw "robotd: perception fixture requires the demo robot";
    var behavior = parseBehavior();
    var multiJoint = args.indexOf("--multi-joint") >= 0;
    var cameraFixtureStream = args.indexOf("--camera-fixture-stream") >= 0 || perceptionFixtureModel != null;
    var cameraFixture = args.indexOf("--camera-fixture") >= 0 || cameraFixtureStream;
    if (cameraFixture && deployment != null)
      throw "robotd: --camera-fixture requires the demo robot";
    // A deployment plans on its binding's model, whose actuator rates are what the step tick can drive.
    var profile = deployment == null ? new robotkit.profile.RobotProfile() : deployment.profile;
    var robot = deployment == null ? new RobotModel(multiJoint ? "demo-forklift" : "demo-arm") : deployment.binding.model;
    if (deployment == null) {
    var base = robot.addLink(new Link("base"));
    if (multiJoint) {
      var leftWheel = robot.addLink(new Link("left wheel", "link/left-wheel"));
      var rightWheel = robot.addLink(new Link("right wheel", "link/right-wheel"));
      var carriage = robot.addLink(new Link("fork carriage", "link/carriage"));
      var leftWheelJoint = robot.addJoint(new Joint("left wheel joint", JointType.Continuous,
        base, leftWheel, "joint/left-wheel"));
      leftWheelJoint.axis = [0.0, 1.0, 0.0];
      leftWheelJoint.limits.lower = -100.0;
      leftWheelJoint.limits.upper = 100.0;
      leftWheelJoint.limits.effort = 100.0;
      leftWheelJoint.limits.maxAcceleration = 1.0;
      var rightWheelJoint = robot.addJoint(new Joint("right wheel joint", JointType.Continuous,
        base, rightWheel, "joint/right-wheel"));
      rightWheelJoint.axis = [0.0, 1.0, 0.0];
      rightWheelJoint.limits.lower = -100.0;
      rightWheelJoint.limits.upper = 100.0;
      rightWheelJoint.limits.effort = 100.0;
      rightWheelJoint.limits.maxAcceleration = 1.0;
      var liftJoint = robot.addJoint(new Joint("mast lift", JointType.Prismatic,
        base, carriage, "joint/lift"));
      liftJoint.limits.lower = 0.0;
      liftJoint.limits.upper = 1.0;
      liftJoint.limits.velocity = 2.0;
      liftJoint.limits.effort = 100.0;
      liftJoint.limits.maxAcceleration = 1.0;
      profile.mobileBase = new RobotMobileConfiguration(
        RobotDriveConfiguration.Differential("joint/left-wheel", "joint/right-wheel",
          0.1, 0.5), 0.5, 1.0, 1.0, 1.0);
      profile.forkMechanism = new RobotForkConfiguration("joint/lift", 1000.0,
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
      if (cameraFixtureStream) {
        var secondCamera = robot.addSensor(new robotkit.model.Sensor("camera-right", "camera", 0,
          "demo/camera-right"));
        secondCamera.frame = mount;
        if (perceptionFixtureModel == null) {
          var oversizeCamera = robot.addSensor(new robotkit.model.Sensor("camera-oversize", "camera", 0,
            "demo/camera-oversize"));
          oversizeCamera.frame = mount;
        }
      }
    }
    }
    var blueprint = RobotRuntimeCompiler.compile(robot, profile);
    if (deployment != null) {
      blueprint.ownerPeriodNs = deployment.ownerPeriodNs;
      blueprint.serialProcessingAllowanceNs = deployment.processingAllowanceNs;
      for (channel in deployment.channels) blueprint.channels.push(channel);
    }
    if (args.indexOf("--server") >= 0) {
      var serverSimulation:Null<SimulationHarness> = null;
      var serverRuntime:Null<RobotRuntime> = null;
      try {
        if (serialPath != null)
          serverRuntime = RobotRuntime.create(blueprint, robotkit.serial.SerialRuntimeEndpoint.create(blueprint, serialPath,
            controller, binding, targetError, baud,
            linkLossTimeoutNs, clockSyncBoundNs));
        else {
          var newServerSimulation = new SimulationHarness();
          serverSimulation = newServerSimulation;
          serverRuntime = newServerSimulation.simulation.addRobot(blueprint);
        }
        if (serverRuntime == null) throw "robotd: failed to create runtime";
        var hostedRuntime:RobotRuntime = serverRuntime;
        var fixtureTick:Null<Void->Void> = null;
        if (cameraFixtureStream) {
          var largeFixture = perceptionFixtureModel == null ||
            args.indexOf("--perception-stall-large") >= 0;
          var fixtureWidth = largeFixture ? 640 : 8;
          var fixtureHeight = largeFixture ? 480 : 4;
          var pixels = haxe.io.Bytes.alloc(fixtureWidth * fixtureHeight * 3);
          for (index in 0...pixels.length) pixels.set(index,
            perceptionFixtureModel == null ? index % 251 : 51);
          var image = new robotkit.world.CameraImage(fixtureWidth, fixtureHeight, "rgb8", pixels);
          var oversizeImage = perceptionFixtureModel == null
            ? new robotkit.world.CameraImage(1920, 1080, "rgb8", haxe.io.Bytes.alloc(1920 * 1080 * 3))
            : null;
          var fixtureSequence = haxe.Int64.ofInt(0);
          var nextFixtureNs = haxe.Int64.ofInt(0);
          fixtureTick = function() {
            var now = nativekit.ffi.NativeKit.nk_time_now_ns();
            if (haxe.Int64.compare(now, nextFixtureNs) < 0) return;
            fixtureSequence = haxe.Int64.add(fixtureSequence, haxe.Int64.ofInt(1));
            if (oversizeImage != null && haxe.Int64.compare(fixtureSequence, haxe.Int64.ofInt(1)) == 0)
              hostedRuntime.publishCameraFrame("demo/camera-oversize", oversizeImage,
                fixtureSequence, now, "camera.fixture");
            hostedRuntime.publishCameraFrame("demo/camera", image, fixtureSequence,
              now, "camera.fixture");
            hostedRuntime.publishCameraFrame("demo/camera-right", image, fixtureSequence,
              now, "camera.fixture");
            nextFixtureNs = haxe.Int64.add(now, haxe.Int64.ofInt(20000000));
          };
        } else if (cameraFixture) {
          var pixels = haxe.io.Bytes.alloc(6);
          for (index in 0...6) pixels.set(index, index + 1);
          hostedRuntime.publishCameraFrame("demo/camera",
            new robotkit.world.CameraImage(2, 1, "rgb8", pixels),
            haxe.Int64.ofInt(1), haxe.Int64.ofInt(1), "camera.fixture");
        }
        var server = new RobotServer(robot, blueprint, hostedRuntime, serverSimulation,
          port, robotId, behavior, listenAddress, bulkBudgetBytes, fixtureTick,
          deployment != null ? deployment.perception : perceptionFixtureModel == null ? [] :
            [new robotkit.deployment.PerceptionPipelineConfig("front_objects", "demo/camera",
              "object_detector", perceptionFixtureModel,
              robotkit.inference.InferenceSession.modelDigest(perceptionFixtureModel),
              "robotd", ["local", "worldd"], 0.4, 0.0, 0.5)]);
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
    var simulation:Null<SimulationHarness> = null;
    var runtime:RobotRuntime;
    if (serialPath != null)
      runtime = RobotRuntime.create(blueprint, robotkit.serial.SerialRuntimeEndpoint.create(blueprint, serialPath, controller, binding, targetError,
        baud, linkLossTimeoutNs, clockSyncBoundNs));
    else if (inMemory)
      runtime = RobotRuntime.create(blueprint);
    else {
      var newSimulation = new SimulationHarness();
      simulation = newSimulation;
      runtime = newSimulation.simulation.addRobot(blueprint);
    }
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

  /** `robotd identify DEVICE_PATH BAUD`: prints the unique id of the board on a serial port. */
  function identify():Void {
    if (args.length != 3) throw "usage: robotd identify DEVICE_PATH BAUD";
    var baud = Std.parseInt(args[2]);
    if (baud == null || [115200, 230400, 460800, 921600].indexOf(baud) < 0)
      throw "robotd: identify baud must be one of 115200, 230400, 460800, 921600";
    Sys.println(robotkit.serial.SerialRuntimeEndpoint.identify(args[1], baud));
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

  function parseBulkBudget():Int {
    var raw = optionValue("--bulk-budget-bytes=");
    if (raw == null) return 0;
    var value = Std.parseInt(raw);
    if (value == null || value <= 0 || value > 4 * 1024 * 1024 - OutboundScheduler.MIN_ESSENTIAL_RESERVE)
      throw "robotd: --bulk-budget-bytes must leave at least 65536 bytes for essential traffic";
    return value;
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
