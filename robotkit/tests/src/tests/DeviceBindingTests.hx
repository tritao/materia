package tests;

import haxe.io.Bytes;
import robotkit.device.DeviceBinding;
import robotkit.device.DeviceChannel;
import robotkit.device.DeviceLayout;
import robotkit.model.Actuator;
import robotkit.model.Joint;
import robotkit.model.JointCoupling;
import robotkit.model.JointLimits;
import robotkit.model.JointType;
import robotkit.model.Link;
import robotkit.model.RobotModel;
import robotkit.model.Transmission;

/** The join of a machine model and a deployment's wiring into the device's actuator layout. */
class DeviceBindingTests {
  public static function run():Int {
    var assertions = 0;
    function check(value:Bool, message:String):Void {
      assertions++;
      if (!value) throw 'assertion failed: $message';
    }
    function near(actual:Float, expected:Float, message:String):Void
      check(Math.abs(actual - expected) <= 1e-9 * Math.max(1.0, Math.abs(expected)), '$message: $actual, expected $expected');
    function fails(action:Void->Void, fragment:String, message:String):Void {
      var text:Null<String> = null;
      try action() catch (error:Dynamic) text = Std.string(error);
      check(text != null && text.indexOf(fragment) >= 0, '$message (got: $text)');
    }

    var model = axisModel();
    // 200 full steps at 16 microsteps a turn, on a 20 kHz step tick.
    var layout = DeviceLayout.decode(Bytes.ofString('{"schemaVersion":1,"channels": [
      {"index": 0, "actuator": "motor", "direction": -1, "direction_setup_ticks": 2}]}'));
    var binding = DeviceBinding.bind(model, layout, 20000);
    var channel = binding.channels[0];
    near(channel.stepsPerUnit, 3200.0 / (2.0 * Math.PI), "steps per radian from the motor's steps and the driver's microsteps");
    near(channel.ratio, Math.PI * 1000.0, "a reversed driver flips the composed screw ratio");
    check(channel.jointIndex == 0 && channel.directionSetupTicks == 2, "the channel follows the independent carriage leader");
    near(channel.maxRate, 20000.0 / channel.stepsPerUnit, "the step tick caps the actuator's rate below its own 100 rad/s");
    near(model.actuators[0].requireRate(), 100.0, "binding leaves the model alone");
    check(binding.model.actuators[0].requireRate() <= channel.maxRate, "the tightened model respects the device rate");
    var leadRatio = Math.PI * 1000.0;
    near(binding.model.coupledLimits("axis").requireVelocity(), binding.model.actuators[0].requireRate() / leadRatio,
      "planning limits see the device's achievable ceiling through the screw");
    near(model.coupledLimits("axis").requireVelocity(), 100.0 / leadRatio, "an unbound model keeps the actuator's own limit");

    var mixed = axisModel();
    var servo = new Actuator("servo", 2.0, 500.0, Transmission.SimpleTransmission("turn", 1.0, 0.0));
    mixed.actuators.insert(0, servo);
    var pulseLayout = DeviceLayout.forSteppers(mixed, 3);
    check(pulseLayout.channels.length == 1 && pulseLayout.channels[0].index == 0 &&
      pulseLayout.channels[0].actuator == "motor", "mixed layouts compact only the stepper pulse channels");
    check(pulseLayout.channels[0].directionSetupTicks == 3, "stepper layouts preserve direction setup timing");
    var mixedBinding = DeviceBinding.bind(mixed, pulseLayout, 20000);
    near(mixedBinding.model.actuators[0].requireRate(), 500.0, "a pulse clock never caps the servo's speed");
    check(DeviceLayout.forSteppers(plainServoModel()).channels.length == 0,
      "a servo-only model has no pulse channels");

    // A slower actuator keeps its own rate; no authored rate takes the ceiling.
    var slow = DeviceBinding.bind(axisModel(10.0), layout, 20000);
    near(slow.channels[0].maxRate, 10.0, "an actuator slower than the step tick keeps its rate");
    // 10 rad/s requests a non-integral pulse interval. A continuous ceiling
    // would leave physical counters chasing the plan after its nominal stop.
    var wire = Bytes.alloc(8);
    wire.setFloat(0, slow.channels[0].maxRate);
    wire.setFloat(4, slow.channels[0].stepsPerUnit);
    var stepInterval = Math.ceil(20000 / (wire.getFloat(0) * wire.getFloat(4)));
    check(slow.model.actuators[0].requireRate() < slow.channels[0].maxRate,
      "integer pulse spacing tightens a non-integral rate ceiling");
    near(slow.model.actuators[0].requireRate(), 20000 / (stepInterval * wire.getFloat(4)),
      "planner speed matches the pulse generator's deployed f32 settings");
    var unlimited = DeviceBinding.bind(axisModel(null), layout, 20000);
    near(unlimited.channels[0].maxRate, 20000.0 / channel.stepsPerUnit, "an unlimited actuator takes the step tick's ceiling");
    near(channel.feedbackRatio, -1.0, "physical shaft feedback retains the direct reversed transmission");
    check(channel.feedbackJointIndex != channel.jointIndex, "shaft feedback remains distinct from its carriage leader");
    var options = binding.virtualActuators();
    check(options.length == 1 && options[0].id == "motor" && options[0].jointIndex == 0 &&
      Math.abs(options[0].stepsPerUnit - channel.stepsPerUnit) < 1e-9, "the virtual device gets the same layout");

    fails(function() DeviceBinding.bind(model, new DeviceLayout([]), 20000), "from 1 to 64",
      "an empty layout is refused");
    fails(function() DeviceBinding.bind(model, new DeviceLayout([new DeviceChannel(0, "ghost")]), 20000),
      "does not have", "a channel for an unknown actuator is refused");
    fails(function() DeviceBinding.bind(model,
      new DeviceLayout([new DeviceChannel(0, "motor"), new DeviceChannel(1, "motor")]), 20000),
      "from channels 0 and 1", "an actuator on two channels is refused");
    fails(function() DeviceBinding.bind(model, new DeviceLayout([new DeviceChannel(1, "motor")]), 20000),
      "out of order", "channels must be contiguous from zero");
    var second = axisModel();
    var spare = new Actuator("spare", null, 10.0, Transmission.SimpleTransmission("turn", 1.0, 0.0));
    spare.fullStepsPerRevolution = 200.0;
    spare.microsteps = 16;
    spare.maxStepRate = 200000;
    second.addActuator(spare);
    fails(function() DeviceBinding.bind(second, layout, 20000), "has no channel",
      "a stepper with no channel is refused, never mapped by default");
    var plain = axisModel();
    plain.actuators[0].fullStepsPerRevolution = 0.0;
    fails(function() DeviceBinding.bind(plain, layout, 20000), "not a stepper",
      "a channel on an actuator with no steps is refused");
    fails(function() DeviceLayout.decode(Bytes.ofString('{"schemaVersion":1,"channels": [{"index": 0, "actuator": "motor", "direction": 2}]}')),
      "direction", "a direction other than 1 or -1 is refused");
    fails(function() DeviceLayout.decode(Bytes.ofString('{"schemaVersion":1,"channels": [{"index": 0, "actuator": "motor", "microsteps": 0}]}')),
      "wiring only", "channel microsteps are refused");
    var settings = axisModel();
    settings.actuators[0].microsteps = 32;
    settings.actuators[0].maxStepRate = 200000;
    var restored = robotkit.model.RobotModelCodec.decode(robotkit.model.RobotModelCodec.encode(settings));
    check(restored.actuators[0].microsteps == 32 && restored.actuators[0].maxStepRate == 200000,
      "driver settings survive the robot model codec");
    var missingDriver = axisModel();
    missingDriver.actuators[0].microsteps = null;
    fails(function() DeviceBinding.bind(missingDriver, layout, 40000), "requires microsteps",
      "a stepper without driver settings is refused");
    var pulseLimited = axisModel(null);
    pulseLimited.actuators[0].microsteps = 16;
    pulseLimited.actuators[0].maxStepRate = 10000;
    var driverCeiling = 10000.0 * 2 * Math.PI / (200 * 16);
    near(pulseLimited.actuators[0].requireRate(), driverCeiling,
      "driver input rate caps the motor before a controller is bound");
    near(pulseLimited.coupledLimits("axis").requireVelocity(), driverCeiling / leadRatio,
      "unbound axis planning includes its driver's pulse ceiling");
    check(pulseLimited.coupledLimits("axis").velocityLimiter == "driver step input",
      "the axis names its driver speed ceiling");
    var automatic = DeviceLayout.forActuators(pulseLimited);
    var fastBoard = DeviceBinding.bind(pulseLimited, automatic, 100000);
    near(fastBoard.channels[0].maxRate, 10000.0 / fastBoard.channels[0].stepsPerUnit,
      "a faster board clock respects the driver's maximum pulse rate");
    var slowBoard = DeviceBinding.bind(pulseLimited, automatic, 5000);
    near(slowBoard.channels[0].maxRate, 5000.0 / slowBoard.channels[0].stepsPerUnit,
      "a slower board clock remains the pulse ceiling");
    check(slowBoard.model.joints[0].limits.velocityLimiter == "controller tick",
      "the axis names its controller speed ceiling");
    check(pulseLimited.actuators[0].maxRate == null, "driver binding keeps the source motor speed unspecified");
    var dual = axisModel();
    dual.joints[0].limits.rackingTolerance = 0.0005;
    var rightLink = dual.addLink(new Link("right screw"));
    var right = dual.addJoint(new Joint("right turn", JointType.Revolute, dual.links[0], rightLink));
    right.limits = new JointLimits(-1e9, 1e9);
    dual.addCoupling(new JointCoupling("right lead", "axis", "right turn", Math.PI * 1000, 0));
    var rightMotor = new Actuator("right motor", 0.6, 100, Transmission.SimpleTransmission("right turn", 1, 0));
    rightMotor.fullStepsPerRevolution = 200;
    rightMotor.microsteps = 16;
    rightMotor.maxStepRate = 200000;
    dual.addActuator(rightMotor);
    var dualLayout = DeviceLayout.forActuators(dual);
    near(dualLayout.channels[0].skewBound, 0.0005, "left drive inherits SI racking tolerance");
    near(dualLayout.channels[1].skewBound, 0.0005, "right drive inherits SI racking tolerance");
    var dualBinding = DeviceBinding.bind(dual, dualLayout, 40000);
    check(dualBinding.channels[0].jointIndex == 0 && dualBinding.channels[1].jointIndex == 0,
      "opposed screw shafts group on their shared leader");
    near(dualBinding.channels[0].ratio, -dualBinding.channels[1].ratio, "opposed shafts retain signed ratios");
    near(automatic.channels[0].skewBound, 0, "a single drive has no skew group");
    var restoredDual = robotkit.model.RobotModelCodec.decode(robotkit.model.RobotModelCodec.encode(dual));
    near(restoredDual.joints[0].limits.rackingTolerance, 0.0005, "racking tolerance survives the model codec");
    // Joint-coordinate zeros compose independently of electrical direction.
    var zeroModel = axisModel();
    zeroModel.actuators[0].transmission = Transmission.SimpleTransmission("turn", 2, 3);
    var zeroBinding = DeviceBinding.bind(zeroModel, layout, 20000);
    near(zeroBinding.channels[0].offset, -3 / leadRatio, "shaft zero is expressed in leader coordinates");
    near(zeroBinding.channels[0].ratio, 2 * leadRatio, "direction changes ratio without changing zero");
    dual.actuators[1].transmission = Transmission.SimpleTransmission("right turn", 1, 1);
    fails(function() DeviceBinding.bind(dual, dualLayout, 40000), "equal leader-coordinate offsets",
      "RKD6 refuses guarded drives with different zeros");
    var switched = axisModel();
    switched.addFrame(new robotkit.model.Frame("home-frame", switched.links[0]));
    switched.addSwitch(new robotkit.model.JointSwitch("home", "axis", "home-frame", "home", -1, -0.01, 0.0, 0.0));
    var inputLayout = DeviceLayout.decode(Bytes.ofString('{"schemaVersion":1,"channels":[{"index":0,"actuator":"motor"}],"inputs":[{"index":0,"switch_id":"home","actuator":"motor","active_high":false,"pin":"PA0"}]}'));
    var inputBinding = DeviceBinding.bind(switched, inputLayout, 20000);
    check(inputBinding.inputs.length == 1 && inputBinding.inputs[0].actuatorChannel == 0 &&
      !inputBinding.inputs[0].wiring.activeHigh, "switch wiring resolves to the physical actuator and polarity");
    var restoredInputs = DeviceLayout.decode(DeviceLayout.encode(inputLayout));
    check(restoredInputs.inputs[0].switchId == "home" && restoredInputs.inputs[0].pin == "PA0",
      "deployment input wiring round trips");
    fails(function() DeviceBinding.bind(switched, layout, 20000), "no deployment input",
      "a machine switch cannot silently lose its input wiring");
    fails(function() DeviceBinding.bind(model, inputLayout, 20000), "one model switch",
      "unknown model switch input is rejected");
    return assertions;
  }

  /** A carriage on a 2 mm lead screw turned by a 200-step motor that can spin at `rate` rad/s. */
  static function axisModel(rate:Null<Float> = 100.0):RobotModel {
    var model = new RobotModel("screw axis");
    var base = model.addLink(new Link("base"));
    var carriage = model.addLink(new Link("carriage"));
    var screw = model.addLink(new Link("screw"));
    var axis = model.addJoint(new Joint("axis", JointType.Prismatic, base, carriage));
    var turn = model.addJoint(new Joint("turn", JointType.Revolute, base, screw));
    axis.limits = new JointLimits(0, 0.3, 0.08, 400, 0.5);
    turn.limits = new JointLimits(-1e9, 1e9);
    model.addCoupling(new JointCoupling("lead", "axis", "turn", -Math.PI * 1000, 0.0));
    var motor = new Actuator("motor", 0.6, rate, Transmission.SimpleTransmission("turn", 1.0, 0.0));
    motor.fullStepsPerRevolution = 200.0;
    motor.microsteps = 16;
    motor.maxStepRate = 200000;
    model.addActuator(motor);
    return model;
  }
  static function plainServoModel():RobotModel {
    var model = axisModel();
    model.actuators[0].fullStepsPerRevolution = 0;
    return model;
  }

}
