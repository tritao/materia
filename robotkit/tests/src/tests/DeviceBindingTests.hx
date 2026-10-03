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
    var layout = DeviceLayout.decode(Bytes.ofString('{"channels": [
      {"index": 0, "actuator": "motor", "direction": -1, "microsteps": 16, "direction_setup_ticks": 2}]}'));
    var binding = DeviceBinding.bind(model, layout, 20000);
    var channel = binding.channels[0];
    near(channel.stepsPerUnit, 3200.0 / (2.0 * Math.PI), "steps per radian from the motor's steps and the driver's microsteps");
    near(channel.ratio, -1.0, "a reversed driver flips the device ratio");
    check(channel.jointIndex == 1 && channel.directionSetupTicks == 2, "the joint comes from the transmission");
    near(channel.maxRate, 20000.0 / channel.stepsPerUnit, "the step tick caps the actuator's rate below its own 100 rad/s");
    near(model.actuators[0].maxRate, 100.0, "binding leaves the model alone");
    near(binding.model.actuators[0].maxRate, channel.maxRate, "the tightened model carries the cap");
    var leadRatio = Math.PI * 1000.0;
    near(binding.model.coupledLimits("axis").velocity, channel.maxRate / leadRatio,
      "planning limits see the device's real ceiling through the screw");
    near(model.coupledLimits("axis").velocity, 100.0 / leadRatio, "an unbound model keeps the actuator's own limit");

    // A slower actuator keeps its own rate; no authored rate takes the ceiling.
    var slow = DeviceBinding.bind(axisModel(10.0), layout, 20000);
    near(slow.channels[0].maxRate, 10.0, "an actuator slower than the step tick keeps its rate");
    var unlimited = DeviceBinding.bind(axisModel(0.0), layout, 20000);
    near(unlimited.channels[0].maxRate, 20000.0 / channel.stepsPerUnit, "an unlimited actuator takes the step tick's ceiling");
    var options = binding.virtualActuators();
    check(options.length == 1 && options[0].id == "motor" && options[0].jointIndex == 1 &&
      Math.abs(options[0].stepsPerUnit - channel.stepsPerUnit) < 1e-9, "the virtual device gets the same layout");

    fails(function() DeviceBinding.bind(model, new DeviceLayout([]), 20000), "from 1 to 64",
      "an empty layout is refused");
    fails(function() DeviceBinding.bind(model, new DeviceLayout([new DeviceChannel(0, "turn")]), 20000),
      "names no actuator", "a joint-only channel cannot be wired");
    fails(function() DeviceBinding.bind(model, new DeviceLayout([new DeviceChannel(0, "", "ghost")]), 20000),
      "does not have", "a channel for an unknown actuator is refused");
    fails(function() DeviceBinding.bind(model, new DeviceLayout([new DeviceChannel(0, "axis", "motor")]), 20000),
      "drives \"turn\"", "a channel that disagrees on the joint is refused");
    fails(function() DeviceBinding.bind(model,
      new DeviceLayout([new DeviceChannel(0, "", "motor"), new DeviceChannel(1, "", "motor")]), 20000),
      "from channels 0 and 1", "an actuator on two channels is refused");
    fails(function() DeviceBinding.bind(model, new DeviceLayout([new DeviceChannel(1, "", "motor")]), 20000),
      "out of order", "channels must be contiguous from zero");
    var second = axisModel();
    var spare = new Actuator("spare", 0.0, 10.0, Transmission.SimpleTransmission("turn", 1.0, 0.0));
    spare.fullStepsPerRevolution = 200.0;
    second.addActuator(spare);
    fails(function() DeviceBinding.bind(second, layout, 20000), "has no channel",
      "a stepper with no channel is refused, never mapped by default");
    var plain = axisModel();
    plain.actuators[0].fullStepsPerRevolution = 0.0;
    fails(function() DeviceBinding.bind(plain, layout, 20000), "not a stepper",
      "a channel on an actuator with no steps is refused");
    fails(function() DeviceLayout.decode(Bytes.ofString('{"channels": [{"index": 0, "actuator": "motor", "direction": 2}]}')),
      "direction", "a direction other than 1 or -1 is refused");
    fails(function() DeviceLayout.decode(Bytes.ofString('{"channels": [{"index": 0, "actuator": "motor", "microsteps": 0}]}')),
      "microsteps", "zero microsteps are refused");
    return assertions;
  }

  /** A carriage on a 2 mm lead screw turned by a 200-step motor that can spin at `rate` rad/s. */
  static function axisModel(rate:Float = 100.0):RobotModel {
    var model = new RobotModel("screw axis");
    var base = model.addLink(new Link("base"));
    var carriage = model.addLink(new Link("carriage"));
    var screw = model.addLink(new Link("screw"));
    var axis = model.addJoint(new Joint("axis", JointType.Prismatic, base, carriage));
    var turn = model.addJoint(new Joint("turn", JointType.Revolute, base, screw));
    axis.limits = new JointLimits(0, 0.3, 0.08, 400, 0.5);
    turn.limits = new JointLimits(-1e9, 1e9, 0, 0, 0);
    model.addCoupling(new JointCoupling("lead", "axis", "turn", -Math.PI * 1000, 0.0));
    var motor = new Actuator("motor", 0.6, rate, Transmission.SimpleTransmission("turn", 1.0, 0.0));
    motor.fullStepsPerRevolution = 200.0;
    model.addActuator(motor);
    return model;
  }
}
