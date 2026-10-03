package robotkit.device;

import robotkit.model.Actuator;
import robotkit.model.RobotModel;
import robotkit.model.RobotModelCodec;
import robotkit.model.Transmission;
import robotkit.runtime.VirtualActuatorOptions;

/** One RKD6 channel as the device sees it: a stepper actuator's transmission into a joint, in steps. */
class BoundChannel {
  public final channel:Int;
  public final actuatorId:String;
  public final jointIndex:Int;
  /** Device actuator coordinate per joint unit; negative when the driver is wired reversed. */
  public final ratio:Float;
  public final offset:Float;
  public final stepsPerUnit:Float;
  /** Actuator units per second the step tick can generate, or less when the actuator is slower. */
  public final maxRate:Float;
  public final directionSetupTicks:Int;
  public final skewBound:Float;

  public function new(channel:Int, actuatorId:String, jointIndex:Int, ratio:Float, offset:Float,
      stepsPerUnit:Float, maxRate:Float, directionSetupTicks:Int, skewBound:Float) {
    this.channel = channel;
    this.actuatorId = actuatorId;
    this.jointIndex = jointIndex;
    this.ratio = ratio;
    this.offset = offset;
    this.stepsPerUnit = stepsPerUnit;
    this.maxRate = maxRate;
    this.directionSetupTicks = directionSetupTicks;
    this.skewBound = skewBound;
  }
}

/**
 * The join of a machine model and a deployment's wiring into the device's actuator layout. The
 * model owns the motor's full steps and the transmission; the layout owns which channel drives
 * which actuator and its direction. Microstepping is a model driver setting, checked against
 * the wiring (legacy models may state it in the layout); the board owns the step tick.
 * Nothing falls back to a default: a stepper without a channel, or a channel without a stepper,
 * is an error.
 */
class DeviceBinding {
  public final channels:Array<BoundChannel>;
  /**
   * A copy of the model whose stepper actuators are no faster than the step tick can drive them,
   * so the limits planning reads from it (RobotModel.coupledLimits) are the device's real ceiling.
   */
  public final model:RobotModel;
  public final stepTickHz:Int;

  function new(channels:Array<BoundChannel>, model:RobotModel, stepTickHz:Int) {
    this.channels = channels;
    this.model = model;
    this.stepTickHz = stepTickHz;
  }

  public static function bind(robot:RobotModel, layout:DeviceLayout, stepTickHz:Int):DeviceBinding {
    if (robot == null || layout == null) throw "Device binding requires a model and a layout";
    if (stepTickHz <= 0) throw "Device binding requires a positive step tick rate";
    if (layout.channels.length == 0 || layout.channels.length > 64)
      throw "Device layout needs from 1 to 64 channels";
    var tightened = RobotModelCodec.decode(RobotModelCodec.encode(robot));
    var wired = new Map<String, Int>();
    var bound:Array<BoundChannel> = [];
    for (position in 0...layout.channels.length) {
      var channel = layout.channels[position];
      if (channel.index != position)
        throw 'Device layout channel ${channel.index} is out of order: expected channel $position';
      var name = channel.actuator;
      if (name == null)
        throw 'Device layout channel $position names no actuator; a joint-only channel cannot be wired';
      if (wired.exists(name))
        throw 'Device layout drives actuator "$name" from channels ${wired.get(name)} and $position';
      wired.set(name, position);
      var actuator = find(robot, name);
      if (actuator == null)
        throw 'Device layout channel $position names actuator "$name", which the model does not have';
      if (actuator.fullStepsPerRevolution <= 0.0)
        throw 'Actuator "$name" on channel $position is not a stepper: the model gives it no full steps per revolution';
      var jointId:String;
      var ratio:Float;
      var offset:Float;
      switch actuator.transmission {
        case SimpleTransmission(target, transmissionRatio, transmissionOffset):
          jointId = target;
          ratio = transmissionRatio;
          offset = transmissionOffset;
      }
      var jointIndex = -1;
      for (index in 0...robot.joints.length) if (robot.joints[index].id == jointId) jointIndex = index;
      if (jointIndex < 0) throw 'Actuator "$name" drives joint "$jointId", which the model does not have';
      if (channel.jointId != "" && channel.jointId != jointId)
        throw 'Device layout channel $position says joint "${channel.jointId}" but actuator "$name" drives "$jointId"';
      // The rotor angle in radians is the actuator coordinate; one turn is the motor's full steps
      // times the driver's microsteps.
      if (channel.microsteps < 1 || channel.microsteps > 1024)
        throw 'Device layout channel $position microsteps must be from 1 to 1024';
      var setting = actuator.microsteps;
      if (setting != null && setting != channel.microsteps)
        throw 'Device layout channel $position microsteps disagree with actuator "$name" driver setting';
      var stepsPerUnit = actuator.fullStepsPerRevolution * channel.microsteps / (2.0 * Math.PI);
      // A faster board clock can idle between pulses; the driver still bounds pulse frequency.
      var driverRate = actuator.maxStepRate;
      var pulseRate:Float = driverRate == null ? stepTickHz : Math.min(stepTickHz, driverRate);
      var ceiling = pulseRate / stepsPerUnit;
      var rate = actuator.maxRate > 0.0 ? Math.min(actuator.maxRate, ceiling) : ceiling;
      bound.push(new BoundChannel(position, name, jointIndex, ratio * channel.direction, offset,
        stepsPerUnit, rate, channel.directionSetupTicks, channel.skewBound));
      var capped = find(tightened, name);
      if (capped != null) capped.maxRate = rate;
    }
    for (actuator in robot.actuators)
      if (actuator.fullStepsPerRevolution > 0.0 && !wired.exists(actuator.id))
        throw 'Stepper actuator "${actuator.id}" has no channel in the device layout';
    return new DeviceBinding(bound, tightened, stepTickHz);
  }

  /** The channels as an in-process virtual device's actuators. */
  public function virtualActuators():Array<VirtualActuatorOptions>
    return [for (channel in channels) new VirtualActuatorOptions(channel.actuatorId,
      channel.jointIndex, channel.ratio, channel.offset, channel.stepsPerUnit, channel.maxRate,
      channel.directionSetupTicks, channel.skewBound)];

  static function find(robot:RobotModel, id:String):Null<Actuator> {
    for (actuator in robot.actuators) if (actuator.id == id) return actuator;
    return null;
  }
}
