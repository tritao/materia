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
  /** Original physical shaft mapping, before collapsing its leader coupling. */
  public final feedbackJointIndex:Int;
  public final feedbackRatio:Float;
  public final feedbackOffset:Float;

  public function new(channel:Int, actuatorId:String, jointIndex:Int, ratio:Float, offset:Float,
      stepsPerUnit:Float, maxRate:Float, directionSetupTicks:Int, skewBound:Float,
      feedbackJointIndex:Int, feedbackRatio:Float, feedbackOffset:Float) {
    this.channel = channel;
    this.actuatorId = actuatorId;
    this.jointIndex = jointIndex;
    this.ratio = ratio;
    this.offset = offset;
    this.stepsPerUnit = stepsPerUnit;
    this.maxRate = maxRate;
    this.directionSetupTicks = directionSetupTicks;
    this.skewBound = skewBound;
    this.feedbackJointIndex = feedbackJointIndex;
    this.feedbackRatio = feedbackRatio;
    this.feedbackOffset = feedbackOffset;
  }
}

/** An input channel joined to the physical actuator whose step count it captures. */
class BoundInput {
  public final wiring:DeviceInput;
  public final actuatorChannel:Int;
  public function new(wiring:DeviceInput, actuatorChannel:Int) {
    this.wiring = wiring; this.actuatorChannel = actuatorChannel;
  }
}

/**
 * The join of a machine model and a deployment's wiring into the device's actuator layout. The
 * model owns the motor's full steps and the transmission; the layout owns which channel drives
 * which actuator and its direction. Microstepping is a model driver setting; the board owns the step tick.
 * Nothing falls back to a default: a stepper without a channel, or a channel without a stepper,
 * is an error.
 */
class DeviceBinding {
  /** Relative error of a rate times steps per unit carried as two f32 values, with margin; the device step generator uses the same. */
  static inline var RATE_ROUNDING:Float = 1e-6;

  public final channels:Array<BoundChannel>;
  public final inputs:Array<BoundInput>;
  /**
   * A copy of the model whose stepper actuators are no faster than the step tick can drive them,
   * so the limits planning reads from it (RobotModel.coupledLimits) are the device's real ceiling.
   */
  public final model:RobotModel;
  public final stepTickHz:Int;

  function new(channels:Array<BoundChannel>, model:RobotModel, stepTickHz:Int, inputs:Array<BoundInput>) {
    this.channels = channels;
    this.inputs = inputs;
    this.model = model;
    this.stepTickHz = stepTickHz;
  }

  /** Controller rate ceilings for host planning; GPIO deployment is not implied. */
  public static function planningModel(robot:RobotModel, stepTickHz:Int):RobotModel {
    var layout = DeviceLayout.forSteppers(robot);
    var model = layout.channels.length == 0 ? RobotModelCodec.decode(RobotModelCodec.encode(robot))
      : bindActuators(robot, layout, stepTickHz).model;
    model.materializeLimits();
    return model;
  }

  static function bindActuators(robot:RobotModel, layout:DeviceLayout, stepTickHz:Int):{
      model:RobotModel, channels:Array<BoundChannel>, wired:Map<String, Int>} {
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
      var mapping = DeviceTransmission.of(robot, actuator);
      if (!mapping.singleLeader && channel.skewBound > 0)
        throw "RKD6 cannot guard skew on a multiple-input transmission";
      var jointIndex = mapping.jointIndex;
      var ratio = mapping.ratio;
      var offset = mapping.offset;
      var setting = actuator.microsteps, driverRate = actuator.maxStepRate;
      if (setting == null || driverRate == null)
        throw 'Stepper actuator "$name" requires microsteps and a driver step-rate ceiling';
      var stepsPerUnit = actuator.fullStepsPerRevolution * setting / (2.0 * Math.PI);
      var pulseRate:Float = Math.min(stepTickHz, driverRate);
      var ceiling = pulseRate / stepsPerUnit;
      var motorRate = actuator.planningRate();
      var rate = motorRate == null ? ceiling : Math.min(motorRate, ceiling);
      var feedbackJoint = -1, feedbackRatio = 0.0, feedbackOffset = 0.0;
      switch actuator.transmission {
        case SimpleTransmission(shaft, directRatio, directOffset):
          for (index in 0...robot.joints.length) if (robot.joints[index].id == shaft) feedbackJoint = index;
          feedbackRatio = directRatio * channel.direction;
          feedbackOffset = directOffset;
      }
      if (feedbackJoint < 0 || !Math.isFinite(feedbackRatio) || feedbackRatio == 0 || !Math.isFinite(feedbackOffset))
        throw "Device actuator has no valid physical feedback mapping";
      bound.push(new BoundChannel(position, name, jointIndex, ratio * channel.direction, offset,
        stepsPerUnit, rate, channel.directionSetupTicks, channel.skewBound,
        feedbackJoint, feedbackRatio, feedbackOffset));
      var capped = find(tightened, name);
      if (capped != null) {
        if ((stepTickHz < driverRate) &&
            (motorRate == null || rate < motorRate)) capped.speedLimiter = "controller tick";
        // The pulse generator enforces an integer number of controller ticks
        // between steps. Plan at that achievable ceiling, using the same f32
        // deployment values, rather than allowing the motor to lag a faster
        // continuous-rate trajectory and keep moving after its nominal stop.
        var wire = haxe.io.Bytes.alloc(8);
        wire.setFloat(0, rate); wire.setFloat(4, stepsPerUnit);
        var wireRate = wire.getFloat(0), wireSteps = wire.getFloat(4);
        var ticks = stepTickHz / (wireRate * wireSteps), whole = Math.floor(ticks);
        // As the device rounds: a remainder within the f32 values' rounding adds no tick (RATE_ROUNDING).
        var interval = Math.max(1, ticks - whole > ticks * RATE_ROUNDING ? whole + 1 : whole);
        capped.maxRate = Math.min(rate, stepTickHz / (interval * wireSteps));
      }
    }
    for (actuator in robot.actuators)
      if (actuator.fullStepsPerRevolution > 0.0 && !wired.exists(actuator.id))
        throw 'Stepper actuator "${actuator.id}" has no channel in the device layout';
    // RKD6 skew comparison normalizes pulse counts by ratio, but carries no
    // coordinate zeros. Equal zeros cancel; distinct zeros need a wire extension.
    for (first in 0...bound.length) for (second in first + 1...bound.length) {
      var a = bound[first], b = bound[second];
      if (a.jointIndex == b.jointIndex && (a.skewBound > 0 || b.skewBound > 0) &&
          Math.abs(a.offset - b.offset) > 1e-12)
        throw "RKD6 skew groups require equal leader-coordinate offsets";
    }
    return {model: tightened, channels: bound, wired: wired};
  }

  public static function bind(robot:RobotModel, layout:DeviceLayout, stepTickHz:Int):DeviceBinding {
    var rates = bindActuators(robot, layout, stepTickHz);
    var tightened = rates.model, bound = rates.channels, wired = rates.wired;
    var inputs:Array<BoundInput> = [];
    var seen = new Map<String, Bool>();
    if (layout.inputs.length > 64) throw "Device layout supports at most 64 inputs";
    for (input in layout.inputs) {
      if (input.index != inputs.length || seen.exists(input.switchId)) throw "Device input indices or switch IDs are duplicated";
      seen.set(input.switchId, true);
      var matching = [for (contact in robot.switches) if (contact.id == input.switchId) contact];
      if (matching.length != 1) throw "Device input must name one model switch";
      var actuatorChannel = wired.get(input.actuator);
      if (actuatorChannel == null) throw "Device input names an unwired actuator";
      var channel = bound[actuatorChannel];
      var contact = matching[0];
      if (robot.joints[channel.jointIndex].id != contact.joint)
        throw "Device input actuator does not drive its switch axis";
      var actuator = find(robot, input.actuator);
      if (actuator == null) throw "Device input actuator is missing";
      var shaft = switch actuator.transmission { case SimpleTransmission(joint, _, _): joint; };
      if (contact.driveJoint != null && shaft != contact.driveJoint)
        throw "Device input actuator does not match its physical switch side";
      inputs.push(new BoundInput(input, actuatorChannel));
    }
    for (contact in robot.switches) if (!seen.exists(contact.id))
      throw "Model switch has no deployment input: " + contact.id;
    tightened.materializeLimits();
    return new DeviceBinding(bound, tightened, stepTickHz, inputs);
  }

  /** Ideal physical switches, quantized conservatively to an emitted step boundary. */
  public function virtualInputs():Array<robotkit.runtime.VirtualInputOptions> {
    var result:Array<robotkit.runtime.VirtualInputOptions> = [];
    for (input in inputs) {
      var matched = [for (contact in model.switches) if (contact.id == input.wiring.switchId) contact];
      if (matched.length != 1) throw "Virtual input has no unique model switch";
      var contact = matched[0];
      var channel = channels[input.actuatorChannel];
      var raw = channel.ratio * (contact.trip - channel.offset) * channel.stepsPerUnit;
      if (!Math.isFinite(raw) || Math.abs(raw) > 9007199254740991.0)
        throw "Virtual switch step threshold exceeds exact integer precision";
      var above = contact.side * channel.ratio > 0;
      var quantized = above ? Math.fceil(raw) : Math.ffloor(raw);
      result.push(new robotkit.runtime.VirtualInputOptions(contact.id, input.actuatorChannel,
        haxe.Int64.fromFloat(quantized), above, input.wiring.activeHigh));
    }
    return result;
  }

  /** The channels as an in-process virtual device's actuators. */
  public function virtualActuators():Array<VirtualActuatorOptions>
    return [for (channel in channels) new VirtualActuatorOptions(channel.actuatorId,
      channel.jointIndex, channel.ratio, channel.offset, channel.stepsPerUnit, channel.maxRate,
      channel.directionSetupTicks, channel.skewBound, channel.feedbackJointIndex,
      channel.feedbackRatio, channel.feedbackOffset)];

  static function find(robot:RobotModel, id:String):Null<Actuator> {
    for (actuator in robot.actuators) if (actuator.id == id) return actuator;
    return null;
  }
}
