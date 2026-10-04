package robotkit.device;

import haxe.Json;
import haxe.io.Bytes;
import robotkit.model.RobotModel;

/**
 * Physical-channel mapping and driver wiring, kept separate from the semantic RobotModel. Each
 * wired channel names a model actuator and its direction; DeviceBinding joins them.
 */
class DeviceLayout {
  public final channels:Array<DeviceChannel>;
  public final inputs:Array<DeviceInput>;

  public function new(channels:Array<DeviceChannel>, ?inputs:Array<DeviceInput>) {
    this.channels = channels;
    this.inputs = inputs == null ? [] : inputs;
  }

  public static inline final VERSION:Int = 1;

  /** One channel per actuator; dual drives inherit their leader's stated racking tolerance. */
  public static function forActuators(model:RobotModel, directionSetupTicks:Int = 0):DeviceLayout {
    if (model == null) throw "Device layout requires a model";
    var mappings = [for (actuator in model.actuators) DeviceTransmission.of(model, actuator)];
    var channels:Array<DeviceChannel> = [];
    for (index in 0...model.actuators.length) {
      var mapping = mappings[index];
      var peers = 0;
      if (mapping.singleLeader) for (candidate in mappings)
        if (candidate.singleLeader && candidate.jointIndex == mapping.jointIndex) peers++;
      var bound = 0.0;
      if (peers > 1) {
        var leader = model.joints[mapping.jointIndex];
        var limits = leader.mechanicalLimits == null ? leader.limits : leader.mechanicalLimits;
        bound = limits.rackingTolerance;
        if (!Math.isFinite(bound) || bound < 0) throw "Device racking tolerance must be finite and non-negative";
      }
      channels.push(new DeviceChannel(index, model.actuators[index].id, 1, directionSetupTicks, bound));
    }
    return new DeviceLayout(channels);
  }

  public static function encode(layout:DeviceLayout):Bytes {
    var bytes = Bytes.ofString(Json.stringify({schemaVersion: VERSION, channels: [for (channel in layout.channels)
      {index: channel.index, actuator: channel.actuator, direction: channel.direction,
        direction_setup_ticks: channel.directionSetupTicks, skew_bound: channel.skewBound}],
      inputs: [for (input in layout.inputs) {index: input.index, switch_id: input.switchId,
        actuator: input.actuator, active_high: input.activeHigh, pin: input.pin}]}));
    var validated = decode(bytes);
    return bytes;
  }

  /** Pulse channels cover only steppers; servo limits remain in their motor/driver model. */
  public static function forSteppers(model:RobotModel, directionSetupTicks:Int = 0):DeviceLayout {
    var channels:Array<DeviceChannel> = [];
    for (actuator in model.actuators) if (actuator.fullStepsPerRevolution > 0) {
      channels.push(new DeviceChannel(channels.length, actuator.id, 1, directionSetupTicks));
    }
    return new DeviceLayout(channels);
  }

  public static function decode(bytes:Bytes):DeviceLayout {
    var root:Dynamic;
    try root = Json.parse(bytes.toString()) catch (_:Dynamic)
      throw "robotd: malformed device layout JSON";
    var version:Dynamic = Reflect.field(root, "schemaVersion");
    if (version != VERSION) throw 'schema v$version is unsupported; expected v$VERSION';
    var records:Dynamic = Reflect.field(root, "channels");
    if (!Std.isOfType(records, Array)) throw "robotd: device layout requires channels";
    var channels:Array<DeviceChannel> = [];
    var channelRecords:Array<Dynamic> = cast records;
    for (record in channelRecords) {
      var index:Dynamic = Reflect.field(record, "index");
      var actuator:Dynamic = Reflect.field(record, "actuator");
      if (!Std.isOfType(index, Int) || !Std.isOfType(actuator, String) || StringTools.trim(actuator).length == 0)
        throw "robotd: device layout channel requires an integer index and an actuator ID";
      if (Reflect.hasField(record, "joint") || Reflect.hasField(record, "microsteps"))
        throw "robotd: device layout channels contain wiring only; driver settings belong to actuators";
      var direction:Dynamic = Reflect.field(record, "direction");
      if (direction == null) direction = 1;
      if (direction != 1 && direction != -1)
        throw 'robotd: device layout channel $index direction must be 1 or -1';
      var setup:Dynamic = Reflect.field(record, "direction_setup_ticks");
      if (setup == null) setup = 0;
      if (!Std.isOfType(setup, Int) || setup < 0 || setup > 65535)
        throw 'robotd: device layout channel $index direction_setup_ticks must be an integer from 0 to 65535';
      var skew:Dynamic = Reflect.field(record, "skew_bound");
      if (skew == null) skew = 0.0;
      if ((!Std.isOfType(skew, Int) && !Std.isOfType(skew, Float)) || !Math.isFinite(skew) || skew < 0)
        throw 'robotd: device layout channel $index skew_bound must be a finite nonnegative number';
      channels.push(new DeviceChannel(index, actuator, direction, setup, skew));
    }
    var inputs:Array<DeviceInput> = [];
    var inputRecords:Dynamic = Reflect.field(root, "inputs");
    if (inputRecords != null) {
      if (!Std.isOfType(inputRecords, Array)) throw "robotd: device layout inputs must be an array";
      var records:Array<Dynamic> = cast inputRecords;
      if (records.length > 64) throw "robotd: device layout supports at most 64 inputs";
      var switches = new Map<String, Bool>();
      var pins = new Map<String, Bool>();
      for (record in records) {
        var index:Dynamic = Reflect.field(record, "index");
        var id:Dynamic = Reflect.field(record, "switch_id");
        var actuator:Dynamic = Reflect.field(record, "actuator");
        var polarity:Dynamic = Reflect.field(record, "active_high");
        var pin:Dynamic = Reflect.field(record, "pin");
        if (pin == null) pin = "";
        if (!Std.isOfType(index, Int) || index != inputs.length ||
            !Std.isOfType(id, String) || StringTools.trim(id).length == 0 ||
            !Std.isOfType(actuator, String) || StringTools.trim(actuator).length == 0 ||
            !Std.isOfType(polarity, Bool) || !Std.isOfType(pin, String))
          throw "robotd: device input requires ordered index, switch ID, actuator ID and electrical polarity";
        if (switches.exists(id)) throw "robotd: duplicate switch input";
        if (pin != "" && pins.exists(pin)) throw "robotd: duplicate input pin";
        switches.set(id, true);
        if (pin != "") pins.set(pin, true);
        inputs.push(new DeviceInput(index, id, actuator, polarity, pin));
      }
    }
    return new DeviceLayout(channels, inputs);
  }

}
