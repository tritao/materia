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

  public function new(channels:Array<DeviceChannel>) {
    this.channels = channels;
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
        direction_setup_ticks: channel.directionSetupTicks, skew_bound: channel.skewBound}]}));
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
    return new DeviceLayout(channels);
  }

}
