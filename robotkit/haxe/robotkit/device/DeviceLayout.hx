package robotkit.device;

import haxe.Json;
import haxe.io.Bytes;
import robotkit.model.RobotModel;

/**
 * Physical-channel mapping and driver wiring, kept separate from the semantic RobotModel. Each
 * wired channel names a model actuator, its direction and microstepping; DeviceBinding joins them.
 */
class DeviceLayout {
  public final channels:Array<DeviceChannel>;

  public function new(channels:Array<DeviceChannel>) {
    this.channels = channels;
  }

  /** One channel per model actuator, in order, using its driver setting. Legacy models use full steps. */
  public static function forActuators(model:RobotModel, directionSetupTicks:Int = 0):DeviceLayout
    return new DeviceLayout([for (index in 0...model.actuators.length) {
      var actuator = model.actuators[index];
      var setting = actuator.microsteps;
      new DeviceChannel(index, "", actuator.id, 1, setting == null ? 1 : setting, directionSetupTicks);
    }]);

  public static function decode(bytes:Bytes):DeviceLayout {
    var root:Dynamic;
    try root = Json.parse(bytes.toString()) catch (_:Dynamic)
      throw "robotd: malformed device layout JSON";
    var records:Dynamic = Reflect.field(root, "channels");
    if (!Std.isOfType(records, Array)) throw "robotd: device layout requires channels";
    var channels:Array<DeviceChannel> = [];
    var channelRecords:Array<Dynamic> = cast records;
    for (record in channelRecords) {
      var index:Dynamic = Reflect.field(record, "index");
      var joint:Dynamic = Reflect.field(record, "joint");
      var actuator:Dynamic = Reflect.field(record, "actuator");
      // A legacy layout names the joint; a wired one names the actuator, whose joint the model gives.
      var namesJoint = Std.isOfType(joint, String) && StringTools.trim(joint).length > 0;
      var namesActuator = Std.isOfType(actuator, String) && StringTools.trim(actuator).length > 0;
      if (!Std.isOfType(index, Int) || !(namesJoint || namesActuator))
        throw "robotd: device layout channel requires an integer index and an actuator or joint ID";
      var direction:Dynamic = Reflect.field(record, "direction");
      if (direction == null) direction = 1;
      if (direction != 1 && direction != -1)
        throw 'robotd: device layout channel $index direction must be 1 or -1';
      var microsteps:Dynamic = Reflect.field(record, "microsteps");
      if (microsteps == null) microsteps = 1;
      if (!Std.isOfType(microsteps, Int) || microsteps < 1 || microsteps > 1024)
        throw 'robotd: device layout channel $index microsteps must be an integer from 1 to 1024';
      var setup:Dynamic = Reflect.field(record, "direction_setup_ticks");
      if (setup == null) setup = 0;
      if (!Std.isOfType(setup, Int) || setup < 0 || setup > 65535)
        throw 'robotd: device layout channel $index direction_setup_ticks must be an integer from 0 to 65535';
      var skew:Dynamic = Reflect.field(record, "skew_bound");
      if (skew == null) skew = 0.0;
      if ((!Std.isOfType(skew, Int) && !Std.isOfType(skew, Float)) || !Math.isFinite(skew) || skew < 0)
        throw 'robotd: device layout channel $index skew_bound must be a finite nonnegative number';
      channels.push(new DeviceChannel(index, namesJoint ? joint : "",
        namesActuator ? actuator : null, direction, microsteps, setup, skew));
    }
    return new DeviceLayout(channels);
  }

  public function validateAgainst(model:RobotModel):Void {
    if (channels.length == 0 || channels.length > 64 || channels.length != model.joints.length)
      throw "robotd: device layout and model must have matching joint counts from 1 to 64";
    var used = new Map<String, Bool>();
    for (index in 0...channels.length) {
      var channel = channels[index];
      if (channel.index != index || channel.jointId != model.joints[index].id)
        throw 'robotd: layout channel $index must map to model joint ${model.joints[index].id}';
      if (used.exists(channel.jointId)) throw 'robotd: duplicate device channel joint ${channel.jointId}';
      used.set(channel.jointId, true);
    }
  }
}
