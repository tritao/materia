package robotkit.device;

import haxe.Json;
import haxe.io.Bytes;
import robotkit.model.RobotModel;

/** RKD5 physical-channel mapping, kept separate from the semantic RobotModel. */
class DeviceLayout {
  public final channels:Array<DeviceChannel>;

  public function new(channels:Array<DeviceChannel>) {
    this.channels = channels;
  }

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
      if (!Std.isOfType(index, Int) || !Std.isOfType(joint, String) ||
          StringTools.trim(joint).length == 0)
        throw "robotd: device layout channel requires an integer index and joint ID";
      channels.push(new DeviceChannel(index, joint));
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
