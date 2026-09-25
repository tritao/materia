package robotd;

import haxe.Json;
import haxe.io.Path;
import robotkit.model.Joint;
import robotkit.model.JointType;
import robotkit.model.Link;
import robotkit.model.RobotModel;

/** A single deployed model and its ordered RKD5 channel layout. */
class RobotDeployment {
  public final robot:RobotModel;
  public final serialPath:String;
  public final baud:Int;
  public final fingerprint:String;
  public final targetError:Float;

  public function new(path:String) {
    var directory = Path.directory(path);
    var config:Dynamic = Json.parse(sys.io.File.getContent(path));
    var name:String = requiredString(config, "name");
    var device:Dynamic = Reflect.field(config, "device");
    if (device == null) throw "robotd: deployment requires device";
    serialPath = requiredString(device, "path");
    var baudValue:Dynamic = Reflect.field(device, "baud");
    if (!Std.isOfType(baudValue, Int))
      throw "robotd: deployment baud must be an integer";
    baud = baudValue;
    if ([115200, 230400, 460800, 921600].indexOf(baud) < 0)
      throw "robotd: deployment baud is unsupported";
    fingerprint = requiredString(device, "fingerprint");
    if (!~/^[0-9a-fA-F]{32}$/.match(fingerprint) ||
        fingerprint.toLowerCase() == "00000000000000000000000000000000")
      throw "robotd: deployment requires a nonzero 32-digit fingerprint";
    var errorValue:Dynamic = Reflect.field(device, "target_error");
    if (!Std.isOfType(errorValue, Int) && !Std.isOfType(errorValue, Float))
      throw "robotd: deployment target_error must be a number";
    targetError = errorValue;
    if (!Math.isFinite(targetError) || targetError < 0)
      throw "robotd: deployment target_error must be finite and nonnegative";

    var layoutPath = Path.join([directory, requiredString(config, "layout")]);
    var layout:Dynamic = Json.parse(sys.io.File.getContent(layoutPath));
    var channels:Array<Dynamic> = Reflect.field(layout, "channels");
    var joints:Array<Dynamic> = Reflect.field(config, "joints");
    if (joints == null || channels == null || joints.length == 0 ||
        joints.length != channels.length || joints.length > 64)
      throw "robotd: deployment joints and layout channels must have matching counts from 1 to 64";
    robot = new RobotModel(name);
    var base = robot.addLink(new Link("base", "link/base"));
    var used = new Map<String, Bool>();
    for (index in 0...joints.length) {
      var record = joints[index];
      var id = requiredString(record, "id");
      var channel = channels[index];
      var channelIndex:Dynamic = Reflect.field(channel, "index");
      if (requiredString(channel, "joint") != id || !Std.isOfType(channelIndex, Int) ||
          channelIndex != index)
        throw 'robotd: layout channel $index must match joint $id';
      if (used.exists(id)) throw 'robotd: duplicate joint $id';
      used.set(id, true);
      var kind = requiredString(record, "type");
      var jointType:JointType = switch (kind) {
        case "continuous": JointType.Continuous;
        case "revolute": JointType.Revolute;
        case "prismatic": JointType.Prismatic;
        case _: throw 'robotd: unsupported deployed joint type $kind';
      };
      var link = robot.addLink(new Link(requiredString(record, "name"), 'link/$id'));
      var joint = robot.addJoint(new Joint(requiredString(record, "name"), jointType,
        base, link, id));
      var limits:Dynamic = Reflect.field(record, "limits");
      if (limits != null) {
        if (Reflect.hasField(limits, "lower")) joint.limits.lower = Reflect.field(limits, "lower");
        if (Reflect.hasField(limits, "upper")) joint.limits.upper = Reflect.field(limits, "upper");
        if (Reflect.hasField(limits, "velocity")) joint.limits.velocity = Reflect.field(limits, "velocity");
        if (Reflect.hasField(limits, "effort")) joint.limits.effort = Reflect.field(limits, "effort");
      }
    }
  }

  static function requiredString(value:Dynamic, field:String):String {
    var result:Dynamic = Reflect.field(value, field);
    if (!Std.isOfType(result, String) || StringTools.trim(result).length == 0)
      throw 'robotd: deployment requires $field';
    return result;
  }
}
