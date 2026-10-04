package robotkit.profile;

import haxe.Json;
import haxe.io.Bytes;

/** The single saved profile format. Binding to mechanical IDs is checked by the compiler. */
class RobotProfileCodec {
  public static function encode(profile:RobotProfile):Bytes
    return Bytes.ofString(Json.stringify(toRecord(profile)));
  public static function decode(bytes:Bytes):RobotProfile return fromRecord(Json.parse(bytes.toString()));
  public static function toRecord(profile:RobotProfile):Dynamic {
    if (profile == null) throw "Robot profile is required";
    var result:Dynamic = {schemaVersion:RobotProfile.CURRENT_VERSION,
      mobileBase:profile.mobileBase == null ? null : mobileRecord(profile.mobileBase),
      forkMechanism:profile.forkMechanism == null ? null : forkRecord(profile.forkMechanism)};
    // Validate authored values by the same contract used to read them.
    fromRecord(result);
    return result;
  }
  public static function fromRecord(value:Dynamic):RobotProfile {
    var version:Dynamic = value == null ? null : Reflect.field(value, "schemaVersion");
    if (!Std.isOfType(version, Int) || version != RobotProfile.CURRENT_VERSION)
      throw "Unsupported RobotProfile schema version";
    for (key in Reflect.fields(value))
      if (key != "schemaVersion" && key != "mobileBase" && key != "forkMechanism")
        throw "Invalid RobotProfile field";
    var mobile = required(value, "mobileBase"), fork = required(value, "forkMechanism");
    return new RobotProfile(mobile == null ? null : readMobile(mobile),
      fork == null ? null : readFork(fork));
  }
  static function mobileRecord(value:RobotMobileConfiguration):Dynamic return {
    drive:driveRecord(value.drive),
    maxLinearSpeed:value.maxLinearSpeed, maxAngularSpeed:value.maxAngularSpeed,
    maxLinearAcceleration:value.maxLinearAcceleration, maxAngularAcceleration:value.maxAngularAcceleration,
    footprintLength:value.footprintLength, footprintWidth:value.footprintWidth
  };
  static function driveRecord(value:RobotDriveConfiguration):Dynamic return switch value {
      case Differential(left, right, radius, track):
        {kind:"differential", leftWheelJoint:left, rightWheelJoint:right, wheelRadius:radius, trackWidth:track};
      case Ackermann(steering, drive, wheelBase, radius, angle):
        {kind:"ackermann", steeringJoint:steering, driveWheelJoint:drive,
          wheelBase:wheelBase, wheelRadius:radius, maxSteeringAngle:angle};
      case Holonomic(wheels, radius, baseRadius):
        {kind:"holonomic", wheelJointIds:wheels.copy(), wheelRadius:radius, baseRadius:baseRadius};
      case _: throw "Invalid RobotProfile drive";
  };
  static function readMobile(value:Dynamic):RobotMobileConfiguration {
    var source = required(value, "drive");
    var drive:RobotDriveConfiguration = switch text(source, "kind") {
      case "differential": Differential(text(source, "leftWheelJoint"), text(source, "rightWheelJoint"),
        positive(source, "wheelRadius"), positive(source, "trackWidth"));
      case "ackermann": Ackermann(text(source, "steeringJoint"), text(source, "driveWheelJoint"),
        positive(source, "wheelBase"), positive(source, "wheelRadius"), positive(source, "maxSteeringAngle"));
      case "holonomic":
        var raw:Dynamic = required(source, "wheelJointIds");
        if (!Std.isOfType(raw, Array)) throw "Invalid RobotProfile wheels";
        var wheels:Array<String> = [];
        for (item in (cast raw:Array<Dynamic>)) {
          if (!Std.isOfType(item, String) || StringTools.trim(item).length == 0 || wheels.contains(item))
            throw "Invalid RobotProfile wheels";
          wheels.push(item);
        }
        if (wheels.length != 3) throw "Invalid RobotProfile wheels";
        Holonomic(wheels, positive(source, "wheelRadius"), positive(source, "baseRadius"));
      case _: throw "Invalid RobotProfile drive";
    };
    var length = optionalPositive(value, "footprintLength"), width = optionalPositive(value, "footprintWidth");
    if ((length == null) != (width == null)) throw "Invalid RobotProfile footprint";
    return new RobotMobileConfiguration(drive, positive(value, "maxLinearSpeed"),
      positive(value, "maxAngularSpeed"), positive(value, "maxLinearAcceleration"),
      positive(value, "maxAngularAcceleration"), length, width);
  }
  static function forkRecord(value:RobotForkConfiguration):Dynamic return {
    liftJoint:value.liftJointId, tiltJoint:value.tiltJointId, spreadJoint:value.spreadJointId,
    maxMassKg:value.maxMassKg, maxLoadMomentKgMeters:value.maxLoadMomentKgMeters,
    maxLiftHeightMeters:value.maxLiftHeightMeters
  };
  static function readFork(value:Dynamic):RobotForkConfiguration
    return new RobotForkConfiguration(text(value, "liftJoint"), positive(value, "maxMassKg"),
      positive(value, "maxLoadMomentKgMeters"), positive(value, "maxLiftHeightMeters"),
      optionalText(value, "tiltJoint"), optionalText(value, "spreadJoint"));
  static function required(value:Dynamic, name:String):Dynamic {
    if (value == null || !Reflect.hasField(value, name)) throw "Missing RobotProfile field";
    return Reflect.field(value, name);
  }
  static function text(value:Dynamic, name:String):String {
    var result:Dynamic = required(value, name);
    if (!Std.isOfType(result, String) || StringTools.trim(result).length == 0)
      throw "Invalid RobotProfile field";
    return result;
  }
  static function optionalText(value:Dynamic, name:String):Null<String>
    return required(value, name) == null ? null : text(value, name);
  static function positive(value:Dynamic, name:String):Float {
    var result:Dynamic = required(value, name);
    if ((!Std.isOfType(result, Int) && !Std.isOfType(result, Float)) ||
        !Math.isFinite(result) || result <= 0.0) throw "Invalid RobotProfile value";
    return result;
  }
  static function optionalPositive(value:Dynamic, name:String):Null<Float>
    return required(value, name) == null ? null : positive(value, name);
}
