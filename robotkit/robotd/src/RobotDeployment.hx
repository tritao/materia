package robotd;

import haxe.Json;
import haxe.io.Path;
import robotkit.device.DeviceLayout;
import robotkit.model.RobotModel;
import robotkit.model.RobotModelCodec;

/** A canonical semantic robot model plus its physical device configuration. */
class RobotDeployment {
  public final robot:RobotModel;
  public final serialPath:String;
  public final baud:Int;
  public final fingerprint:String;
  public final targetError:Float;

  public function new(path:String) {
    var directory = Path.directory(path);
    var config:Dynamic = Json.parse(sys.io.File.getContent(path));
    var version:Dynamic = Reflect.field(config, "schemaVersion");
    if (version != 1) throw "robotd: unsupported deployment schema version";
    var modelPath = Path.join([directory, requiredString(config, "model")]);
    robot = RobotModelCodec.decode(sys.io.File.getBytes(modelPath));
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

    var layoutPath = Path.join([directory, requiredString(device, "layout")]);
    var layoutBytes = sys.io.File.getBytes(layoutPath);
    var lockPath = Path.join([directory, requiredString(device, "schema_lock")]);
    var actualFingerprint = DeviceFingerprint.compute(layoutBytes,
      sys.io.File.getBytes(lockPath));
    if (actualFingerprint != fingerprint.toLowerCase())
      throw 'robotd: deployment fingerprint does not match layout and schema lock (expected $actualFingerprint)';
    DeviceLayout.decode(layoutBytes).validateAgainst(robot);
  }

  static function requiredString(value:Dynamic, field:String):String {
    var result:Dynamic = Reflect.field(value, field);
    if (!Std.isOfType(result, String) || StringTools.trim(result).length == 0)
      throw 'robotd: deployment requires $field';
    return result;
  }
}
