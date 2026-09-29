package robotkit.deployment;

import haxe.Json;
import haxe.io.Path;
import haxe.crypto.Sha256;
import visionkit.CameraCalibration;
import robotkit.device.DeviceLayout;
import robotkit.device.DeviceFingerprint;
import robotkit.model.RobotModel;
import robotkit.model.RobotModelCodec;
import robotkit.world.ProcessChannelDeclaration;
import robotkit.world.ProcessEventValue;

/** A canonical semantic robot model plus its physical device configuration. */
class SerialDeployment {
  public final robot:RobotModel;
  public final serialPath:String;
  public final baud:Int;
  public final fingerprint:String;
  public final targetError:Float;
  public final ownerPeriodNs:haxe.Int64;
  public final processingAllowanceNs:haxe.Int64;
  public final protocol:String;
  public final stepTickHz:Int;
  public final linkLossTimeoutNs:haxe.Int64;
  public final clockSyncBoundNs:haxe.Int64;
  public final channels:Array<ProcessChannelDeclaration>;
  public final cameras:Map<String, CameraCalibration>;

  public function new(path:String) {
    var directory = Path.directory(path);
    var config:Dynamic = Json.parse(sys.io.File.getContent(path));
    var version:Dynamic = Reflect.field(config, "schemaVersion");
    if (version != 3 && version != 4 && version != 5) throw "robotd: unsupported deployment schema version";
    var modelPath = Path.join([directory, requiredString(config, "model")]);
    robot = RobotModelCodec.decode(sys.io.File.getBytes(modelPath));
    cameras = new Map();
    var cameraRows:Dynamic = Reflect.field(config, "cameras");
    if (cameraRows != null) {
      if (version != 5) throw "robotd: deployment cameras require schema version 5";
      if (!Std.isOfType(cameraRows, Array)) throw "robotd: deployment cameras must be an array";
      for (entry in (cast cameraRows:Array<Dynamic>)) {
        if (entry == null) throw "robotd: deployment camera cannot be null";
        for (field in Reflect.fields(entry))
          if (["sensorId", "calibration", "sha256"].indexOf(field) < 0)
            throw 'robotd: unknown camera field $field';
        var sensorId = requiredString(entry, "sensorId");
        if (cameras.exists(sensorId)) throw 'robotd: duplicate camera $sensorId';
        var sensor:Null<robotkit.model.Sensor> = null;
        for (candidate in robot.sensors) if (candidate.id == sensorId) sensor = candidate;
        if (sensor == null || sensor.kind != "camera")
          throw 'robotd: camera $sensorId must name a camera sensor';
        var hash = requiredString(entry, "sha256");
        if (!~/^[0-9a-f]{64}$/.match(hash)) throw 'robotd: camera $sensorId requires lowercase SHA-256';
        var calibrationPath = Path.join([directory, requiredString(entry, "calibration")]);
        var bytes = sys.io.File.getBytes(calibrationPath);
        if (sha256Hex(bytes) != hash)
          throw 'robotd: camera $sensorId calibration SHA-256 mismatch';
        cameras.set(sensorId, CameraCalibration.fromJson(bytes.toString()));
      }
    }
    channels = [];
    var declared:Dynamic = Reflect.field(config, "channels");
    if (declared != null) {
      if (!Std.isOfType(declared, Array))
        throw "robotd: deployment channels must be an array";
      var channelRows:Array<Dynamic> = cast declared;
      for (entry in channelRows) {
        if (entry == null) throw "robotd: deployment channel cannot be null";
        var id = requiredString(entry, "id");
        for (existing in channels)
          if (existing.id == id) throw 'robotd: duplicate channel $id';
        var safe:Dynamic = Reflect.field(entry, "safeValue");
        if (safe == null) throw 'robotd: channel $id needs a safe value';
        var kind = requiredString(safe, "kind");
        var value:ProcessEventValue = switch kind {
          case "digital":
            var digital:Dynamic = Reflect.field(safe, "digital");
            if (!Std.isOfType(digital, Bool)) throw 'robotd: channel $id safe digital must be a bool';
            ProcessEventValue.Digital(digital);
          case "analog":
            var analog:Dynamic = Reflect.field(safe, "analog");
            if (!Std.isOfType(analog, Int) && !Std.isOfType(analog, Float))
              throw 'robotd: channel $id safe analog must be a number';
            ProcessEventValue.Analog(analog);
          case "process":
            var argument:Dynamic = Reflect.field(safe, "argument");
            if (!Std.isOfType(argument, Int) && !Std.isOfType(argument, Float))
              throw 'robotd: channel $id safe process argument must be a number';
            ProcessEventValue.Process(requiredString(safe, "command"), argument);
          default: throw 'robotd: unsupported channel kind $kind';
        };
        channels.push(new ProcessChannelDeclaration(id, value));
      }
    }
    var device:Dynamic = Reflect.field(config, "device");
    if (device == null) throw "robotd: deployment requires device";
    var declaredProtocol:Dynamic = Reflect.field(device, "protocol");
    if (version == 3 && declaredProtocol != "rkd6")
      throw "robotd: v3 deployment requires rkd6; rkd5 is unsupported";
    if ((version == 4 || version == 5) && declaredProtocol != null)
      throw "robotd: v4/v5 deployment must omit protocol (RKD6 is implied)";
    protocol = "rkd6";
    if (version == 3 || version == 4 || version == 5) {
      var stepRate:Dynamic = Reflect.field(device, "step_tick_hz");
      if (!Std.isOfType(stepRate, Int) || stepRate <= 0)
        throw "robotd: deployment step_tick_hz must be a positive integer";
      stepTickHz = stepRate;
      linkLossTimeoutNs = requiredNanoseconds(device, "link_loss_timeout_ns");
      clockSyncBoundNs = requiredNanoseconds(device, "clock_sync_bound_ns");
    }
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
    ownerPeriodNs = requiredNanoseconds(device, "owner_period_ns");
    processingAllowanceNs = requiredNanoseconds(device, "processing_allowance_ns");

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

  static function sha256Hex(bytes:haxe.io.Bytes):String {
    var digest = Sha256.make(bytes);
    var result = new StringBuf();
    for (i in 0...digest.length) result.add(StringTools.hex(digest.get(i), 2).toLowerCase());
    return result.toString();
  }

  static function requiredNanoseconds(value:Dynamic, field:String):haxe.Int64 {
    var source:Dynamic = Reflect.field(value, field);
    if (!Std.isOfType(source, Int) || source <= 0)
      throw 'robotd: deployment $field must be a positive integer nanosecond count';
    return haxe.Int64.ofInt(source);
  }
}
