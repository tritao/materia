package robotkit.deployment;

import haxe.Json;
import haxe.io.Path;
import haxe.crypto.Sha256;
import visionkit.CameraCalibration;
import robotkit.device.DeviceBinding;
import robotkit.device.DeviceLayout;
import robotkit.model.RobotModel;
import robotkit.model.RobotModelCodec;
import robotkit.execution.ProcessChannelDeclaration;
import robotkit.execution.ProcessEventValue;
import robotkit.perception.PerceptionPipelineRegistry;

/**
 * A canonical semantic robot model plus its physical device configuration: which board it is for
 * (`controller`) and how its channels are wired (`binding`, derived from the model and the layout).
 */
class SerialDeployment {
  public final robot:RobotModel;
  public final profile:robotkit.profile.RobotProfile;
  /** The channel layout derived from the model and the wiring, with the tightened model to plan on. */
  public final binding:DeviceBinding;
  public final layout:DeviceLayout;
  public final serialPath:String;
  public final baud:Int;
  /** Unique id of the board this deployment is for: 32 lowercase hex digits. */
  public final controller:String;
  public final targetError:Float;
  public final ownerPeriodNs:haxe.Int64;
  public final processingAllowanceNs:haxe.Int64;
  public final protocol:String;
  public final stepTickHz:Int;
  public final linkLossTimeoutNs:haxe.Int64;
  public final clockSyncBoundNs:haxe.Int64;
  public final channels:Array<ProcessChannelDeclaration>;
  public final cameras:Map<String, CameraCalibration>;
  public final perception:Array<PerceptionPipelineConfig>;

  public function openRobot(id:robotkit.core.RobotId):robotkit.serial.SerialRobot
    return new robotkit.serial.SerialRobot(id, robot, profile, serialPath, controller,
      layout, targetError, baud, ownerPeriodNs, processingAllowanceNs, channels,
      stepTickHz, linkLossTimeoutNs, clockSyncBoundNs);

  public function new(path:String) {
    var directory = Path.directory(path);
    var config:Dynamic = Json.parse(sys.io.File.getContent(path));
    var version:Dynamic = Reflect.field(config, "schemaVersion");
    if (version != 6) throw "Unsupported robot deployment schema version";
    var modelPath = Path.join([directory, requiredString(config, "model")]);
    robot = RobotModelCodec.decode(sys.io.File.getBytes(modelPath));
    profile = robotkit.profile.RobotProfileCodec.fromRecord(Reflect.field(config, "profile"));
    cameras = new Map();
    var cameraRows:Dynamic = Reflect.field(config, "cameras");
    if (cameraRows != null) {
      if (!Std.isOfType(cameraRows, Array)) throw "robotd: deployment cameras must be an array";
      for (entry in (cast cameraRows:Array<Dynamic>)) {
        if (entry == null) throw "robotd: deployment camera cannot be null";
        exactKeys(entry, ["sensorId", "calibration", "sha256"], "camera");
        var sensorId = requiredString(entry, "sensorId");
        if (cameras.exists(sensorId)) throw 'robotd: duplicate camera $sensorId';
        var sensor:Null<robotkit.model.Sensor> = null;
        for (candidate in robot.sensors) if (candidate.id == sensorId) sensor = candidate;
        if (sensor == null || sensor.kind != "camera")
          throw 'robotd: camera $sensorId must name a camera sensor';
        var hash = requiredString(entry, "sha256");
        if (!~/^[0-9a-f]{64}$/.match(hash)) throw 'robotd: camera $sensorId requires lowercase SHA-256';
        var calibrationName = requiredString(entry, "calibration");
        var normalizedName = StringTools.replace(calibrationName, "\\", "/");
        if (StringTools.startsWith(normalizedName, "/") ||
            ~/^[A-Za-z]:/.match(normalizedName) ||
            normalizedName.split("/").indexOf("..") >= 0)
          throw 'robotd: camera $sensorId calibration must stay within the deployment directory';
        var calibrationPath = Path.join([directory, calibrationName]);
        var deploymentDirectory = sys.FileSystem.fullPath(directory);
        var resolvedCalibration = sys.FileSystem.fullPath(calibrationPath);
        var directoryPrefix = StringTools.endsWith(deploymentDirectory, "/") ?
          deploymentDirectory : deploymentDirectory + "/";
        if (!StringTools.startsWith(resolvedCalibration, directoryPrefix))
          throw 'robotd: camera $sensorId calibration resolves outside the deployment directory';
        var bytes:haxe.io.Bytes;
        try bytes = sys.io.File.getBytes(calibrationPath)
        catch (_:Dynamic) throw 'robotd: camera $sensorId calibration file cannot be read';
        if (sha256Hex(bytes) != hash)
          throw 'robotd: camera $sensorId calibration SHA-256 mismatch';
        var parsedCalibration:CameraCalibration;
        try parsedCalibration = CameraCalibration.fromJson(bytes.toString())
        catch (_:Dynamic) throw 'robotd: camera $sensorId calibration JSON is invalid';
        cameras.set(sensorId, parsedCalibration);
      }
    }
    perception = [];
    var configured:Dynamic = Reflect.field(config, "perception");
    if (configured != null) {
      if (!Std.isOfType(configured, Array)) throw "robotd: perception must be an array";
      var rows:Array<Dynamic> = cast configured;
      for (entry in rows) {
        if (entry == null) throw "robotd: perception entry cannot be null";
        exactKeys(entry, ["id", "input", "pipeline", "model", "modelSha256", "host", "consumers", "options"], "perception");
        var id = requiredString(entry, "id");
        for (existing in perception) if (existing.id == id) throw 'robotd: duplicate perception id $id';
        var input = requiredString(entry, "input");
        var sensor:Null<robotkit.model.Sensor> = null;
        for (candidate in robot.sensors) if (candidate.id == input) sensor = candidate;
        if (sensor == null || sensor.kind != "camera" || sensor.frame == null)
          throw 'robotd: perception input $input must name a camera with an existing frame';
        var pipeline = requiredString(entry, "pipeline");
        if (!PerceptionPipelineRegistry.supports(pipeline))
          throw 'robotd: unknown perception pipeline $pipeline';
        var modelFile = Path.join([directory, requiredString(entry, "model")]);
        var digest = requiredString(entry, "modelSha256").toLowerCase();
        if (!~/^[0-9a-f]{64}$/.match(digest) || sha256Hex(sys.io.File.getBytes(modelFile)) != digest)
          throw 'robotd: perception model SHA-256 mismatch for $id';
        var host = requiredString(entry, "host");
        if (host != "robotd" && host != "worldd") throw 'robotd: unsupported perception host $host';
        var consumersValue:Dynamic = Reflect.field(entry, "consumers");
        if (!Std.isOfType(consumersValue, Array)) throw 'robotd: perception consumers for $id must be an array';
        var consumerRows:Array<Dynamic> = cast consumersValue;
        if (consumerRows.length == 0) throw 'robotd: perception $id needs consumers';
        var consumers:Array<String> = [];
        for (consumer in consumerRows) {
          if (consumer != "local" && consumer != "worldd") throw 'robotd: unsupported perception consumer $consumer';
          if (consumers.indexOf(consumer) >= 0) throw 'robotd: duplicate perception consumer $consumer';
          consumers.push(consumer);
        }
        if (host == "worldd" && consumers.indexOf("local") >= 0)
          throw 'robotd: worldd perception cannot serve a local consumer without a network round trip';
        var options:Dynamic = Reflect.field(entry, "options");
        if (options == null) options = {};
        exactKeys(options, ["scoreThreshold", "maxRateHz", "iouThreshold", "threads",
          "dynamicWidth", "dynamicHeight"], "perception options");
        var score = optionalNumber(options, "scoreThreshold", 0.4);
        var rate = optionalNumber(options, "maxRateHz", 0.0);
        var iou = optionalNumber(options, "iouThreshold", 0.5);
        var threads = optionalNumber(options, "threads", 1);
        var dynamicWidth = optionalNumber(options, "dynamicWidth", 0);
        var dynamicHeight = optionalNumber(options, "dynamicHeight", 0);
        if (score < 0 || score > 1 || iou < 0 || iou > 1 || rate < 0)
          throw 'robotd: perception options for $id are out of range';
        if (threads < 1 || threads > 64 || threads != Math.floor(threads) ||
            dynamicWidth < 0 || dynamicWidth > 8192 || dynamicWidth != Math.floor(dynamicWidth) ||
            dynamicHeight < 0 || dynamicHeight > 8192 || dynamicHeight != Math.floor(dynamicHeight))
          throw 'robotd: inference dimensions or threads for $id are invalid';
        perception.push(new PerceptionPipelineConfig(id, input, pipeline, modelFile,
          digest, host, consumers, score, rate, iou,
          Std.int(threads), Std.int(dynamicWidth), Std.int(dynamicHeight)));
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
        var keep:Dynamic = Reflect.field(entry, "keepOnStop");
        if (keep != null && !Std.isOfType(keep, Bool)) throw 'robotd: channel $id keepOnStop must be a bool';
        channels.push(new ProcessChannelDeclaration(id, value, keep == true));
      }
    }
    var device:Dynamic = Reflect.field(config, "device");
    if (device == null) throw "robotd: deployment requires device";
    if (Reflect.field(device, "protocol") != null)
      throw "robotd: deployment must omit protocol (RKD6 is implied)";
    if (Reflect.field(device, "fingerprint") != null)
      throw "robotd: device.fingerprint is gone; name the board with device.controller (see `robotd identify`)";
    protocol = "rkd6";
    var stepRate:Dynamic = Reflect.field(device, "step_tick_hz");
    if (!Std.isOfType(stepRate, Int) || stepRate <= 0)
      throw "robotd: deployment step_tick_hz must be a positive integer";
    stepTickHz = stepRate;
    linkLossTimeoutNs = requiredNanoseconds(device, "link_loss_timeout_ns");
    clockSyncBoundNs = requiredNanoseconds(device, "clock_sync_bound_ns");
    serialPath = requiredString(device, "path");
    var baudValue:Dynamic = Reflect.field(device, "baud");
    if (!Std.isOfType(baudValue, Int))
      throw "robotd: deployment baud must be an integer";
    baud = baudValue;
    if ([115200, 230400, 460800, 921600].indexOf(baud) < 0)
      throw "robotd: deployment baud is unsupported";
    var declaredController = requiredString(device, "controller");
    if (!~/^[0-9a-fA-F]{32}$/.match(declaredController) ||
        declaredController.toLowerCase() == "00000000000000000000000000000000")
      throw "robotd: deployment requires a nonzero 32-digit controller id";
    controller = declaredController.toLowerCase();
    var errorValue:Dynamic = Reflect.field(device, "target_error");
    if (!Std.isOfType(errorValue, Int) && !Std.isOfType(errorValue, Float))
      throw "robotd: deployment target_error must be a number";
    targetError = errorValue;
    if (!Math.isFinite(targetError) || targetError < 0)
      throw "robotd: deployment target_error must be finite and nonnegative";
    ownerPeriodNs = requiredNanoseconds(device, "owner_period_ns");
    processingAllowanceNs = requiredNanoseconds(device, "processing_allowance_ns");

    var layoutPath = Path.join([directory, requiredString(device, "layout")]);
    layout = DeviceLayout.decode(sys.io.File.getBytes(layoutPath));
    binding = DeviceBinding.bind(robot, layout, stepTickHz);
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

  static function exactKeys(value:Dynamic, allowed:Array<String>, section:String):Void {
    for (key in Reflect.fields(value))
      if (allowed.indexOf(key) < 0) throw 'robotd: unknown $section key $key';
  }

  static function optionalNumber(value:Dynamic, field:String, fallback:Float):Float {
    var source:Dynamic = Reflect.field(value, field);
    if (source == null) return fallback;
    if ((!Std.isOfType(source, Int) && !Std.isOfType(source, Float)) || !Math.isFinite(source))
      throw 'robotd: perception $field must be a finite number';
    return source;
  }
}
