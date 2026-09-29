package robotkit.deployment;

import haxe.Json;
import haxe.io.Path;
import robotkit.device.DeviceLayout;
import robotkit.device.DeviceFingerprint;
import robotkit.model.RobotModel;
import robotkit.model.RobotModelCodec;
import robotkit.world.ProcessChannelDeclaration;
import robotkit.world.ProcessEventValue;
import robotkit.inference.InferenceSession;
import robotkit.perception.PerceptionPipelineRegistry;

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
  public final perception:Array<PerceptionPipelineConfig>;

  public function new(path:String) {
    var directory = Path.directory(path);
    var config:Dynamic = Json.parse(sys.io.File.getContent(path));
    var version:Dynamic = Reflect.field(config, "schemaVersion");
    if (version != 3 && version != 4 && version != 5) throw "robotd: unsupported deployment schema version";
    var modelPath = Path.join([directory, requiredString(config, "model")]);
    robot = RobotModelCodec.decode(sys.io.File.getBytes(modelPath));
    perception = [];
    var configured:Dynamic = Reflect.field(config, "perception");
    if (configured != null) {
      if (version != 5) throw "robotd: perception requires deployment schema version 5";
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
        if (!~/^[0-9a-f]{64}$/.match(digest) || InferenceSession.modelDigest(modelFile) != digest)
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
        exactKeys(options, ["scoreThreshold", "maxRateHz", "iouThreshold"], "perception options");
        var score = optionalNumber(options, "scoreThreshold", 0.4);
        var rate = optionalNumber(options, "maxRateHz", 0.0);
        var iou = optionalNumber(options, "iouThreshold", 0.5);
        if (score < 0 || score > 1 || iou < 0 || iou > 1 || rate < 0)
          throw 'robotd: perception options for $id are out of range';
        perception.push(new PerceptionPipelineConfig(id, input, pipeline, modelFile,
          digest, host, consumers, score, rate, iou));
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
