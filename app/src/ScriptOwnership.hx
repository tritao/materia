package app;

import nativekit.ui.core.EditOperation;
import nativekit.ui.core.EditorDocument;
import robotkit.runtime.RobotRuntimeCompiler;
import haxe.Json;
import haxe.crypto.Sha256;

typedef ScriptMaterialization = {var scene: EditorScene;
var sensors:SensorConfiguration;
var backend:Int;
var timestep:Float;
}
private typedef ScriptSettings = {var backend: Int;
var timestep:Float;
}

/** Owns a script baseline plus stable-ID document overrides. Scripts are unsandboxed compiled Haxe. */
class ScriptOwnership {
  public static inline var OVERRIDE_VERSION:Int = 2;
  public static inline var SIMULATION_TARGET:String = "simulation";
  public final document:EditorDocument;
  public var reference(default, null):String;
  public var configurationVersion(default, null):Int;
  public var configurationSha256(default, null):String;
  public var savedVersion(default, null):Int;
  public var overridesEnabled(default, null):Bool;
  public var diagnostics(default, null):Array<String> = [];
  public var staleOverrides(default, null):Array<String> = [];
  var baseline:ScriptedSetup;
  final overrides:Map<String, ScriptOverrideRecord> = new Map();

  public function new(reference:String, ? record:ScriptOwnershipRecord,
      ?sharedDocument:EditorDocument) {
    document = sharedDocument == null ? new EditorDocument("script-overrides") : sharedDocument;
    this.reference = reference;
    if (record == null) {
      overridesEnabled = false;
      savedVersion = 0;
    } else {
      var loaded:ScriptOwnershipRecord = record;
      overridesEnabled = loaded.overridesEnabled;
      savedVersion = loaded.version;
      for (item in loaded.overrides) overrides.set(key(item.targetId, item.property), copyOverride(item));
    }
    baseline = new ScriptedSetup();
    var provider = SetupScriptRegistry.provider(reference);
    try provider().evaluate(baseline) catch (error:Dynamic) {
      baseline.dispose();
      throw error;
    }
    try validateSetup(baseline, reference) catch (error:Dynamic) {
      baseline.dispose();
      throw error;
    }
    configurationVersion = baseline.version;
    configurationSha256 = contentHash(baseline);
    if (record != null) try verifyIdentity(record) catch (error:Dynamic) {
      baseline.dispose();
      throw error;
    }
    if (record != null) refreshDiagnostics();
    document.markSaved();
  }
  public function reload():ScriptMaterialization {
    var candidate = new ScriptedSetup(), previous = baseline;
    var provider = SetupScriptRegistry.provider(reference);
    try provider().evaluate(candidate) catch (error:Dynamic) {
      candidate.dispose();
      throw error;
    }
    try validateSetup(candidate, reference) catch (error:Dynamic) {
      candidate.dispose();
      throw error;
    }
    baseline = candidate;
    configurationVersion = candidate.version;
    configurationSha256 = contentHash(candidate);
    refreshDiagnostics();
    try {
      var result = materialize();
      previous.dispose();
      return result;
    } catch (error:Dynamic) {
      baseline = previous;
      configurationVersion = previous.version;
      configurationSha256 = contentHash(previous);
      candidate.dispose();
      refreshDiagnostics();
      throw error;
    }
  }
  public function materialize():ScriptMaterialization {
    var source:SensorConfiguration = baseline.sensors;
    var record:Dynamic = Json.parse(Json.stringify(source.records()));
    var sensors = new SensorConfiguration(record, document);
    try {
      var objects = cloneObjects();
      var settings:ScriptSettings = {backend: baseline.backend, timestep: baseline.timestep};
      applyOverrides(sensors, objects, settings);
      validateSensors(sensors);
      if ((settings.backend != ApplicationSimulation.DETERMINISTIC && settings.backend != ApplicationSimulation.MUJOCO)
        || !Math.isFinite(settings.timestep) || settings.timestep <= 0) throw "Invalid overridden simulation settings";
      var scene = new EditorScene(objects, document);
      return {scene: scene, sensors: sensors, backend: settings.backend, timestep: settings.timestep};
    } catch (error:Dynamic) {
      sensors.dispose();
      throw error;
    }
  }
  public function setOverridesEnabled(value:Bool):Bool {
    if (value == overridesEnabled) return false;
    var before = overridesEnabled;
    document.apply(new EditOperation(
      value ? "Enable script overrides" : "Disable script overrides",
      function() overridesEnabled = value,
      function() overridesEnabled = before
    ));
    return true;
  }
  public function setSensorRate(robotId:String, sensorId:String, value:Float):Bool {
    return setOverride(robotId + "/" + sensorId, "updateRate", "number", value);
  }
  public function revertSensorRate(robotId:String, sensorId:String):Bool {
    return revertOverride(robotId + "/" + sensorId, "updateRate");
  }
  public function sensorRateOrigin(robotId:String, sensorId:String):String return origin(
    robotId + "/" + sensorId,
    "updateRate"
  );
  public function setOverride(targetId:String, property:String, kind:String, value:Dynamic):Bool {
    if (!overridesEnabled) throw "Enable overrides before changing script-owned values";
    var next = overrideRecord(targetId, property, kind, value);
    validateOverride(next);
    var k = key(targetId, property), had = overrides.exists(k), before = overrides.get(k);
    if (had && before != null && before.kind == next.kind && before.encodedValue == next.encodedValue) return false;
    overrides.set(k, next);
    try validateCandidate(k) catch (error:Dynamic) {
      if (had) overrides.set(k, cast before);
      else overrides.remove(k);
      refreshDiagnostics();
      throw error;
    }
    if (had) overrides.set(k, cast before);
    else overrides.remove(k);
    document.apply(new EditOperation("Override " + property, function() overrides.set(k, next), function() {
      if (had) overrides.set(k, cast before);
      else overrides.remove(k);
    }
    ));
    refreshDiagnostics();
    return true;
  }
  public function revertOverride(targetId:String, property:String):Bool {
    var k = key(targetId, property);
    if (!overrides.exists(k)) return false;
    var before = overrides.get(k);
    document.apply(new EditOperation(
      "Revert " + property,
      function() overrides.remove(k),
      function() overrides.set(
        k,
        cast before
      )
    ));
    refreshDiagnostics();
    return true;
  }
  public function origin(targetId:String, property:String):String return overridesEnabled && overrides.exists(key(
    targetId,
    property
  )) ? "override" : "script";
  public function propertyOrigins(targetId:String,
    properties:Array<String>):Array < String> return [for (property in properties) property + ": " + origin(
      targetId,
      property
    )];
  public function revertTarget(targetId:String):Bool {
    var removed:Array<ScriptOverrideRecord> = [];
    for (value in overrides) {
      var item = value;
      if (item.targetId == targetId) removed.push(item);
    }
    if (removed.length == 0) return false;
    document.apply(new EditOperation("Revert script overrides for " + targetId, function() {
      for (item in removed) overrides.remove(key(item.targetId, item.property));
    }, function() {
      for (item in removed) overrides.set(key(item.targetId, item.property), item);
    }
    ));
    refreshDiagnostics();
    return true;
  }
  public function removeStaleOverrides():Bool {
    if (staleOverrides.length == 0) return false;
    var removed:Array<ScriptOverrideRecord> = [];
    for (k in staleOverrides) {
      var item = overrides.get(k);
      if (item != null) removed.push(item);
    }
    document.apply(new EditOperation("Remove stale script overrides", function() {
      for (item in removed) overrides.remove(key(item.targetId, item.property));
    }, function() {
      for (item in removed) overrides.set(key(item.targetId, item.property), item);
    }
    ));
    refreshDiagnostics();
    return true;
  }
  public function backend():Int {
    var item = overrides.get(key(SIMULATION_TARGET, "backend"));
    return overridesEnabled && item != null ? cast overrideValue(item) : baseline.backend;
  }
  public function timestep():Float {
    var item = overrides.get(key(SIMULATION_TARGET, "timestep"));
    return overridesEnabled && item != null ? cast overrideValue(item) : baseline.timestep;
  }
  public function markSaved():Void {
    savedVersion = configurationVersion;
    document.markSaved();
    refreshDiagnostics();
  }
  public function record():ScriptOwnershipRecord {
    var packageIdentity = SetupScriptRegistry.identity(reference);
    var values:Array<ScriptOverrideRecord> = [];
    for (k in overrides.keys()) values.push(copyOverride(overrides.get(k)));
    values.sort(function(a, b) return Reflect.compare(a.targetId + "/" + a.property, b.targetId + "/" + b.property));
    return {
      reference: reference,
      version: configurationVersion,
      identityVersion: 1,
      packageId: packageIdentity.packageId,
      packageVersion: packageIdentity.packageVersion,
      sourceSha256: packageIdentity.sourceSha256,
      configurationSha256: configurationSha256,
      overrideVersion: OVERRIDE_VERSION,
      overridesEnabled: overridesEnabled,
      overrides: values
    };
  }
  static function contentHash(value:ScriptedSetup):String {
    var normalized:Dynamic = Json.parse(Json.stringify({version: value.version,
      sensors: value.sensors.records(), objects: value.objects,
      backend: value.backend, timestep: value.timestep}));
    return Sha256.encode(ScriptCanonicalJson.encode(normalized));
  }
  function verifyIdentity(record:ScriptOwnershipRecord):Void {
    var packageIdentity = SetupScriptRegistry.identity(reference);
    // Earlier files have no identity. They acquire one on their next save.
    if (record.identityVersion == null && record.packageId == null && record.packageVersion == null
      && record.sourceSha256 == null && record.configurationSha256 == null) return;
    if (record.identityVersion != 1 || record.packageId != packageIdentity.packageId
      || record.packageVersion != packageIdentity.packageVersion
      || record.sourceSha256 != packageIdentity.sourceSha256
      || record.configurationSha256 != configurationSha256)
      throw 'Setup script identity mismatch: $reference';
  }
  function applyOverrides(sensors:SensorConfiguration, objects:Array<SceneObjectData>, settings:ScriptSettings):Void {
    var found = new Map<String, Bool>();
    var orderedKeys = [for (k in overrides.keys()) k];
    orderedKeys.sort(function(left, right) {
      var leftItem:ScriptOverrideRecord = cast overrides.get(left);
      var rightItem:ScriptOverrideRecord = cast overrides.get(right);
      var leftPriority = leftItem.property == "mount.frameId" ? 0 : 1;
      var rightPriority = rightItem.property == "mount.frameId" ? 0 : 1;
      return leftPriority == rightPriority ? Reflect.compare(left, right) : leftPriority - rightPriority;
    }
    );
    for (k in orderedKeys) {
      var item:ScriptOverrideRecord = cast overrides.get(k);
      var applied = false;
      if (item.targetId == SIMULATION_TARGET) applied = applySimulation(item, settings);
      if (!applied) for (robot in sensors.robotModels()) {
        if (item.targetId == robot.id) applied = applyRobot(item, robot.position, robot.rotation, sensors, robot.id);
        for (sensor in robot.model.sensors) if (item.targetId == robot.id + "/" + sensor.id) applied = applySensor(
          item,
          sensor,
          robot.model
        );
      }
      if (!applied) for (object in objects) if (item.targetId == object.id) applied = applyObject(item, object);
      if (applied) found.set(k, true);
    }
    diagnostics = [];
    staleOverrides = [];
    if (savedVersion > 0 && savedVersion != configurationVersion) diagnostics
      .push('Configuration version changed from $savedVersion to $configurationVersion; stable-ID overrides were revalidated');
    for (k in orderedKeys) if (!found.exists(k)) {
      staleOverrides.push(k);
      diagnostics.push("Stale or unsupported override: " + k);
    }
    staleOverrides.sort(Reflect.compare);
  }
  function refreshDiagnostics():Void {
    var sensors:Null<SensorConfiguration> = null;
    try {
      var source:SensorConfiguration = baseline.sensors;
      var records:Dynamic = Json.parse(Json.stringify(source.records()));
      sensors = new SensorConfiguration(records);
      var objects = cloneObjects();
      var settings:ScriptSettings = {backend: baseline.backend, timestep: baseline.timestep};
      applyOverrides(sensors, objects, settings);
      validateSensors(sensors);
    } catch (error:Dynamic) {
      if (diagnostics.length == 0) diagnostics = [Std.string(error)];
    }
    if (sensors != null) sensors.dispose();
  }
  function validateCandidate(changedKey:String):Void {
    var source:SensorConfiguration = baseline.sensors;
    var records:Dynamic = Json.parse(Json.stringify(source.records()));
    var sensors = new SensorConfiguration(records);
    try {
      var objects = cloneObjects();
      var settings:ScriptSettings = {backend: baseline.backend, timestep: baseline.timestep};
      applyOverrides(sensors, objects, settings);
      validateSensors(sensors);
      if (staleOverrides.indexOf(changedKey) >= 0) throw "Unknown override target or property: " + changedKey;
      if ((settings.backend != ApplicationSimulation.DETERMINISTIC && settings.backend != ApplicationSimulation.MUJOCO)
        || !Math.isFinite(settings.timestep) || settings.timestep <= 0) throw "Invalid overridden simulation settings";
      for (object in objects) if (!Math.isFinite(object.x) || !Math.isFinite(object.y)
        || !Math.isFinite(object.z) || !Math.isFinite(object.width) || !Math.isFinite(object.height)
        || !Math.isFinite(object.depth) || object.width <= 0 || object.height <= 0 || object.depth <= 0
        || !Math.isFinite(object.mass) || object.mass <= 0) throw "Invalid overridden environment object";
    } catch (error:Dynamic) {
      sensors.dispose();
      throw error;
    }
    sensors.dispose();
  }
  static function validateSensors(sensors:SensorConfiguration):Void for (robot in sensors.robotModels()) {
    var issues = RobotRuntimeCompiler.validate(robot.model);
    if (issues.length > 0) throw issues[0].code + ": " + issues[0].message;
  }
  function cloneObjects():Array<SceneObjectData> return SceneCodec.decode(Json.stringify({
    format: SceneCodec.FORMAT,
    version: SceneCodec.VERSION,
    objects: baseline.objects
  }));
  static function validateSetup(value:ScriptedSetup, expectedReference:String):Void {
    if (value.reference != expectedReference || StringTools.trim(value.reference).length == 0
      || value.version < 1 || value.sensors == null || value.objects == null
      || !Math.isFinite(value.timestep) || value.timestep <= 0) throw "Invalid setup script result";
    if (value.backend != ApplicationSimulation.DETERMINISTIC
      && value.backend != ApplicationSimulation.MUJOCO) throw "Unsupported setup physics backend";
    var sensors:SensorConfiguration = value.sensors;
    validateSensors(sensors);
  }
  function applySimulation(item:ScriptOverrideRecord, settings:ScriptSettings):Bool switch item.property {
    case "backend":
      if (item.kind != "integer") return false;
      if (overridesEnabled) settings.backend = cast overrideValue(item);
      return true;
    case "timestep":
      if (item.kind != "number") return false;
      if (overridesEnabled) settings.timestep = cast overrideValue(item);
      return true;
    default:
      return false;
  }
  function applyRobot(
    item:ScriptOverrideRecord,
    position:Array<Float>,
    rotation:Array<Float>,
    sensors:SensorConfiguration,
    robotId:String
  ):Bool switch item.property {
    case "position":
      if (item.kind != "vector") return false;
      if (overridesEnabled) sensors.setRobotPose(robotId, vectorValue(item), sensors.robotRotation(robotId));
      return true;
    case "rotation":
      if (item.kind != "vector") return false;
      if (overridesEnabled) sensors.setRobotPose(robotId, sensors.robotPosition(robotId), vectorValue(item));
      return true;
    default:
      return false;
  }
  function applySensor(item:ScriptOverrideRecord, sensor:robotkit.model.Sensor, model:robotkit.model.RobotModel):Bool {
    var frame = sensor.frame;
    switch item.property {
      case "updateRate":
        if (item.kind != "number") return false;
        if (overridesEnabled) sensor.updateRate = cast overrideValue(item);
      case "rayCount":
        if (item.kind != "integer") return false;
        if (overridesEnabled) sensor.rayCount = cast overrideValue(item);
      case "maxRange":
        if (item.kind != "number") return false;
        if (overridesEnabled) sensor.maxRange = cast overrideValue(item);
      case "noiseStddev":
        if (item.kind != "number") return false;
        if (overridesEnabled) sensor.noiseStddev = cast overrideValue(item);
      case "noiseSeed":
        if (item.kind != "integer") return false;
        if (overridesEnabled) sensor.noiseSeed = cast overrideValue(item);
      case "mount.position":
        if (item.kind != "vector" || frame == null) return false;
        if (overridesEnabled) frame.position = vectorValue(item);
      case "mount.rotation":
        if (item.kind != "vector" || frame == null) return false;
        if (overridesEnabled) frame.rotation = vectorValue(item);
      case "mount.frameId":
        if (item.kind != "text") return false;
        var next:Null<robotkit.model.Frame> = null;
        for (candidate in model.frames) if (candidate.id == overrideValue(item)) next = candidate;
        if (next == null) return false;
        if (overridesEnabled) sensor.frame = next;
      default:
        return false;
    }
    return true;
  }
  function applyObject(item:ScriptOverrideRecord, object:SceneObjectData):Bool {
    if (!overridesEnabled) return objectPropertyExists(item.property, item.kind);
    switch item.property {
      case "position":
        if (item.kind != "vector") return false;
        var value = vectorValue(item);
        object.x = value[0];
        object.y = value[1];
        object.z = value[2];
      case "dimensions":
        if (item.kind != "vector") return false;
        var value = vectorValue(item);
        object.width = value[0];
        object.height = value[1];
        object.depth = value[2];
      case "mass":
        if (item.kind != "number") return false;
        object.mass = cast overrideValue(item);
      case "collisionEnabled":
        if (item.kind != "boolean") return false;
        object.collisionEnabled = cast overrideValue(item);
      case "dynamicBody":
        if (item.kind != "boolean") return false;
        object.dynamicBody = cast overrideValue(item);
      case "visible":
        if (item.kind != "boolean") return false;
        object.visible = cast overrideValue(item);
      default:
        return false;
    }
    return objectPropertyExists(item.property, item.kind);
  }
  static function objectPropertyExists(property:String, kind:String):Bool return switch property {
    case "position" | "dimensions":
      kind == "vector";
    case "mass":
      kind == "number";
    case "collisionEnabled" | "dynamicBody" | "visible":
      kind == "boolean";
    default:
      false;
  }
  static function validateOverride(item:ScriptOverrideRecord):Void {
    if (item.targetId == null || StringTools.trim(item.targetId).length == 0 || item.property == null
      || StringTools.trim(item.property).length == 0) throw "Override target and property are required";
    switch item.kind {
      case "number":
        var raw = overrideValue(item);
        if (!Std.isOfType(raw, Float) && !Std.isOfType(raw, Int)) throw "Override must be numeric";
        var value:Float = cast raw;
        if (!Math.isFinite(value)) throw "Override must be finite";
      case "integer":
        if (!Std.isOfType(overrideValue(item), Int)) throw "Override must be an integer";
      case "boolean":
        if (!Std.isOfType(overrideValue(item), Bool)) throw "Override must be boolean";
      case "text":
        if (!Std.isOfType(overrideValue(item), String)) throw "Override must be text";
      case "vector":
        var value:Array<Dynamic> = cast overrideValue(item);
        if (value == null
          || value.length < 2 || value.length > 4) throw "Override vector must contain two to four values";
        for (component in value) {
          var number = Std.parseFloat(Std.string(component));
          if (Std.isOfType(component,
            Bool) || Std.isOfType(component, String) || !Math.isFinite(number)) throw "Override vector must be finite";
        }
      default:
        throw "Unsupported override type";
    }
    var numeric:Float =(item.kind == "number" || item.kind == "integer") ? numberValue(overrideValue(item)) : 0.0;
    switch item.property {
      case "backend":
        if (item.kind != "integer" ||(numeric != ApplicationSimulation.DETERMINISTIC
          && numeric != ApplicationSimulation.MUJOCO)) throw "Unsupported setup physics backend";
      case "timestep":
        if (item.kind != "number" || numeric <= 0) throw "Simulation timestep must be positive";
      case "updateRate":
        if (numeric < 0 || numeric > 10000) throw "Invalid sensor update rate";
      case "rayCount":
        if (item.kind != "integer" || numeric < 1 || numeric > 64) throw "Invalid LiDAR ray count";
      case "maxRange" | "mass":
        if (numeric <= 0) throw "Override value must be positive";
      case "noiseStddev" | "noiseSeed":
        if (numeric < 0) throw "Override value cannot be negative";
      case "position":
        requireVector(item, 3);
      case "dimensions":
        requireVector(item, 3);
        var dimensions = vectorValue(item);
        for (component in dimensions) if (component <= 0) throw "Object dimensions must be positive";
      case "rotation" | "mount.rotation":
        requireVector(item, 4);
        var norm = 0.0;
        var rotation = vectorValue(item);
        for (component in rotation) norm += component * component;
        if (Math.abs(norm - 1.0) > 0.000001) throw "Override rotation must be a unit quaternion";
      case "mount.position":
        requireVector(item, 3);
      default:
    }
  }
  static function requireVector(item:ScriptOverrideRecord, length:Int):Void {
    var values:Array<Dynamic> = overrideValue(item);
    if (item.kind != "vector" || values.length != length) throw 'Override ${item.property} requires $length values';
  }
  static function numberValue(value:Dynamic):Float return value;
  static function vectorValue(item:ScriptOverrideRecord):Array<Float> {
    var values:Array<Dynamic> = overrideValue(item);
    return [for (value in values) numberValue(value)];
  }
  public static function overrideRecord(targetId:String,
    property:String, kind:String, value:Dynamic):ScriptOverrideRecord {
    return {targetId: targetId, property: property, kind: kind, encodedValue: Json.stringify(value)};
  }
  static function overrideValue(item:ScriptOverrideRecord):Dynamic {
    try {
      return Json.parse(item.encodedValue);
    } catch (error:Dynamic) {
      throw 'Invalid ${item.property} override value: ' + Std.string(error);
    }
  }
  static function copyOverride(parsed:ScriptOverrideRecord):ScriptOverrideRecord {
    return {
      targetId: parsed.targetId,
      property: parsed.property,
      kind: parsed.kind,
      encodedValue: parsed.encodedValue
    };
  }
  static inline function key(target:String, property:String):String return target + "|" + property;
  public function dispose():Void baseline.dispose();
}
