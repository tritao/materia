package app;

import haxe.Json;
import humankit.HumanJobSpec;
import bimkit.BimCodec;
import bimkit.BimDocument;
import materia.project.Appearance;
import materia.project.Appearance.Appearances;
import materia.project.MaterialDef;
import materia.project.MaterialLibrary;

class SceneCodec {
  public static inline var FORMAT:String = "materia.scene";
  public static inline var VERSION:Int = 1;

  public static function encode(scene:EditorScene,
    ? sensors:SensorConfiguration, ? script:ScriptOwnershipRecord, ? bim:BimDocument,
    ?project:ProjectSceneRecord, ?authoredObjects:Array<SceneObjectData>,
    ?customMaterials:Array<MaterialDef>, ?recipeDocument:String,
    ?robotMotions:Array<RobotMotionTrack>):String {
    var custom = customMaterials == null ? [] : customMaterials;
    MaterialLibrary.validateCustom(custom);
    var objects = script == null ? (authoredObjects == null ? scene.recordsForSave() : authoredObjects) : [];
    var encodedObjects:Array<Dynamic> = [];
    for (object in objects) encodedObjects.push(encodeObject(object, custom));
    return Json.stringify({format: FORMAT, version: VERSION, objects: encodedObjects,
      sensors: script == null && sensors != null ? sensors.records() : null,
      script: script, project: project, materials: MaterialLibrary.all().concat(custom),
      bim: bim == null ? null : Json.parse(BimCodec.encode(bim)),
      recipeDocument: recipeDocument,
      robotMotions: robotMotions == null || robotMotions.length == 0 ? null :
        [for (track in robotMotions) track.record()]}, null, "  ") + "\n";
  }

  /** Serialize editor color fields as sparse material visuals. */
  public static function encodeObject(object:SceneObjectData, ?customMaterials:Array<MaterialDef>):Dynamic {
    var custom = customMaterials == null ? [] : customMaterials;
    var finish = object.appearance == null ? "neutral" : object.appearance.finish;
    var id = object.materialId == null ?
      (hasMaterial(finish, custom) ? finish : "neutral") : object.materialId;
    var material = resolveMaterial(id, custom);
    var result:Dynamic = {};
    for (name in Reflect.fields(object)) if (["red", "green", "blue", "appearance", "materialId"].indexOf(name) < 0 &&
        (name != "worker" || object.worker != null))
      Reflect.setField(result, name, Reflect.field(object, name));
    Reflect.setField(result, "materialId", id);
    var visual:Dynamic = {};
    var color = material.visual.baseColor;
    if (Math.abs(object.red - color[0]) > 1e-6 || Math.abs(object.green - color[1]) > 1e-6 ||
        Math.abs(object.blue - color[2]) > 1e-6)
      Reflect.setField(visual, "baseColor", [object.red, object.green, object.blue]);
    var appearance = object.appearance == null ? Appearances.neutral() : object.appearance;
    if (appearance.finish != id) Reflect.setField(visual, "finish", appearance.finish);
    if (Math.abs(appearance.metallic - material.visual.metallic) > 1e-6)
      Reflect.setField(visual, "metallic", appearance.metallic);
    if (Math.abs(appearance.roughness - material.visual.roughness) > 1e-6)
      Reflect.setField(visual, "roughness", appearance.roughness);
    if (Reflect.fields(visual).length > 0) Reflect.setField(result, "visualOverrides", visual);
    return result;
  }

  static function resolveMaterial(id:String, custom:Array<MaterialDef>):MaterialDef {
    var builtIn = MaterialLibrary.get(id);
    if (builtIn != null) return builtIn;
    for (item in custom) if (item.id == id) return item;
    throw 'Unknown scene material "$id"';
  }

  static function hasMaterial(id:String, custom:Array<MaterialDef>):Bool {
    if (MaterialLibrary.get(id) != null) return true;
    for (item in custom) if (item.id == id) return true;
    return false;
  }

  /** Parse once before decoding independently validated sections. */
  public static function parse(text:String):Dynamic {
    var root:Dynamic = Json.parse(text);
    if (stringField(root, "format") != FORMAT || numberField(root, "version") != VERSION)
      throw "Unsupported scene document";
    return root;
  }

  public static function decodeProject(text:String):Null<ProjectSceneRecord> return decodeProjectRoot(parse(text));

  public static function decodeCustomMaterialsRoot(root:Dynamic):Array<MaterialDef> {
    if (!Reflect.hasField(root, "materials") || Reflect.field(root, "materials") == null) return [];
    var raw:Dynamic = Reflect.field(root, "materials");
    if (!Std.isOfType(raw, Array)) throw "Scene materials must be an array";
    var custom:Array<MaterialDef> = [];
    var seen = new Map<String, Bool>();
    var materials:Array<Dynamic> = cast raw;
    for (material in materials) {
      var id = stringField(material, "id");
      if (seen.exists(id)) throw "Duplicate scene material";
      seen.set(id, true);
      if (MaterialLibrary.get(id) != null) continue;
      var visual = field(material, "visual"), physical = field(material, "physical");
      var parsed:MaterialDef = {
        id: id, name: stringField(material, "name"),
        visual: {baseColor: materialColor(field(visual, "baseColor")),
          metallic: numberField(visual, "metallic"), roughness: numberField(visual, "roughness")},
        physical: {density: numberField(physical, "density"), spec: stringField(physical, "spec")}
      };
      if (Reflect.hasField(visual, "emissive") && Reflect.field(visual, "emissive") != null)
        parsed.visual.emissive = materialColor(Reflect.field(visual, "emissive"));
      if (Reflect.hasField(visual, "alpha") && Reflect.field(visual, "alpha") != null)
        parsed.visual.alpha = numberField(visual, "alpha");
      custom.push(parsed);
    }
    MaterialLibrary.validateCustom(custom);
    return custom;
  }

  static function materialColor(raw:Dynamic):Array<Float> {
    if (!Std.isOfType(raw, Array)) throw "Scene material color must be an array";
    var values:Array<Dynamic> = cast raw;
    if (values.length != 3) throw "Scene material color must have three channels";
    var color:Array<Float> = [];
    for (value in values) {
      if ((!Std.isOfType(value, Float) && !Std.isOfType(value, Int)) || !Math.isFinite(cast value))
        throw "Scene material color must be finite";
      color.push(cast value);
    }
    return color;
  }

  public static function decodeProjectRoot(root:Dynamic):Null<ProjectSceneRecord> {
    var value:Dynamic = Reflect.field(root, "project");
    if (value == null) return null;
    if (Reflect.field(root, "script") != null) throw "A scene cannot have both script and project owners";
    if (numberField(value, "version") != 1) throw "Unsupported generated project record version";
    var reference = stringField(value, "reference");
    var assemblyState = optionalText(value, "assemblyState");
    if (assemblyState != null && assemblyState.length > 2000000)
      throw "Generated project assembly state is too large";
    var assemblyDependentJoints = optionalTextArray(value, "assemblyDependentJoints", 4000);
    var overrides:Dynamic = field(value, "overrides");
    var removed:Dynamic = field(value, "removed");
    var instances:Dynamic = field(value, "instances");
    if (!Std.isOfType(overrides, Array) || !Std.isOfType(removed, Array) ||
        !Std.isOfType(instances, Array)) throw "Invalid generated project edits";
    var overrideValues:Array<Dynamic> = cast overrides;
    var removedValues:Array<Dynamic> = cast removed;
    var instanceValues:Array<Dynamic> = cast instances;
    if (overrideValues.length > 10000 || removedValues.length > 10000 || instanceValues.length > 10000)
      throw "Generated project has too many edits";
    var seen = new Map<String, Bool>();
    var overrideRecords:Array<app.ProjectSceneRecord.ProjectFieldOverride> = [];
    for (item in overrideValues) {
      var id = stringField(item, "targetId");
      var editKey = id + ":" + stringField(item, "property");
      if (seen.exists(editKey) || Reflect.hasField(item, "meshSnapshot") || Reflect.hasField(item, "cadGraph"))
        throw "Invalid generated project override";
      var kind = stringField(item, "kind");
      if (["number", "boolean", "text", "vector", "appearance"].indexOf(kind) < 0 ||
          !Reflect.hasField(item, "value")) throw "Invalid generated project field edit";
      seen.set(editKey, true);
      overrideRecords.push({targetId: id, property: stringField(item, "property"),
        kind: kind, value: Reflect.field(item, "value")});
    }
    var removedIds:Array<String> = [];
    for (item in removedValues) {
      if (!Std.isOfType(item, String) || StringTools.trim(cast item).length == 0 || containsNul(cast item))
        throw "Invalid generated project removal ID";
      var id:String = cast item;
      if (seen.exists(id)) throw "Duplicate generated project edit ID";
      seen.set(id, true);
      removedIds.push(id);
    }
    var instanceRecords:Array<app.ProjectSceneRecord.ProjectSceneInstance> = [];
    for (item in instanceValues) {
      var sourceId = stringField(item, "sourceId");
      var id = stringField(item, "id");
      if (!Std.isOfType(field(item, "overrides"), Array) || seen.exists(id))
        throw "Invalid generated project instance";
      seen.set(id, true);
      var instanceEdits:Array<app.ProjectSceneRecord.ProjectFieldOverride> = [];
      var rawEdits:Array<Dynamic> = cast field(item, "overrides");
      for (edit in rawEdits) {
        var editKind = stringField(edit, "kind");
        if (["number", "boolean", "text", "vector", "appearance"].indexOf(editKind) < 0 ||
            !Reflect.hasField(edit, "value")) throw "Invalid generated project instance edit";
        instanceEdits.push({targetId: stringField(edit, "targetId"),
          property: stringField(edit, "property"), kind: editKind,
          value: Reflect.field(edit, "value")});
      }
      instanceRecords.push({sourceId: sourceId, id: id, overrides: instanceEdits});
    }
    return {version: 1, reference: reference, overrides: overrideRecords,
      removed: removedIds, instances: instanceRecords, assemblyState: assemblyState,
      assemblyDependentJoints: assemblyDependentJoints};
  }

  public static function decodeScript(text:String):Null<ScriptOwnershipRecord> return decodeScriptRoot(parse(text));

  public static function decodeScriptRoot(root:Dynamic):Null<ScriptOwnershipRecord> {
    var value:Dynamic = Reflect.field(root, "script");
    if (value == null) return null;
    var reference = stringField(value, "reference"), version = Std.int(numberField(value, "version"));
    var enabled:Dynamic = field(value, "overridesEnabled");
    if (!Std.isOfType(enabled, Bool)) throw "Invalid script override mode";
    var overridesEnabled:Bool = enabled;
    var raw:Dynamic = field(value, "overrides");
    if (!Std.isOfType(raw, Array)) throw "Invalid script overrides";
    var overrideVersion = Reflect.hasField(value,
      "overrideVersion") ? Std.int(numberField(value, "overrideVersion")) : 1;
    if (overrideVersion < 1
      || overrideVersion > ScriptOwnership.OVERRIDE_VERSION) throw "Unsupported script override version";
    var overrides:Array<ScriptOverrideRecord> = [];
    var items:Array<Dynamic> = raw;
    for (item in items) {
      var kind = overrideVersion == 1 ? "number" : stringField(item, "kind");
      var overrideValue:Dynamic;
      if (overrideVersion == 1) overrideValue = field(item, "value");
      else {
        var encoded = stringField(item, "encodedValue");
        try overrideValue = Json.parse(encoded) catch (_:Dynamic) throw "Invalid encoded script override";
      }
      validateOverrideValue(kind, overrideValue);
      overrides.push(ScriptOwnership.overrideRecord(
        stringField(item, "targetId"),
        stringField(item, "property"),
        kind,
        overrideValue
      ));
    }
    var identityFields = ["identityVersion", "packageId", "packageVersion",
      "sourceSha256", "configurationSha256"];
    var identityCount = 0;
    for (name in identityFields) if (Reflect.hasField(value, name)) identityCount++;
    if (identityCount != 0 && identityCount != identityFields.length)
      throw "Incomplete setup script identity";
    var packageId:Null<String> = null, packageVersion:Null<String> = null;
    var sourceSha256:Null<String> = null, configurationSha256:Null<String> = null;
    var identityVersion:Null<Int> = null;
    if (identityCount != 0) {
      identityVersion = Std.int(numberField(value, "identityVersion"));
      if (identityVersion != 1) throw "Unsupported setup script identity version";
      packageId = stringField(value, "packageId");
      packageVersion = stringField(value, "packageVersion");
      sourceSha256 = stringField(value, "sourceSha256");
      configurationSha256 = stringField(value, "configurationSha256");
      if (!~/^[0-9a-f]{64}$/.match(sourceSha256)
        || !~/^[0-9a-f]{64}$/.match(configurationSha256))
        throw "Invalid setup script digest";
    }
    return {
      reference: reference,
      version: version,
      identityVersion: identityVersion,
      packageId: packageId,
      packageVersion: packageVersion,
      sourceSha256: sourceSha256,
      configurationSha256: configurationSha256,
      overrideVersion: ScriptOwnership.OVERRIDE_VERSION,
      overridesEnabled: overridesEnabled,
      overrides: overrides
    };
  }

  /** Optional versioned BimKit section; absent sections migrate to an empty BIM model. */
  public static function decodeBim(text:String):BimDocument return decodeBimRoot(parse(text));

  public static function decodeBimRoot(root:Dynamic):BimDocument {
    var value:Dynamic = Reflect.field(root, "bim");
    return value == null ? new BimDocument() : BimCodec.decode(Json.stringify(value));
  }

  static function validateOverrideValue(kind:String, value:Dynamic):Void switch kind {
    case "number":
      if ((!Std.isOfType(value,
        Float) && !Std.isOfType(value, Int)) || !Math.isFinite(cast value)) throw "Invalid numeric script override";
    case "integer":
      if (!Std.isOfType(value, Int)) throw "Invalid integer script override";
    case "boolean":
      if (!Std.isOfType(value, Bool)) throw "Invalid boolean script override";
    case "text":
      if (!Std.isOfType(value, String) || containsNul(cast value)) throw "Invalid text script override";
    case "vector":
      if (!Std.isOfType(value, Array)) throw "Invalid vector script override";
      var values:Array<Dynamic> = cast value;
      if (values.length < 2 || values.length > 4) throw "Invalid vector script override";
      for (item in values) if ((!Std.isOfType(item,
        Float) && !Std.isOfType(item, Int)) || !Math.isFinite(cast item)) throw "Invalid vector script override";
    default:
      throw "Unsupported script override type";
  }

  public static function decodeSensors(text:String):Null<Dynamic> return decodeSensorsRoot(parse(text));

  public static function decodeSensorsRoot(root:Dynamic):Null<Dynamic> {
    return Reflect.hasField(root, "sensors") ? Reflect.field(root, "sensors") : null;
  }

  /** Validate the entire file before allocating native scene resources. */
  public static function decode(text:String):Array<SceneObjectData> return decodeRoot(parse(text));

  public static function decodeRoot(root:Dynamic):Array<SceneObjectData> {
    var raw:Dynamic = field(root, "objects");
    if (!Std.isOfType(raw, Array)) throw "Scene objects must be an array";
    var values:Array<Dynamic> = cast raw;
    if (values.length > 10000) throw "Scene documents support at most 10000 objects";
    var result:Array<SceneObjectData> = [];
    var ids:Map<String, Bool> = new Map();
    var sketchDraftCount = 0;
    var custom = decodeCustomMaterialsRoot(root);
    for (value in values) {
      var id = stringField(value, "id");
      if (id == "scene" || ids.exists(id)) throw "Duplicate or reserved object ID: " + id;
      ids.set(id, true);
      var kind = stringField(value, "type");
      if (kind != "rectangle" && kind != "cad-plate" && kind != "cad-bracket" && kind != "cad-step" &&
          kind != "cad-part" && kind != "cad-preview" && kind != StockSimulationSession.KIND &&
          kind != "human-worker")
        throw "Unsupported scene object type: " + kind;
      var materialId = stringField(value, "materialId");
      var material = resolveMaterial(materialId, custom);
      var visual:Dynamic = Reflect.field(value, "visualOverrides");
      var color = material.visual.baseColor;
      if (visual != null && Reflect.hasField(visual, "baseColor")) {
        var rawColor:Dynamic = Reflect.field(visual, "baseColor");
        if (!Std.isOfType(rawColor, Array)) throw "Invalid scene material color";
        color = cast rawColor;
        if (color.length != 3) throw "Invalid scene material color";
        for (channel in color) if (!Math.isFinite(channel) || channel < 0 || channel > 1)
          throw "Invalid scene material color";
      }
      var appearance:Appearance = {finish: visual == null || !Reflect.hasField(visual, "finish")
        ? materialId : stringField(visual, "finish"),
        metallic: visual == null ? material.visual.metallic : optionalBounded(visual, "metallic", 0, 1, material.visual.metallic),
        roughness: visual == null ? material.visual.roughness : optionalBounded(visual, "roughness", 0, 1, material.visual.roughness)};
      var visible:Dynamic = field(value, "visible");
      if (!Std.isOfType(visible, Bool)) throw "Object visibility must be a boolean";
      var visibleValue:Bool = visible;
      var collisionEnabled = optionalBool(value, "collisionEnabled", visibleValue);
      var dynamicBody = optionalBool(value, "dynamicBody", false);
      var cadGraph = optionalText(value, "cadGraph");
      var meshSnapshot = optionalText(value, "meshSnapshot");
      var sketchDraft = optionalText(value, "sketchDraft");
      var worker:Null<WorkerObjectData> = null;
      if (kind == "human-worker") {
        var source:Dynamic = field(value, "worker");
        if (source == null || Std.isOfType(source, Array)) throw "Worker data must be an object";
        var zonesRaw:Dynamic = field(source, "zones");
        if (!Std.isOfType(zonesRaw, Array)) throw "Worker zones must be an array";
        var zones:Array<String> = [];
        for (zone in (cast zonesRaw:Array<Dynamic>)) {
          if (!Std.isOfType(zone, String) || zone == "") throw "Worker zone needs an object ID";
          zones.push(zone);
        }
        worker = {asset: stringField(source, "asset"), job: stringField(source, "job"), zones: zones};
        if (Reflect.hasField(source, "migrationNote")) worker.migrationNote = optionalText(source, "migrationNote");
        // A malformed job remains editable and is reported by the worker inspector.
        try HumanJobSpec.parse(worker.job) catch (_:Dynamic) {}
      } else if (Reflect.hasField(value, "worker") && Reflect.field(value, "worker") != null)
        throw "Worker data belongs only to human-worker objects";
      if ((kind == "cad-plate" || kind == "cad-bracket" || kind == "cad-step" || kind == "cad-part")
          && cadGraph != null && cadGraph.length > 10000000) throw "CAD feature graph is too large";
      if (kind == "cad-preview" && (meshSnapshot == null || meshSnapshot.length > 50000000))
        throw "CAD preview object requires a bounded mesh snapshot";
      if (kind != "cad-preview" && meshSnapshot != null)
        throw "Mesh snapshots are only valid on CAD preview objects";
      if (sketchDraft != null) {
        sketchDraftCount++;
        if (sketchDraftCount > 1) throw "Scene documents can contain only one active sketch draft";
        if (sketchDraft.length > 10000000 ||
            (kind != "cad-plate" && kind != "cad-bracket" && kind != "cad-step" && kind != "cad-part"))
          throw "Invalid sketch draft owner";
        var draft = SketchDraftCodec.decode(sketchDraft);
        if (draft.featureIndex < 0 && kind != "cad-part")
          throw "new sketch drafts require a generic CAD part";
      }
      result.push({
        id: id,
        label: stringField(value, "label"),
        type: kind,
        x: bounded(value, "x", -1000000, 1000000),
        y: bounded(
          value,
          "y",
          -1000000,
          1000000
        ),
        z: kind == "human-worker" ? 0.0 : bounded(
          value,
          "z",
          -1000000,
          1000000
        ),
        width: bounded(
          value,
          "width",
          0.000001,
          1000000
        ),
        height: bounded(
          value,
          "height",
          0.000001,
          1000000
        ),
        depth: optionalBounded(
          value,
          "depth",
          0.000001,
          1000000,
          0.1
        ),
        collisionEnabled: kind == "human-worker" ? false : collisionEnabled,
        dynamicBody: kind == "human-worker" ? false : dynamicBody,
        mass: optionalBounded(
          value,
          "mass",
          0.000001,
          1000000,
          1.0
        ),
        red: color[0],
        green: color[1],
        blue: color[2],
        appearance: appearance,
        materialId: materialId,
        visible: visibleValue,
        rotation: kind == "human-worker" ? workerYawRotation(optionalRotation(value)) : optionalRotation(value),
        cadGraph: cadGraph,
        meshSnapshot: meshSnapshot,
        sketchDraft: sketchDraft,
        worker: worker
      }
      );
    }
    return result;
  }

  static function workerYawRotation(rotation:Null<Array<Float>>):Null<Array<Float>> {
    if (rotation == null) return null;
    var yaw = Math.atan2(2 * (rotation[3] * rotation[2] + rotation[0] * rotation[1]),
      1 - 2 * (rotation[1] * rotation[1] + rotation[2] * rotation[2]));
    return [0.0, 0.0, Math.sin(yaw * 0.5), Math.cos(yaw * 0.5)];
  }

  static function field(value:Dynamic, name:String):Dynamic {
    if (value == null || !Reflect.hasField(value, name)) throw "Missing scene field: " + name;
    return Reflect.field(value, name);
  }
  static function stringField(value:Dynamic, name:String):String {
    var data = field(value, name);
    if (!Std.isOfType(data, String)) throw "Scene field must be text: " + name;
    var result:String = cast data;
    if (StringTools.trim(result).length == 0
      || containsNul(result)) throw "Scene field cannot be empty or contain NUL: " + name;
    return result;
  }
  public static function containsNul(value:String):Bool {
    for (index in 0...value.length) if (value.charCodeAt(index) == 0) return true;
    return false;
  }
  static function numberField(value:Dynamic, name:String):Float {
    var data = field(value, name);
    if (!Std.isOfType(data, Float) && !Std.isOfType(data, Int)) throw "Scene field must be numeric: " + name;
    var result:Float = cast data;
    if (result != result || result - result != 0) throw "Scene field must be finite: " + name;
    return result;
  }
  static function bounded(value:Dynamic, name:String, minimum:Float, maximum:Float):Float {
    var result = numberField(value, name);
    if (result < minimum || result > maximum) throw "Scene field is out of range: " + name;
    return result;
  }
  static function optionalBounded(value:Dynamic,
    name:String, minimum:Float, maximum:Float, fallback:Float):Float return Reflect.hasField(
      value,
      name
    ) ? bounded(
      value,
      name,
      minimum,
      maximum
    ) : fallback;
  static function optionalBool(value:Dynamic, name:String, fallback:Bool):Bool {
    if (!Reflect.hasField(value, name)) return fallback;
    var result = Reflect.field(value, name);
    if (!Std.isOfType(result, Bool)) throw 'Scene field must be boolean: $name';
    return cast result;
  }
  static function optionalText(value:Dynamic, name:String):Null < String > {
    if (!Reflect.hasField(value, name) || Reflect.field(value, name) == null) return null;
    var result = Reflect.field(value, name);
    if (!Std.isOfType(result, String)) throw 'Scene field must be text: $name';
    return cast result;
  }

  static function optionalTextArray(value:Dynamic, name:String, maximum:Int):Null<Array<String>> {
    if (!Reflect.hasField(value, name) || Reflect.field(value, name) == null) return null;
    var raw:Dynamic = Reflect.field(value, name);
    if (!Std.isOfType(raw, Array)) throw 'Scene field must be an array: $name';
    var items:Array<Dynamic> = cast raw;
    if (items.length > maximum) throw 'Scene field has too many entries: $name';
    var result:Array<String> = [];
    var seen = new Map<String, Bool>();
    for (item in items) {
      if (!Std.isOfType(item, String)) throw 'Scene field entries must be text: $name';
      var id:String = cast item;
      if (StringTools.trim(id).length == 0 || containsNul(id) || seen.exists(id))
        throw 'Scene field contains an invalid or duplicate entry: $name';
      seen.set(id, true);
      result.push(id);
    }
    return result;
  }

  static function optionalRotation(value:Dynamic):Null<Array<Float>> {
    if (!Reflect.hasField(value, "rotation") || Reflect.field(value, "rotation") == null) return null;
    var raw:Dynamic = Reflect.field(value, "rotation");
    if (!Std.isOfType(raw, Array)) throw "Object rotation must be a quaternion";
    var items:Array<Dynamic> = cast raw;
    if (items.length != 4) throw "Object rotation must have four components";
    var result:Array<Float> = [];
    for (item in items) {
      if ((!Std.isOfType(item, Float) && !Std.isOfType(item, Int)) || !Math.isFinite(cast item))
        throw "Object rotation must be finite";
      result.push(cast item);
    }
    var length = result[0] * result[0] + result[1] * result[1] +
      result[2] * result[2] + result[3] * result[3];
    if (Math.abs(length - 1) > 1e-4) throw "Object rotation must be normalized";
    return result;
  }
}
