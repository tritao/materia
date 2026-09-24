package app;

import haxe.Json;
import bimkit.BimCodec;
import bimkit.BimDocument;

class SceneCodec {
  public static inline var FORMAT:String = "materia.scene";
  public static inline var VERSION:Int = 1;

  public static function encode(scene:EditorScene,
    ? sensors:SensorConfiguration, ? script:ScriptOwnershipRecord, ? bim:BimDocument,
    ?project:ProjectSceneRecord, ?authoredObjects:Array<SceneObjectData>):String return Json.stringify(
      {
    format: FORMAT,
    version: VERSION,
    objects: script == null ? (authoredObjects == null ? scene.recordsForSave() : authoredObjects) :[],
    sensors : script == null && sensors != null ? sensors.records() : null,
    script : script,
    project: project,
    bim : bim == null ? null : Json.parse(BimCodec.encode(bim))
  },
    null,
    "  "
  ) + "\n";

  public static function decodeProject(text:String):Null<ProjectSceneRecord> {
    var root:Dynamic = Json.parse(text);
    if (stringField(root, "format") != FORMAT || numberField(root, "version") != VERSION)
      throw "Unsupported scene document";
    var value:Dynamic = Reflect.field(root, "project");
    if (value == null) return null;
    if (Reflect.field(root, "script") != null) throw "A scene cannot have both script and project owners";
    if (numberField(value, "version") != 1) throw "Unsupported generated project record version";
    var reference = stringField(value, "reference");
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
    for (item in overrideValues) {
      var id = stringField(item, "id");
      if (seen.exists(id) || Reflect.hasField(item, "meshSnapshot") || Reflect.hasField(item, "cadGraph"))
        throw "Invalid generated project override";
      seen.set(id, true);
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
      var object = field(item, "object");
      var id = stringField(object, "id");
      if (seen.exists(id) || Reflect.hasField(object, "meshSnapshot") || Reflect.hasField(object, "cadGraph"))
        throw "Invalid generated project instance";
      seen.set(id, true);
      instanceRecords.push({sourceId: sourceId, object: object});
    }
    return {version: 1, reference: reference, overrides: cast overrides,
      removed: removedIds, instances: instanceRecords};
  }

  public static function decodeScript(text:String):Null < ScriptOwnershipRecord > {
    var root:Dynamic = Json.parse(text);
    if (stringField(root,
      "format") != FORMAT || numberField(root, "version") != VERSION) throw "Unsupported scene document";
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
  public static function decodeBim(text:String):BimDocument {
    var root:Dynamic = Json.parse(text);
    if (stringField(root, "format") != FORMAT || numberField(root, "version") != VERSION)
      throw "Unsupported scene document";
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

  public static function decodeSensors(text:String):Null < Dynamic > {
    var root:Dynamic = Json.parse(text);
    if (stringField(root,
      "format") != FORMAT || numberField(root, "version") != VERSION) throw "Unsupported scene document";
    return Reflect.hasField(root, "sensors") ? Reflect.field(root, "sensors") : null;
  }

  /** Validate the entire file before allocating native scene resources. */
  public static function decode(text:String):Array < SceneObjectData > {
    var root:Dynamic = Json.parse(text);
    if (stringField(root, "format") != FORMAT) throw "This is not a Materia scene document";
    if (numberField(root, "version") != VERSION) throw "Unsupported scene document version";
    var raw:Dynamic = field(root, "objects");
    if (!Std.isOfType(raw, Array)) throw "Scene objects must be an array";
    var values:Array<Dynamic> = cast raw;
    if (values.length > 10000) throw "Scene documents support at most 10000 objects";
    var result:Array<SceneObjectData> = [];
    var ids:Map<String, Bool> = new Map();
    var sketchDraftCount = 0;
    for (value in values) {
      var id = stringField(value, "id");
      if (id == "scene" || ids.exists(id)) throw "Duplicate or reserved object ID: " + id;
      ids.set(id, true);
      var kind = stringField(value, "type");
      if (kind != "rectangle" && kind != "cad-plate" && kind != "cad-bracket" && kind != "cad-step" &&
          kind != "cad-part" && kind != "cad-preview")
        throw "Unsupported scene object type: " + kind;
      var visible:Dynamic = field(value, "visible");
      if (!Std.isOfType(visible, Bool)) throw "Object visibility must be a boolean";
      var visibleValue:Bool = visible;
      var collisionEnabled = optionalBool(value, "collisionEnabled", visibleValue);
      var dynamicBody = optionalBool(value, "dynamicBody", false);
      var cadGraph = optionalText(value, "cadGraph");
      var meshSnapshot = optionalText(value, "meshSnapshot");
      var sketchDraft = optionalText(value, "sketchDraft");
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
        z: bounded(
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
        collisionEnabled: collisionEnabled,
        dynamicBody: dynamicBody,
        mass: optionalBounded(
          value,
          "mass",
          0.000001,
          1000000,
          1.0
        ),
        red: bounded(
          value,
          "red",
          0,
          1
        ),
        green: bounded(
          value,
          "green",
          0,
          1
        ),
        blue: bounded(
          value,
          "blue",
          0,
          1
        ),
        visible: visibleValue,
        rotation: optionalRotation(value),
        cadGraph: cadGraph,
        meshSnapshot: meshSnapshot,
        sketchDraft: sketchDraft
      }
      );
    }
    return result;
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
