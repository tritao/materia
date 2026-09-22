package app;

import haxe.Json;

class SceneCodec {
  public static inline var FORMAT:String = "materia.scene";
  public static inline var VERSION:Int = 1;

  public static function encode(scene:EditorScene, ?sensors:SensorConfiguration,
      ?script:ScriptOwnershipRecord):String
    return Json.stringify({format: FORMAT, version: VERSION,
      objects:script == null ? scene.records() : [],
      sensors:script == null && sensors != null ? sensors.records() : null,
      script:script}, null, "  ") + "\n";

  public static function decodeScript(text:String):Null<ScriptOwnershipRecord> {
    var root:Dynamic=Json.parse(text);
    if(stringField(root,"format")!=FORMAT||numberField(root,"version")!=VERSION)
      throw "Unsupported scene document";
    var value:Dynamic=Reflect.field(root,"script");if(value==null)return null;
    var reference=stringField(value,"reference"),version=Std.int(numberField(value,"version"));
    var enabled:Dynamic=field(value,"overridesEnabled");if(!Std.isOfType(enabled,Bool))throw "Invalid script override mode";
    var overridesEnabled:Bool=enabled;
    var raw:Dynamic=field(value,"overrides");if(!Std.isOfType(raw,Array))throw "Invalid script overrides";
    var overrides:Array<ScriptOverrideRecord> = [];
    var items:Array<Dynamic> = raw;
    for(item in items)overrides.push({targetId:stringField(item,"targetId"),
      property:stringField(item,"property"),value:numberField(item,"value")});
    return {reference:reference,version:version,overridesEnabled:overridesEnabled,overrides:overrides};
  }

  public static function decodeSensors(text:String):Null<Dynamic> {
    var root:Dynamic = Json.parse(text);
    if (stringField(root, "format") != FORMAT || numberField(root, "version") != VERSION)
      throw "Unsupported scene document";
    return Reflect.hasField(root, "sensors") ? Reflect.field(root, "sensors") : null;
  }

  /** Validate the entire file before allocating native scene resources. */
  public static function decode(text:String):Array<SceneObjectData> {
    var root:Dynamic = Json.parse(text);
    if (stringField(root, "format") != FORMAT) throw "This is not a Materia scene document";
    if (numberField(root, "version") != VERSION) throw "Unsupported scene document version";
    var raw:Dynamic = field(root, "objects");
    if (!Std.isOfType(raw, Array)) throw "Scene objects must be an array";
    var values:Array<Dynamic> = cast raw;
    if (values.length > 10000) throw "Scene documents support at most 10000 objects";
    var result:Array<SceneObjectData> = [];
    var ids:Map<String, Bool> = new Map();
    for (value in values) {
      var id = stringField(value, "id");
      if (id == "scene" || ids.exists(id)) throw "Duplicate or reserved object ID: " + id;
      ids.set(id, true);
      var kind = stringField(value, "type");
      if (kind != "rectangle" && kind != "cad-plate")
        throw "Unsupported scene object type: " + kind;
      var visible:Dynamic = field(value, "visible");
      if (!Std.isOfType(visible, Bool)) throw "Object visibility must be a boolean";
      var visibleValue:Bool=visible;
      var collisionEnabled=optionalBool(value,"collisionEnabled",visibleValue);
      var dynamicBody=optionalBool(value,"dynamicBody",false);
      var cadGraph=optionalText(value,"cadGraph");
      if(kind=="cad-plate"&&cadGraph!=null&&cadGraph.length>10000000)
        throw "CAD feature graph is too large";
      result.push({id: id, label: stringField(value, "label"), type: kind,
        x: bounded(value, "x", -1000000, 1000000), y: bounded(value, "y", -1000000, 1000000),
        z: bounded(value, "z", -1000000, 1000000),
        width: bounded(value, "width", 0.000001, 1000000),
        height: bounded(value, "height", 0.000001, 1000000),
        depth: optionalBounded(value,"depth",0.000001,1000000,0.1),
        collisionEnabled:collisionEnabled,dynamicBody:dynamicBody,
        mass:optionalBounded(value,"mass",0.000001,1000000,1.0),
        red: bounded(value, "red", 0, 1), green: bounded(value, "green", 0, 1),
        blue: bounded(value, "blue", 0, 1), visible: visibleValue,cadGraph:cadGraph});
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
    if (StringTools.trim(result).length == 0 || containsNul(result))
      throw "Scene field cannot be empty or contain NUL: " + name;
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
  static function optionalBounded(value:Dynamic,name:String,minimum:Float,maximum:Float,
      fallback:Float):Float return Reflect.hasField(value,name)
    ? bounded(value,name,minimum,maximum) : fallback;
  static function optionalBool(value:Dynamic,name:String,fallback:Bool):Bool {
    if(!Reflect.hasField(value,name))return fallback;
    var result=Reflect.field(value,name);if(!Std.isOfType(result,Bool))throw 'Scene field must be boolean: $name';
    return cast result;
  }
  static function optionalText(value:Dynamic,name:String):Null<String> {
    if(!Reflect.hasField(value,name)||Reflect.field(value,name)==null)return null;
    var result=Reflect.field(value,name);if(!Std.isOfType(result,String))throw 'Scene field must be text: $name';
    return cast result;
  }
}
