package app;

import haxe.Json;

/** Portable scene data. Native occurrence handles are deliberately not persisted. */
typedef SceneObjectData = {
  var id:String;
  var label:String;
  var type:String;
  var x:Float;
  var y:Float;
  var z:Float;
  var width:Float;
  var height:Float;
  var depth:Float;
  var collisionEnabled:Bool;
  var dynamicBody:Bool;
  var mass:Float;
  var red:Float;
  var green:Float;
  var blue:Float;
  var visible:Bool;
}

class SceneCodec {
  public static inline var FORMAT:String = "materia.scene";
  public static inline var VERSION:Int = 1;

  public static function encode(scene:EditorScene, ?sensors:SensorConfiguration):String
    return Json.stringify({format: FORMAT, version: VERSION, objects: scene.records(),
      sensors:sensors == null ? null : sensors.records()}, null, "  ") + "\n";

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
      if (kind != "rectangle") throw "Unsupported scene object type: " + kind;
      var visible:Dynamic = field(value, "visible");
      if (!Std.isOfType(visible, Bool)) throw "Object visibility must be a boolean";
      var collisionEnabled=optionalBool(value,"collisionEnabled",cast visible);
      var dynamicBody=optionalBool(value,"dynamicBody",false);
      result.push({id: id, label: stringField(value, "label"), type: kind,
        x: bounded(value, "x", -1000000, 1000000), y: bounded(value, "y", -1000000, 1000000),
        z: bounded(value, "z", -1000000, 1000000),
        width: bounded(value, "width", 0.000001, 1000000),
        height: bounded(value, "height", 0.000001, 1000000),
        depth: optionalBounded(value,"depth",0.000001,1000000,0.1),
        collisionEnabled:collisionEnabled,dynamicBody:dynamicBody,
        mass:optionalBounded(value,"mass",0.000001,1000000,1.0),
        red: bounded(value, "red", 0, 1), green: bounded(value, "green", 0, 1),
        blue: bounded(value, "blue", 0, 1), visible: cast visible});
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
}
