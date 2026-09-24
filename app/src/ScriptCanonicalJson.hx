package app;

import haxe.Json;

/** Stable JSON for script configuration identity, independent of field order. */
class ScriptCanonicalJson {
  public static function encode(value:Dynamic):String {
    if (value == null) return "null";
    if (Std.isOfType(value, Array)) {
      var values:Array<Dynamic> = cast value;
      return "[" + [for (item in values) encode(item)].join(",") + "]";
    }
    if (Reflect.isObject(value) && !Std.isOfType(value, String)) {
      var names = Reflect.fields(value);
      names.sort(Reflect.compare);
      return "{" + [for (name in names) Json.stringify(name) + ":"
        + encode(Reflect.field(value, name))].join(",") + "}";
    }
    return Json.stringify(value);
  }
}
