package processkit.moonraker;

/**
 * Compares a Klipper configuration as written with the one Klipper reports running (`configfile.settings` and
 * `configfile.warnings`). Klipper lower-cases keys and returns numbers and booleans as such, and adds defaults for what the
 * file leaves out; only what the file says is compared. A pin that is still a placeholder is for the user to replace, so a
 * caller passes the text it actually uploaded.
 */
class ConfigReadback {
  /** Relative tolerance of two numbers: Klipper reads a decimal and writes a double. */
  public static inline var TOLERANCE:Float = 1e-6;

  static function number(text:String):Null<Float> {
    var trimmed = StringTools.trim(text);
    if (trimmed.length == 0) return null;
    for (index in 0...trimmed.length) {
      var code = trimmed.charCodeAt(index);
      if (!((code >= 48 && code <= 57) || code == 43 || code == 45 || code == 46 || code == 101 || code == 69)) return null;
    }
    var value = Std.parseFloat(trimmed);
    return Math.isFinite(value) ? value : null;
  }

  static function flatten(value:Dynamic):Array<Float> {
    if (Std.isOfType(value, Array)) {
      var result:Array<Float> = [];
      for (item in (value:Array<Dynamic>)) result = result.concat(flatten(item));
      return result;
    }
    if (Std.isOfType(value, Float) || Std.isOfType(value, Int)) return [(value:Float)];
    return [Math.NaN];
  }

  /** The differences, each a sentence: a section or key the machine does not have, a value that differs, a warning Klipper gave. */
  public static function compare(expected:Map<String, Map<String, String>>, settings:Dynamic, warnings:Array<Dynamic>):Array<String> {
    var problems:Array<String> = [];
    if (settings == null) return ["Klipper reports no running configuration"];
    var sections = [for (name in expected.keys()) name];
    sections.sort(Reflect.compare);
    for (name in sections) {
      var running:Dynamic = Reflect.field(settings, name.toLowerCase());
      if (running == null) { problems.push('[$name] is not in the running configuration'); continue; }
      var keys = expected.get(name);
      var names = [for (key in keys.keys()) key];
      names.sort(Reflect.compare);
      for (key in names) {
        var wanted = StringTools.trim(keys.get(key));
        var actual:Dynamic = Reflect.field(running, key.toLowerCase());
        if (actual == null) { problems.push('[$name] $key is not in the running configuration'); continue; }
        var same:Bool;
        if (Std.isOfType(actual, Array)) {
          // A list Klipper parsed (`gear_ratio: 54:18` is [[54, 18]]): the same numbers in the same order.
          var values = flatten(actual);
          var parts = [for (part in ~/[,:\n]/g.split(wanted)) number(part)];
          same = parts.length == values.length;
          for (index in 0...parts.length) {
            var part = parts[index];
            if (!same) break;
            same = part != null && Math.abs(part - values[index]) <= TOLERANCE * Math.max(1.0, Math.abs(part));
          }
        } else if (Std.isOfType(actual, Bool)) same = wanted.toLowerCase() == (actual ? "true" : "false");
        else if (Std.isOfType(actual, Float) || Std.isOfType(actual, Int)) {
          var parsed = number(wanted);
          var value:Float = actual;
          same = parsed != null && Math.abs(parsed - value) <= TOLERANCE * Math.max(1.0, Math.abs(parsed));
        } else same = StringTools.trim(Std.string(actual)).toLowerCase() == wanted.toLowerCase();
        if (!same) problems.push('[$name] $key is $actual, the file says $wanted');
      }
    }
    for (warning in warnings) {
      var message:Dynamic = Reflect.field(warning, "message");
      problems.push("Klipper warns: " + (message == null ? Std.string(warning) : Std.string(message)));
    }
    return problems;
  }
}
