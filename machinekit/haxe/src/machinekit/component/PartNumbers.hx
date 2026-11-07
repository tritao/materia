package machinekit.component;

/** Compact geometry identities; human role text and exact defining inputs have separate jobs. */
class PartNumbers {
  public static function geometry(family:String, role:String, definingKey:String):String {
    if (definingKey == null || definingKey.length == 0) throw "A geometry number needs defining inputs";
    var prefix = token(family);
    var purpose = token(role);
    var available = PartNumberLint.MAX_LENGTH - prefix.length - 10;
    if (available < 1) throw "Part-number family leaves no room for a role";
    if (purpose.length > available) purpose = purpose.substr(0, available);
    while (StringTools.endsWith(purpose, "-")) purpose = purpose.substr(0, purpose.length - 1);
    if (purpose.length == 0) throw "Part-number role must not be empty";
    return prefix + "-" + purpose + "-" + haxe.crypto.Sha256.encode(definingKey).substr(0, 8).toUpperCase();
  }

  /** Canonical records of defining scalar inputs; binary64 numbers never use display rounding. */
  public static function exactKey(value:Dynamic):String {
    if (value == null) return "n";
    if (Std.isOfType(value, String)) {
      var text:String = cast value;
      return "s" + text.length + ":" + text;
    }
    if (Std.isOfType(value, Bool)) return value ? "b1" : "b0";
    if (Std.isOfType(value, Int) || Std.isOfType(value, Float))
      return "f" + ComponentNumberKey.encode(cast value) + ";";
    if (Std.isOfType(value, Array)) {
      var items:Array<Dynamic> = cast value;
      return "a[" + items.map(exactKey).join("") + "]";
    }
    var fields = [for (field in Reflect.fields(value)) if (Reflect.field(value, field) != null) field];
    fields.sort(Reflect.compare);
    return "o{" + [for (field in fields) exactKey(field) + exactKey(Reflect.field(value, field))].join("") + "}";
  }

  static function token(text:String):String {
    if (text == null || text.length == 0) throw "Part-number family and role must not be empty";
    var result = new StringBuf();
    var separator = false;
    for (index in 0...text.length) {
      var c = text.charAt(index).toUpperCase();
      var code = StringTools.fastCodeAt(c, 0);
      if ((code >= 65 && code <= 90) || (code >= 48 && code <= 57)) {
        if (separator && result.length > 0) result.add("-");
        result.add(c);
        separator = false;
      } else separator = true;
    }
    var value = result.toString();
    if (value.length == 0) throw "Part-number family and role need letters or digits";
    return value;
  }
}
