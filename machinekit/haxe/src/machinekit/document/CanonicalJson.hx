package machinekit.document;

/** A JSON value built in code, written with sorted keys so a record's text depends only on its content. */
enum JsonValue {
  JNull;
  JBool(value:Bool);
  JNumber(value:Float);
  JString(value:String);
  JArray(items:Array<JsonValue>);
  JObject(fields:Map<String, JsonValue>);
}

/**
 * Deterministic JSON for records other tools read and hash (pcbkit's envelope, for one): keys sorted, two-space indent,
 * numbers rounded to `digits` decimals and written without a trailing ".0", so regenerating an unchanged record gives the
 * same bytes and a change shows as a small diff.
 */
class CanonicalJson {
  public static function write(value:JsonValue, digits:Int = 4):String return text(value, digits, "") + "\n";

  public static function number(value:Float, digits:Int = 4):Float {
    var scale = Math.pow(10, digits);
    var rounded = Math.round(value * scale) / scale;
    return rounded == 0 ? 0.0 : rounded; // no "-0"
  }

  static function text(value:JsonValue, digits:Int, indent:String):String {
    var inner = indent + "  ";
    return switch value {
      case JNull:
        "null";
      case JBool(b):
        b ? "true" : "false";
      case JNumber(n):
        numberText(number(n, digits));
      case JString(s):
        quote(s);
      case JArray(items):
        if (items.length == 0) "[]" else if (flat(items)) "[" + [for (i in items) text(i,
          digits, inner)].join(", ") + "]" else "[\n" + [for (i in items) inner + text(
            i,
            digits,
            inner
          )].join(",\n") + "\n" + indent + "]";
      case JObject(fields):
        var keys = [for (k in fields.keys()) k];
        keys.sort((a, b) -> a < b ? -1 : (a > b ? 1 : 0));
        if (keys.length == 0) "{}" else "{\n" + [for (k in keys) inner + quote(k) + ": " + text(
          fields.get(k),
          digits,
          inner
        )].join(",\n") + "\n" + indent + "}";
    };
  }

  /** Arrays of plain numbers (a point, a rectangle) stay on one line. */
  static function flat(items:Array<JsonValue>):Bool {
    for (i in items) switch i {
      case JNumber(_):
      default:
        return false;
    }
    return true;
  }

  static function numberText(n:Float):String {
    if (Math.isNaN(n) || !Math.isFinite(n)) throw "CanonicalJson: a number must be finite";
    if (n == Math.ffloor(n) && Math.abs(n) < 1e9) return Std.string(Std.int(n));
    return Std.string(n);
  }

  static function quote(s:String):String {
    var out = new StringBuf();
    out.add('"');
    for (i in 0...s.length) {
      var c = s.charCodeAt(i);
      switch c {
        case 0x22:
          out.add('\\"');
        case 0x5C:
          out.add("\\\\");
        case 0x0A:
          out.add("\\n");
        case 0x0D:
          out.add("\\r");
        case 0x09:
          out.add("\\t");
        default:
          if (c < 0x20) out.add("\\u" + StringTools.hex(c, 4)) else out.addChar(c);
      }
    }
    out.add('"');
    return out.toString();
  }
}
