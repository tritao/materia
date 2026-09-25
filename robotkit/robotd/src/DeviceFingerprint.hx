package robotd;

import haxe.Json;
import haxe.crypto.Sha256;
import haxe.io.Bytes;

/** The RKD5 layout fingerprint, matching tools/device_fingerprint.py. */
class DeviceFingerprint {
  static final DOMAIN = Bytes.ofString("RobotKit RKD5 device layout fingerprint v1");

  public static function compute(layout:Bytes, schemaLock:Bytes):String {
    if (layout.length == 0) throw "robotd: deployment layout must not be empty";
    var lock:Dynamic = Json.parse(schemaLock.toString());
    if (lock == null || Std.isOfType(lock, Array) || Std.isOfType(lock, String) ||
        Std.isOfType(lock, Int) || Std.isOfType(lock, Float) || Std.isOfType(lock, Bool))
      throw "robotd: device schema lock must be a JSON object";
    var canonical = Bytes.ofString(canonicalJson(lock));
    var input = Bytes.alloc(DOMAIN.length + 1 + 8 + canonical.length + 8 + layout.length);
    input.blit(0, DOMAIN, 0, DOMAIN.length);
    input.set(DOMAIN.length, 0);
    var offset = addBlock(input, DOMAIN.length + 1, canonical);
    addBlock(input, offset, layout);
    var digest = Sha256.make(input);
    var result = "";
    for (index in 0...16) result += StringTools.hex(digest.get(index), 2);
    return result.toLowerCase();
  }

  static function addBlock(output:Bytes, offset:Int, bytes:Bytes):Int {
    var size = bytes.length;
    for (index in 0...8) output.set(offset + index,
      index < 4 ? (size >>> (index * 8)) & 255 : 0);
    output.blit(offset + 8, bytes, 0, size);
    return offset + 8 + size;
  }

  static function canonicalJson(value:Dynamic):String {
    if (Std.isOfType(value, Array)) {
      var values:Array<Dynamic> = value;
      return "[" + [for (item in values) canonicalJson(item)].join(",") + "]";
    }
    if (value != null && !Std.isOfType(value, String) && !Std.isOfType(value, Int) &&
        !Std.isOfType(value, Float) && !Std.isOfType(value, Bool)) {
      var fields = Reflect.fields(value);
      fields.sort(Reflect.compare);
      return "{" + [for (field in fields)
        Json.stringify(field) + ":" + canonicalJson(Reflect.field(value, field))].join(",") + "}";
    }
    return Json.stringify(value);
  }
}
