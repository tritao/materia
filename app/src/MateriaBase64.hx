package app;

import haxe.io.Bytes;

/** Small bounded Base64 codec for binary buffers in project snapshots. */
class MateriaBase64 {
  static inline var ALPHABET:String = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";

  public static function encode(bytes:Bytes):String {
    var output = new StringBuf();
    var offset = 0;
    while (offset < bytes.length) {
      var first = bytes.get(offset++);
      var hasSecond = offset < bytes.length;
      var second = hasSecond ? bytes.get(offset++) : 0;
      var hasThird = offset < bytes.length;
      var third = hasThird ? bytes.get(offset++) : 0;
      output.add(ALPHABET.charAt(first >> 2));
      output.add(ALPHABET.charAt(((first & 3) << 4) | (second >> 4)));
      output.add(hasSecond ? ALPHABET.charAt(((second & 15) << 2) | (third >> 6)) : "=");
      output.add(hasThird ? ALPHABET.charAt(third & 63) : "=");
    }
    return output.toString();
  }

  public static function decode(value:String, maximumBytes:Int):Bytes {
    if (value == null || value.length == 0 || value.length % 4 != 0)
      throw "Invalid base64 geometry buffer";
    var padding = value.charAt(value.length - 1) == "=" ? 1 : 0;
    if (value.charAt(value.length - 2) == "=") padding++;
    if (padding > 2) throw "Invalid base64 geometry buffer";
    var outputLength = Std.int(value.length / 4) * 3 - padding;
    if (outputLength <= 0 || outputLength > maximumBytes)
      throw "Base64 geometry buffer is empty or too large";
    var output = Bytes.alloc(outputLength);
    var destination = 0;
    for (offset in 0...Std.int(value.length / 4)) {
      var source = offset * 4;
      var last = offset == Std.int(value.length / 4) - 1;
      var first = digit(value.charCodeAt(source));
      var second = digit(value.charCodeAt(source + 1));
      var thirdCode = value.charCodeAt(source + 2);
      var fourthCode = value.charCodeAt(source + 3);
      var third = thirdCode == 61 && last ? 0 : digit(thirdCode);
      var fourth = fourthCode == 61 && last ? 0 : digit(fourthCode);
      if (first < 0 || second < 0 || third < 0 || fourth < 0 ||
          (!last && (thirdCode == 61 || fourthCode == 61)) ||
          (thirdCode == 61 && fourthCode != 61) ||
          (thirdCode == 61 && offset != Std.int(value.length / 4) - 1))
        throw "Invalid base64 geometry buffer";
      var packed = (first << 18) | (second << 12) | (third << 6) | fourth;
      if (destination < outputLength) output.set(destination++, (packed >>> 16) & 0xff);
      if (destination < outputLength) output.set(destination++, (packed >>> 8) & 0xff);
      if (destination < outputLength) output.set(destination++, packed & 0xff);
    }
    return output;
  }

  static function digit(code:Null<Int>):Int {
    if (code == null) return -1;
    if (code >= 65 && code <= 90) return code - 65;
    if (code >= 97 && code <= 122) return code - 71;
    if (code >= 48 && code <= 57) return code + 4;
    if (code == 43) return 62;
    if (code == 47) return 63;
    return -1;
  }
}
