package app;

/** Checks for the lowercase hexadecimal SHA-256 digests used in project and script identities. */
class Sha256Digest {
  public static function isHex(value:Null<String>):Bool {
    if (value == null || value.length != 64) return false;
    for (index in 0...64) {
      var code = StringTools.fastCodeAt(value, index);
      if (!((code >= "0".code && code <= "9".code) || (code >= "a".code && code <= "f".code))) return false;
    }
    return true;
  }
}
