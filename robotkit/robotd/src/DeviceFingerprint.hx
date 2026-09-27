package robotd;

/** Source-compatible facade for the shared deployment fingerprint. */
class DeviceFingerprint {
  public static function compute(layout:haxe.io.Bytes, schemaLock:haxe.io.Bytes):String
    return robotkit.device.DeviceFingerprint.compute(layout, schemaLock);
}
