package robotkit.runtime;

import haxe.Int64;

/** Physical device observation; timestamps share the endpoint snapshot's source clock. */
class DeviceInputReading {
  public final active:Bool;
  public final sourceTimestampNs:Int64;
  public final closingCount:Int64;
  public final openingCount:Int64;
  public final capturedSteps:Int64;
  public final capturedTimestampNs:Int64;
  public final capturedPosition:Float;

  public function new(active:Bool, sourceTimestampNs:Int64, closingCount:Int64,
      openingCount:Int64, capturedSteps:Int64, capturedTimestampNs:Int64, capturedPosition:Float) {
    this.active = active; this.sourceTimestampNs = sourceTimestampNs;
    this.closingCount = closingCount; this.openingCount = openingCount;
    this.capturedSteps = capturedSteps; this.capturedTimestampNs = capturedTimestampNs;
    this.capturedPosition = capturedPosition;
  }
}
