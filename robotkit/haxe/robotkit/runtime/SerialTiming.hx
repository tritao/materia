package robotkit.runtime;

import haxe.Int64;

/** Mirrors the RKD5 one-way command-frame qualification before opening a device. */
class SerialTiming {
  public static function requireQualified(blueprint:RobotRuntimeBlueprint, baud:Int):Void {
    var period = Int64.toFloat(blueprint.ownerPeriodNs);
    var allowance = Int64.toFloat(blueprint.serialProcessingAllowanceNs);
    if (period == 0.0) period = 10000000.0;
    if (allowance == 0.0) allowance = 2000000.0;
    var frame = baud > 0
      ? Math.ceil((12 + 20 + blueprint.jointCount * 8) * 10000000000.0 / baud)
      : 0.0;
    var minimum = frame + allowance;
    if (baud > 0 && period > 0.0 && allowance >= 0.0 && period >= minimum)
      return;
    throw 'Serial runtime timing unsupported: baud=$baud joint_count=${blueprint.jointCount} '
      + 'frame_time_ns=$frame processing_allowance_ns=$allowance '
      + 'minimum_owner_period_ns=$minimum configured_owner_period_ns=$period';
  }
}
