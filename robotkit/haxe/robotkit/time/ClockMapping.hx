package robotkit.time;

import haxe.Int64;

/** Explicit affine mapping between clocks whose timestamps are nanoseconds. */
class ClockMapping {
  static final MAX_EXACT_DELTA = Int64.parseString("9007199254740991");
  static final MIN_EXACT_DELTA = Int64.parseString("-9007199254740991");

  public final fromClockId:String;
  public final toClockId:String;
  /** Target minus source time at the anchor. */
  public final offsetNs:Int64;
  /** Target clock rate relative to source, in parts per billion. */
  public final skewPpb:Float;
  public final errorBoundNs:Int64;
  /** Earliest valid timestamp in the target clock. Also the offset anchor. */
  public final validFromNs:Int64;
  /** Provenance of this measured or configured relationship. */
  public final source:String;

  public function new(fromClockId:String, toClockId:String, offsetNs:Int64,
      skewPpb:Float, errorBoundNs:Int64, validFromNs:Int64, source:String) {
    if (fromClockId == null || fromClockId.length == 0 ||
        toClockId == null || toClockId.length == 0 || fromClockId == toClockId ||
        source == null || source.length == 0 || !Math.isFinite(skewPpb) ||
        skewPpb <= -1000000000.0 ||
        Int64.compare(errorBoundNs, Int64.ofInt(0)) < 0)
      throw "Clock mapping requires distinct clocks, provenance, positive rate, and a non-negative bound";
    this.fromClockId = fromClockId; this.toClockId = toClockId;
    this.offsetNs = offsetNs; this.skewPpb = skewPpb;
    this.errorBoundNs = errorBoundNs; this.validFromNs = validFromNs;
    this.source = source;
  }

  /** Returns null before validity or when an exact integer-ns result cannot be represented. */
  public function map(timestampNs:Int64):Null<MappedTimestamp> {
    var sourceAnchor = checkedSubtract(validFromNs, offsetNs);
    if (sourceAnchor == null) return null;
    var delta = checkedSubtract(timestampNs, sourceAnchor);
    if (delta == null || Int64.compare(delta, MAX_EXACT_DELTA) > 0 ||
        Int64.compare(delta, MIN_EXACT_DELTA) < 0) return null;
    // The relative interval is exact as Float below 2^53 ns. Keeping the
    // absolute epoch in Int64 avoids loss of nanoseconds near today's dates.
    var correction = Math.round(Int64.toFloat(delta) * skewPpb / 1000000000.0);
    if (!Math.isFinite(correction) || Math.abs(correction) > 9007199254740991.0)
      return null;
    var shifted = checkedAdd(timestampNs, offsetNs);
    if (shifted == null) return null;
    var value = checkedAdd(shifted, Int64.fromFloat(correction));
    if (value == null || Int64.compare(value, validFromNs) < 0) return null;
    return new MappedTimestamp(value, errorBoundNs);
  }

  public static function checkedAdd(left:Int64, right:Int64):Null<Int64> {
    var result = Int64.add(left, right);
    if (Int64.compare(right, Int64.ofInt(0)) > 0 && Int64.compare(result, left) < 0 ||
        Int64.compare(right, Int64.ofInt(0)) < 0 && Int64.compare(result, left) > 0)
      return null;
    return result;
  }

  public static function checkedSubtract(left:Int64, right:Int64):Null<Int64> {
    var result = Int64.sub(left, right);
    if (Int64.compare(right, Int64.ofInt(0)) > 0 && Int64.compare(result, left) > 0 ||
        Int64.compare(right, Int64.ofInt(0)) < 0 && Int64.compare(result, left) < 0)
      return null;
    return result;
  }
}
