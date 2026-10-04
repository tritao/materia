package robotkit.time;

import haxe.Int64;

/** A timestamp in the requested clock and the accumulated mapping error. */
class MappedTimestamp {
  public final valueNs:Int64;
  public final errorBoundNs:Int64;

  public function new(valueNs:Int64, errorBoundNs:Int64) {
    if (Int64.compare(errorBoundNs, Int64.ofInt(0)) < 0)
      throw "Mapped timestamp error bound must be non-negative";
    this.valueNs = valueNs;
    this.errorBoundNs = errorBoundNs;
  }
}
