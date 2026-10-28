package toolpathkit.setup;

import toolpathkit.path.Provenance;

/** One toolpath point outside the configured machine travel bounds. */
class TravelViolation {
  public final axis:Int;
  public final value:Float;
  public final lower:Float;
  public final upper:Float;
  public final provenance:Provenance;

  public function new(axis:Int, value:Float, lower:Float, upper:Float, provenance:Provenance) {
    this.axis = axis;
    this.value = value;
    this.lower = lower;
    this.upper = upper;
    this.provenance = provenance;
  }

  public function message():String return '${["X", "Y", "Z"][axis]} travel $value m outside [$lower, $upper] m';
}
