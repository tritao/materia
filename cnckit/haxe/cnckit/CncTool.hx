package cnckit;

/** Declared cutter dimensions in metres. */
class CncTool {
  public final number:Int;
  public final length:Float;
  public final diameter:Float;

  public function new(number:Int, length:Float, diameter:Float) {
    if (number < 0 || !Math.isFinite(length) ||
        !Math.isFinite(diameter) || diameter < 0.0)
      throw "CNC tool needs a non-negative number and finite dimensions";
    this.number = number;
    this.length = length;
    this.diameter = diameter;
  }
}
