package cnckit;

import cnckit.tool.CutterProfile;

/** Declared cutter dimensions in metres, with an optional exact shape. */
class CncTool {
  /** Flute length assumed for a tool with neither a shape nor a length. */
  public static inline final DEFAULT_FLUTE_LENGTH = 0.05;

  public final number:Int;
  public final length:Float;
  public final diameter:Float;
  /** The tool's shape; null when only its diameter is known. */
  public final cutter:Null<CutterProfile>;

  public function new(number:Int, length:Float, diameter:Float,
      ?cutter:CutterProfile) {
    if (number < 0 || !Math.isFinite(length) ||
        !Math.isFinite(diameter) || diameter < 0.0)
      throw "CNC tool needs a non-negative number and finite dimensions";
    if (cutter != null &&
        Math.abs(cutter.cuttingDiameter() - diameter) > 1e-9 * Math.max(1.0, diameter))
      throw 'CNC tool $number diameter does not match its cutter shape';
    this.number = number;
    this.length = length;
    this.diameter = diameter;
    this.cutter = cutter;
  }

  /** A tool whose diameter comes from its shape. */
  public static function shaped(number:Int, length:Float,
      cutter:CutterProfile):CncTool
    return new CncTool(number, length, cutter.cuttingDiameter(), cutter);

  /**
    The shape to simulate. Without an explicit shape this is a flat end mill
    of the declared diameter whose flutes run the whole tool length (or
    `DEFAULT_FLUTE_LENGTH` when no length is known), so it removes material
    but never reports shank or holder collisions.
  **/
  public function profile():CutterProfile {
    if (cutter != null) return cutter;
    if (diameter <= 0.0) throw 'CNC tool $number has no diameter or cutter shape';
    return CutterProfile.flat(diameter, length > 0.0 ? length : DEFAULT_FLUTE_LENGTH);
  }

  /** The same tool with a different stored length. */
  public function withLength(length:Float):CncTool
    return new CncTool(number, length, diameter, cutter);
}
