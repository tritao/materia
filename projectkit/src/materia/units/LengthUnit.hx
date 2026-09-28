package materia.units;

/** Supported model coordinate units and their SI scale. */
class LengthUnit {
  public static function metresPerUnit(unit:String):Float return switch (unit) {
    case "mm": 0.001;
    case "cm": 0.01;
    case "m": 1.0;
    case "in": 0.0254;
    default: throw 'Unsupported length unit "$unit"';
  };

  public static function fromScale(scale:Float):String {
    for (unit in ["mm", "cm", "m", "in"])
      if (Math.abs(metresPerUnit(unit) - scale) < 1e-12) return unit;
    throw "Unsupported model length scale";
  }
}
