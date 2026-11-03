package machinekit.structural;

import machinekit.component.Dimension;

/**
 * One ground stock size: a plate or flat bar whose faces are ground, so a face that only needs to be flat and parallel
 * needs no machining. `thicknessTolerance` is the symmetric deviation of the ground thickness (mm); `flatness` and
 * `parallelism` are over any 100 mm; `roughnessRa` in micrometres. The values are catalogue-typical and assumed until a
 * supplier's row replaces them.
 */
typedef GroundStockRow = {
  var id:String;
  /** "plate" or "flat bar". */
  var kind:String;
  /** A material library spec. */
  var material:String;
  var thickness:Float;
  var thicknessTolerance:Float;
  var flatness:Float;
  var parallelism:Float;
  var roughnessRa:Float;
}

/**
 * Ground plate and ground flat bar stock rows, the standard thicknesses and grinding tolerances a part is cut from:
 *
 * - **plate**: ground aluminium tooling plate (cast or rolled, the maker's grade stands in for "aluminium 6061");
 * - **flat bar**: ground steel flat stock ("gauge plate" bar), sold in widths of 10 to 100 mm.
 *
 * Both are looked up by thickness; `atLeast` picks the smallest standard thickness that leaves a required finished
 * thickness, so a part is drawn on stock rather than on an arbitrary dimension.
 */
class GroundStock {
  /** Standard thicknesses, mm. */
  public static function plateThicknesses():Array<Float> return [5, 6, 8, 10, 12, 15, 16, 20, 25, 30, 40, 50];

  public static function barThicknesses():Array<Float> return [3, 4, 5, 6, 8, 10, 12, 15, 16, 20, 25, 30, 40];

  /** Standard bar widths, mm. */
  public static function barWidths():Array<Float> return [10, 12, 15, 16, 20, 25, 30, 32, 40, 50, 60, 80, 100];

  static function contains(values:Array<Float>, value:Float):Bool {
    for (candidate in values) if (Math.abs(candidate - value) < 1e-9) return true;
    return false;
  }

  /** Grinding tolerance on thickness, ±mm: tighter on thin stock. */
  static function thicknessTolerance(thickness:Float):Float return thickness <= 12 ? 0.01 : thickness <= 30 ? 0.015 : 0.02;

  static function row(kind:String, material:String, thickness:Float, flatness:Float):GroundStockRow {
    var tag = kind == "plate" ? "GROUND-PLATE" : "GROUND-BAR";
    return {
      id: tag + "-" + (material == "steel" ? "STEEL" : "AL") + "-" + Dimension.format(thickness),
      kind: kind,
      material: material,
      thickness: thickness,
      thicknessTolerance: thicknessTolerance(thickness),
      flatness: flatness,
      parallelism: 0.01,
      roughnessRa: 0.8
    };
  }

  /** The ground aluminium plate row of a standard `thickness`. */
  public static function plate(thickness:Float):GroundStockRow {
    if (!contains(plateThicknesses(), thickness)) throw 'No ground plate ${Dimension.format(thickness)} mm thick; standard: ${plateThicknesses()}';
    return row("plate", "aluminium 6061", thickness, 0.02);
  }

  /** The ground steel flat bar row of a standard `thickness`; `width` must be a standard width and exceed it. */
  public static function bar(width:Float, thickness:Float):GroundStockRow {
    if (!contains(barThicknesses(), thickness)) throw 'No ground flat bar ${Dimension.format(thickness)} mm thick; standard: ${barThicknesses()}';
    if (!contains(barWidths(), width)) throw 'No ground flat bar ${Dimension.format(width)} mm wide; standard: ${barWidths()}';
    if (width < thickness) throw "A ground flat bar is at least as wide as it is thick";
    return row("flat bar", "steel", thickness, 0.01);
  }

  /** The smallest standard `kind` thickness of at least `thickness`, mm: the stock a part of that thickness is cut from. */
  public static function atLeast(kind:String, thickness:Float):Float {
    var sizes = switch kind {
      case "plate": plateThicknesses();
      case "flat bar": barThicknesses();
      default: throw 'Unknown ground stock kind "$kind"';
    };
    for (size in sizes) if (size >= thickness - 1e-9) return size;
    throw 'No ground $kind of ${Dimension.format(thickness)} mm or more';
  }
}
