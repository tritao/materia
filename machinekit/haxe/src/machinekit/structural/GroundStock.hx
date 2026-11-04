package machinekit.structural;

import machinekit.component.Dimension;

/**
 * One ground stock size: a plate or flat bar whose faces are ground, so a face that only needs to be flat and parallel
 * needs no machining. `thicknessTolerance` is the symmetric deviation of the ground thickness (mm); `flatness` and
 * `parallelism` are over any 100 mm; `roughnessRa` in micrometres. They are the tolerances of the stock **as bought**: a
 * face that needs better is ground after cutting and says so on its `ManufacturingRoute` machined faces. The values are
 * catalogue-typical (`assumed`) until a supplier's row replaces them.
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
  /** Where the tolerances come from: a standard or a supplier datasheet. */
  var source:String;
  /** True while the values are typical figures not yet taken from a supplier's row for the grade bought. */
  var assumed:Bool;
}

/**
 * Ground plate and ground flat bar stock rows, the standard thicknesses and grinding tolerances a part is cut from:
 *
 * - **steel plate**: Blanchard- or precision-ground steel plate (C45 or 1.2842 tool steel), the material named by the caller;
 * - **aluminium plate**: cast tooling plate (MIC-6 / 5083 type), the material named by the caller;
 * - **flat bar**: precision-ground steel flat stock (DIN 59350 type, "gauge plate" bar), widths of 10 to 100 mm.
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

  public static inline final STEEL = "steel";
  public static inline final ALUMINIUM = "aluminium 6061";

  /**
   * The stock rows' tolerances as sold, by kind and material: thickness (±mm, tighter up to 12 mm), flatness and parallelism (mm over
   * 100 mm), and the source. ASSUMED: DIN 59350 gives precision-ground flat steel its own thickness classes and the Blanchard-ground
   * plate and cast tooling plate (MIC-6 datasheet: about ±0.1 mm thickness, flatness 0.1 mm over a metre) are looser still; the figures
   * here are typical catalogue values taken at the loose end of those, not a quoted grade, until a supplier row replaces them.
   */
  static function figures(kind:String, material:String, thickness:Float): {tolerance:Float, flatness:Float, parallelism:Float, source:String} {
    var thin = thickness <= 12;
    if (kind == "flat bar") return {
      tolerance: thin ? 0.03 : 0.05, flatness: 0.05, parallelism: 0.03,
      source: "precision-ground flat steel, DIN 59350 type (typical catalogue values, assumed)"
    };
    if (material == STEEL) return {
      tolerance: thin ? 0.05 : 0.10, flatness: 0.10, parallelism: 0.05,
      source: "Blanchard/precision-ground steel plate, C45 or 1.2842 (typical catalogue values, assumed)"
    };
    return {
      tolerance: thin ? 0.08 : 0.10, flatness: 0.10, parallelism: 0.05,
      source: "cast aluminium tooling plate, MIC-6 / 5083 type (datasheet-typical +-0.1 mm, assumed)"
    };
  }

  static function row(kind:String, material:String, thickness:Float):GroundStockRow {
    var tag = kind == "plate" ? "GROUND-PLATE" : "GROUND-BAR", f = figures(kind, material, thickness);
    return {
      id: tag + "-" + (material == STEEL ? "STEEL" : "AL") + "-" + Dimension.format(thickness),
      kind: kind,
      material: material,
      thickness: thickness,
      thicknessTolerance: f.tolerance,
      flatness: f.flatness,
      parallelism: f.parallelism,
      roughnessRa: 0.8,
      source: f.source,
      assumed: true
    };
  }

  /** The ground plate row of a standard `thickness` in `material` (`STEEL` or `ALUMINIUM`); there is no default material. */
  public static function plate(material:String, thickness:Float):GroundStockRow {
    if (material != STEEL && material != ALUMINIUM) throw 'No ground plate in "$material"; use "$STEEL" or "$ALUMINIUM"';
    if (!contains(plateThicknesses(), thickness)) throw 'No ground plate ${Dimension.format(thickness)} mm thick; standard: ${plateThicknesses()}';
    return row("plate", material, thickness);
  }

  /** The ground steel flat bar row of a standard `thickness`; `width` must be a standard width and exceed it. */
  public static function bar(width:Float, thickness:Float):GroundStockRow {
    if (!contains(barThicknesses(), thickness)) throw 'No ground flat bar ${Dimension.format(thickness)} mm thick; standard: ${barThicknesses()}';
    if (!contains(barWidths(), width)) throw 'No ground flat bar ${Dimension.format(width)} mm wide; standard: ${barWidths()}';
    if (width < thickness) throw "A ground flat bar is at least as wide as it is thick";
    return row("flat bar", STEEL, thickness);
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
