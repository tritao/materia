package machinekit.motion;

/** How a sealed scale's housing and reading head must be mounted, mm. Tolerances are as drawn; checks use them later. */
typedef SealedScaleMounting = {
  /** Housing mounting face parallel to the travel, over the full length (datum F). */
  var housingParallelism:Float;
  /** Reading head parallel to the travel (datum A on JFT-40, F on JFT-10). */
  var headParallelism:Float;
  /** Reading head offset across the travel from the housing datum, as +-. */
  var headLateral:Float;
  /** Nominal running gap between housing and head, and its +- tolerance (null: not given). */
  var gap:Float;
  var gapTolerance:Null<Float>;
}

/** One series of sealed absolute scale. Lengths mm, mass kg. Dimensions are read from the maker's datasheet drawing. */
typedef SealedScaleSeries = {
  var series:String;
  /** Standard measuring lengths, ascending. */
  var lengths:Array<Float>;
  /** Overall housing length is ML plus this, without / with the mounting plate (null: not offered). */
  var overhang:Float;
  var overhangWithPlate:Null<Float>;
  /** Measuring start measured from the housing end, without / with plate. */
  var startOffset:Float;
  var startOffsetWithPlate:Null<Float>;
  /** Housing section across the travel (height, toward the head) and in plan (depth). */
  var housingHeight:Float;
  var housingDepth:Float;
  var headLength:Float;
  var headDepth:Float;
  var headHeight:Float;
  /** M5 mounting holes of the reading head, spaced along the travel. */
  var headHolePitch:Float;
  /** Housing mass: fixed part, and per metre of ML; the head's mass. */
  var housingBaseMass:Float;
  var massPerMetre:Float;
  var headMass:Float;
  /** Largest ML offered at +-3 um. */
  var maxMlFor3um:Float;
  /** Thermal expansion of the glass scale, 1/K, without / with the mounting plate. */
  var thermalExpansion:Float;
  var thermalExpansionWithPlate:Null<Float>;
  /** Mounting plate is mandatory from this ML (null: never). */
  var plateRequiredFrom:Null<Float>;
  var mounting:SealedScaleMounting;
  /** End fixing holes of the housing (JFT-40 only): through diameter and counterbore. */
  var holeDiameter:Null<Float>;
  var holeCounterbore:Null<Float>;
  /**
   * Depth of that counterbore from the face toward the head, mm. ASSUMED: the drawing's ~5 mm is read as the floor left under the screw head (13.5 housing less
   * 9.5 = 4.0 here: M8 ISO 4762 head 8 mm plus 1.5 mm sunk below the face, so the head never meets the reading head's gap), not as the depth, which no socket head fits.
   */
  var holeCounterboreDepth:Null<Float>;
  /** Thread of the reading head's two mounting holes and its depth in the head body, mm (JFT-40: M5; the depth is not on the datasheet: ASSUMED 8 of the 11.2 mm head). */
  var headThread:String;
  var headThreadDepth:Null<Float>;
}

/**
 * Yuheng Optics JFT-40 (slim) and JFT-10 (full size) sealed absolute glass scales: BiSS-C, 40.96 um grid, resolution
 * 2.5 / 5 / 10 nm, up to 180 m/min and 100 m/s2, 4 N movement force, IP53 (IP64 with air). Source: the maker's
 * datasheets `JFT-40.pdf` and `JFT-10.pdf` (build/suppliers/yuheng, not committed).
 */
class SealedScaleData {
  public static inline final SOURCE = "Yuheng Optics JFT-40 and JFT-10 datasheets (build/suppliers/yuheng/JFT-40.pdf, JFT-10.pdf)";
  public static inline final MAKER = "Yuheng Optics";
  public static inline final GRID_PITCH_UM = 40.96;
  public static inline final MAX_SPEED_M_MIN = 180.0;
  public static inline final MAX_ACCELERATION = 100.0;
  public static inline final MOVEMENT_FORCE = 4.0;

  static function range(from:Float, to:Float, step:Float):Array<Float> {
    var out:Array<Float> = [], v = from;
    while (v <= to + 1e-9) {
      out.push(v);
      v += step;
    }
    return out;
  }

  public static function all():Array<SealedScaleSeries> return [slim(), full()];

  public static function series(name:String):SealedScaleSeries {
    for (s in all()) if (s.series == name) return s;
    throw 'Unknown sealed scale series "$name"; offered: JFT-40, JFT-10';
  }

  /** JFT-40: 70 to 820 in 50 mm steps, then 920, 1020, 1140 and each 100 to 1840, 2040. */
  public static function slim():SealedScaleSeries return {
    series: "JFT-40",
    lengths: range(70, 820, 50).concat([920, 1020]).concat(range(1140, 1840, 100)).concat([2040]),
    overhang: 138,
    overhangWithPlate: 105,
    startOffset: 16.5,
    startOffsetWithPlate: 10.3,
    housingHeight: 13.5,
    housingDepth: 18,
    headLength: 115.3,
    headDepth: 18.25,
    headHeight: 11.2,
    headHolePitch: 97.5,
    housingBaseMass: 0.2,
    massPerMetre: 0.5,
    headMass: 0.2,
    maxMlFor3um: 2040,
    thermalExpansion: 8e-6,
    thermalExpansionWithPlate: 9e-6,
    plateRequiredFrom: 1340,
    mounting: {housingParallelism: 0.1, headParallelism: 0.05, headLateral: 0.2, gap: 1, gapTolerance: 0.3},
    holeDiameter: 9.6,
    holeCounterbore: 15,
    holeCounterboreDepth: 9.5,
    headThread: "M5",
    headThreadDepth: 8
  };

  /** JFT-10: 140 to 1840 in 100 mm steps, then 2040 to 4240 in 200 mm steps (the datasheet's list). */
  public static function full():SealedScaleSeries return {
    series: "JFT-10",
    lengths: range(140, 1840, 100).concat(range(2040, 4240, 200)),
    overhang: 121,
    overhangWithPlate: null,
    startOffset: 18.5,
    startOffsetWithPlate: null,
    housingHeight: 58.5,
    housingDepth: 37,
    headLength: 107,
    headDepth: 35,
    headHeight: 17,
    headHolePitch: 76,
    housingBaseMass: 0.4,
    massPerMetre: 2.6,
    headMass: 0.4,
    maxMlFor3um: 3040,
    thermalExpansion: 8e-6,
    thermalExpansionWithPlate: null,
    plateRequiredFrom: null,
    mounting: {housingParallelism: 0.1, headParallelism: 0.2, headLateral: 0.1, gap: 1.5, gapTolerance: null},
    holeDiameter: null,
    holeCounterbore: null,
    holeCounterboreDepth: null,
    headThread: "M5",
    headThreadDepth: null
  };

  /** The smallest standard measuring length of `series` that covers `travel`; throws past the longest. */
  public static function standardLength(name:String, travel:Float):Float {
    if (!(travel > 0) || !Math.isFinite(travel)) throw "Sealed scale needs a positive travel";
    var s = series(name);
    for (ml in s.lengths) if (ml >= travel - 1e-9) return ml;
    throw 'No $name measuring length covers ${travel} mm; longest is ${s.lengths[s.lengths.length - 1]}';
  }
}
