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

/**
 * One series of sealed absolute scale. Lengths mm, mass kg. JFT-40 is read from the maker's STEP models (`YuhengScaleCad`, ML 70, 120 and 220, checked
 * by `SealedScaleStepTests`); JFT-10 from its datasheet drawing only (no STEP yet), so its housing and head sections are unverified.
 *
 * The part frame (`SealedScalePart`): travel along +Z; Y runs from the free face (0) to the mounting face (`housingThickness`) of the housing, through
 * the fixing holes; X lies across the travel with 0 on the fixing-hole axis (the housing centre when the series has no end holes). The reading head
 * stands beside the housing, on the reading face's side, not above it: the sandwich is seat, housing, with the head level with it and carried from the
 * free side.
 */
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
  /** Housing thickness along the fixing holes (free face to mounting face) and its width across the travel (reading face to the opposite face). */
  var housingThickness:Float;
  var housingWidth:Float;
  /** The fixing-hole axis's distance from the face opposite the reading face (JFT-40 only: null for no end holes). */
  var holeOffset:Null<Float>;
  /** The reading finger's depth into the housing below the reading face: the measuring line lies this far under it. */
  var readingDepth:Float;
  /** Reading head body: along the travel, thickness (along Y, as the housing's), and height from the gap face away from the housing. */
  var headLength:Float;
  var headThickness:Float;
  var headWidth:Float;
  /** The reading finger's centre (the reading datum) from the head body's centre along the travel, positive away from the cable end. */
  var headFingerOffset:Float;
  /** Strain relief at the cable end: length beyond the body's end, and its square section; the cable leaves straight, `cableDiameter`, `cableStraight` long. */
  var bootLength:Float;
  var bootSize:Float;
  var cableDiameter:Float;
  var cableStraight:Float;
  /** The head's two mounting holes along Y (through the thickness), spaced along the travel; null when not drawn (JFT-10). */
  var headHolePitch:Float;
  /** Pair centre from the reading finger's centre along the travel, positive away from the cable. */
  var headHoleOffset:Float;
  /** Hole axis height above the gap face. */
  var headHoleHeight:Float;
  /** Slot width, slot length along the travel (a round hole is 0), counterbore width and depth from each face. */
  var headHoleDiameter:Null<Float>;
  var headHoleSlot:Float;
  var headCounterbore:Null<Float>;
  var headCounterboreDepth:Null<Float>;
  /** The screw the head's holes take. */
  var headThread:String;
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
  /** End fixing holes of the housing (JFT-40 only): through diameter, counterbore diameter and its depth from each face (both faces, so either mounts). */
  var holeDiameter:Null<Float>;
  var holeCounterbore:Null<Float>;
  var holeCounterboreDepth:Null<Float>;
  /** Air purge port: thread, and its position from the reading face (toward the opposite face), on both housing ends; null: no port. */
  var airThread:Null<String>;
  var airFromReadingFace:Float;
}

/**
 * Yuheng Optics JFT-40 (slim) and JFT-10 (full size) sealed absolute glass scales: BiSS-C, 40.96 um grid, resolution
 * 2.5 / 5 / 10 nm, up to 180 m/min and 100 m/s2, 4 N movement force, IP53 (IP64 with air). Source: the maker's
 * datasheets `JFT-40.pdf` and `JFT-10.pdf` (build/suppliers/yuheng, not committed).
 */
class SealedScaleData {
  public static inline final SOURCE = "Yuheng Optics JFT-40 and JFT-10 datasheets (build/suppliers/yuheng/JFT-40.pdf, JFT-10.pdf); JFT-40 section, holes, head and ports from supplier STEP "
    + YuhengScaleCad.STEP_70 + " sha " + YuhengScaleCad.STEP_70_SHA256 + ", " + YuhengScaleCad.STEP_120 + " sha " + YuhengScaleCad.STEP_120_SHA256 + ", "
    + YuhengScaleCad.STEP_220 + " sha " + YuhengScaleCad.STEP_220_SHA256;
  public static inline final MAKER = "Yuheng Optics";
  public static inline final GRID_PITCH_UM = 40.96;
  public static inline final MAX_SPEED_M_MIN = 180.0;
  public static inline final MAX_ACCELERATION = 100.0;
  public static inline final MOVEMENT_FORCE = 4.0;
  /** Air purge connection: the maker names only the thread (M5); the straight push-in fitting for a 4 mm tube and its tube run are ASSUMED. */
  public static inline final AIR_FITTING_DIAMETER = 9.0;
  public static inline final AIR_FITTING_LENGTH = 18.0;
  public static inline final AIR_TUBE_DIAMETER = 4.0;
  public static inline final AIR_TUBE_STRAIGHT = 25.0;
  /** Smallest cable bend radius: the cable datasheet's flexing life (more than 10 million cycles) holds from 75 mm (YH-DL, Ø6.7 PUR); no static radius is given. */
  public static inline final CABLE_BEND_RADIUS = 75.0;

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

  /**
   * JFT-40: 70 to 820 in 50 mm steps, then 920, 1020, 1140 and each 100 to 1840, 2040. Dimensions from the maker's STEP models (ML 70, 120, 220; the
   * three agree on everything but the length) unless marked: housing 18 thick (end caps 18.25) x 32.2 across, ML + 138 long, fixing holes Ø9.6 along the
   * thickness 11.5 from each end and 13.5 above the bottom face with a Ø15 counterbore 6 deep from each face (5.75 from the extrusion's), a groove in the
   * reading face (8 x 3.2) that the head's finger runs in; head 97.5 long and 13 high above a 1 mm gap, two Ø4.6 x 2.4 slots 56 apart with Ø7.6
   * counterbores 5 deep from each face, a 13 mm square strain relief 17.5 beyond the body and a Ø6 cable stub 54.5 from the body's end.
   * The drawing's "13.5" is the hole axis height, not the housing's height (it is 32.2), and its 97.5 is the head body, with 115.3 its length with the boot.
   * ASSUMED: the reading finger's centre (the finger is the 25 mm nub in the groove; the maker gives no datum) and the measuring line at the groove floor.
   */
  public static function slim():SealedScaleSeries return {
    series: "JFT-40",
    lengths: range(70, 820, 50).concat([920, 1020]).concat(range(1140, 1840, 100)).concat([2040]),
    overhang: 138,
    overhangWithPlate: 105,
    startOffset: 16.5,
    startOffsetWithPlate: 10.3,
    housingThickness: 18,
    housingWidth: 32.2,
    holeOffset: 13.5,
    readingDepth: 3.2,
    headLength: 97.5,
    headThickness: 18,
    headWidth: 13,
    headFingerOffset: -2.75,
    bootLength: 17.5,
    bootSize: 13,
    cableDiameter: 6.7,
    cableStraight: 37,
    headHolePitch: 56,
    headHoleOffset: 10.5,
    headHoleHeight: 9,
    headHoleDiameter: 4.6,
    headHoleSlot: 2.4,
    headCounterbore: 7.6,
    headCounterboreDepth: 5,
    headThread: "M4",
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
    holeCounterboreDepth: 5.75,
    airThread: "M5",
    airFromReadingFace: 7
  };

  /** JFT-10: 140 to 1840 in 100 mm steps, then 2040 to 4240 in 200 mm steps (the datasheet's list). Section from the datasheet drawing only, not checked against a STEP. */
  public static function full():SealedScaleSeries return {
    series: "JFT-10",
    lengths: range(140, 1840, 100).concat(range(2040, 4240, 200)),
    overhang: 121,
    overhangWithPlate: null,
    startOffset: 18.5,
    startOffsetWithPlate: null,
    housingThickness: 37,
    housingWidth: 58.5,
    holeOffset: null,
    readingDepth: 0,
    headLength: 107,
    headThickness: 35,
    headWidth: 17,
    headFingerOffset: 0,
    bootLength: 0,
    bootSize: 0,
    cableDiameter: 0,
    cableStraight: 0,
    headHolePitch: 76,
    headHoleOffset: 0,
    headHoleHeight: 0,
    headHoleDiameter: null,
    headHoleSlot: 0,
    headCounterbore: null,
    headCounterboreDepth: null,
    headThread: "M5",
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
    airThread: null,
    airFromReadingFace: 0
  };

  /** The smallest standard measuring length of `series` that covers `travel`; throws past the longest. */
  public static function standardLength(name:String, travel:Float):Float {
    if (!(travel > 0) || !Math.isFinite(travel)) throw "Sealed scale needs a positive travel";
    var s = series(name);
    for (ml in s.lengths) if (ml >= travel - 1e-9) return ml;
    throw 'No $name measuring length covers ${travel} mm; longest is ${s.lengths[s.lengths.length - 1]}';
  }
}
