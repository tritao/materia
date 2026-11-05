package machinekit.hobbing;

import machinekit.catalog.Catalog;
import machinekit.component.ComponentValues;

/** Purchased D80 BT30 belt spindle. Only verifiedFields are supplier facts.
 * The listing's linked drawing says NT30 and contradicts its bearing text: it
 * cannot establish BT30 axial dimensions. Packaging below is ASSUMED pending
 * the selected option's outline; it is not a manufacturing drawing. */
class Bt30SpindleData {
  public static inline final DESIGNATION = "ROBOTDIGG-2049-BT30-D80-6000";
  public static inline final SOURCE = "https://www.robotdigg.com/product/2049";
  public static inline final SHANK_SOURCE = "https://www.ptpcnctoolings.com/shanks/bt";
  public static inline final OUTSIDE = 80.0;
  public static inline final MAX_RPM = 6000.0;
  // ASSUMED axial packaging and register, deliberately separate from catalogue facts.
  public static inline final GAUGE_Z = 180.0;
  public static inline final CLAMP_FROM = 10.0;
  public static inline final CLAMP_TO = 168.0;
  public static inline final FLANGE_FROM = 170.0;
  public static inline final FLANGE_DIAMETER = 110.0;
  public static inline final ROTOR_DIAMETER = 60.0;
  public static inline final STUB_DIAMETER = 20.0;
  public static inline final STUB_REAR = -36.0;
  // MAS BT30 shank, Sumitomo standard-tapers table; diametral taper 7/24.
  public static inline final TAPER_DIAMETER = 31.75;
  public static inline final TAPER_LENGTH = 48.4;
  public static inline final TAPER_RATIO = 7.0 / 24;
  public static inline final HOLDER_FLANGE = 46.0;
  public static inline final HOLDER_GAP = 2.0;
  public static inline final HOLDER_LENGTH = 20.0;
  public static inline final DRIVE_SLOT_WIDTH = 16.1;
  public static inline final DRIVE_SLOT_FLOOR = 16.3;
  /** ASSUMED axial dog engagement; matched to the purchased spindle before manufacture. */
  public static inline final DRIVE_DOG_LENGTH = 10.0;

  public static function catalog():Catalog<ComponentValues> return new Catalog(
    "D80 BT30 belt-driven spindle", _ -> DESIGNATION,
    [new ComponentValues().setNumber("outside", OUTSIDE).setNumber("maxRpm", MAX_RPM)
      .setToken("interface", "BT30").setToken("beltProfile", "HTD5M").setToken("bearingFamily", "7008+7006")],
    _ -> ({source: SOURCE, standard: null, standardEdition: null, dimensionKind: Mixed,
      conformance: GenericApproximation,
      verifiedFields: ["outside", "maxRpm", "interface", "beltProfile", "bearingFamily"]})
  );

  /** Shared male/female taper, with its gauge plane at z. */
  public static function taper(z:Float):cadkit.modeling.Part {
    var r = TAPER_DIAMETER / 2;
    return cadkit.modeling.Part.revolve([
      {r: 0.0, z: z - TAPER_LENGTH},
      {r: r - TAPER_LENGTH * TAPER_RATIO / 2, z: z - TAPER_LENGTH},
      {r: r, z: z}, {r: 0.0, z: z}
    ]);
  }
}
