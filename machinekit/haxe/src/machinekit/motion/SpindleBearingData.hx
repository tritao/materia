package machinekit.motion;

import machinekit.catalog.Catalog;

typedef SpindleBearingRow = {
  var model:String;
  var bore:Float;
  var outside:Float;
  var width:Float;
  var contactAngle:Float;
  var mass:Float;
  var dynamicLoad:Float;
  var staticLoad:Float;
  var preloadClass:String;
  var pairPreload:Float;
  var pairAxialStiffness:Float;
}

/** Maker facts only. The sheet states 20 N unmounted preload and 17 N/um
 * axial stiffness for a class-A DB/DF pair. Installed radial stiffness and
 * error allowances are separate project assumptions. */
class SpindleBearingData {
  public static inline final SKF_SOURCE = "https://docs.rs-online.com/4857/A700000013821392.pdf";
  public static inline final SKF_MODEL = "7002 CDGA/P4A";
  static var table:Null<Catalog<SpindleBearingRow>>;
  public static function catalog():Catalog<SpindleBearingRow> {
    if (table == null) table = new Catalog("super-precision spindle bearing", r -> r.model, [{
      model: SKF_MODEL, bore: 15.0, outside: 32.0, width: 9.0,
      contactAngle: Math.PI / 12, mass: 0.03, dynamicLoad: 5200.0, staticLoad: 2450.0, preloadClass: "GA", pairPreload: 20.0, pairAxialStiffness: 17000000.0
    }], _ -> ({
      source: SKF_SOURCE, standard: null, standardEdition: null,
      dimensionKind: Nominal, conformance: NominalEnvelope,
      verifiedFields: ["bore", "outside", "width", "contactAngle", "mass", "dynamicLoad", "staticLoad", "preloadClass", "pairPreload", "pairAxialStiffness"]
    }));
    return table;
  }
}
