package machinekit.hmi;

import machinekit.catalog.Catalog;
import machinekit.catalog.CatalogMetadata;
import machinekit.component.ComponentValues;

/**
 * The hand wheels and emergency stops we buy, as catalog rows: a designation, the recipe inputs that
 * describe the part, and the provenance of those numbers. A recipe's `model` input selects a row; `CUSTOM`
 * takes whatever the caller gives. Replace a row's numbers when the part is measured.
 */
class ControlCatalogs {
  public static inline var CUSTOM:String = "CUSTOM";
  public static inline var YUMO:String = "YUMO-ISM6045-100B";
  public static inline var VALLDER:String = "VALLDER-1165001";
  public static inline var XB2:String = "XB2-ES542";
  public static inline var EAO:String = "EAO-84-COMPACT";
  static var wheelCatalog:Null<Catalog<ControlRow>>;
  static var eStopCatalog:Null<Catalog<ControlRow>>;

  static function custom():ControlRow return {
    designation: CUSTOM,
    values: new ComponentValues(),
    metadata: {
      source: "Values given by the caller",
      standard: null,
      standardEdition: null,
      dimensionKind: Unverified,
      conformance: GenericApproximation
    }
  };

  public static function wheels():Catalog<ControlRow> {
    if (wheelCatalog == null) wheelCatalog = new Catalog(
      "hand wheel",
      row -> row.designation,
      [
        custom(),
        {
          designation: YUMO,
          values: new ComponentValues().setToken("basis", "catalog").setNumber("outerDiameter", 60).setNumber(
            "flangeThickness",
            6
          ).setNumber(
            "coneHeight",
            11
          ).setNumber(
            "ringDiameter",
            49
          ).setNumber(
            "ringHeight",
            9
          ).setNumber(
            "dialDiameter",
            44
          ).setNumber(
            "dialRecess",
            2
          ).setNumber(
            "handleDiameter",
            10
          ).setNumber(
            "handleProtrusion",
            12
          ).setNumber(
            "handleRadius",
            17
          ).setNumber(
            "bossDiameter",
            43
          ).setNumber(
            "bossDepth",
            3
          ).setNumber(
            "panelClearance",
            1
          ).setNumber(
            "studCircleDiameter",
            50.8
          ).setNumber(
            "studDiameter",
            3
          ).setNumber(
            "studHoleDiameter",
            3.2
          ).setNumber(
            "studLength",
            10
          ).setInteger(
            "studCount",
            3
          ).setNumber(
            "terminalLength",
            28
          ).setNumber(
            "terminalWidth",
            11
          ).setNumber(
            "terminalDepth",
            5.5
          ).setNumber(
            "maxPanelThickness",
            6
          ).setNumber(
            "massGrams",
            250
          ),
          metadata: {
            source: "YUMO ISM6045 maker drawing and the yumoelectric.com product page",
            standard: null,
            standardEdition: null,
            dimensionKind: Nominal,
            conformance: NominalEnvelope,
            verifiedFields: [
              "outerDiameter",
              "studCircleDiameter",
              "bossDiameter",
              "studLength",
              "studCount",
              "massGrams"
            ]
          }
        },
        {
          designation: VALLDER,
          values: new ComponentValues().setToken("basis", "catalog").setNumber("outerDiameter", 80).setNumber(
            "flangeThickness",
            7
          ).setNumber(
            "coneHeight",
            6
          ).setNumber(
            "ringDiameter",
            59.5
          ).setNumber(
            "ringHeight",
            14
          ).setNumber(
            "dialDiameter",
            50
          ).setNumber(
            "handleProtrusion",
            14
          ).setNumber(
            "handleRadius",
            18
          ).setNumber(
            "bossDiameter",
            44
          ).setNumber(
            "panelClearance",
            1
          ).setNumber(
            "studCircleDiameter",
            72
          ).setNumber(
            "studDiameter",
            4
          ).setNumber(
            "studHoleDiameter",
            4.8
          ).setNumber(
            "studLength",
            12
          ).setNumber(
            "terminalDepth",
            12
          ).setNumber(
            "terminalLength",
            30
          ).setNumber(
            "terminalWidth",
            12
          ).setNumber(
            "maxPanelThickness",
            3
          ).setNumber(
            "massGrams",
            276
          ),
          metadata: {
            source: "vallder.com/product/mpg-hand-wheel (SKU 1165001) dimension drawing",
            standard: null,
            standardEdition: null,
            dimensionKind: Nominal,
            conformance: NominalEnvelope,
            verifiedFields: ["outerDiameter", "ringDiameter", "studCircleDiameter", "bossDiameter", "studCount"]
          }
        }
      ],
      row -> row.metadata
    );
    return wheelCatalog;
  }

  public static function eStops():Catalog<ControlRow> {
    if (eStopCatalog == null) eStopCatalog = new Catalog(
      "emergency stop",
      row -> row.designation,
      [
        custom(),
        {
          designation: XB2,
          values: new ComponentValues().setToken("basis", "stated").setNumber("headDiameter", 40).setNumber(
            "collarDiameter",
            30
          ).setNumber(
            "collarHeight",
            9
          ).setNumber(
            "headHeight",
            22
          ).setNumber(
            "panelHole",
            22.3
          ).setNumber(
            "panelThickness",
            3
          ).setNumber(
            "nutDiameter",
            30
          ).setNumber(
            "nutThickness",
            5
          ).setNumber(
            "baseWidth",
            42
          ).setNumber(
            "baseDepth",
            30
          ).setNumber(
            "baseLength",
            11
          ).setNumber(
            "blockWidth",
            29
          ).setNumber(
            "blockThickness",
            14.8
          ).setNumber(
            "blockLength",
            24
          ).setNumber(
            "massGrams",
            52.4
          ),
          metadata: {
            source: "AliExpress item 1005002447999037 and a seller's size drawing for the XB2-ES542 family",
            standard: null,
            standardEdition: null,
            dimensionKind: Nominal,
            conformance: NominalEnvelope,
            verifiedFields: ["headDiameter", "massGrams"]
          }
        },
        {
          designation: EAO,
          values: new ComponentValues().setToken("basis", "assumed").setNumber("headHeight", 16).setNumber(
            "nutThickness",
            3
          ).setNumber(
            "baseLength",
            4.7
          ).setNumber(
            "blockLength",
            8
          ).setNumber(
            "baseWidth",
            30
          ).setNumber(
            "baseDepth",
            30
          ).setNumber(
            "blockWidth",
            22
          ).setNumber(
            "blockThickness",
            12
          ).setNumber(
            "massGrams",
            40
          ),
          metadata: {
            source: "EAO Series 84 and APEM EC Compact product pages (depth behind the panel only)",
            standard: "EN 60947-5-5",
            standardEdition: null,
            dimensionKind: Unverified,
            conformance: GenericApproximation
          }
        }
      ],
      row -> row.metadata
    );
    return eStopCatalog;
  }

  /** `inputs` with the selected model's values applied over them (the model's values win); `model` is set. */
  public static function withModel(catalog:Catalog<ControlRow>,
    defaultModel:String, inputs:Null<ComponentValues>):ComponentValues {
    var result = inputs == null ? new ComponentValues() : inputs.copy();
    var model = result.get("model") == null ? defaultModel : result.token("model");
    var row = catalog.get(model);
    for (name in row.values.names()) result.set(name, row.values.get(name));
    return result.setToken("model", model);
  }
}
