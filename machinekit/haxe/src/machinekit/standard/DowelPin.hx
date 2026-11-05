package machinekit.standard;

import machinekit.component.ComponentType;
import machinekit.component.ComponentValues;
import machinekit.component.ComponentValue.*;
import machinekit.component.ComponentRecipeSupport;
import machinekit.component.Dimension;

import cadkit.modeling.Part;
import machinekit.catalog.Catalog;
import machinekit.catalog.CatalogMetadata.DimensionKind;
import machinekit.catalog.CatalogMetadata.Conformance;
import machinekit.component.ComponentDetail;
import machinekit.component.MachineComponent;
import machinekit.component.Solids;

/**
 * ISO 8734 type A dowel pin: hardened steel, diameter tolerance m6, driven into holes reamed H7 (a light press fit) in both
 * parts. CAD frame: axis along +Z, from z=0 to z=length; the chamfers are not modelled. Connector: `axis` at mid-length.
 * The preferred length series is `standardLengths()`; `standardLengthAtLeast` picks from it like a screw's.
 */
class DowelPin extends MachineComponent {
  static var table:Null<Catalog<DowelPinSpec>>;

  public final spec:DowelPinSpec;
  public final length:Float;

  static function rows():Array<DowelPinSpec> return [
    {size: "3", diameter: 3, chamfer: 0.4},
    {size: "4", diameter: 4, chamfer: 0.5},
    {size: "5", diameter: 5, chamfer: 0.6},
    {size: "6", diameter: 6, chamfer: 0.8},
    {size: "8", diameter: 8, chamfer: 1.0},
    {size: "10", diameter: 10, chamfer: 1.2},
    {size: "12", diameter: 12, chamfer: 1.5},
    {size: "16", diameter: 16, chamfer: 2.0},
  ];

  public static function catalog():Catalog<DowelPinSpec> {
    if (table == null) table = new Catalog(
      "dowel pin size",
      spec -> spec.size,
      rows(),
      _ -> ({
        source: "https://www.iso.org/standard/4408.html",
        standard: "ISO 8734",
        standardEdition: null,
        dimensionKind: Nominal,
        conformance: NominalEnvelope,
        sources: ["https://www.bossard.com/eshop/global-en/pins/dowel-pins-cylindrical-pins/"],
        verifiedFields: ["diameter"]
      })
    );
    return table;
  }

  /** Preferred nominal lengths (mm) of ISO 8734: the series the pin is bought in. */
  public static function standardLengths():Array<Float> return [6.0, 8, 10, 12, 14, 16, 18, 20, 22, 24, 26, 28, 30, 32, 35, 40, 45, 50, 55, 60, 70, 80, 90, 100];

  /** The shortest standard length of at least `minimum` mm, or null where the series ends before it. */
  public static function standardLengthAtLeast(minimum:Float):Null<Float> {
    for (length in standardLengths()) if (length >= minimum - 1e-9) return length;
    return null;
  }

  public static function metric(size:String, length:Float):DowelPin return new DowelPin(catalog().get(size), length);

  public static function custom(spec:DowelPinSpec, length:Float):DowelPin return new DowelPin(spec, length, true);

  private function new(spec:DowelPinSpec, length:Float, codeOnly:Bool = false) {
    if (!(length > 0) || !Math.isFinite(length)) throw "Dowel pin needs a positive length";
    if (spec == null || !(spec.diameter > 0) || !(spec.chamfer >= 0) || !(length > 2 * spec.chamfer))
      throw "Dowel pin specification has inconsistent ISO 8734 dimensions";
    var designation = 'ISO8734-${spec.size}x${Dimension.format(length)}';
    var customName = '${spec.size}-D${Dimension.format(spec.diameter)}-L${Dimension.format(length)}';
    super(
      codeOnly ? customDesignation(customName) : designation,
      'Dowel pin ${Dimension.format(spec.diameter)} m6 x ${Dimension.format(length)} mm',
      "steel",
      codeOnly
    );
    if (codeOnly) addFacet(new machinekit.component.ProcurementIntent(Buy(null, null, null)));
    addFacet(new machinekit.component.EngineeringMetadata(new machinekit.component.ComponentValues()
      .setToken("ratingsBasis", codeOnly ? "Stated" : "Catalog").setToken("errorsBasis", "Assumed")
      .setToken("source", codeOnly ? "" : "ISO 8734")));
    this.spec = spec;
    this.length = length;
    addConnector("axis", Axis, Solids.axial(0, 0, length / 2));
  }

  override public function hasGeometry():Bool return true;

  override public function geometryFidelity():machinekit.component.GeometryFidelity return machinekit.component.GeometryFidelity.Datasheet;

  override public function geometry(detail:ComponentDetail = Preview):Part return Solids.named(Part.cylinderSpan(spec.diameter / 2, 0, length), "pin");

  private static var recipeTypeCache:Null<ComponentType>;

  public static function recipeType():ComponentType {
    if (recipeTypeCache == null) recipeTypeCache = new ComponentType(
      "machinekit.standard.dowel-pin",
      [ComponentRecipeSupport.catalog("size", DowelPin.catalog(), "5"), ComponentRecipeSupport.length("length", 16)],
      v -> DowelPin.metric(v.token("size"), v.number("length")),
      true
    ).procured(_ -> Buy(null, null, "ISO 8734"));
    return recipeTypeCache;
  }

  override public function componentType():Null<ComponentType> return codeOnly ? null : recipeType();

  override public function values():ComponentValues {
    return new ComponentValues().set("size", Token(spec.size)).set("length", Number(length)).setToken("material", materialSpec());
  }

}
