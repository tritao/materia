package machinekit.printing;

import cadkit.modeling.Part;
import machinekit.catalog.Catalog;
import machinekit.catalog.CatalogMetadata.DimensionKind;
import machinekit.catalog.CatalogMetadata.Conformance;
import machinekit.component.ComponentDetail;
import machinekit.component.ComponentRecipeSupport;
import machinekit.component.ComponentType;
import machinekit.component.ComponentValues;
import machinekit.component.MachineComponent;
import machinekit.component.Solids;

typedef NozzleSpec = {
  var designation:String;
  /** Diameter of the orifice, mm. */
  var orifice:Float;
  /** Across the hex flats, and the heights of the hex and the cone below it, mm. */
  var hexSize:Float;
  var hexHeight:Float;
  var coneHeight:Float;
  var tipDiameter:Float;
}

/** A brass M6 nozzle. Origin: the orifice, with the nozzle standing up along +Z; `tip` is there. */
class Nozzle extends MachineComponent {
  static var table:Null<Catalog<NozzleSpec>>;
  static var recipe:Null<ComponentType>;
  public final spec:NozzleSpec;

  public static function catalog():Catalog<NozzleSpec> {
    if (table == null) table = new Catalog(
      "nozzle",
      s -> s.designation,
      [{designation: "BRASS-M6-0.4", orifice: 0.4, hexSize: 6.0, hexHeight: 3.0, coneHeight: 4.0, tipDiameter: 2.0}],
      _ -> ({
        source: "Assumed brass M6 nozzle; no vendor verification",
        standard: null,
        standardEdition: null,
        dimensionKind: Unverified,
        conformance: GenericApproximation,
        verifiedFields: []
      })
    );
    return table;
  }

  public function new(designation:String = "BRASS-M6-0.4") {
    var row = catalog().get(designation);
    super(designation, 'Nozzle, ${Std.string(row.orifice)} mm', "brass");
    if (Std.isExactType(this, Nozzle)) addFacet(new machinekit.component.EngineeringMetadata(new ComponentValues()
      .setToken("ratingsBasis", "Assumed").setToken("errorsBasis", "Assumed").setToken("source", "")));
    spec = row;
    addConnector("tip", Face, Solids.axial(0, 0, 0));
    addConnector("mount", Mount, Solids.axial(0, 0, row.coneHeight + row.hexHeight));
  }

  /** Cross-section of the orifice, mm^2. */
  public function orificeArea():Float return Math.PI * spec.orifice * spec.orifice / 4.0;

  public static function recipeType():ComponentType {
    if (recipe == null) recipe = new ComponentType(
      "machinekit.printing.nozzle",
      [ComponentRecipeSupport.catalog("designation", catalog(), "BRASS-M6-0.4")],
      v -> new Nozzle(v.token("designation"))
    ).numbered(v -> v.token("designation")).procured(_ -> Buy(null, null, null)).requiring(v -> {
      var row = catalog().get(v.token("designation"));
      return [machinekit.component.Procurement.ProcurementCodec.requirement("orifice", "mm", null, null, row.orifice),
        machinekit.component.Procurement.ProcurementCodec.choice("thread", "M6"),
        machinekit.component.Procurement.ProcurementCodec.choice("material", "brass")];
    });
    return recipe;
  }
  override public function componentType():Null<ComponentType> return Std.isExactType(this,
    Nozzle) ? recipeType() : null;
  override public function values():ComponentValues return new ComponentValues().setToken("designation",
    designation).setToken(
      "material",
      materialSpec()
    );
  override public function hasGeometry():Bool return true;
  /** The tip cone and its wrench hex. Envelope detail turns the hex's corners; the mass is the turned body across its flats. */
  override public function geometry(detail:ComponentDetail = Preview):Part {
    if (detail == Envelope) return envelope();
    var tip = spec.tipDiameter / 2, top = spec.hexSize / 2;
    return Solids.union([
      Solids.named(
        Part.revolve([{r: 0, z: 0}, {r: tip, z: 0}, {r: top * 0.9, z: spec.coneHeight}, {r: 0, z: spec.coneHeight}]),
        "cone"
      ),
      Solids.named(
        Part.prism(
          cadkit.modeling.Polygon.regular(6, spec.hexSize),
          spec.coneHeight - 0.01,
          spec.coneHeight + spec.hexHeight
        ),
        "hex"
      )
    ]);
  }

  override public function massGeometry():Null<Part> return body(spec.hexSize / 2);

  function envelope():Part return body(spec.hexSize / Math.sqrt(3));

  function body(top:Float):Part {
    var tip = spec.tipDiameter / 2;
    return Solids.named(
      Part.revolve([
        {r: 0, z: 0},
        {r: tip, z: 0},
        {r: top, z: spec.coneHeight},
        {r: top, z: spec.coneHeight + spec.hexHeight},
        {r: 0, z: spec.coneHeight + spec.hexHeight}
      ]),
      "body"
    );
  }
}
