package machinekit.pneumatic;

import cadkit.InertiaTensor;
import cadkit.modeling.Part;
import cadkit.modeling.Vector;
import machinekit.catalog.Catalog;
import machinekit.catalog.CatalogMetadata.DimensionKind;
import machinekit.catalog.CatalogMetadata.Conformance;
import machinekit.component.ComponentDetail;
import machinekit.component.MachineComponent;
import machinekit.component.PortInterface;
import machinekit.component.PortKind;
import machinekit.component.PortRole;
import machinekit.component.Solids;
import machinekit.component.ComponentRecipeSupport;
import machinekit.component.ComponentType;

typedef PushInFittingRow = {
  var maker:String;
  var designation:String;
  var label:String;
  var envelopeDiameterMm:Float;
  var envelopeLengthMm:Float;
  var tubeOdMm:Float;
  var thread:String;
  var massKg:Float;
}

/** A threaded push-in fitting from a maker's catalogue: thread to tube OD. Cylindrical envelope and centre/inertia
 * are approximations; the published weight is retained. */
class PushInFitting extends MachineComponent {
  static var table:Null<Catalog<PushInFittingRow>>;
  public final spec:PushInFittingRow;

  public static function catalog():Catalog<PushInFittingRow> {
    if (table == null) table = new Catalog(
      "push-in fitting",
      row -> row.designation,
      [{
        maker: "Schmalz",
        designation: "10.08.02.00203",
        label: "STV-GE G1/4-AG 4",
        envelopeDiameterMm: 8.6,
        envelopeLengthMm: 19.6,
        tubeOdMm: 4,
        thread: "G1/4-M",
        massKg: 0.015
      }],
      _ -> ({
        source: "https://www.schmalz.com/en-gb/products/vacuum-technology-for-automation-301607/vacuum-components-301608/filters-and-connections-308965/hoses-and-connections-309034/screw-in-push-fittings-309092/10.08.02.00203",
        standard: null,
        standardEdition: null,
        dimensionKind: Mixed,
        conformance: NominalEnvelope,
        verifiedFields: ["envelopeDiameterMm", "envelopeLengthMm", "tubeOdMm", "thread", "massKg"]
      })
    );
    return table;
  }

  public function new(designation:String) {
    var row = catalog().get(designation);
    super(row.designation, '${row.maker} ${row.label} vacuum fitting', "brass");
    if (Std.isExactType(this, PushInFitting)) addFacet(new machinekit.component.EngineeringMetadata(new machinekit.component.ComponentValues()
      .setToken("ratingsBasis", "Catalog").setToken("errorsBasis", "Assumed")
      .setToken("source", catalog().metadata(designation).source)));
    this.spec = row;
    addConnector("mount", Mount, Solids.axial(0, 0, 0));
    addPort({name: "hose", kind: Vacuum, role: Consumer, iface: PushIn(row.tubeOdMm), required: true});
    addPort({name: "thread", kind: Vacuum, role: Supply, iface: Thread(row.thread), required: false});
    addBridge("hose", "thread");
    var r = row.envelopeDiameterMm / 2, h = row.envelopeLengthMm, m = row.massKg;
    var transverse = m * (3 * r * r + h * h) / 12;
    declareMass(m, new Vector(0, 0, -h / 2), new InertiaTensor(transverse, 0, 0, transverse, 0, m * r * r / 2));
  }

  static var recipeTypeCache:Null<ComponentType>;

  public static function recipeType():ComponentType {
    if (recipeTypeCache == null) recipeTypeCache = new ComponentType(
      "machinekit.pneumatic.push-in-fitting",
      [ComponentRecipeSupport.catalog("designation", PushInFitting.catalog(), "10.08.02.00203")],
      v -> new PushInFitting(v.token("designation")),
      true
    ).procured(v -> {var row = catalog().get(v.token("designation")); return Buy(row.maker, row.designation, null);});
    return recipeTypeCache;
  }

  /** Subclasses must declare their own recipe and saved values. */
  override public function componentType():Null<ComponentType> return Std.isExactType(this,
    PushInFitting) ? recipeType() : null;

  override public function values():machinekit
    .component.ComponentValues return new machinekit.component.ComponentValues().setToken(
      "designation",
      spec.designation
    ).setToken(
      "material",
      materialSpec()
    );

  override public function hasGeometry():Bool return true;

  override public function geometry(detail:ComponentDetail = Preview):Part return Part.cylinderSpan(
    spec.envelopeDiameterMm / 2,
    -spec.envelopeLengthMm,
    0
  );
}
