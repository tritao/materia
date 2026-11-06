package machinekit.robotics;

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

typedef BayonetChangerToolRow = {
  var maker:String;
  var designation:String;
  var label:String;
  var nominalPipeDiameterMm:Float;
  var channels:Int;
  var envelopeDiameterMm:Float;
  var lengthMm:Float;
  var massKg:Float;
  var outsideThread:String;
}

/** The tool-side half of a manual bayonet tool changer, from a maker's catalogue, with its pneumatic passages.
 * Cylindrical envelope and mass centre are estimates from published bounds. */
class BayonetChangerTool extends MachineComponent {
  static final table = new machinekit.cache.Once<Catalog<BayonetChangerToolRow>>();
  public final spec:BayonetChangerToolRow;

  public static function catalog():Catalog<BayonetChangerToolRow> {
    return table.get(() -> new Catalog(
      "tool-side bayonet changer",
      row -> row.designation,
      [{
        maker: "Schmalz",
        designation: "10.07.13.00018",
        label: "SXT-QC-CON-40-M-3",
        nominalPipeDiameterMm: 40,
        channels: 3,
        envelopeDiameterMm: 68.3,
        lengthMm: 156.5,
        massKg: 0.55,
        outsideThread: "G1/8-F"
      }],
      _ -> ({
        source: "https://www.schmalz.com/en-ca/products/vacuum-technology-for-automation-301607/vacuum-components-301608/mounting-elements-307004/tooling-system-sxt-307259/quick-change-robot-bayonets-sxt-qc-con-307260/10.07.13.00018",
        standard: null,
        standardEdition: null,
        dimensionKind: Mixed,
        conformance: NominalEnvelope,
        verifiedFields: [
          "nominalPipeDiameterMm",
          "channels",
          "envelopeDiameterMm",
          "lengthMm",
          "massKg",
          "outsideThread"
        ]
      })
    ));
  }


  public function new(designation:String) {
    var row = catalog().get(designation);
    super(row.designation, '${row.maker} ${row.label} tool-side bayonet', "aluminium 6061");
    this.spec = row;
    addConnector("master", Mount, Solids.axial(0, 0, 0));
    addConnector("payload", Mount, Solids.axial(0, 0, row.lengthMm));
    for (i in 1...row.channels + 1) {
      addPort({
        name: 'airIn$i',
        kind: Pneumatic,
        role: Consumer,
        iface: Coupling('schmalz:sxt:${spec.nominalPipeDiameterMm}:${spec.channels}', i),
        required: false
      });
      addPort({name: 'airOut$i', kind: Pneumatic, role: Supply, iface: Thread(row.outsideThread), required: false});
      addBridge('airIn$i', 'airOut$i');
    }
    addFacet(new machinekit.component.CouplingFacet(
      'schmalz:sxt:${spec.nominalPipeDiameterMm}:${spec.channels}',
      "master"
    ));
    var r = row.envelopeDiameterMm / 2, z = row.lengthMm, m = row.massKg;
    var transverse = m * (3 * r * r + z * z) / 12;
    declareMass(m, new Vector(0, 0, z / 2), new InertiaTensor(transverse, 0, 0, transverse, 0, m * r * r / 2));
  }

  static final recipeTypeCache = new machinekit.cache.Once<ComponentType>();

  public static function recipeType():ComponentType {
    return recipeTypeCache.get(() -> new ComponentType(
      "machinekit.robotics.bayonet-changer-tool",
      [ComponentRecipeSupport.catalog("designation", BayonetChangerTool.catalog(), "10.07.13.00018")],
      v -> new BayonetChangerTool(v.token("designation")),
      true
    ));
  }

  /** Subclasses must declare their own recipe and saved values. */
  override public function componentType():Null<ComponentType> return Std.isExactType(this,
    BayonetChangerTool) ? recipeType() : null;

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
    0,
    spec.lengthMm
  );
}
