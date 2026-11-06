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

typedef BayonetChangerMasterRow = {
  var maker:String;
  var designation:String;
  var label:String;
  var nominalPipeDiameterMm:Float;
  var channels:Int;
  var widthMm:Float;
  var depthMm:Float;
  var lengthMm:Float;
  var massKg:Float;
  var outsideThread:String;
}

/** The robot-side half of a manual bayonet tool changer (no lock actuator or signals, unlike `ToolChangerMaster`),
 * from a maker's catalogue, with its pneumatic passages. Box envelope and mass centre are estimates from published bounds. */
class BayonetChangerMaster extends MachineComponent {
  static final table = new machinekit.cache.Once<Catalog<BayonetChangerMasterRow>>();
  public final spec:BayonetChangerMasterRow;

  public static function catalog():Catalog<BayonetChangerMasterRow> {
    return table.get(() -> new Catalog(
      "robot-side bayonet changer",
      row -> row.designation,
      [{
        maker: "Schmalz",
        designation: "10.07.13.00013",
        label: "SXT-QC-CON-40-F-3",
        nominalPipeDiameterMm: 40,
        channels: 3,
        widthMm: 109,
        depthMm: 85,
        lengthMm: 133,
        massKg: 1.7,
        outsideThread: "G1/8-F"
      }],
      _ -> ({
        source: "https://www.schmalz.com/en-us/products/vacuum-technology-for-automation-301607/vacuum-components-301608/mounting-elements-307004/tooling-system-sxt-307259/quick-change-robot-bayonets-sxt-qc-con-307260/10.07.13.00013",
        standard: null,
        standardEdition: null,
        dimensionKind: Mixed,
        conformance: NominalEnvelope,
        verifiedFields: [
          "nominalPipeDiameterMm",
          "channels",
          "widthMm",
          "depthMm",
          "lengthMm",
          "massKg",
          "outsideThread"
        ]
      })
    ));
  }


  public function new(designation:String) {
    var row = catalog().get(designation);
    super(row.designation, '${row.maker} ${row.label} robot-side bayonet', "aluminium 6061");
    this.spec = row;
    addConnector("robot", Mount, Solids.axial(0, 0, 0));
    addConnector("tool", Mount, Solids.axial(0, 0, row.lengthMm));
    for (i in 1...row.channels + 1) {
      addPort({name: 'airIn$i', kind: Pneumatic, role: Consumer, iface: Thread(row.outsideThread), required: false});
      addPort({
        name: 'airOut$i',
        kind: Pneumatic,
        role: Supply,
        iface: Coupling('schmalz:sxt:${spec.nominalPipeDiameterMm}:${spec.channels}', i),
        required: false
      });
      addBridge('airIn$i', 'airOut$i');
    }
    addFacet(new machinekit.component.CouplingFacet(
      'schmalz:sxt:${spec.nominalPipeDiameterMm}:${spec.channels}',
      "tool"
    ));
    var x = row.widthMm, y = row.depthMm, z = row.lengthMm, m = row.massKg;
    declareMass(
      m,
      new Vector(0, 0, z / 2),
      new InertiaTensor(m * (y * y + z * z) / 12, 0, 0, m * (x * x + z * z) / 12, 0, m * (x * x + y * y) / 12)
    );
  }

  static final recipeTypeCache = new machinekit.cache.Once<ComponentType>();

  public static function recipeType():ComponentType {
    return recipeTypeCache.get(() -> new ComponentType(
      "machinekit.robotics.bayonet-changer-master",
      [ComponentRecipeSupport.catalog("designation", BayonetChangerMaster.catalog(), "10.07.13.00013")],
      v -> new BayonetChangerMaster(v.token("designation")),
      true
    ));
  }

  /** Subclasses must declare their own recipe and saved values. */
  override public function componentType():Null<ComponentType> return Std.isExactType(this,
    BayonetChangerMaster) ? recipeType() : null;

  override public function values():machinekit
    .component.ComponentValues return new machinekit.component.ComponentValues().setToken(
      "designation",
      spec.designation
    ).setToken(
      "material",
      materialSpec()
    );

  override public function hasGeometry():Bool return true;

  override public function geometry(detail:ComponentDetail = Preview):Part return Part.box(
    spec.widthMm,
    spec.depthMm,
    spec.lengthMm
  );
}
