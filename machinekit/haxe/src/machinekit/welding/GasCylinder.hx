package machinekit.welding;

import cadkit.modeling.Part;
import cadkit.modeling.Vector;
import machinekit.component.ComponentDetail;
import machinekit.component.MachineComponent;
import machinekit.component.Solids;

/** Shielding gas cylinder with its regulator: a 50 litre steel bottle 1.2 m tall.
 *
 * CAD frame: standing on its base at z=0 on the origin, the regulator on top. Connector: `base`
 * (+Y along +Z). Port: the regulated `gas` outlet.
 */
class GasCylinder extends MachineComponent {
  public static inline var DIAMETER:Float = 230;
  public static inline var BOTTLE_HEIGHT:Float = 1100;
  public static inline var HEIGHT:Float = 1200;
  static inline var REGULATOR_DIAMETER:Float = 120;

  /** What the bottle holds, such as "Ar/CO2 82/18". */
  public final mixture:String;

  public function new(mixture:String = "Ar/CO2 82/18") {
    if (mixture == null || mixture.length == 0) throw "Gas cylinder needs a gas mixture";
    super(
      'GAS-CYLINDER-50L-${mixture.split(" ").join("-").split("/").join("-")}',
      'Shielding gas cylinder, 50 L, $mixture, with regulator',
      "painted steel",
      true
    );
    addFacet(new machinekit.component.ProcurementIntent(Buy(null, null, null), [
      machinekit.component.Procurement.ProcurementCodec.choice("mixture", mixture),
      machinekit.component.Procurement.ProcurementCodec.requirement("cylinderCapacity", "l", null, null, 50),
      machinekit.component.Procurement.ProcurementCodec.choice("regulator", "included")
    ]));
    addFacet(new machinekit.component.EngineeringMetadata(new machinekit.component.ComponentValues()
      .setToken("ratingsBasis", "Assumed").setToken("errorsBasis", "Assumed").setToken("source", "")));
    this.mixture = mixture;
    addConnector("base", Mount, Solids.axial(0, 0, 0));
    addPort({name: "gas", kind: Gas, role: Supply, iface: WeldingInterfaces.gas(), required: false});
    declareMass(75, new Vector(0, 0, 0.4 * HEIGHT));
  }

  override function get_bom():machinekit.component.BomItem {
    var item = super.get_bom();
    item.partNumber = 'GAS-CYLINDER-50L-${mixture.toUpperCase().split(" ").join("-").split("/").join("-")}';
    return item;
  }

  override public function revisionFingerprint():Null<String> return machinekit.component.BomRevision.fingerprint(
    ["welding-gas-cylinder", mixture, machinekit.component.ComponentNumberKey.encode(DIAMETER),
      machinekit.component.ComponentNumberKey.encode(BOTTLE_HEIGHT), machinekit.component.ComponentNumberKey.encode(HEIGHT),
      machinekit.component.ComponentNumberKey.encode(REGULATOR_DIAMETER)].join("|"), 1);

  override public function hasGeometry():Bool return true;

  override public function geometry(detail:ComponentDetail = Preview):Part return Solids.union([
    Solids.named(Part.cylinderSpan(DIAMETER / 2, 0, BOTTLE_HEIGHT), "bottle"),
    Solids.named(Part.cylinderSpan(REGULATOR_DIAMETER / 2, BOTTLE_HEIGHT, HEIGHT), "regulator")
  ]);
}
