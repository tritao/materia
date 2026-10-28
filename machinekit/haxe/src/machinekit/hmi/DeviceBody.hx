package machinekit.hmi;

import cadkit.modeling.Part;
import cadkit.modeling.Vector;
import machinekit.component.*;

/**
 * One body of an expanded device. The first body is the purchase: it carries the device's bill-of-materials line, so the
 * BOM reads as it did before the device was split. Every other body is included in that purchase. Mass is the device's
 * declared mass shared out by volume.
 */
class DeviceBody extends MachineComponent {
  final build:() -> Part;
  final purchase:HmiPart;
  final carriesPurchase:Bool;

  public function new(
    designation:String,
    description:String,
    material:String,
    build:() -> Part,
    purchase:HmiPart,
    carriesPurchase:Bool,
    massKg:Float,
    centreOfMass:Vector,
    ?ports:Array<ComponentPort>
  ) {
    super(designation, description, material, true);
    this.build = build;
    this.purchase = purchase;
    this.carriesPurchase = carriesPurchase;
    if (!carriesPurchase) addFacet(new IncludedInPurchase(purchase.bom.partNumber));
    declareMass(massKg, centreOfMass);
    if (ports != null) for (port in ports) {
      // The body is built in the device's frame, so a port keeps the device's frame for its terminals.
      if (port.connector != null && !Lambda.exists(connectors(), c -> c.name == port.connector)) {
        var terminals = purchase.connector(port.connector);
        addConnector(terminals.name, terminals.role, terminals.frame);
      }
      addPort(port);
    }
  }

  override function get_bom():BomItem return carriesPurchase ? purchase.bom : super.get_bom();

  override public function hasGeometry():Bool return true;
  override public function geometry(detail:ComponentDetail = Preview):Part return build();
}
