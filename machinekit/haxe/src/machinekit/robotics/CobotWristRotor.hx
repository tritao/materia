package machinekit.robotics;

import cadkit.modeling.Part;
import cadkit.modeling.Vector;
import machinekit.component.ComponentDetail;
import machinekit.component.MachineComponent;
import machinekit.component.Solids;

/** Assumed bearing-supported output flange, carried by the driven link. Its
 * annular skirt clears the installed gearhead. The bearing running clearance is
 * represented geometrically; this component does not specify rolling elements.
 */
class CobotWristRotor extends MachineComponent {
  public final module:CobotJoint;
  public final clocking:Float;
  public function new(drawing:String, module:CobotJoint, clocking:Float = 0) {
    super('COBOT-WRIST-ROTOR-${module.size}-C${clocking}', "Rotating wrist output flange", "aluminium 6061", true);
    declareDrawingNumber(drawing);
    addFacet(new machinekit.component.ProcurementIntent(Make("CNC machining"), [
      machinekit.component.Procurement.ProcurementCodec.requirement("moduleSize", "1", null, null, module.size),
      machinekit.component.Procurement.ProcurementCodec.requirement("clocking", "rad", null, null, clocking)
    ]));
    addFacet(machinekit.component.EngineeringMetadata.assumed(""));
    this.module = module;
    this.clocking = clocking;
    addConnector("face", Mount, Solids.axial(0, 0, 0));
  }
  override public function revisionFingerprint():Null<String> {
    if (drawingNumber == null || module.bom.fingerprint == null) return null;
    return machinekit.component.BomRevision.fingerprint([drawingNumber, materialSpec(), module.bom.fingerprint,
      machinekit.component.ComponentNumberKey.encode(clocking)].join("|"), 1);
  }
  override public function hasGeometry():Bool return true;
  override public function geometry(detail:ComponentDetail = Preview):Part {
    var radius = CobotWristMount.hubRadius(module);
    var ring = Solids.cut(
      Part.cylinderSpan(radius, -8, 0),
      [Part.cylinderSpan(module.pocketDiameter / 2 + 0.5, -8.1, 0.1)]
    );
    var body = Solids.union([
      Solids.named(ring, "bearing-skirt"),
      Solids.named(Part.cylinderSpan(radius, -4, 0), "output-plate")
    ]);
    var cuts = CobotWristMount.housingCuts(module, true, clocking);
    var moved:Array<Part> = [];
    for (cut in cuts) {
      moved.push(cut.translated(new Vector(0, 0, -module.length)));
      cut.close();
    }
    return Solids.cut(body, moved);
  }
}
