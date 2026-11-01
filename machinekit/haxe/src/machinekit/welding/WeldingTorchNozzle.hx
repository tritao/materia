package machinekit.welding;

import cadkit.modeling.Part;
import cadkit.modeling.Vector;
import machinekit.component.ComponentDetail;
import machinekit.component.Dimension;
import machinekit.component.MachineComponent;
import machinekit.component.Solids;

/**
 * The gas nozzle and contact tip of a robot MIG torch (`WeldingTorch`), along +Z of its own frame, mated to the end of the
 * swan neck (`WeldingTorchNeck`). Its own part, so that its collision shape is the nozzle and not a hull around the step
 * from the neck's radius to the nozzle's.
 *
 * CAD frame: the nozzle's base at the origin, the tip pointing along +Z. Connector `base`.
 */
class WeldingTorchNozzle extends MachineComponent {
  final bendDegrees:Float;
  public function new(bendDegrees:Float = 45) {
    super(
      'WELD-TORCH-NOZZLE-${Dimension.format(bendDegrees)}DEG',
      'Robot MIG torch gas nozzle and contact tip',
      "brass",
      true
    );
    addFacet(new machinekit.component.ProcurementIntent(Buy(null, null, null), [
      machinekit.component.Procurement.ProcurementCodec.choice("process", "MIG"),
      machinekit.component.Procurement.ProcurementCodec.requirement("neckBend", "deg", null, null, bendDegrees)
    ]));
    addFacet(new machinekit.component.EngineeringMetadata(new machinekit.component.ComponentValues()
      .setToken("ratingsBasis", "Assumed").setToken("errorsBasis", "Assumed").setToken("source", "")));
    this.bendDegrees = bendDegrees;
    addConnector("base", Mount, Solids.axial(0, 0, 0));
    declareMass(WeldingTorch.NOZZLE_MASS, new Vector(0, 0, 0.5 * WeldingTorch.NOZZLE_LENGTH));
  }

  override public function revisionFingerprint():Null<String> return machinekit.component.BomRevision.fingerprint(
    ["WeldingTorchNozzle", materialSpec(), machinekit.component.ComponentNumberKey.encode(bendDegrees)].join("|"), 1);

  override public function hasGeometry():Bool return true;

  override public function geometry(detail:ComponentDetail = Preview):Part {
    var nozzle = WeldingTorch.NOZZLE_LENGTH;
    return Solids.union([
      Solids.named(Part.cylinderSpan(WeldingTorch.NOZZLE_R, 0, nozzle), "nozzle"),
      Solids.named(Part.cylinderSpan(WeldingTorch.TIP_R, nozzle, nozzle + WeldingTorch.TIP_PROTRUSION), "tip")
    ]);
  }
}
