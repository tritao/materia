package machinekit.welding;

import cadkit.modeling.Part;
import cadkit.modeling.Vector;
import machinekit.component.ComponentDetail;
import machinekit.component.Dimension;
import machinekit.component.MachineComponent;
import machinekit.component.Solids;

/**
 * The swan neck of a robot MIG torch (`WeldingTorch`): the elbow at the bend and the neck, along +Z of its own frame.
 * It is mated to the torch's `neck` connector, which puts +Z along the nozzle's axis. Its own part, so that its collision
 * shape is the neck and not a hull around the whole bent torch.
 *
 * CAD frame: the elbow's centre at the origin, the neck along +Z. Connectors: `base` (the elbow's centre) and `nozzle` (the
 * neck's end, where `WeldingTorchNozzle` is mated).
 */
class WeldingTorchNeck extends MachineComponent {
	public final bendDegrees:Float;

	public function new(bendDegrees:Float = 45) {
		super('WELD-TORCH-NECK-${Dimension.format(bendDegrees)}DEG', 'Robot MIG torch swan neck, ${Dimension.format(bendDegrees)} degrees',
			"brass", true);
		this.bendDegrees = bendDegrees;
		addConnector("base", Mount, Solids.axial(0, 0, 0));
		addConnector("nozzle", Mount, Solids.axial(0, 0, WeldingTorch.NECK_LENGTH));
		declareMass(WeldingTorch.NECK_MASS, new Vector(0, 0, 0.5 * WeldingTorch.NECK_LENGTH));
	}

	override public function hasGeometry():Bool return true;

	override public function geometry(detail:ComponentDetail = Preview):Part {
		return Solids.union([
			Solids.named(Part.sphere(WeldingTorch.NECK_R), "elbow"),
			Solids.named(Part.cylinderSpan(WeldingTorch.NECK_R, 0, WeldingTorch.NECK_LENGTH), "neck")
		]);
	}
}
