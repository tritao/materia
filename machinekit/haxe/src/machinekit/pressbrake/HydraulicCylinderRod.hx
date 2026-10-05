package machinekit.pressbrake;

import cadkit.modeling.Part;
import machinekit.component.ComponentDetail;
import machinekit.component.Dimension;
import machinekit.component.MachineComponent;
import machinekit.component.Solids;

/** Moving hydraulic rod, mounted at its tip and extending back along local -Z. */
class HydraulicCylinderRod extends MachineComponent {
	public final diameter:Float;
	public final length:Float;

	public function new(diameter:Float, length:Float) {
		if (!Math.isFinite(diameter) || !Math.isFinite(length) || !(diameter > 0 && length > 0))
			throw "Hydraulic rod needs positive finite dimensions";
		super('HYDRAULIC-ROD-${Dimension.format(diameter)}x${Dimension.format(length)}',
			"Assumed press-brake hydraulic rod", "steel", true);
		this.diameter = diameter; this.length = length;
		addConnector("mount", Mount, Solids.axial(0, 0, 0));
	}
	override public function hasGeometry():Bool return true;
	override public function geometry(detail:ComponentDetail = Preview):Part
		return Solids.named(Part.cylinderSpan(diameter / 2, -length, 0), "rod");
}
