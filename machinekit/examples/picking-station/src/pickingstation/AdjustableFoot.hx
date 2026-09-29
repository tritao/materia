package pickingstation;

import cadkit.modeling.Part;
import machinekit.component.ComponentDetail;
import machinekit.component.MachineComponent;
import machinekit.component.Solids;

/** Small adjustable foot used by the example rack. */
class AdjustableFoot extends MachineComponent {
	override public function componentType():machinekit.component.ComponentType return PickingStationRecipes.foot();
	override public function values():machinekit.component.ComponentValues return PickingStationRecipes.values(this);
	public final diameter:Float;
	public final height:Float;

	public function new(diameter:Float = 35, height:Float = 25) {
		if (!Math.isFinite(diameter) || !Math.isFinite(height) || diameter <= 0 || height <= 0)
			throw "Adjustable foot dimensions must be finite and positive";
		this.diameter = diameter;
		this.height = height;
		super('ADJUSTABLE-FOOT-${fmt(diameter)}', 'Simplified adjustable machine foot', "steel");
	}

	override public function geometry(detail:ComponentDetail = Preview):Part
		return Solids.union([Part.cylinderSpan(diameter / 2, 0, Math.min(8, height)),
			Part.cylinderSpan(diameter * 0.22, Math.min(7, height - 1), height)]);

	static function fmt(value:Float):String return Std.string(Math.round(value * 10) / 10);
}
