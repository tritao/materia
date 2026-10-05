package machinekit.pressbrake;

import cadkit.modeling.Part;
import machinekit.component.ComponentType;
import machinekit.component.ComponentValues;
import machinekit.component.ComponentRecipeSupport;
import machinekit.component.ComponentDetail;
import machinekit.component.Dimension;
import machinekit.component.MachineComponent;
import machinekit.component.Solids;

/** Moving hydraulic rod, mounted at its tip and extending back along local -Z. */
class HydraulicCylinderRod extends MachineComponent {
	static var recipe:Null<ComponentType>;
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
	public static function recipeType():ComponentType {
		if (recipe == null) recipe = new ComponentType("machinekit.pressbrake.hydraulic-cylinder-rod", [
			ComponentRecipeSupport.length("diameter", 100), ComponentRecipeSupport.length("length", 190)
		], v -> new HydraulicCylinderRod(v.number("diameter"), v.number("length")));
		return recipe;
	}
	override public function componentType():Null<ComponentType> return recipeType();
	override public function values():ComponentValues return new ComponentValues().setNumber("diameter", diameter)
		.setNumber("length", length).setToken("material", materialSpec());

}
