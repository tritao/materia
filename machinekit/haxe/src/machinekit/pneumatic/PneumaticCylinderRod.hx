package machinekit.pneumatic;

import cadkit.modeling.Part;
import machinekit.component.ComponentDetail;
import machinekit.component.ComponentType;
import machinekit.component.ComponentValues;
import machinekit.component.ComponentRecipeSupport;
import machinekit.component.MachineComponent;
import machinekit.component.Solids;

/** Assumed cylinder rod envelope. Mount at the tip, with the rod extending along -Z. */
class PneumaticCylinderRod extends MachineComponent {
	public final diameter:Float;
	public final length:Float;
	static var recipe:Null<ComponentType>;
	public function new(diameter:Float, length:Float) {
		if (!(diameter > 0) || !(length > 0) || !Math.isFinite(diameter + length)) throw "Invalid cylinder rod dimensions";
		super('CYLINDER-ROD-${diameter}x$length', "Reference pneumatic cylinder rod", "steel");
		this.diameter = diameter; this.length = length;
		addConnector("mount", Mount, Solids.axial(0, 0, 0));
	}
	override public function hasGeometry():Bool return true;
	override public function geometry(detail:ComponentDetail = Preview):Part
		return Solids.named(Part.cylinderSpan(diameter / 2, -length, 0), "rod");
	public static function recipeType():ComponentType {
		if (recipe == null) recipe = new ComponentType("machinekit.pneumatic.cylinder-rod", [
			ComponentRecipeSupport.length("diameter", 10), ComponentRecipeSupport.length("length", 115)
		], v -> new PneumaticCylinderRod(v.number("diameter"), v.number("length")));
		return recipe;
	}
	override public function componentType():Null<ComponentType> return recipeType();
	override public function values():ComponentValues return new ComponentValues().setNumber("diameter", diameter)
		.setNumber("length", length).setToken("material", materialSpec());
}
