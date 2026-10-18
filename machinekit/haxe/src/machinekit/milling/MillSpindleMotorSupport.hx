package machinekit.milling;

import cadkit.modeling.Part;
import cadkit.modeling.Vector;
import machinekit.component.*;
import machinekit.component.ComponentRecipeSupport as P;

/** Offset spindle-motor shelf support. Bottom foot lands on the tapered collar;
 * the upper bridge reaches the existing shelf. Origin at spindle X/Y, head top.
 * Nominal fabricated geometry; welds and fasteners are not qualified.
 */
class MillSpindleMotorSupport extends MachineComponent {
	static var recipe:Null<ComponentType>;
	public static function recipeType():ComponentType {
		if (recipe == null) recipe = new ComponentType("machinekit.milling.spindle-motor-support", [P.length("height", 27)],
			v -> new MillSpindleMotorSupport(v.number("height")));
		return recipe;
	}
	public function new(height:Float) {
		if (!Math.isFinite(height) || height < 16) throw "Spindle motor support needs room for its bridge";
		super("MILL-SPINDLE-MOTOR-SUPPORT", "Spindle motor shelf bridge on tapered head", "steel", false,
			new ComponentValues().setNumber("height", height));
		addConnector("mount", Mount, Solids.axial(0, 0, 0));
	}
	override public function componentType():Null<ComponentType> return recipeType();
	override public function hasGeometry():Bool return true;
	override public function geometry(detail:ComponentDetail = Preview):Part {
		var h = parameterNumber("height");
		return Solids.union([Part.box(20, 80, h).translated(new Vector(48, 0, 0)),
			Part.box(62, 80, 8).translated(new Vector(69, 0, h - 8))]);
	}
}
