package machinekit.milling;

import cadkit.modeling.Part;
import cadkit.modeling.Vector;
import machinekit.component.*;
import machinekit.component.ComponentRecipeSupport as P;

/** Horizontal direct-drive cage. +Z goes from pedestal toward motor; +Y is up.
 * Low anchor holes run through the legs and meet the pedestal's front tapped holes.
 * Motor pattern is an explicit generic envelope, pending vendor selection.
 */
class MillServoMount extends MachineComponent {
	static var recipe:Null<ComponentType>;

	public static function parameters():Array<ComponentParameter> return [
		P.length("width", 100),
		P.length("shaftHeight", 68),
		P.length("pedestalHeight", 43)
	];

	public static function recipeType():ComponentType {
		if (recipe == null) recipe = new ComponentType("machinekit.milling.servo-mount", parameters(), v -> new MillServoMount(v));
		return recipe;
	}

	public function new(?values:ComponentValues) {
		var v = P.resolve(parameters(), values);
		if (v.number("width") < 84 || v.number("shaftHeight") < 40 || v.number("pedestalHeight") < 15) throw "Motor mount cannot fit drive";
		super("MILL-SERVO-MOUNT", "Two plate direct servo mount", "steel", false, v);
		addConnector("mount", Mount, Solids.axial(0, 0, 0));
	}

	override public function componentType():Null<ComponentType> return recipeType();

	override public function hasGeometry():Bool return true;

	override public function geometry(detail:ComponentDetail = Preview):Part {
		var w = parameterNumber("width"), s = parameterNumber("shaftHeight"), d = s + 33, cy = (33 - s) / 2;
		var pieces = [
			Part.box(w, d, 8).translated(new Vector(0, cy, 0)), Part.box(w, d, 8).translated(new Vector(0, cy, 47))
		];
		for (side in [-1, 1]) pieces.push(Part.box(12, d, 39).translated(new Vector(side * (w / 2 - 6), cy, 8)));
		var tools = [Part.cylinderSpan(14.1, -1, 9), Part.cylinderSpan(8.5, 46, 56)];
		for (x in [-15.0, 15.0]) for (y in [-15.0, 15.0]) tools.push(Part.cylinderSpan(2.75, 46, 56, x, y));
		for (side in [-1, 1]) tools.push(Part.cylinderSpan(
			3.3,
			-1,
			56,
			side * (w / 2 - 8),
			parameterNumber("pedestalHeight") / 2 - s
		));
		return Solids.cut(Solids.union(pieces), tools);
	}
}
